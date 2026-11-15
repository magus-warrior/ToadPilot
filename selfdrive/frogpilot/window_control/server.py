"""Authenticated loopback REST service for explicit parked window jobs.

Fixed body commands use the existing normal Toyota CAN sender.
Percentage requests require an explicit prior calibration.
"""
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import re
import secrets
import signal
import threading
import time
import uuid

from .protocol import WINDOWS, decode_status, decode_roof, validate_command
from .stack import ROOT, atomic_json, restore_manager, wait_healthy
from .transport import Cancelled, WindowTransport

HOST, PORT = '127.0.0.1', 8766
TOKEN_PATH = ROOT / 'api-token'


def load_json(path, default):
  try:
    return json.loads(path.read_text())
  except (OSError, ValueError):
    return default


def load_token():
  ROOT.mkdir(parents=True, exist_ok=True, mode=0o700)
  os.chmod(ROOT, 0o700)
  try:
    fd = os.open(TOKEN_PATH, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as out:
      out.write(secrets.token_urlsafe(32) + '\n')
  except FileExistsError:
    pass
  os.chmod(TOKEN_PATH, 0o600)
  token = TOKEN_PATH.read_text().strip()
  if len(token) < 32:
    raise RuntimeError('Window API token is invalid')
  return token


def validate_job(data, calibration):
  if not isinstance(data, dict) or set(data) - {'commands', 'confirm_clear', 'request_id'}:
    raise ValueError('Use commands, confirm_clear, and optional request_id')
  if not isinstance(data.get('commands'), list) or not 1 <= len(data['commands']) <= 8:
    raise ValueError('commands must contain one to eight operations')
  commands = [validate_command(command) for command in data['commands']]
  if any(command['action'] == 'stop' for command in commands):
    raise ValueError('Use POST /v1/stop for immediate cancellation')
  needs_clearance = any(command['windows'] == ['moonroof'] or command['action'] in ('up', 'calibrate') or
                        (command['action'] == 'position' and command['open_percent'] < 100.) for command in commands)
  if needs_clearance and data.get('confirm_clear') is not True:
    raise ValueError('Upward movement requires confirm_clear=true; diagnostic operation disables jam protection')
  if 'confirm_clear' in data and not isinstance(data['confirm_clear'], bool):
    raise ValueError('confirm_clear must be boolean')
  available = set(calibration)
  for command in commands:
    if command['action'] == 'calibrate':
      available.update(command['windows'])
    elif command['action'] == 'position' and 0. < command['open_percent'] < 100.:
      missing = set(command['windows']) - available
      if missing:
        raise ValueError('Calibration required before percentage positioning: ' + ', '.join(sorted(missing)))
  request_id = data.get('request_id')
  if request_id is not None and (not isinstance(request_id, str) or not 1 <= len(request_id) <= 96):
    raise ValueError('request_id must be a string from one to 96 characters')
  return commands, request_id


class Controller:
  def __init__(self):
    self.lock, self.pending = threading.RLock(), queue.Queue()
    self.cancelled, self.shutdown = threading.Event(), threading.Event()
    self.active = None
    self.phase = 'ready'
    self.jobs, self.requests = {}, {}
    self.window_state, self.window_mono = {}, 0.
    self.roof_state, self.roof_mono = {}, 0.
    self.calibration = load_json(ROOT / 'calibration.json', {})
    self.verified = load_json(ROOT / 'verification.json', {})
    self.vehicle = {'fresh': False}

  def snapshot(self):
    with self.lock:
      vehicle = dict(self.vehicle)
      vehicle['fresh'] = bool(vehicle.get('fresh') and time.monotonic() - vehicle.get('mono', 0.) < .75)
      return copy.deepcopy({'service': 'window_control', 'phase': self.phase, 'active_job': self.active,
                            'vehicle': vehicle, 'windows': self.window_state,
                            'moonroof': self.roof_state,
                            'moonroof_status_age_s': round(time.monotonic() - self.roof_mono, 3) if self.roof_mono else None,
                            'window_status_age_s': round(time.monotonic() - self.window_mono, 3) if self.window_mono else None,
                            'calibration': self.calibration, 'hardware_verified': self.verified,
                            'window_map': {key: dict(label=value['label'], subaddress=hex(value['subaddress'])) for key, value in WINDOWS.items()},
                            'position_units': 'percent open: 0=closed, 100=open; intermediate positions are timed estimates',
                            'actuation_mode': 'normal Toyota safety mode; parked body IO through card',
                            'jam_protection_during_actuation': False})

  def observe_windows(self, state, mono):
    with self.lock:
      if 'moonroof' in state:
        self.roof_state, self.roof_mono = state['moonroof'], mono
      if any(window in state for window in WINDOWS):
        self.window_state = {key: value for key, value in state.items() if key in WINDOWS}
        self.window_mono = mono

  def observe_vehicle(self, state):
    with self.lock:
      self.vehicle = state

  def job(self, job_id):
    if not re.fullmatch('[a-f0-9]{32}', job_id):
      return None
    with self.lock:
      return copy.deepcopy(self.jobs.get(job_id) or load_json(ROOT / 'jobs' / (job_id + '.json'), None))

  def submit(self, data):
    with self.lock:
      commands, request_id = validate_job(data, self.calibration)
      fingerprint = hashlib.sha256(json.dumps(commands, sort_keys=True).encode()).hexdigest()
      if request_id is not None and request_id in self.requests:
        previous_fingerprint, job_id = self.requests[request_id]
        if previous_fingerprint != fingerprint:
          raise ValueError('request_id was already used with different commands')
        previous = self.job(job_id)
        if previous is None:
          raise RuntimeError('The previous result for this request_id has expired; use a new request_id')
        return previous
      if self.active is not None or self.phase != 'ready':
        raise RuntimeError('Another window job or recovery is active')
      # The transport and card independently validate fresh state again before motion.
      if not self.vehicle.get('fresh') or time.monotonic() - self.vehicle.get('mono', 0.) > .75 or self.vehicle.get('gear') != 'park' or abs(self.vehicle.get('speed', 1.)) > .01 or self.vehicle.get('controls_active'):
        raise RuntimeError('Fresh stationary Park with driving controls disabled is required')
      job_id = uuid.uuid4().hex
      job = {'id': job_id, 'status': 'accepted', 'created_at': time.time(), 'commands': commands,
             'results': [], 'cancel_requested': False, 'request_id': request_id}
      self.jobs[job_id] = job
      self.active, self.phase = job_id, 'accepted'
      self.cancelled.clear()
      if request_id is not None:
        self.requests[request_id] = fingerprint, job_id
        atomic_json(ROOT / 'request-ids.json', self.requests)
      atomic_json(ROOT / 'jobs' / (job_id + '.json'), job)
      self.pending.put(job_id)
      return copy.deepcopy(job)

  def stop(self):
    with self.lock:
      self.cancelled.set()
      if self.active:
        self.jobs[self.active]['cancel_requested'] = True
      return {'stop_requested': True, 'active_job': self.active}

  def run_job(self, job_id):
    job = self.jobs[job_id]
    recovery_failed = False

    def update(phase, **fields):
      with self.lock:
        self.phase = phase
        job.update(status='running', phase=phase, progress=fields)
        atomic_json(ROOT / 'jobs' / (job_id + '.json'), job)

    try:
      if self.cancelled.is_set():
        raise Cancelled('Window job stopped before execution')
      with WindowTransport(self.cancelled, update, self.observe_windows) as transport:
        for command in job['commands']:
          results = transport.execute(command, self.calibration)
          with self.lock:
            job['results'].extend(results)
            for result in results:
              if result['action'] == 'calibrate':
                self.verified[result['window']] = {'up': True, 'down': True}
              if result.get('motor_acknowledged') and result['before']['state'] != result['after']['state']:
                self.verified.setdefault(result['window'], {})[result['action']] = True
            atomic_json(ROOT / 'verification.json', self.verified)
      job['status'] = 'completed'
    except Cancelled as error:
      job.update(status='cancelled', error=str(error))
    except Exception as error:
      job.update(status='failed', error=str(error))
    finally:
      # A leftover marker means restoration needs attention, not another motor command.
      recovery_failed = (ROOT / 'handover-active.json').exists()
      with self.lock:
        job.update(finished_at=time.time(), driving_software_restored=not recovery_failed)
        atomic_json(ROOT / 'jobs' / (job_id + '.json'), job)
        self.active = None
        self.phase = 'recovery_required' if recovery_failed else 'ready'
        # Bounded job retention; never retain bearer tokens or manager environments in a response.
        for path in sorted((ROOT / 'jobs').glob('*.json'), key=lambda p: p.stat().st_mtime)[:-50]:
          path.unlink(missing_ok=True)
        while len(self.jobs) > 50:
          self.jobs.pop(next(iter(self.jobs)))
        self.requests = {key: value for key, value in self.requests.items()
                         if (ROOT / 'jobs' / (value[1] + '.json')).exists()}
        atomic_json(ROOT / 'request-ids.json', self.requests)

  def recover(self):
    marker = ROOT / 'handover-active.json'
    if marker.exists():
      previous = load_json(marker, {})
      boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
      if previous.get('boot_id') != boot_id:
        # The normal device boot owns manager startup after a power cycle.
        marker.unlink()
        return
      self.phase = 'recovering_driving_software'
      restore_manager()
      wait_healthy()
      marker.unlink()
      self.phase = 'ready'

  def run(self):
    import cereal.messaging as messaging
    self.requests = load_json(ROOT / 'request-ids.json', {})
    self.recover()
    observation_errors = []

    def observe():
      try:
        self.observe()
      except BaseException as error:
        observation_errors.append(str(error))
        self.observe_vehicle({'fresh': False})
        self.cancelled.set()
        self.shutdown.set()

    observer = threading.Thread(target=observe, name='body_state_observer', daemon=True)
    observer.start()
    try:
      while not self.shutdown.is_set():
        try:
          job_id = self.pending.get(timeout=.1)
        except queue.Empty:
          continue
        self.run_job(job_id)
    finally:
      self.shutdown.set()
      observer.join(timeout=2.)
    if observation_errors:
      raise RuntimeError('Vehicle state observer failed: ' + '; '.join(observation_errors))

  def observe(self):
    import cereal.messaging as messaging
    sm = messaging.SubMaster(['carState', 'controlsState'])
    can = messaging.sub_sock('can')
    while not self.shutdown.is_set():
      sm.update(0)
      now = time.monotonic()
      fresh = all(sm.valid[s] and now - sm.recv_time[s] < .5 for s in ('carState', 'controlsState'))
      with self.lock:
        self.vehicle = {'fresh': fresh, 'mono': now, 'speed': float(sm['carState'].vEgo),
                        'gear': str(sm['carState'].gearShifter),
                        'controls_active': bool(sm['controlsState'].enabled or sm['controlsState'].active)}
      for _ in range(200):
        message = messaging.recv_one_or_none(can)
        if message is None:
          break
        for frame in message.can:
          if frame.src == 0 and frame.address == 0x638 and len(frame.dat) == 8:
            self.observe_windows(decode_status(frame.dat), time.monotonic())
          if frame.src == 0 and frame.address == 0x620 and len(frame.dat) == 8:
            self.observe_windows({'moonroof': decode_roof(frame.dat)}, time.monotonic())
      self.shutdown.wait(.05)


def make_handler(controller, token):
  class Handler(BaseHTTPRequestHandler):
    server_version = 'WindowControl/1'

    def log_message(self, *_):
      pass

    def respond(self, status, value):
      payload = json.dumps(value, allow_nan=False).encode()
      self.send_response(status)
      self.send_header('Content-Type', 'application/json')
      self.send_header('Cache-Control', 'no-store')
      self.send_header('Content-Length', str(len(payload)))
      self.end_headers()
      self.wfile.write(payload)

    def authorized(self):
      header = self.headers.get('Authorization', '')
      if not secrets.compare_digest(header, 'Bearer ' + token):
        self.respond(401, {'error': 'Bearer authentication required'})
        return False
      return True

    def body(self):
      if self.headers.get_content_type() != 'application/json':
        raise ValueError('Content-Type must be application/json')
      try:
        size = int(self.headers.get('Content-Length', '0'))
      except ValueError:
        raise ValueError('Invalid Content-Length') from None
      if not 1 <= size <= 8192 or self.headers.get('Transfer-Encoding'):
        raise ValueError('JSON body must be from one to 8192 bytes')
      self.connection.settimeout(3.)
      return json.loads(self.rfile.read(size))

    def do_GET(self):
      if not self.authorized():
        return
      if self.path == '/v1/windows':
        self.respond(200, controller.snapshot())
      elif self.path.startswith('/v1/jobs/'):
        job = controller.job(self.path.removeprefix('/v1/jobs/'))
        self.respond(200 if job else 404, job or {'error': 'Unknown job'})
      elif self.path == '/openapi.json':
        self.respond(200, load_json(Path(__file__).with_name('openapi.json'), {}))
      else:
        self.respond(404, {'error': 'Unknown route'})

    def do_POST(self):
      if not self.authorized():
        return
      try:
        data = self.body()
        if self.path == '/v1/stop':
          if data != {}:
            raise ValueError('Stop accepts an empty JSON object')
          self.respond(200, controller.stop())
        elif self.path == '/v1/window-jobs':
          self.respond(202, controller.submit(data))
        else:
          self.respond(404, {'error': 'Unknown route'})
      except (ValueError, TypeError, UnicodeError) as error:
        self.respond(400, {'error': str(error)})
      except RuntimeError as error:
        self.respond(409, {'error': str(error)})

  return Handler


def main():
  token = load_token()
  controller = Controller()
  server = ThreadingHTTPServer((HOST, PORT), make_handler(controller, token))
  worker_errors = []

  def run_worker():
    try:
      controller.run()
    except BaseException as error:
      worker_errors.append(str(error))
      controller.shutdown.set()
      # serve_forever might not have started yet; wait briefly in another thread.
      threading.Thread(target=server.shutdown, daemon=True).start()

  thread = threading.Thread(target=run_worker, name='window_worker')
  thread.start()

  def shutdown(*_):
    controller.stop()
    controller.shutdown.set()
    threading.Thread(target=server.shutdown, daemon=True).start()

  signal.signal(signal.SIGTERM, shutdown)
  signal.signal(signal.SIGINT, shutdown)
  try:
    server.serve_forever(poll_interval=.2)
  finally:
    controller.stop()
    controller.shutdown.set()
    thread.join(timeout=235.)
    server.server_close()
  if worker_errors or thread.is_alive():
    raise RuntimeError('Window worker requires recovery: ' + '; '.join(worker_errors))


if __name__ == '__main__':
  main()
