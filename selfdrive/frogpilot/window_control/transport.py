"""Normal Toyota-mode transport through card; no direct USB or safety-mode changes."""
import json
from pathlib import Path
import socket
import tempfile
import time
import uuid

from .bridge import SOCKET_PATH
from .protocol import WINDOWS, decode_status, decode_roof, guard_stationary, operation_response
from .stack import ROOT, atomic_json


class Cancelled(RuntimeError):
  pass


class WindowTransport:
  def __init__(self, cancelled, update, state_update, vehicle_update=None):
    self.cancelled, self.update, self.state_update = cancelled, update, state_update
    self.vehicle_update = vehicle_update
    self.states, self.replies = {}, []
    self.bus, self.active_window = 0, None
    self.reachable = set()
    self.deadline = time.monotonic() + 90.
    self.sock = None
    self.directory = None

  def __enter__(self):
    import cereal.messaging as messaging
    try:
      self.sm = messaging.SubMaster(['carState', 'controlsState', 'pandaStates'])
      self.can = messaging.sub_sock('can')
      self.directory = tempfile.TemporaryDirectory(prefix='toad-body-', dir='/dev/shm')
      self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
      self.sock.bind(str(Path(self.directory.name) / 'client.sock'))
      self.sock.settimeout(.35)
      self.pump(1.3)
      self.guard()
      if not Path(SOCKET_PATH).exists():
        raise RuntimeError('Normal Toyota body-control bridge is unavailable; restart required')
      self.update('toyota_ready', bus=0)
      return self
    except BaseException:
      self.__exit__()
      raise

  def __exit__(self, *_):
    try:
      if self.active_window is not None and self.sock is not None:
        self.release(self.active_window)
    finally:
      if self.sock is not None:
        self.sock.close()
        self.sock = None
      if self.directory is not None:
        self.directory.cleanup()

  def pump(self, duration, cancellable=True, check_park=False):
    import cereal.messaging as messaging
    end = time.monotonic() + duration
    while time.monotonic() < end:
      if cancellable and self.cancelled.is_set():
        raise Cancelled('Window job stopped')
      if cancellable and time.monotonic() > self.deadline:
        raise RuntimeError('Window job exceeded its ninety-second budget')
      self.sm.update(0)
      if self.vehicle_update is not None:
        now = time.monotonic()
        cs, controls = self.sm['carState'], self.sm['controlsState']
        self.vehicle_update({'fresh': all(self.sm.valid[s] and now - self.sm.recv_time[s] < .5 for s in ('carState', 'controlsState')),
                             'mono': now, 'speed': float(cs.vEgo), 'gear': str(cs.gearShifter),
                             'controls_active': bool(controls.enabled or controls.active)})
      for _ in range(200):
        message = messaging.recv_one_or_none(self.can)
        if message is None:
          break
        now = time.monotonic()
        for frame in message.can:
          if frame.src != 0:
            continue
          address, data = frame.address, bytes(frame.dat)
          if address in (0xaa, 0x3bc, 0x638, 0x620):
            self.states.setdefault(0, {})[address] = (now, data)
          if address == 0x638:
            self.state_update(decode_status(data), now)
          if address == 0x620:
            self.state_update({'moonroof': decode_roof(data)}, now)
          if address == 0x758:
            self.replies.append(data)
            self.replies = self.replies[-128:]
      if check_park:
        self.guard()
      time.sleep(.005)

  def guard(self):
    now = time.monotonic()
    if not all(self.sm.valid[s] and now - self.sm.recv_time[s] < .5 for s in ('carState', 'controlsState')):
      raise RuntimeError('Fresh vehicle and driving-controls state required')
    controls = self.sm['controlsState']
    cs = self.sm['carState']
    if str(cs.gearShifter) != 'park' or abs(cs.vEgo) > .01:
      raise RuntimeError('Stationary Park required')
    pandas = self.sm['pandaStates']
    if not pandas or now - self.sm.recv_time['pandaStates'] > 2. or any(str(p.safetyModel) != 'toyota' for p in pandas):
      raise RuntimeError('Normal Toyota Panda safety mode required')
    guard_stationary(self.states.get(0, {}), now, bool(controls.enabled or controls.active or any(p.controlsAllowed for p in pandas)))

  def status(self):
    row = self.states.get(0, {}).get(0x638)
    if row is None or time.monotonic() - row[0] > 2.:
      raise RuntimeError('Window-position status is stale')
    result = decode_status(row[1])
    roof = self.states.get(0, {}).get(0x620)
    if roof is not None and time.monotonic() - roof[0] < 2.:
      result['moonroof'] = decode_roof(roof[1])
    return result

  def transmit(self, window, action, seconds=1.):
    if action != 'stop':
      self.guard()
      if self.cancelled.is_set():
        raise Cancelled('Window job stopped')
    request_id = uuid.uuid4().hex
    command = {'id': request_id, 'window': window, 'action': action, 'duration_s': seconds,
               'expires': time.monotonic() + .75}
    self.sock.sendto(json.dumps(command).encode(), SOCKET_PATH)
    reply = json.loads(self.sock.recv(1024))
    if not reply.get('ok') or reply.get('id') != request_id:
      raise RuntimeError(reply.get('error', 'Body bridge did not accept command'))

  def probe(self, window):
    if window in self.reachable:
      return
    # Empty the short response buffer so a previous motor's reply cannot qualify.
    self.replies.clear()
    self.transmit(window, 'probe')
    self.pump(.3, check_park=True)
    replies = [operation_response(row, window) for row in self.replies]
    if not any(reply == b'\x7e' for reply in replies):
      raise RuntimeError(f'{window}: window-motor diagnostic handshake was not accepted')
    self.reachable.add(window)

  def release(self, window):
    # Releasing an output remains available if the car leaves Park or STOP is requested.
    self.transmit(window, 'stop')
    self.pump(.2, cancellable=False)
    self.active_window = None

  def move(self, window, action, duration=None):
    self.probe(window)
    before = self.status()[window]
    target = 'closed' if action == 'up' else 'open'
    if duration is None and before['state'] == target:
      return {'window': window, 'action': action, 'before': before, 'after': before, 'already_at_endpoint': True}
    seconds = 6. if duration is None else duration
    self.replies.clear()
    started = time.monotonic()
    self.update('moving', window=window, action=action, duration_s=seconds)
    self.active_window = window
    accepted = False
    try:
      self.transmit(window, action, seconds)
      while time.monotonic() - started < seconds:
        self.pump(min(.05, max(0., seconds - (time.monotonic() - started))), check_park=True)
        for row in self.replies:
          reply = operation_response(row, window)
          if reply == b'\x70\x01':
            accepted = True
          elif reply and reply.startswith(b'\x7f\x30'):
            raise RuntimeError(f'{window}: motor rejected the operation ({reply.hex()})')
        if time.monotonic() - started > .5 and not accepted:
          raise RuntimeError(f'{window}: no positive motor acknowledgement')
        if duration is None and accepted and self.status()[window]['state'] == target:
          break
    finally:
      self.release(window)
    elapsed = time.monotonic() - started - .2
    self.pump(.15, check_park=True)
    after = self.status()[window]
    if not accepted:
      raise RuntimeError(f'{window}: movement was not acknowledged')
    if duration is None and after['state'] != target:
      raise RuntimeError(f'{window}: endpoint was not reported within six seconds')
    return {'window': window, 'action': action, 'before': before, 'after': after,
            'motor_acknowledged': True, 'elapsed_s': round(max(0., elapsed), 3),
            'opening_percentage_measured': after['open_percent'] is not None}

  def execute(self, command, calibration):
    results = []
    for window in command['windows']:
      action = command['action']
      if window == 'moonroof':
        results.append(self.move_roof(action, command.get('duration_s')))
        continue
      if action == 'stop':
        self.release(window)
        results.append({'window': window, 'action': 'stop'})
      elif action == 'calibrate':
        self.move(window, 'up')
        down = self.move(window, 'down')
        up = self.move(window, 'up')
        if not all(.5 <= result['elapsed_s'] <= 6. for result in (down, up)):
          raise RuntimeError('Measured travel time was implausible; calibration was not saved')
        calibration[window] = {'down_s': down['elapsed_s'], 'up_s': up['elapsed_s'], 'measured_at': time.time()}
        atomic_json(ROOT / 'calibration.json', calibration)
        results.append({'window': window, 'action': action, 'calibration': calibration[window], 'after': self.status()[window]})
      elif action == 'position':
        percent = command['open_percent']
        if percent in (0., 100.):
          results.append(self.move(window, 'up' if percent == 0. else 'down'))
          continue
        if window not in calibration:
          raise RuntimeError(f'{window}: run an explicit calibration before requesting a percentage')
        self.move(window, 'up')
        duration = max(.25, min(6., calibration[window]['down_s'] * percent / 100.))
        result = self.move(window, 'down', duration)
        result.update(action='position', requested_open_percent=percent,
                      estimated_open_percent=percent, opening_percentage_measured=False,
                      positioning_method='closed endpoint followed by calibrated timed movement')
        results.append(result)
      else:
        results.append(self.move(window, action, command.get('duration_s')))
    return results

  def move_roof(self, action, duration=None):
    window = 'moonroof'
    self.probe(window)
    before = self.status().get(window)
    if before is None:
      raise RuntimeError('Fresh roof status required')
    closing = action in ('close_slide', 'tilt_down')
    if closing and before['state'] == 'closed':
      return {'window': window, 'action': action, 'before': before, 'after': before, 'already_at_endpoint': True}
    if action == 'tilt_up' and before['state'] != 'closed':
      raise RuntimeError('Tilt up requires the roof to be fully closed first')
    direction = 'roof_raise' if action in ('close_slide', 'tilt_up') else 'down'
    seconds = duration if duration is not None else (2. if action == 'tilt_up' else 12.)
    accepted = False
    self.replies.clear()
    self.active_window = window
    started = time.monotonic()
    self.update('moving', window=window, action=action, duration_s=seconds)
    try:
      self.transmit(window, direction, min(6., seconds))
      next_pulse = started + min(6., seconds) - .1
      while time.monotonic() - started < seconds:
        if time.monotonic() >= next_pulse and seconds - (time.monotonic() - started) > .2:
          pulse = min(6., seconds - (time.monotonic() - started))
          self.transmit(window, direction, pulse)
          next_pulse = time.monotonic() + pulse - .1
        self.pump(.03, check_park=True)
        for row in self.replies:
          reply = operation_response(row, window)
          if reply == b'\x70\x01':
            accepted = True
          elif reply and reply.startswith(b'\x7f\x30'):
            raise RuntimeError('Moonroof motor rejected command: ' + reply.hex())
        if time.monotonic() - started > .5 and not accepted:
          raise RuntimeError('Moonroof motor did not acknowledge the command')
        if closing and accepted and self.status().get(window, {}).get('state') == 'closed':
          break
    finally:
      self.release(window)
    self.pump(.15, check_park=True)
    after = self.status().get(window)
    if not accepted:
      raise RuntimeError('Moonroof movement was not acknowledged')
    if closing and duration is None and after['state'] != 'closed':
      raise RuntimeError('Roof did not report closed within twelve seconds')
    return {'window': window, 'action': action, 'before': before, 'after': after,
            'motor_acknowledged': True, 'elapsed_s': round(time.monotonic() - started, 3),
            'opening_percentage_measured': False, 'positioning_method': 'bounded timed movement with closed-state feedback'}
