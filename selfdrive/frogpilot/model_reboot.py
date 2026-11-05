"""Explicitly requested model reboot, deferred until controls are fully disengaged.

No cruise-cancel or actuator commands are sent. A daemon thread exists only while
an explicit request is pending; restarting Fleet Manager cancels that request.
"""
import threading
import time

SERVICES = ('carState', 'carControl', 'controlsState')
TIMEOUT = 120.0
STABLE_TIME = 1.0


def disengagement_snapshot(sm, now, params):
  device_fresh = (sm.valid['deviceState'] and sm.alive['deviceState'] and
                  0 <= now - sm.recv_time['deviceState'] <= 1.5 and
                  0 <= now - sm.logMonoTime['deviceState'] / 1e9 <= 1.5)
  if not device_fresh:
    return False, 'Waiting for fresh device state'
  if not sm['deviceState'].started:
    offroad = params.get_bool('IsOffroad') and not params.get_bool('IsOnroad')
    return offroad, 'Offroad' if offroad else 'Waiting for confirmed road state'
  for name in SERVICES:
    if not (sm.valid[name] and sm.alive[name] and 0 <= now - sm.recv_time[name] <= .5 and
            0 <= now - sm.logMonoTime[name] / 1e9 <= .5):
      return False, 'Waiting for fresh control state'
  car = sm['carState']
  command = sm['carControl']
  if car.cruiseState.enabled:
    return False, 'Switch cruise off to continue'
  if sm['controlsState'].enabled or command.enabled or command.latActive or command.longActive:
    return False, 'Waiting for steering and acceleration control to disengage'
  return True, 'Disengagement confirmed'


class ModelReboot:
  def __init__(self, params, installed, clock=time.monotonic):
    self.params = params
    self.installed = installed
    self.clock = clock
    self.lock = threading.RLock()
    self.state = dict(state='idle', pending=False)
    self.generation = 0
    self.ready_since = None
    self.last_check = None

  def status(self):
    with self.lock:
      return dict(self.state)

  def submit(self, model, *, start_thread=True):
    if not isinstance(model, str) or not self.installed(model):
      raise ValueError('Choose one installed model with its matching runtime.')
    with self.lock:
      if self.state['state'] == 'rebooting':
        raise ValueError('Reboot has already started.')
      if self.state['pending']:
        raise ValueError('Cancel the pending reboot before requesting another.')
      self.generation += 1
      generation = self.generation
      self.ready_since = None
      self.last_check = None
      self.state = dict(state='waiting', pending=True, model=model,
                        requested_at=self.clock(), message='Switch cruise off to continue')
      result = dict(self.state)
    if start_thread:
      threading.Thread(target=self._run, args=(generation,), daemon=True, name='model-reboot').start()
    return result

  def cancel(self):
    with self.lock:
      if self.state['state'] == 'rebooting':
        raise ValueError('Reboot has already started.')
      self.generation += 1
      self.ready_since = None
      self.state = dict(state='cancelled', pending=False, message='Cancelled; current model retained')
      return dict(self.state)

  def step(self, generation, ready, message, now=None):
    now = self.clock() if now is None else now
    with self.lock:
      if generation != self.generation or not self.state['pending']:
        return False
      if now - self.state['requested_at'] >= TIMEOUT:
        self.state.update(state='expired', pending=False, message='Timed out; current model retained')
        return False
      if self.last_check is not None and (now < self.last_check or now - self.last_check > .5):
        self.ready_since = None
      self.last_check = now
      self.state['message'] = message
      if not ready:
        self.ready_since = None
        return True
      if self.ready_since is None:
        self.ready_since = now
        return True
      if now - self.ready_since < STABLE_TIME:
        return True
      model = self.state['model']
      if not self.installed(model):
        self.state.update(state='failed', pending=False, message='Model runtime is no longer installed')
        return False
      previous = self.params.get('Model')
      randomizer = self.params.get_bool('ModelRandomizer')
      try:
        # Do not refresh live toggles: the current model continues until manager
        # cleanup. New driving processes load this selection after the reboot.
        self.params.put('Model', model)
        self.params.put_bool('ModelRandomizer', False)
        self.params.put_bool('DoReboot', True)
      except Exception:
        if previous is not None:
          self.params.put('Model', previous)
        self.params.put_bool('ModelRandomizer', randomizer)
        self.state.update(state='failed', pending=False, message='Could not request reboot; model restored')
        return False
      self.state.update(state='rebooting', pending=False, message='Disengaged; rebooting with selected model')
      return False

  def _run(self, generation):
    from cereal import messaging
    try:
      sm = messaging.SubMaster(['deviceState', *SERVICES], frequency=5)
      while True:
        sm.update(0)
        now = self.clock()
        ready, message = disengagement_snapshot(sm, now, self.params)
        if not self.step(generation, ready, message, now):
          return
        time.sleep(.2)
    except Exception:
      with self.lock:
        if generation == self.generation and self.state['pending']:
          self.state.update(state='failed', pending=False, message='Control state unavailable; reboot cancelled')


_instance = None
_instance_lock = threading.Lock()


def coordinator():
  global _instance
  with _instance_lock:
    if _instance is None:
      from openpilot.common.params import Params
      from openpilot.selfdrive.frogpilot.frogpilot_variables import DEFAULT_CLASSIC_MODEL, MODELS_PATH
      from openpilot.selfdrive.toad_modeld.artifact import installed, MODELS
      def available(model):
        if model == DEFAULT_CLASSIC_MODEL:
          return (MODELS_PATH / f'{model}.thneed').is_file()
        return model in MODELS and installed(model)
      _instance = ModelReboot(Params(), available)
    return _instance
