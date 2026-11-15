"""Fixed Toyota window protocol; percentages are timed estimates, not sensors."""
import math

WINDOWS = {
  'front_left': {'subaddress': 0x90, 'shift': 6, 'label': 'Driver'},
  'front_right': {'subaddress': 0x91, 'shift': 4, 'label': 'Front passenger'},
  'rear_left': {'subaddress': 0x93, 'shift': 0, 'label': 'Rear left'},
  'rear_right': {'subaddress': 0x92, 'shift': 2, 'label': 'Rear right'},
}
ACTUATORS = {**WINDOWS, 'moonroof': {'subaddress': 0xad, 'label': 'Moonroof'}}
GROUPS = {'all': tuple(WINDOWS), 'front': ('front_left', 'front_right'), 'rear': ('rear_left', 'rear_right')}
ALIASES = {'driver': 'front_left', 'passenger': 'front_right', 'front_passenger': 'front_right',
           'rear_driver': 'rear_left', 'rear_passenger': 'rear_right'}
MAX_MOVE_SECONDS = 6.


def number(value, low, high, name):
  if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
    raise ValueError(f'{name} must be a number from {low} to {high}')
  return float(value)


def names(window):
  if not isinstance(window, str):
    raise ValueError('window must be a window name or group')
  window = ALIASES.get(window, window)
  if window in GROUPS:
    return GROUPS[window]
  if window in ACTUATORS:
    return (window,)
  raise ValueError('Unknown window; use front_left, front_right, rear_left, rear_right, all, front, or rear')


def validate_command(data):
  if not isinstance(data, dict) or set(data) - {'window', 'action', 'duration_s', 'open_percent'}:
    raise ValueError('Command must contain only window, action, duration_s, or open_percent')
  windows = names(data.get('window', 'all'))
  action = data.get('action')
  if not isinstance(action, str):
    raise ValueError('action is required')
  if windows == ('moonroof',):
    if action not in ('slide_open', 'close_slide', 'tilt_up', 'tilt_down'):
      raise ValueError('Moonroof action must be slide_open, close_slide, tilt_up, or tilt_down')
    if 'open_percent' in data:
      raise ValueError('Moonroof percentage feedback is unavailable')
    result = {'windows': ['moonroof'], 'action': action}
    if 'duration_s' in data:
      result['duration_s'] = number(data['duration_s'], .25, MAX_MOVE_SECONDS, 'duration_s')
    return result
  action = {'open': 'down', 'close': 'up'}.get(action, action)
  if action not in ('up', 'down', 'stop', 'position', 'calibrate'):
    raise ValueError('action must be up, down, stop, position, or calibrate')
  out = {'windows': list(windows), 'action': action}
  if action == 'position':
    if 'duration_s' in data:
      raise ValueError('position uses open_percent, not duration_s')
    out['open_percent'] = number(data.get('open_percent'), 0., 100., 'open_percent')
  elif 'open_percent' in data:
    raise ValueError('open_percent requires action=position')
  if 'duration_s' in data:
    if action not in ('up', 'down'):
      raise ValueError('duration_s is only accepted for up or down')
    out['duration_s'] = number(data['duration_s'], .25, MAX_MOVE_SECONDS, 'duration_s')
  return out


def packet(window, action, duration_s=1.):
  if window not in ACTUATORS or action not in ('up', 'down', 'stop', 'probe', 'roof_raise'):
    raise ValueError('Unknown window operation')
  if action == 'roof_raise' and window != 'moonroof':
    raise ValueError('Roof direction is restricted to the roof actuator')
  sub = ACTUATORS[window]['subaddress']
  if action == 'probe':
    return bytes([sub, 1, 0x3e, 0, 0, 0, 0, 0])
  seconds = 0 if action == 'stop' else math.ceil(number(duration_s, .1, MAX_MOVE_SECONDS, 'duration_s'))
  output = {'up': 0x80, 'down': 0x40, 'roof_raise': 0x20, 'stop': 0}[action]
  return bytes([sub, 4, 0x30, 1, seconds, output, 0, 0])


def decode_status(payload):
  if len(payload) != 8:
    raise ValueError('Window status must be eight bytes')
  labels = {0: 'unknown', 1: 'open', 2: 'closed', 3: 'partly_open'}
  return {name: {'state': labels[(payload[4] >> spec['shift']) & 3],
                 'open_percent': {1: 100., 2: 0.}.get((payload[4] >> spec['shift']) & 3)}
          for name, spec in WINDOWS.items()}


def guard_stationary(state, now, controls_allowed):
  # Gear is broadcast at approximately 1 Hz; wheel speeds arrive much faster.
  for address, max_age in ((0xaa, .5), (0x3bc, 2.)):
    if address not in state or not 0. <= now - state[address][0] <= max_age or len(state[address][1]) != 8:
      raise RuntimeError('Fresh raw wheel-speed and gear signals are required')
  wheels, gear = state[0xaa][1], state[0x3bc][1]
  speeds = [int.from_bytes(wheels[i:i + 2], 'big') * .01 - 67.67 for i in (0, 2, 4, 6)]
  if (gear[1] & 0x3f) != 32 or any(abs(speed) > .2 for speed in speeds) or controls_allowed:
    raise RuntimeError('Window diagnostics require stationary Park and disabled driving controls')


def operation_response(payload, window):
  if len(payload) < 3 or payload[0] != ACTUATORS[window]['subaddress']:
    return None
  length = payload[1] & 15
  return bytes(payload[2:2 + length])


def decode_roof(payload):
  if len(payload) != 8:
    raise ValueError('Roof status must be eight bytes')
  return {'state': 'not_fully_closed' if payload[7] & 1 else 'closed',
          'open_percent': None, 'slide_or_tilt_measured': False}
