"""Offline window protocol, request validation, cancellation and HTTP checks."""
import json
from pathlib import Path
import threading
import time
from http.server import ThreadingHTTPServer
from types import SimpleNamespace as NS
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from openpilot.selfdrive.frogpilot.window_control import protocol, server, transport


@pytest.mark.parametrize('name,sub,shift', [('front_left', 0x90, 6), ('front_right', 0x91, 4),
                                          ('rear_left', 0x93, 0), ('rear_right', 0x92, 2)])
def test_window_commands_and_independent_status_bits(name, sub, shift):
  assert protocol.packet(name, 'down', 1.) == bytes([sub, 4, 0x30, 1, 1, 0x40, 0, 0])
  assert protocol.packet(name, 'up', .5) == bytes([sub, 4, 0x30, 1, 1, 0x80, 0, 0])
  assert protocol.packet(name, 'stop') == bytes([sub, 4, 0x30, 1, 0, 0, 0, 0])
  for bits, state, percent in ((1, 'open', 100.), (2, 'closed', 0.), (3, 'partly_open', None)):
    payload = bytearray(8)
    payload[4] = bits << shift
    decoded = protocol.decode_status(payload)
    assert decoded[name] == {'state': state, 'open_percent': percent}
    assert all(value['state'] == 'unknown' for key, value in decoded.items() if key != name)


def test_fractional_duration_rounds_ecu_timeout_up():
  assert protocol.packet('front_left', 'down', 1.25)[4] == 2


@pytest.mark.parametrize('bad', [True, -1., 0., .01, 6.1, float('nan'), float('inf'), '1'])
def test_invalid_durations_never_become_commands(bad):
  with pytest.raises(ValueError):
    protocol.validate_command({'window': 'driver', 'action': 'down', 'duration_s': bad})


def test_groups_and_aliases():
  assert protocol.names('all') == tuple(protocol.WINDOWS)
  assert protocol.names('driver') == ('front_left',)
  assert protocol.names('passenger') == ('front_right',)
  assert protocol.names('rear') == ('rear_left', 'rear_right')
  assert protocol.validate_command({'window': 'driver', 'action': 'close'})['action'] == 'up'


def test_no_raw_can_or_unrelated_controls():
  for command in ({'window': 'driver', 'action': 'down', 'can_id': 0x750},
                  {'window': 'roof', 'action': 'up'}, {'window': 'driver', 'action': 'unlock'}):
    with pytest.raises(ValueError):
      protocol.validate_command(command)


def parked_state(now=10.):
  zero_speed = (6767).to_bytes(2, 'big') * 4
  return {0xaa: (now, zero_speed), 0x3bc: (now, b'\x00\x20' + bytes(6))}


def test_guard_uses_gear_message_rate_and_rejects_motion():
  state = parked_state()
  state[0x3bc] = (8.6, state[0x3bc][1])
  protocol.guard_stationary(state, 10., False)
  state[0xaa] = (10., (6800).to_bytes(2, 'big') * 4)
  with pytest.raises(RuntimeError):
    protocol.guard_stationary(state, 10., False)


@pytest.mark.parametrize('failure', ['drive', 'stale_wheels', 'stale_gear', 'engaged', 'missing'])
def test_park_guard_blocks_invalid_inputs(failure):
  state = parked_state()
  if failure == 'drive':
    state[0x3bc] = (10., bytes(8))
  elif failure == 'stale_wheels':
    state[0xaa] = (9., state[0xaa][1])
  elif failure == 'stale_gear':
    state[0x3bc] = (7., state[0x3bc][1])
  elif failure == 'missing':
    del state[0xaa]
  with pytest.raises(RuntimeError):
    protocol.guard_stationary(state, 10., failure == 'engaged')


def test_up_requires_clearance_and_partial_position_requires_calibration():
  with pytest.raises(ValueError, match='confirm_clear'):
    server.validate_job({'commands': [{'window': 'driver', 'action': 'up'}]}, {})
  with pytest.raises(ValueError, match='Calibration required'):
    server.validate_job({'commands': [{'window': 'driver', 'action': 'position', 'open_percent': 25}], 'confirm_clear': True}, {})
  commands, _ = server.validate_job({'commands': [{'window': 'driver', 'action': 'calibrate'},
                                                {'window': 'driver', 'action': 'position', 'open_percent': 25}],
                                   'confirm_clear': True}, {})
  assert commands[1]['open_percent'] == 25.


def test_endpoint_percentage_needs_no_travel_calibration():
  commands, _ = server.validate_job({'commands': [{'window': 'all', 'action': 'position', 'open_percent': 100}]}, {})
  assert commands[0]['windows'] == list(protocol.WINDOWS)


@pytest.fixture
def controller(tmp_path, monkeypatch):
  monkeypatch.setattr(server, 'ROOT', tmp_path)
  result = server.Controller()
  result.vehicle = {'fresh': True, 'mono': time.monotonic(), 'gear': 'park', 'speed': 0., 'controls_active': False}
  return result


def down_job(request_id='one'):
  return {'commands': [{'window': 'driver', 'action': 'down', 'duration_s': 1.}], 'request_id': request_id}


def test_retries_are_idempotent_and_commands_do_not_queue_behind_active_job(controller):
  first = controller.submit(down_job())
  duplicate = controller.submit(down_job())
  assert duplicate['id'] == first['id']
  assert controller.pending.qsize() == 1
  with pytest.raises(RuntimeError, match='Another window job'):
    controller.submit(down_job('two'))
  changed = down_job()
  changed['commands'][0]['duration_s'] = 2.
  with pytest.raises(ValueError, match='different commands'):
    controller.submit(changed)


@pytest.mark.parametrize('change', [{'gear': 'drive'}, {'speed': 1.}, {'fresh': False},
                                  {'controls_active': True}, {'mono': 0.}])
def test_unparked_or_stale_api_state_cannot_start_handover(controller, change):
  controller.vehicle.update(change)
  with pytest.raises(RuntimeError, match='Park'):
    controller.submit(down_job())
  assert controller.pending.empty()


def test_stop_is_immediate_and_does_not_require_fresh_park(controller):
  accepted = controller.submit(down_job())
  controller.vehicle['fresh'] = False
  assert controller.stop()['active_job'] == accepted['id']
  assert controller.cancelled.is_set()


def test_observation_continues_during_a_blocking_motor_job(controller, monkeypatch):
  job_started, observed_during_job = threading.Event(), threading.Event()

  def observe():
    assert job_started.wait(2.)
    controller.observe_vehicle({'fresh': True, 'mono': time.monotonic(), 'gear': 'park', 'speed': 0., 'controls_active': False})
    observed_during_job.set()
    controller.shutdown.wait(2.)

  def run_job(_):
    job_started.set()
    assert observed_during_job.wait(2.)
    controller.shutdown.set()

  monkeypatch.setattr(controller, 'observe', observe)
  monkeypatch.setattr(controller, 'run_job', run_job)
  controller.pending.put('job')
  controller.run()
  assert observed_during_job.is_set()


def test_snapshot_marks_old_vehicle_state_stale(controller):
  controller.vehicle['mono'] = time.monotonic() - 1.
  assert not controller.snapshot()['vehicle']['fresh']


def test_verified_roof_actions_accept_clearance_and_reject_percentages():
  for action in ('slide_open', 'close_slide', 'tilt_up', 'tilt_down'):
    commands, _ = server.validate_job({'commands': [{'window': 'moonroof', 'action': action}], 'confirm_clear': True}, {})
    assert commands[0]['action'] == action
  with pytest.raises(ValueError, match='confirm_clear'):
    server.validate_job({'commands': [{'window': 'moonroof', 'action': 'tilt_up'}]}, {})
  with pytest.raises(ValueError):
    server.validate_job({'commands': [{'window': 'moonroof', 'action': 'position', 'open_percent': 25}], 'confirm_clear': True}, {})
  assert protocol.packet('moonroof', 'roof_raise', .4) == bytes.fromhex('ad04300101200000')
  with pytest.raises(ValueError):
    protocol.packet('front_left', 'roof_raise', .4)


def test_roof_full_close_renews_bounded_pulse_until_closed(monkeypatch):
  clock, sent = [100.], []
  motor = transport.WindowTransport(threading.Event(), lambda *a, **k: None, lambda *a: None)
  monkeypatch.setattr(transport.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(motor, 'probe', lambda _: None)
  monkeypatch.setattr(motor, 'status', lambda: {'moonroof': {'state': 'closed' if clock[0] >= 106.3 else 'not_fully_closed'}})
  monkeypatch.setattr(motor, 'transmit', lambda w, a, seconds=1.: sent.append((w, a, seconds)))

  def pump(duration, **_):
    clock[0] += duration
    motor.replies = [bytes.fromhex('ad02700100000000')]

  def release(_):
    clock[0] += .2
    motor.active_window = None

  monkeypatch.setattr(motor, 'pump', pump)
  monkeypatch.setattr(motor, 'release', release)
  result = motor.move_roof('close_slide')
  assert result['after']['state'] == 'closed'
  assert len(sent) == 2
  assert all(w == 'moonroof' and a == 'roof_raise' and 0. < seconds <= 6. for w, a, seconds in sent)


def test_cancelled_motion_releases_motor_even_after_park_guard_fails(monkeypatch):
  released = []
  motor = transport.WindowTransport(threading.Event(), lambda *a, **k: None, lambda *a: None)
  monkeypatch.setattr(motor, 'probe', lambda window: None)
  monkeypatch.setattr(motor, 'status', lambda: {'front_left': {'state': 'closed', 'open_percent': 0.}})
  monkeypatch.setattr(motor, 'transmit', lambda *args: None)
  monkeypatch.setattr(motor, 'release', lambda window: released.append(window))

  def cancel(*args, **kwargs):
    raise transport.Cancelled('stop requested')

  monkeypatch.setattr(motor, 'pump', cancel)
  with pytest.raises(transport.Cancelled):
    motor.move('front_left', 'down', 1.)
  assert released == ['front_left']


def test_percentage_reports_estimate_and_homes_to_known_endpoint(monkeypatch):
  calls = []
  motor = transport.WindowTransport(threading.Event(), lambda *a, **k: None, lambda *a: None)

  def move(window, action, duration=None):
    calls.append((window, action, duration))
    return {'window': window, 'action': action, 'after': {'state': 'partly_open', 'open_percent': None}}

  monkeypatch.setattr(motor, 'move', move)
  result = motor.execute({'windows': ['front_left'], 'action': 'position', 'open_percent': 25.},
                         {'front_left': {'down_s': 3.}})[0]
  assert calls == [('front_left', 'up', None), ('front_left', 'down', .75)]
  assert result['estimated_open_percent'] == 25.
  assert result['opening_percentage_measured'] is False
  assert result['after']['open_percent'] is None


def test_http_authentication_status_validation_and_stop(controller):
  token = 'unit-test-token-with-at-least-thirty-two-characters'
  http = ThreadingHTTPServer(('127.0.0.1', 0), server.make_handler(controller, token))
  thread = threading.Thread(target=http.serve_forever, daemon=True)
  thread.start()
  url = f'http://127.0.0.1:{http.server_port}'

  def call(path, data=None, authenticated=True):
    headers = {'Authorization': 'Bearer ' + token} if authenticated else {}
    if data is not None:
      headers['Content-Type'] = 'application/json'
    req = Request(url + path, data=json.dumps(data).encode() if data is not None else None, headers=headers)
    try:
      with urlopen(req, timeout=3.) as response:
        return response.status, json.load(response)
    except HTTPError as error:
      return error.code, json.load(error)

  try:
    assert call('/v1/windows', authenticated=False)[0] == 401
    status, body = call('/v1/windows')
    assert status == 200 and token not in json.dumps(body)
    assert call('/v1/window-jobs', {'commands': [{'window': 'driver', 'action': 'up'}]})[0] == 400
    status, body = call('/v1/window-jobs', down_job())
    assert status == 202
    assert call('/v1/jobs/' + body['id'])[0] == 200
    assert call('/v1/stop', {})[0] == 200
    assert controller.cancelled.is_set()
  finally:
    http.shutdown()
    http.server_close()
    thread.join(timeout=3.)


@pytest.mark.parametrize('current_boot', [False, True])
def test_recovery_does_not_duplicate_normal_manager_startup_after_reboot(controller, monkeypatch, current_boot):
  boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
  marker = server.ROOT / 'handover-active.json'
  marker.write_text(json.dumps({'boot_id': boot_id if current_boot else 'previous-device-boot'}))
  calls = []
  monkeypatch.setattr(server, 'restore_manager', lambda: calls.append('restore'))
  monkeypatch.setattr(server, 'wait_healthy', lambda: calls.append('healthy'))
  controller.recover()
  assert calls == (['restore', 'healthy'] if current_boot else [])
  assert not marker.exists()
  assert controller.phase == 'ready'
