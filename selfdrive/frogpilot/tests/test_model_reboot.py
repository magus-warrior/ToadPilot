from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from openpilot.selfdrive.frogpilot.model_reboot import ModelReboot, disengagement_snapshot


class Params:
  def __init__(self):
    self.data = {'Model': b'old', 'ModelRandomizer': True, 'IsOffroad': False, 'IsOnroad': True}
  def get(self, key):
    return self.data.get(key)
  def get_bool(self, key):
    return bool(self.data.get(key, False))
  def put(self, key, value):
    self.data[key] = value
  def put_bool(self, key, value):
    self.data[key] = value


class State(dict):
  def __init__(self):
    super().__init__(deviceState=NS(started=True), carState=NS(cruiseState=NS(enabled=False)),
      controlsState=NS(enabled=False), carControl=NS(enabled=False, latActive=False, longActive=False))
    self.valid = dict.fromkeys(self, True)
    self.alive = dict.fromkeys(self, True)
    self.recv_time = dict.fromkeys(self, 10.)
    self.logMonoTime = dict.fromkeys(self, int(10e9))


@pytest.mark.parametrize('blocker', ['cruise', 'controls', 'enabled', 'steering', 'acceleration', 'invalid', 'dead', 'stale', 'old_source', 'future', 'missing_device'])
def test_reboot_requires_every_disengagement_check(blocker):
  state, params = State(), Params()
  assert disengagement_snapshot(state, 10., params)[0]
  if blocker == 'cruise': state['carState'].cruiseState.enabled = True
  if blocker == 'controls': state['controlsState'].enabled = True
  if blocker == 'enabled': state['carControl'].enabled = True
  if blocker == 'steering': state['carControl'].latActive = True
  if blocker == 'acceleration': state['carControl'].longActive = True
  if blocker == 'invalid': state.valid['carControl'] = False
  if blocker == 'dead': state.alive['controlsState'] = False
  if blocker == 'stale': state.recv_time['carState'] = 9.
  if blocker == 'old_source': state.logMonoTime['carState'] = int(9e9)
  if blocker == 'future': state.logMonoTime['carState'] = int(11e9)
  if blocker == 'missing_device': state.alive['deviceState'] = False
  assert not disengagement_snapshot(state, 10., params)[0]


def test_offroad_requires_fresh_device_and_matching_flags():
  state, params = State(), Params()
  state['deviceState'].started = False
  assert not disengagement_snapshot(state, 10., params)[0]
  params.data.update(IsOnroad=False, IsOffroad=True)
  assert disengagement_snapshot(state, 10., params)[0]
  assert not disengagement_snapshot(state, 12., params)[0]


def test_model_is_committed_only_after_sustained_disengagement():
  params = Params()
  control = ModelReboot(params, lambda mid: mid == 'new', clock=lambda: 10.)
  control.submit('new', start_thread=False)
  generation = control.generation
  assert params.get('Model') == b'old'
  assert control.step(generation, False, 'Cruise active', 10.)
  for tick in (10.2, 10.4, 10.6, 10.8):
    assert control.step(generation, True, 'Ready', tick)
  assert params.get('Model') == b'old'
  # Re-engaging or stale data resets the entire confirmation window.
  assert control.step(generation, False, 'Steering active', 10.9)
  for tick in (11., 11.2, 11.4, 11.6, 11.8):
    assert control.step(generation, True, 'Ready', tick)
  assert not params.get_bool('DoReboot')
  assert not control.step(generation, True, 'Ready', 12.)
  assert params.get('Model') == 'new'
  assert not params.get_bool('ModelRandomizer')
  assert params.get_bool('DoReboot')
  assert control.status()['state'] == 'rebooting'


@pytest.mark.parametrize('action', ['cancel', 'timeout', 'uninstalled'])
def test_cancel_timeout_or_lost_runtime_never_changes_model(action):
  params = Params()
  installed = Mock(return_value=True)
  control = ModelReboot(params, installed, clock=lambda: 10.)
  control.submit('new', start_thread=False)
  generation = control.generation
  assert control.step(generation, True, 'Ready', 10.)
  if action == 'cancel': control.cancel()
  if action == 'uninstalled': installed.return_value = False
  if action == 'uninstalled':
    for tick in (10.2, 10.4, 10.6, 10.8):
      assert control.step(generation, True, 'Ready', tick)
  assert not control.step(generation, True, 'Ready', 130. if action == 'timeout' else 11.)
  assert params.get('Model') == b'old'
  assert not params.get_bool('DoReboot')
  assert not control.status()['pending']


def test_invalid_and_overlapping_requests_are_rejected():
  control = ModelReboot(Params(), lambda mid: mid == 'new')
  with pytest.raises(ValueError): control.submit('unknown', start_thread=False)
  with pytest.raises(ValueError): control.submit({}, start_thread=False)
  control.submit('new', start_thread=False)
  with pytest.raises(ValueError): control.submit('new', start_thread=False)


def test_reboot_write_failure_restores_model_and_randomizer():
  params = Params()
  control = ModelReboot(params, lambda _: True, clock=lambda: 10.)
  control.submit('new', start_thread=False)
  original_put_bool = params.put_bool
  def put_bool(key, value):
    if key == 'DoReboot': raise OSError('unavailable')
    original_put_bool(key, value)
  params.put_bool = put_bool
  control.step(control.generation, True, 'Ready', 10.)
  for tick in (10.2, 10.4, 10.6, 10.8):
    assert control.step(control.generation, True, 'Ready', tick)
  assert not control.step(control.generation, True, 'Ready', 11.)
  assert params.get('Model') == b'old'
  assert params.get_bool('ModelRandomizer')
  assert control.status()['state'] == 'failed'


def test_a_worker_pause_restarts_the_disengagement_confirmation_window():
  control = ModelReboot(Params(), lambda _: True, clock=lambda: 10.)
  control.submit('new', start_thread=False)
  generation = control.generation
  control.step(generation, True, 'Ready', 10.)
  control.step(generation, True, 'Ready', 10.2)
  assert control.step(generation, True, 'Ready', 20.)
  assert not control.params.get_bool('DoReboot')
  for tick in (20.2, 20.4, 20.6, 20.8):
    assert control.step(generation, True, 'Ready', tick)
  assert not control.step(generation, True, 'Ready', 21.)


def test_http_request_only_queues_and_get_is_read_only():
  from openpilot.selfdrive.frogpilot.fleetmanager.fleet_manager import app
  control = ModelReboot(Params(), lambda mid: mid == 'new')
  with patch('openpilot.selfdrive.frogpilot.model_reboot.coordinator', return_value=control), \
       patch('threading.Thread.start') as start:
    client = app.test_client()
    assert client.get('/toadpilot/model-reboot').json['state'] == 'idle'
    start.assert_not_called()
    assert client.post('/toadpilot/model-reboot', json={'model': 'unknown'}).status_code == 400
    response = client.post('/toadpilot/model-reboot', json={'model': 'new'})
    assert response.json['pending']
    assert response.headers['Cache-Control'] == 'no-store'
    assert control.params.get('Model') == b'old'
    assert not control.params.get_bool('DoReboot')
    start.assert_called_once()
    assert not client.post('/toadpilot/model-reboot', json={'cancel': True}).json['pending']
    assert client.post('/toadpilot/model-reboot', json={'cancel': 'yes'}).status_code == 400
