import json
import socket
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.frogpilot.window_control.bridge import BodyBridge


@pytest.fixture
def bridge(tmp_path):
  server = BodyBridge(str(tmp_path / 'server'))
  client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
  client.bind(str(tmp_path / 'client'))
  client.settimeout(.1)
  yield server, client, str(tmp_path / 'server')
  client.close()
  server.sock.close()


def command(client, path, action='down', request_id='a' * 32):
  client.sendto(json.dumps({'id': request_id, 'window': 'front_left', 'action': action,
                            'duration_s': .35, 'expires': 10.75}).encode(), path)


def state(**changes):
  return NS(**dict(canValid=True, gearShifter='park', vEgo=0., **changes))


CC = NS(enabled=False, latActive=False, longActive=False)


def test_release_deadline_survives_api_disappearance(bridge):
  server, client, path = bridge
  command(client, path)
  frames = server.update(state(), CC, True, 10.)
  assert len(frames) == 1 and frames[0][2][5] == 0x40
  assert json.loads(client.recv(1024))['ok']
  assert server.update(state(), CC, True, 10.34) == []
  assert server.update(state(), CC, True, 10.36)[0][2][4:6] == bytes(2)


@pytest.mark.parametrize('failure', ['gear', 'speed', 'can', 'engaged', 'stale'])
def test_guard_releases_active_motor_and_blocks_new_motion(bridge, failure):
  server, client, path = bridge
  command(client, path)
  server.update(state(), CC, True, 10.)
  client.recv(1024)
  cs, cc, fresh = state(), NS(**vars(CC)), True
  if failure == 'gear': cs.gearShifter = 'drive'
  if failure == 'speed': cs.vEgo = 1.
  if failure == 'can': cs.canValid = False
  if failure == 'engaged': cc.enabled = True
  if failure == 'stale': fresh = False
  command(client, path, request_id='b' * 32)
  frames = server.update(cs, cc, fresh, 10.1)
  assert len(frames) == 1 and frames[0][2][4:6] == bytes(2)
  assert not json.loads(client.recv(1024))['ok']


def test_expired_and_arbitrary_commands_never_send(bridge):
  server, client, path = bridge
  command(client, path)
  assert server.update(state(), CC, True, 11.) == []
  assert not json.loads(client.recv(1024))['ok']
  client.sendto(b'{"address":1872,"data":"9004300101400000"}', path)
  assert server.update(state(), CC, True, 11.) == []
  assert not json.loads(client.recv(1024))['ok']
