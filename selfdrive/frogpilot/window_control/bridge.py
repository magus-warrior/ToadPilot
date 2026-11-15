"""Bounded body commands appended to card's existing Toyota sendcan stream."""
import json
import os
from pathlib import Path
import socket
import time

from .protocol import ACTUATORS, packet, number

SOCKET_PATH = '/dev/shm/toad-body-control.sock'


class BodyBridge:
  def __init__(self, path=SOCKET_PATH):
    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    self.sock.setblocking(False)
    Path(path).unlink(missing_ok=True)
    self.sock.bind(path)
    os.chmod(path, 0o600)
    self.active, self.seen = {}, {}
    self.ready = False

  def update(self, cs, cc, controls_fresh, now=None):
    now = time.monotonic() if now is None else now
    parked = (controls_fresh and cs.canValid and str(cs.gearShifter) == 'park' and
              abs(cs.vEgo) < .01 and not (cc.enabled or cc.latActive or cc.longActive))
    frames = []
    for window, deadline in list(self.active.items()):
      if now >= deadline or not parked:
        frames.append((0x750, 0, packet(window, 'stop'), 0))
        del self.active[window]
    self.ready = parked
    # A bounded drain prevents another local process from starving the controls loop.
    for _ in range(8):
      try:
        raw, peer = self.sock.recvfrom(1024)
      except BlockingIOError:
        break
      except OSError:
        break
      response = {'ok': False}
      try:
        command = json.loads(raw)
        if not isinstance(command, dict) or set(command) != {'id', 'window', 'action', 'duration_s', 'expires'}:
          raise ValueError('Invalid body command')
        request_id = command['id']
        if not isinstance(request_id, str) or len(request_id) != 32:
          raise ValueError('Invalid request ID')
        number(command['expires'], now, now + 2., 'expires')
        window, action = command['window'], command['action']
        if window not in ACTUATORS or (action not in ('up', 'down', 'stop', 'probe') and
                                       not (window == 'moonroof' and action == 'roof_raise')):
          raise ValueError('Unsupported body operation')
        duration = number(command['duration_s'], .1, 6., 'duration_s')
        if request_id in self.seen:
          response = self.seen[request_id]
        else:
          if action != 'stop' and not parked:
            raise ValueError('Fresh stationary Park and disabled controls required')
          if window in self.active:
            frames.append((0x750, 0, packet(window, 'stop'), 0))
            del self.active[window]
          frames.append((0x750, 0, packet(window, action, duration), 0))
          if action in ('up', 'down', 'roof_raise'):
            self.active[window] = now + duration
          response = {'ok': True, 'id': request_id}
          self.seen[request_id] = response
          if len(self.seen) > 128:
            self.seen.pop(next(iter(self.seen)))
      except (ValueError, TypeError, KeyError) as error:
        response['error'] = str(error)
      if peer:
        try:
          self.sock.sendto(json.dumps(response).encode(), peer)
        except OSError:
          pass
    return frames
