"""Temporarily hand the Panda to diagnostics and restore the driving manager."""
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

ROOT = Path('/data/toadpilot/window-control')
REPO = Path('/data/openpilot')
SETTINGS = ('Model', 'ModelVersion', 'ToadPilotConfig', 'ExperimentalMode', 'ConditionalExperimental',
            'ForceStops', 'StartAccelRamp', 'HumanFollowing', 'AccelerationProfile', 'DecelerationProfile')


def atomic_json(path, value, mode=0o600):
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(path.name + '.tmp')
  fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, mode)
  with os.fdopen(fd, 'w') as out:
    json.dump(value, out, indent=2, allow_nan=False)
    out.write('\n')
    out.flush()
    os.fsync(out.fileno())
  os.replace(temporary, path)


def manager_pids():
  result = []
  for proc in Path('/proc').iterdir():
    if proc.name.isdigit():
      try:
        args = (proc / 'cmdline').read_bytes().split(b'\0')
        if len(args) > 1 and args[1] in (b'./manager.py', b'/data/openpilot/system/manager/manager.py') and (proc / 'cwd').resolve() == REPO / 'system/manager':
          result.append(int(proc.name))
      except (OSError, RuntimeError):
        pass
  return sorted(result)


def check_parked():
  import cereal.messaging as messaging
  from cereal import car
  from openpilot.common.params import Params
  params = Params()
  raw = params.get('CarParams')
  if not raw:
    raise RuntimeError('CarParams is unavailable')
  with car.CarParams.from_bytes(raw) as cp:
    if cp.carFingerprint != 'TOYOTA_CAMRY_TSS2' or cp.carName != 'toyota':
      raise RuntimeError('This window protocol is restricted to the validated Camry platform')
  if any(params.get_bool(k) for k in ('DoReboot', 'DoShutdown', 'DoUninstall')):
    raise RuntimeError('Another lifecycle operation is pending')
  sm = messaging.SubMaster(['carState', 'controlsState'])
  end, good = time.monotonic() + 5., 0
  while time.monotonic() < end:
    sm.update(100)
    if not sm.updated['carState'] or not all(sm.valid[s] and time.monotonic() - sm.recv_time[s] < .5 for s in ('carState', 'controlsState')):
      continue
    cs, controls = sm['carState'], sm['controlsState']
    if abs(cs.vEgo) > .01 or str(cs.gearShifter) != 'park' or controls.enabled or controls.active:
      raise RuntimeError('Window diagnostics require stationary Park and disabled driving controls')
    good += 1
    if good >= 5:
      return
  raise RuntimeError('Fresh parked vehicle state could not be confirmed')


def launch():
  env = json.loads((ROOT / 'manager-environment.json').read_text())
  os.chdir(REPO / 'system/manager')
  os.execve(sys.executable, [sys.executable, './manager.py'], env)


def restore_manager():
  if manager_pids():
    return
  command = shlex.join([sys.executable, '-m', 'openpilot.selfdrive.frogpilot.window_control.stack', '--launch'])
  restored = subprocess.run(['tmux', 'respawn-pane', '-k', '-t', '%0', command], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  if restored.returncode:
    subprocess.run(['tmux', 'new-session', '-d', '-s', 'comma', '-n', 'openpilot', command], check=True)


def wait_healthy():
  import cereal.messaging as messaging
  services = ['managerState', 'carState', 'controlsState', 'modelV2', 'longitudinalPlan', 'pandaStates']
  sm = messaging.SubMaster(services)
  end, good = time.monotonic() + 100., 0
  while time.monotonic() < end:
    sm.update(100)
    missing = [p.name for p in sm['managerState'].processes if p.shouldBeRunning and not p.running]
    fresh = all(sm.valid[s] and time.monotonic() - sm.recv_time[s] < 2. for s in services)
    restored = any(str(p.safetyModel) == 'toyota' for p in sm['pandaStates'])
    good = good + 1 if fresh and not missing and restored else 0
    if good >= 5:
      return
  raise RuntimeError('Driving manager was restarted, but service health has not been confirmed')


class DrivingHandover:
  def __init__(self, update):
    self.update, self.stopped = update, False

  def __enter__(self):
    from openpilot.common.params import Params
    check_parked()
    self.params = Params()
    self.settings = {key: self.params.get(key) for key in SETTINGS}
    pids = manager_pids()
    if not pids:
      raise RuntimeError('Driving manager was not found')
    raw = Path('/proc', str(pids[0]), 'environ').read_bytes()
    env = {entry.split(b'=', 1)[0].decode(): entry.split(b'=', 1)[1].decode() for entry in raw.split(b'\0') if b'=' in entry}
    env['PYTHONPATH'] = str(REPO)
    atomic_json(ROOT / 'manager-environment.json', env)
    # A durable marker lets the separate API service recover after its own crash.
    atomic_json(ROOT / 'handover-active.json', {'started_at': time.time(), 'owner_pid': os.getpid(),
                                             'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip()})
    self.update('stopping_driving_software')
    try:
      os.kill(pids[0], signal.SIGTERM)
      self.stopped = True
      end = time.monotonic() + 40.
      while manager_pids() and time.monotonic() < end:
        time.sleep(.1)
      if manager_pids():
        raise RuntimeError('Clean driving-software shutdown did not finish')
    except BaseException:
      self.__exit__(*sys.exc_info())
      raise
    return self

  def __exit__(self, *_):
    if self.stopped:
      self.update('restoring_driving_software')
      restore_manager()
      wait_healthy()
      if any(self.params.get(key) != value for key, value in self.settings.items()):
        raise RuntimeError('A driving preference changed during the diagnostic handover')
    (ROOT / 'handover-active.json').unlink(missing_ok=True)


if __name__ == '__main__':
  if sys.argv[1:] == ['--launch']:
    launch()
