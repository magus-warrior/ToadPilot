#!/usr/bin/env python3
"""Render the entire offroad UI in isolated queues and copied settings."""
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid

from cereal import messaging
from openpilot.common.params import Params

ROOT = Path(__file__).resolve().parents[2]


def main():
  selected = ['LanguageSetting', 'IsMetric', 'HasAcceptedTerms', 'CompletedTrainingVersion', 'TermsVersion', 'TrainingVersion',
              'CarMake', 'CarModel', 'CarModelName',
              'CarParamsPersistent', 'PrimeType', 'Model', 'AvailableModels', 'AvailableModelNames',
              'ModelVersions', 'ExperimentalMode', 'ApiCache_DriveStats', 'TuningLevel', 'ToadPilotConfig', 'Version', 'GitBranch']
  persistent = Params()
  shared = Params('/dev/shm/params')
  snapshot = {key: persistent.get(key) for key in selected}
  memory = {key: shared.get(key) for key in ['FrogPilotToggles', 'FrogPilotTuningLevels']}
  if not memory['FrogPilotToggles']:
    raise RuntimeError('A running device toggle snapshot is required for this smoke check')
  tracking = Params('/cache/tracking')
  trip_stats = {key: tracking.get(key) for key in ('FrogPilotDrives', 'FrogPilotKilometers', 'FrogPilotMinutes')}
  # Read one parked device sample for the sidebar, before entering the preview namespace.
  panel = os.getenv('ABRASAX_PREVIEW_PANEL', 'offroad')
  if panel not in ('offroad', 'models', 'gas', 'controls'):
    raise ValueError('Unknown preview panel')
  sample = None
  if panel == 'offroad':
    state = messaging.recv_one(messaging.sub_sock('deviceState', timeout=1500))
    if state is not None and state.valid and not state.deviceState.started:
      sample = state.as_builder().to_bytes()
  prefix = 'abrasax-smoke-' + uuid.uuid4().hex[:8]
  os.environ['OPENPILOT_PREFIX'] = prefix
  queue_dir = Path('/dev/shm') / prefix
  queue_dir.mkdir(mode=0o700)
  aliases = []
  try:
    for base in ['/data/params', '/dev/shm/params', '/dev/shm/params_default', '/cache/params', '/cache/tracking']:
      clone = Params(base)
      alias = Path(clone.get_param_path())
      aliases.append((alias, alias.resolve()))
      values = memory if base == '/dev/shm/params' else snapshot if base == '/data/params' else {}
      if base == '/cache/tracking':
        values = trip_stats
      for key, value in values.items():
        if value is not None:
          clone.put(key, value)
    preview_bus = messaging.PubMaster(['deviceState']) if sample else None
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen', QT_QUICK_BACKEND='software',
               ABRASAX_SCREENSHOT=str(ROOT / f'.native-preview/{panel}.png'))
    screenshot = Path(env['ABRASAX_SCREENSHOT'])
    screenshot.unlink(missing_ok=True)
    with (ROOT / '.native-preview/offroad.log').open('wb') as log:
      process = subprocess.Popen([str(ROOT / '.native-build/ui-abrasax'), '--abrasax-offroad-preview',
                                  '-stylesheet', '../frogpilot/assets/toad_neon/neon.qss'],
        cwd=ROOT / 'selfdrive/ui', env=env, stdout=log, stderr=subprocess.STDOUT)
      try:
        deadline = time.monotonic() + float(os.getenv('ABRASAX_PREVIEW_TIMEOUT_S', '20'))
        next_sample = 0.0
        while not screenshot.exists() and process.poll() is None and time.monotonic() < deadline:
          if preview_bus and time.monotonic() >= next_sample:
            preview_bus.send('deviceState', sample)
            next_sample = time.monotonic() + .5
          time.sleep(.1)
        if not screenshot.exists() or process.wait(timeout=12) != 0:
          raise RuntimeError(f'Full UI preview failed (exit={process.poll()}); inspect offroad.log')
      finally:
        if process.poll() is None:
          process.terminate()
          try:
            process.wait(timeout=3)
          except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    print('Full offroad UI rendered:', env['ABRASAX_SCREENSHOT'])
  finally:
    for alias, target in aliases:
      if target.name.startswith('.tmp') or alias.name == prefix:
        alias.unlink(missing_ok=True)
        if target != alias:
          shutil.rmtree(target)
    shutil.rmtree(queue_dir)


if __name__ == '__main__':
  main()
