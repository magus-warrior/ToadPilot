#!/usr/bin/env python3
"""Render the entire offroad UI in isolated queues and copied settings."""
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid

from openpilot.common.params import Params

ROOT = Path(__file__).resolve().parents[2]


def main():
  selected = ['LanguageSetting', 'IsMetric', 'HasAcceptedTerms', 'CompletedTrainingVersion', 'TermsVersion', 'TrainingVersion',
              'CarMake', 'CarModel',
              'CarParamsPersistent', 'PrimeType', 'Model', 'AvailableModels', 'AvailableModelNames',
              'ModelVersions', 'TuningLevel', 'ToadPilotConfig', 'Version', 'GitBranch']
  persistent = Params()
  shared = Params('/dev/shm/params')
  snapshot = {key: persistent.get(key) for key in selected}
  memory = {key: shared.get(key) for key in ['FrogPilotToggles', 'FrogPilotTuningLevels']}
  if not memory['FrogPilotToggles']:
    raise RuntimeError('A running device toggle snapshot is required for this smoke check')
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
      for key, value in values.items():
        if value is not None:
          clone.put(key, value)
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen', QT_QUICK_BACKEND='software',
               ABRASAX_SCREENSHOT=str(ROOT / '.native-preview/offroad.png'))
    screenshot = Path(env['ABRASAX_SCREENSHOT'])
    screenshot.unlink(missing_ok=True)
    with (ROOT / '.native-preview/offroad.log').open('wb') as log:
      process = subprocess.Popen([str(ROOT / '.native-build/ui-abrasax'), '--abrasax-offroad-preview'],
        cwd=ROOT / 'selfdrive/ui', env=env, stdout=log, stderr=subprocess.STDOUT)
      try:
        deadline = time.monotonic() + 20
        while not screenshot.exists() and process.poll() is None and time.monotonic() < deadline:
          time.sleep(.1)
        if not screenshot.exists():
          raise RuntimeError('Full UI did not produce its preview; inspect offroad.log')
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
