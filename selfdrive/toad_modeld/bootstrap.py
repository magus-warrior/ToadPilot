"""Load the matching tinygrad only inside the dedicated model process."""
import os
import sys
import hashlib
import json

from openpilot.selfdrive.toad_modeld.artifact import RUNTIME_PATH, installed


def configure_runtime():
  if not installed():
    raise RuntimeError('Install the pinned ToadPilot model runtime before loading RDF4')
  if 'tinygrad' in sys.modules:
    raise RuntimeError('RDF4 must run in a fresh process, isolated from legacy tinygrad')
  receipt = json.loads((RUNTIME_PATH / 'receipt.json').read_text())
  for name, expected in receipt['runtime_files'].items():
    if hashlib.sha256((RUNTIME_PATH / name).read_bytes()).hexdigest() != expected:
      raise RuntimeError(f'Pinned runtime file changed: {name}')
  os.environ['DEV'] = 'QCOM'
  os.environ['QCOM'] = '1'
  os.environ['GMMU'] = '0'
  sys.path.insert(0, str(RUNTIME_PATH))
