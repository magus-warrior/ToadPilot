"""Load the matching tinygrad only inside the dedicated model process."""
import os
import sys
import hashlib
import json

from openpilot.selfdrive.toad_modeld.artifact import (MODEL_ID, MODELS, V23_REFERENCE_REVISION,
                                                   DEEP_RL3_V2_REFERENCE_REVISION, installed)


def configure_runtime(model_id=MODEL_ID):
  runtime = MODELS[model_id]["runtime"]
  if not installed(model_id):
    raise RuntimeError('Install the pinned ToadPilot model runtime before loading a modern model')
  if 'tinygrad' in sys.modules:
    raise RuntimeError('The modern model must run in a fresh process, isolated from legacy tinygrad')
  receipt = json.loads((runtime / 'receipt.json').read_text())
  for name, expected in receipt['runtime_files'].items():
    if hashlib.sha256((runtime / name).read_bytes()).hexdigest() != expected:
      raise RuntimeError(f'Pinned runtime file changed: {name}')
  os.environ['DEV'] = 'QCOM'
  os.environ['QCOM'] = '1'
  os.environ['GMMU'] = '0'
  sys.path.insert(0, str(runtime))

  if MODELS[model_id]['revision'] in {V23_REFERENCE_REVISION, DEEP_RL3_V2_REFERENCE_REVISION}:
    # AGNOS 10.1 uses glibc 2.31, where these POSIX symbols live in
    # libpthread. These runtimes assume glibc >= 2.34's merged libc.
    # Resolve the existing ABI from its proper library; keep GPU code intact.
    import ctypes
    import ctypes.util
    from tinygrad.runtime.autogen import libc
    pthread = ctypes.CDLL(ctypes.util.find_library('pthread'), use_errno=True)
    for symbol in ('sem_open', 'sem_unlink', 'sem_wait', 'sem_close', 'sem_post', 'pthread_exit'):
      if not hasattr(libc.dll, symbol):
        setattr(libc.dll, symbol, getattr(pthread, symbol))
