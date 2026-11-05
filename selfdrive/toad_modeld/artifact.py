"""Pinned on-device model and runtime. Importing this module never opens a GPU."""
import hashlib
import json
from pathlib import Path

MODEL_ID = "toad-rdf4"
MODEL_NAME = "Regret Driven Framework V4 (ToadPilot)"
MODEL_VERSION = "v15"
REFERENCE_REVISION = "4a553da9ac22a14e34513c011aed04c1e93b4619"
ARTIFACT_SHA256 = "27969d9da00f74ba0c1f56de575665121a967528753500cd2f809b61664e0e3f"
RUNTIME_PATH = Path('/data/toadpilot/model-runtime') / REFERENCE_REVISION
ARTIFACT_PATH = RUNTIME_PATH / 'driving_tinygrad.pkl'


def is_modern_model(toggles):
  return (getattr(toggles, 'model', '') == MODEL_ID and
          getattr(toggles, 'model_version', '') == MODEL_VERSION)


def installed():
  try:
    receipt = json.loads((RUNTIME_PATH / 'receipt.json').read_text())
    return (receipt['reference_revision'] == REFERENCE_REVISION and
            receipt['artifact_sha256'] == ARTIFACT_SHA256 and
            ARTIFACT_PATH.stat().st_size == receipt['artifact_bytes'] and
            (RUNTIME_PATH / 'tinygrad/__init__.py').is_file())
  except (OSError, ValueError, KeyError, TypeError):
    return False


def verify_artifact(path=ARTIFACT_PATH):
  checksum = hashlib.sha256()
  with path.open('rb') as model:
    for block in iter(lambda: model.read(1024 * 1024), b''):
      checksum.update(block)
  if checksum.hexdigest() != ARTIFACT_SHA256:
    raise ValueError('RDF4 artifact checksum differs from the pinned model')


def catalog_entry():
  return dict(id=MODEL_ID, name=MODEL_NAME, version=MODEL_VERSION, experimental=True)
