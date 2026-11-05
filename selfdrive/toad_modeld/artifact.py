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


MODELS = {
  MODEL_ID: dict(name=MODEL_NAME, version=MODEL_VERSION, revision=REFERENCE_REVISION,
                 sha256=ARTIFACT_SHA256, runtime=RUNTIME_PATH),
  'toad-pop-v2': dict(name='Pop Model V2 (ToadPilot)', version='v11',
                      revision='6bd48c28e8feea277cfdd3916c313dfec6236a93',
                      sha256='7f1edb92c85a6316b5ec7187634b9f2522da55bf17f1cade4475f02e24b9d4c2',
                      runtime=Path('/data/toadpilot/model-runtime/pop-v2-v23')),
  'toad-south-carolina': dict(name='South Carolina (ToadPilot)', version='v11',
                            revision='6bd48c28e8feea277cfdd3916c313dfec6236a93',
                            sha256='d1c40648f28b83e6bb74313cc3796056a80e529b9716e5870aeeeee2e772be00',
                            runtime=Path('/data/toadpilot/model-runtime/south-carolina-v23')),
}


def artifact_path(model_id=MODEL_ID):
  return MODELS[model_id]['runtime'] / 'driving_tinygrad.pkl'


def is_modern_model(toggles):
  spec = MODELS.get(getattr(toggles, 'model', ''))
  return spec is not None and getattr(toggles, 'model_version', '') == spec['version']


def installed(model_id=MODEL_ID):
  spec = MODELS[model_id]
  runtime = spec['runtime']
  try:
    receipt = json.loads((runtime / 'receipt.json').read_text())
    return (receipt['reference_revision'] == spec['revision'] and
            receipt['artifact_sha256'] == spec['sha256'] and
            artifact_path(model_id).stat().st_size == receipt['artifact_bytes'] and
            (runtime / 'tinygrad/__init__.py').is_file())
  except (OSError, ValueError, KeyError, TypeError):
    return False


def verify_artifact(path=None, model_id=MODEL_ID):
  checksum = hashlib.sha256()
  with (path or artifact_path(model_id)).open('rb') as model:
    for block in iter(lambda: model.read(1024 * 1024), b''):
      checksum.update(block)
  if checksum.hexdigest() != MODELS[model_id]['sha256']:
    raise ValueError(f'{model_id} artifact checksum differs from the pinned model')


def catalog_entry(model_id=MODEL_ID):
  spec = MODELS[model_id]
  return dict(id=model_id, name=spec['name'], version=spec['version'], experimental=True)
