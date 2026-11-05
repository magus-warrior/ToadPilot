"""Pinned on-device model and runtime. Importing this module never opens a GPU."""
import hashlib
import json
from pathlib import Path

MODEL_ID = "toad-rdf4"
MODEL_NAME = "Regret Driven Framework V4 (ToadPilot)"
MODEL_VERSION = "v15"
REFERENCE_REVISION = "4a553da9ac22a14e34513c011aed04c1e93b4619"
V23_REFERENCE_REVISION = "6bd48c28e8feea277cfdd3916c313dfec6236a93"
DEEP_RL3_V2_REFERENCE_REVISION = "d97100bd14beb93810f93cd080c5c887e915471a"
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
  # QCOM artifacts from firestar5683/StarPilot-Resources at
  # b141c7dc3cc421ca68fa308d22343b7d2f897fa0 (Models branch).
  'toad-rdf5': dict(name='Regret Driven Framework V5 (ToadPilot)', version='v15',
                   revision=V23_REFERENCE_REVISION,
                   sha256='ef350dee3c5d74c213d6bbcc673f92f73036bb6b5111e6c4ce4b4df6c6a2c756',
                   runtime=Path('/data/toadpilot/model-runtime/rdf5-v23')),
  'toad-rdf6': dict(name='Regret Driven Framework V6 (ToadPilot)', version='v15',
                   revision=V23_REFERENCE_REVISION,
                   sha256='f98b7b2d0777f67a1855c209c2d4b00840f6e9dd5d4240bd41a4cf214973453e',
                   runtime=Path('/data/toadpilot/model-runtime/rdf6-v23')),
  'toad-tsf-do': dict(name='Terrific Super Fantastic Do Over (ToadPilot)', version='v15',
                     revision=V23_REFERENCE_REVISION,
                     sha256='177161a55a46fafaaafda2958bdd516652af04861a56019568c6b67f90209aaf',
                     runtime=Path('/data/toadpilot/model-runtime/tsf-do-v23')),
  'toad-macrostiff': dict(name='MacroStiff (ToadPilot)', version='v11',
                        revision=V23_REFERENCE_REVISION,
                        sha256='7ba5fb924841b5d9bc01ce28d44ea4b81ab460dfc09cd6e7d7b92c723cf87e4b',
                        runtime=Path('/data/toadpilot/model-runtime/macrostiff-v23')),
  'toad-wmi11': dict(name='Watermelon Indigo V11 (ToadPilot)', version='v11',
                    revision=V23_REFERENCE_REVISION,
                    sha256='7a02a36a0aaf3f5ff568b988bfacb87692323cfa43e4349aa73b574fdf693b3e',
                    runtime=Path('/data/toadpilot/model-runtime/wmi11-v23')),
  'toad-rebellious-hope': dict(name='Rebellious Hope (ToadPilot)', version='v15',
                             revision=V23_REFERENCE_REVISION,
                             sha256='f4c839f2edee0f2e1960936102cd111f133255090528062baf584ae9b5f3a021',
                             runtime=Path('/data/toadpilot/model-runtime/rebellious-hope-v23')),
  'toad-deep-rl3-v2': dict(name='Deep RL 3 V2 (ToadPilot)', version='v15',
                         revision=DEEP_RL3_V2_REFERENCE_REVISION,
                         image_history_pipeline='warp',
                         sha256='43c10d9bd611160f02bdf638b1a6b8c2d343e204bfb94626f836cadca1767a19',
                         runtime=Path('/data/toadpilot/model-runtime/deep-rl3-v2-20260623')),
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
