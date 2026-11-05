"""Install a pinned runtime from a verified StarPilot reference checkout, offroad only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from openpilot.selfdrive.toad_modeld.artifact import MODEL_ID, MODELS, installed, verify_artifact


def require_offroad():
  if Path('/data/params/d/IsOnroad').read_text().strip() != '0':
    raise RuntimeError('Installation requires a confirmed offroad state')


def install(reference, model_id=MODEL_ID, artifact=None):
  spec = MODELS[model_id]
  runtime = spec["runtime"]
  require_offroad()
  reference = Path(reference).resolve()

  def git(*args):
    return subprocess.check_output(['git', '-C', str(reference), *args])

  if git('rev-parse', 'HEAD').decode().strip() != spec["revision"]:
    raise ValueError('Reference checkout is not the pinned StarPilot revision')
  git('diff', '--exit-code', 'HEAD', '--', 'tinygrad_repo/tinygrad', 'selfdrive/modeld/models')
  if runtime.exists():
    if not installed(model_id):
      raise RuntimeError('Existing runtime is incomplete; inspect it before reinstalling')
    verify_artifact(model_id=model_id)
  else:
    runtime.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='.install-', dir=runtime.parent))
    try:
      runtime_hashes = {}
      for name in git('ls-files', 'tinygrad_repo/tinygrad').decode().splitlines():
        relative = Path(name).relative_to('tinygrad_repo')
        target = staging / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(reference / name, target)
        runtime_hashes[str(relative)] = hashlib.sha256(target.read_bytes()).hexdigest()
      (staging / 'tinygrad-LICENSE').write_bytes(git('show', 'HEAD:tinygrad_repo/LICENSE'))
      (staging / 'StarPilot-LICENSE').write_bytes(git('show', 'HEAD:LICENSE'))
      model = staging / 'driving_tinygrad.pkl'
      with model.open('wb') as output:
        if artifact is not None:
          with Path(artifact).open('rb') as source:
            shutil.copyfileobj(source, output)
        elif model_id == MODEL_ID:
          for part in range(1, 4):
            path = reference / f'selfdrive/modeld/models/driving_tinygrad.pkl.chunk{part:02d}of03'
            with path.open('rb') as source:
              shutil.copyfileobj(source, output)
        else:
          raise ValueError('A verified artifact file is required for this model')
        output.flush()
        os.fsync(output.fileno())
      verify_artifact(model, model_id)
      receipt = dict(reference_revision=spec["revision"], artifact_sha256=spec["sha256"],
                     artifact_bytes=model.stat().st_size, runtime_files=runtime_hashes)
      (staging / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
      require_offroad()
      staging.rename(runtime)
    finally:
      if staging.exists():
        shutil.rmtree(staging)
  print(f'Installed verified runtime: {runtime}')


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--reference', required=True, type=Path)
  parser.add_argument("--model", choices=MODELS, default=MODEL_ID)
  parser.add_argument("--artifact", type=Path)
  args = parser.parse_args()
  install(args.reference, args.model, args.artifact)


if __name__ == '__main__':
  main()
