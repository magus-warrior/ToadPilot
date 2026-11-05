"""Offroad inference, parsing and cereal round-trip validation; no control publication."""
import argparse
import json
from pathlib import Path
import time

from openpilot.selfdrive.toad_modeld.bootstrap import configure_runtime
from openpilot.selfdrive.toad_modeld.action import action_from_outputs
from openpilot.selfdrive.toad_modeld.artifact import MODEL_ID, MODELS
from openpilot.selfdrive.toad_modeld.install import require_offroad


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--frames', type=int, default=100)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument("--model", choices=MODELS, default=MODEL_ID)
  args = parser.parse_args()
  if not 1 <= args.frames <= 10000:
    parser.error('--frames must be between 1 and 10000')
  require_offroad()
  configure_runtime(args.model)
  import numpy as np
  from cereal import log
  import cereal.messaging as messaging
  from openpilot.selfdrive.toad_modeld.runner import ModelState
  from openpilot.selfdrive.toad_modeld.fill_model_msg import PublishState, fill_model_msg, fill_pose_msg
  model = ModelState(1928, 1208, args.model)
  model.warmup()
  state = PublishState()
  previous = log.ModelDataV2.Action(longitudinalValid=True)
  timings = []
  for frame in range(args.frames):
    require_offroad()
    started = time.perf_counter()
    outputs = model.synthetic()
    latency = time.perf_counter() - started
    curvature, acceleration, stop = action_from_outputs(outputs, 15, (0.3, 0.5), previous.desiredCurvature, previous.desiredAcceleration)
    action = log.ModelDataV2.Action(desiredCurvature=curvature, desiredAcceleration=acceleration, shouldStop=stop, longitudinalValid=True)
    base = messaging.new_message('drivingModelData')
    extended = messaging.new_message('modelV2')
    pose = messaging.new_message('cameraOdometry')
    fill_model_msg(base, extended, outputs, action, state, frame, frame, frame, 0, 1234, latency, True)
    fill_pose_msg(pose, outputs, frame, 0, 1234, True)
    for message in (base, extended, pose):
      with log.Event.from_bytes(message.to_bytes()) as decoded:
        assert decoded.valid
        if decoded.which() == 'modelV2':
          assert decoded.modelV2.action.longitudinalValid
          assert len(decoded.modelV2.position.x) == 33
          assert len(decoded.modelV2.leadsV3) == 3
          assert len(decoded.modelV2.laneLines) == 6
          assert len(decoded.modelV2.temporalPose.trans) == 3
    timings.append(latency * 1000)
    previous = action
    if frame % 50 == 0:
      print(f'Validated frame {frame}: {latency * 1000:.1f} ms', flush=True)
  result = dict(model=args.model, frames=args.frames, parsed_and_serialized=True, finite_outputs=True,
                latency_ms={name: float(np.percentile(timings, p)) for name, p in [('p50', 50), ('p95', 95), ('max', 100)]},
                frames_over_50ms=sum(t > 50 for t in timings),
                limitation='Synthetic frames; live camera and driving behavior not validated by this test.')
  args.output.write_text(json.dumps(result, indent=2) + '\n')
  print(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
