"""Lateral-delay learner adapted to ToadPilot's calibrated localizer messages."""
import hashlib
import json
import math
from pathlib import Path
import time

from cereal import car, messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import config_realtime_process
from openpilot.selfdrive.toad_modeld.lag_estimator import LateralLagEstimator, BLOCK_NUM

CACHE = Path('/data/toadpilot/live_delay/cache.json')
STATUS = Path('/dev/shm/toadpilot_lateral_delay.json')


def cache_key(cp_bytes, model):
  return hashlib.sha256(cp_bytes + model.encode() + b':toad-delay-v1').hexdigest()


def restore_cache(path, key):
  try:
    data = json.loads(path.read_text())
    lag, count = data['delay'], data['blocks']
    if (data['key'] != key or data['version'] != 1 or not isinstance(lag, (int, float)) or
        isinstance(lag, bool) or not math.isfinite(lag) or not 0.15 <= lag <= 0.65 or
        type(count) is not int or not 5 <= count <= BLOCK_NUM):
      return None
    # Require fresh evidence on this route before applying a cached estimate.
    return float(lag), min(count, 4)
  except (OSError, ValueError, KeyError, TypeError):
    return None


def atomic_json(path, value):
  try:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, allow_nan=False))
    temp.replace(path)
  except OSError:
    cloudlog.exception('Unable to save lateral delay status/cache')


def main():
  config_realtime_process([0, 1, 2, 3], 5)
  params = Params()
  cp_bytes = params.get('CarParams', block=True)
  with car.CarParams.from_bytes(cp_bytes) as reader:
    CP = reader.as_builder()
  key = cache_key(cp_bytes, params.get('Model', encoding='utf-8') or '')
  estimator = LateralLagEstimator(CP, 0.05)
  restored = restore_cache(CACHE, key)
  if restored:
    estimator.reset(*restored)
  inputs = ['carControl', 'carState', 'controlsState', 'liveCalibration', 'liveLocationKalman']
  sm = messaging.SubMaster(inputs, poll='liveLocationKalman')
  pm = messaging.PubMaster(['liveDelay'])
  last_sample_t = None
  while True:
    sm.update()
    sample_t = sm.logMonoTime['liveLocationKalman'] * 1e-9
    valid = sm.all_checks() and sm.updated['liveLocationKalman']
    if valid and last_sample_t is not None and not 0.025 <= sample_t - last_sample_t <= 0.075:
      # Cross-correlation assumes uniformly sampled time. Clear history on gaps.
      estimator.reset(estimator.initial_lag, 0)
      estimator.last_estimate_t = 0.0
      estimator.last_pose_invalid_t = sample_t
      valid = False
    if sm.updated['liveLocationKalman']:
      last_sample_t = sample_t
    if valid:
      for name in sorted(inputs, key=lambda name: sm.logMonoTime[name]):
        if sm.updated[name]:
          estimator.handle_log(sm.logMonoTime[name] * 1e-9, name, sm[name])
      estimator.t = sample_t
      # Reject EPS faults and driver overrides, including the recovery window.
      if sm['carState'].steerFaultTemporary or sm['carState'].steerFaultPermanent:
        estimator.pose_valid = False
      estimator.update_points()
    else:
      estimator.pose_valid = False
      estimator.last_pose_invalid_t = sample_t
      estimator.points.update(sample_t, 0.0, 0.0, False)
    if sm.frame % 5 == 0:
      if valid:
        estimator.update_estimate()
      message = estimator.get_msg(valid)
      pm.send('liveDelay', message)
      if sm.frame % 20 == 0:
        ld = message.liveDelay
        atomic_json(STATUS, dict(status=str(ld.status), delay=ld.lateralDelay, progress=ld.calPerc,
                                 valid=bool(valid), updated=time.time()))
      if sm.frame % 1200 == 0 and valid and str(message.liveDelay.status) == 'estimated':
        atomic_json(CACHE, dict(version=1, key=key, delay=message.liveDelay.lateralDelay,
                                blocks=message.liveDelay.validBlocks))


if __name__ == '__main__':
  main()
