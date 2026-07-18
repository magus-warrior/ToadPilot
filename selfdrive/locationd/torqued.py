#!/usr/bin/env python3
import json
import numpy as np
from collections import deque, defaultdict

import cereal.messaging as messaging
from cereal import car, log
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.locationd.helpers import PointBuckets, ParameterEstimator

from openpilot.selfdrive.frogpilot.frogpilot_variables import get_frogpilot_toggles
from openpilot.selfdrive.frogpilot.toadpilot import (TOADPILOT_FINGERPRINT, TOADPILOT_LAT_ACCEL_OFFSET_MAX,
                                                     TOADPILOT_RESET_TUNE_KEY, TOADPILOT_TORQUE_CACHE_VERSION,
                                                     TOADPILOT_TUNE_STATUS_KEY, ToadPilotHighAuthorityObserver)

HISTORY = 5  # secs
POINTS_PER_BUCKET = 1500
MIN_POINTS_TOTAL = 4000
MIN_POINTS_TOTAL_QLOG = 600
FIT_POINTS_TOTAL = 2000
FIT_POINTS_TOTAL_QLOG = 600
MIN_VEL = 15  # m/s
FRICTION_FACTOR = 1.5  # ~85% of data coverage
FACTOR_SANITY = 0.3
FACTOR_SANITY_QLOG = 0.5
FRICTION_SANITY = 0.5
FRICTION_SANITY_QLOG = 0.8
STEER_MIN_THRESHOLD = 0.02
MIN_FILTER_DECAY = 50
MAX_FILTER_DECAY = 250
LAT_ACC_THRESHOLD = 1
STEER_BUCKET_BOUNDS = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0), (0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]
MIN_BUCKET_POINTS = np.array([100, 300, 500, 500, 500, 500, 300, 100])
MIN_ENGAGE_BUFFER = 2  # secs

VERSION = TOADPILOT_TORQUE_CACHE_VERSION  # v2 caches always store pure learner values, never manual overrides
ALLOWED_CARS = ['toyota', 'hyundai']


def slope2rot(slope):
  sin = np.sqrt(slope**2 / (slope**2 + 1))
  cos = np.sqrt(1 / (slope**2 + 1))
  return np.array([[cos, -sin], [sin, cos]])


def live_location_sample_valid(msg):
  """Fail closed when a localizer sample used by the torque learner is invalid."""
  try:
    yaw_rate_values = msg.angularVelocityCalibrated.value
    orientation_values = msg.orientationNED.value
    return bool(msg.angularVelocityCalibrated.valid and msg.orientationNED.valid and
                msg.inputsOK and msg.sensorsOK and msg.posenetOK and
                len(yaw_rate_values) >= 3 and len(orientation_values) >= 1 and
                np.isfinite(yaw_rate_values[2]) and np.isfinite(orientation_values[0]))
  except (AttributeError, IndexError, TypeError, ValueError):
    return False


class TorqueBuckets(PointBuckets):
  def add_point(self, x, y):
    for bound_min, bound_max in self.x_bounds:
      if (x >= bound_min) and (x < bound_max):
        self.buckets[(bound_min, bound_max)].append([x, 1.0, y])
        break


class TorqueEstimator(ParameterEstimator):
  def __init__(self, CP, decimated=False):
    self.hist_len = int(HISTORY / DT_MDL)
    self.lag = CP.steerActuatorDelay + .2   # from controlsd
    if decimated:
      self.min_bucket_points = MIN_BUCKET_POINTS / 10
      self.min_points_total = MIN_POINTS_TOTAL_QLOG
      self.fit_points = FIT_POINTS_TOTAL_QLOG
      self.factor_sanity = FACTOR_SANITY_QLOG
      self.friction_sanity = FRICTION_SANITY_QLOG

    else:
      self.min_bucket_points = MIN_BUCKET_POINTS
      self.min_points_total = MIN_POINTS_TOTAL
      self.fit_points = FIT_POINTS_TOTAL
      self.factor_sanity = FACTOR_SANITY
      self.friction_sanity = FRICTION_SANITY

    self.offline_friction = 0.0
    self.is_torque_tuning = CP.lateralTuning.which() == "torque"
    self.toadpilot_capable = (CP.carName == "toyota" and CP.carFingerprint == TOADPILOT_FINGERPRINT and
                              self.is_torque_tuning)
    self.toadpilot_observer = ToadPilotHighAuthorityObserver(CP.carFingerprint)
    self.toadpilot_observe_enabled = False
    self.toadpilot_collecting = False
    self.offline_latAccelFactor = 0.0
    self.resets = 0.0
    self.use_params = CP.carName in ALLOWED_CARS and self.is_torque_tuning

    if self.is_torque_tuning:
      self.offline_friction = CP.lateralTuning.torque.friction
      self.offline_latAccelFactor = CP.lateralTuning.torque.latAccelFactor

    self.reset()

    initial_params = {
      'latAccelFactor': self.offline_latAccelFactor,
      'latAccelOffset': 0.0,
      'frictionCoefficient': self.offline_friction,
      'points': []
    }
    self.decay = MIN_FILTER_DECAY
    self.min_lataccel_factor = (1.0 - self.factor_sanity) * self.offline_latAccelFactor
    self.max_lataccel_factor = (1.0 + self.factor_sanity) * self.offline_latAccelFactor
    self.min_friction = (1.0 - self.friction_sanity) * self.offline_friction
    self.max_friction = (1.0 + self.friction_sanity) * self.offline_friction
    # VERSION is global, so its wire contract must not depend on estimator cadence
    # or vehicle. Full-estimator sanity bounds are the conservative common subset.
    self.cache_min_lataccel_factor = (1.0 - FACTOR_SANITY) * self.offline_latAccelFactor
    self.cache_max_lataccel_factor = (1.0 + FACTOR_SANITY) * self.offline_latAccelFactor
    self.cache_min_friction = (1.0 - FRICTION_SANITY) * self.offline_friction
    self.cache_max_friction = (1.0 + FRICTION_SANITY) * self.offline_friction

    # try to restore cached params
    params = Params()
    params_cache = params.get("CarParamsPrevRoute")
    torque_cache = params.get("LiveTorqueParameters")
    if params_cache is not None and torque_cache is not None:
      try:
        with log.Event.from_bytes(torque_cache) as log_evt:
          cache_ltp = log_evt.liveTorqueParameters
        with car.CarParams.from_bytes(params_cache) as msg:
          cache_CP = msg
        if self.get_restore_key(cache_CP, cache_ltp.version) == self.get_restore_key(CP, VERSION):
          if cache_ltp.liveValid:
            cached_factor = float(cache_ltp.latAccelFactorFiltered)
            cached_offset = float(cache_ltp.latAccelOffsetFiltered)
            cached_friction = float(cache_ltp.frictionCoefficientFiltered)
            cached_values = (cached_factor, cached_offset, cached_friction)
            if not all(np.isfinite(value) for value in cached_values):
              raise ValueError("cached torque parameters are non-finite")
            factor_tolerance = np.finfo(np.float32).eps * max(abs(cached_factor), abs(self.cache_min_lataccel_factor),
                                                              abs(self.cache_max_lataccel_factor), 1.0)
            friction_tolerance = np.finfo(np.float32).eps * max(abs(cached_friction), abs(self.cache_min_friction),
                                                                abs(self.cache_max_friction), 1.0)
            cached_in_range = (self.cache_min_lataccel_factor - factor_tolerance <= cached_factor <=
                               self.cache_max_lataccel_factor + factor_tolerance and
                               self.cache_min_friction - friction_tolerance <= cached_friction <=
                               self.cache_max_friction + friction_tolerance)
            if not cached_in_range:
              raise ValueError("cached torque parameters exceed learner sanity bounds")
            if self.toadpilot_capable and abs(cached_offset) > TOADPILOT_LAT_ACCEL_OFFSET_MAX:
              raise ValueError("cached TOADPILOT offset exceeds its sanity bound")
            cached_factor = float(np.clip(cached_factor, self.cache_min_lataccel_factor, self.cache_max_lataccel_factor))
            cached_friction = float(np.clip(cached_friction, self.cache_min_friction, self.cache_max_friction))

            initial_params = {
              'latAccelFactor': cached_factor,
              'latAccelOffset': cached_offset,
              'frictionCoefficient': cached_friction
            }
          initial_params['points'] = cache_ltp.points
          cached_decay = float(cache_ltp.decay)
          if not np.isfinite(cached_decay) or not MIN_FILTER_DECAY <= cached_decay <= MAX_FILTER_DECAY:
            raise ValueError("cached torque decay is invalid")
          self.decay = cached_decay
          self.filtered_points.load_points(initial_params['points'])
          cloudlog.info("restored torque params from cache")
      except Exception:
        cloudlog.exception("failed to restore cached torque params")
        params.remove("LiveTorqueParameters")
        self.reset()
        initial_params = {
          'latAccelFactor': self.offline_latAccelFactor,
          'latAccelOffset': 0.0,
          'frictionCoefficient': self.offline_friction,
          'points': [],
        }

    if self.toadpilot_capable:
      initial_params["latAccelOffset"] = float(np.clip(initial_params["latAccelOffset"],
                                                       -TOADPILOT_LAT_ACCEL_OFFSET_MAX, TOADPILOT_LAT_ACCEL_OFFSET_MAX))
    self.filtered_params = {}
    for param in initial_params:
      self.filtered_params[param] = FirstOrderFilter(initial_params[param], self.decay, DT_MDL)

  def get_restore_key(self, CP, version):
    a, b = None, None
    if CP.lateralTuning.which() == 'torque':
      a = CP.lateralTuning.torque.friction
      b = CP.lateralTuning.torque.latAccelFactor
    return (CP.carFingerprint, CP.lateralTuning.which(), a, b, version)

  def configure_toadpilot_observer(self, frogpilot_toggles):
    self.toadpilot_observe_enabled = (self.toadpilot_capable and
                                      getattr(frogpilot_toggles, "toadpilot_enabled", False) is True and
                                      getattr(frogpilot_toggles, "toadpilot_telemetry", False) is True)
    if not self.toadpilot_observe_enabled:
      self.toadpilot_collecting = False

  def reset(self):
    self.resets += 1.0
    self.decay = MIN_FILTER_DECAY
    self.raw_points = defaultdict(lambda: deque(maxlen=self.hist_len))
    self.toadpilot_collecting = False
    self.filtered_points = TorqueBuckets(x_bounds=STEER_BUCKET_BOUNDS,
                                         min_points=self.min_bucket_points,
                                         min_points_total=self.min_points_total,
                                         points_per_bucket=POINTS_PER_BUCKET,
                                         rowsize=3)

  def reset_to_offline(self):
    fingerprint = self.toadpilot_observer.fingerprint
    self.reset()
    self.filtered_params = {
      "latAccelFactor": FirstOrderFilter(self.offline_latAccelFactor, self.decay, DT_MDL),
      "latAccelOffset": FirstOrderFilter(0.0, self.decay, DT_MDL),
      "frictionCoefficient": FirstOrderFilter(self.offline_friction, self.decay, DT_MDL),
    }
    self.toadpilot_observer = ToadPilotHighAuthorityObserver(fingerprint)

  def estimate_params(self):
    points = self.filtered_points.get_points(self.fit_points)
    # total least square solution as both x and y are noisy observations
    # this is empirically the slope of the hysteresis parallelogram as opposed to the line through the diagonals
    try:
      _, _, v = np.linalg.svd(points, full_matrices=False)
      slope, offset = -v.T[0:2, 2] / v.T[2, 2]
      _, spread = np.matmul(points[:, [0, 2]], slope2rot(slope)).T
      friction_coeff = np.std(spread) * FRICTION_FACTOR
    except np.linalg.LinAlgError as e:
      cloudlog.exception(f"Error computing live torque params: {e}")
      slope = offset = friction_coeff = np.nan
    return slope, offset, friction_coeff

  def update_params(self, params):
    self.decay = min(self.decay + DT_MDL, MAX_FILTER_DECAY)
    for param, value in params.items():
      self.filtered_params[param].update(value)
      self.filtered_params[param].update_alpha(self.decay)

  def handle_log(self, t, which, msg):
    if which == "carControl":
      self.raw_points["carControl_t"].append(t + self.lag)
      self.raw_points["active"].append(msg.latActive)
    elif which == "carOutput":
      self.raw_points["carOutput_t"].append(t + self.lag)
      self.raw_points["steer_torque"].append(-msg.actuatorsOutput.steer)
    elif which == "carState":
      self.raw_points["carState_t"].append(t + self.lag)
      self.raw_points["vego"].append(msg.vEgo)
      self.raw_points["steer_override"].append(msg.steeringPressed)
    elif which == "liveLocationKalman":
      self.toadpilot_collecting = False
      # This branch has no embedded monotonic sensor timestamp equivalent to
      # livePose.timestamp. Keep using the event logMonoTime supplied by main;
      # wall-clock and time-since-reset fields are not interchangeable.
      if len(self.raw_points['steer_torque']) == self.hist_len and live_location_sample_valid(msg):
        yaw_rate = msg.angularVelocityCalibrated.value[2]
        roll = msg.orientationNED.value[0]
        active = np.interp(np.arange(t - MIN_ENGAGE_BUFFER, t, DT_MDL), self.raw_points['carControl_t'], self.raw_points['active']).astype(bool)
        steer_override = np.interp(np.arange(t - MIN_ENGAGE_BUFFER, t, DT_MDL), self.raw_points['carState_t'], self.raw_points['steer_override']).astype(bool)
        vego = np.interp(t, self.raw_points['carState_t'], self.raw_points['vego'])
        steer = np.interp(t, self.raw_points['carOutput_t'], self.raw_points['steer_torque'])
        lateral_acc = (vego * yaw_rate) - (np.sin(roll) * ACCELERATION_DUE_TO_GRAVITY)
        sample_valid = all(active) and (not any(steer_override)) and vego > MIN_VEL
        self.toadpilot_collecting = self.toadpilot_observe_enabled and sample_valid
        if self.toadpilot_observe_enabled and sample_valid:
          self.toadpilot_observer.add_sample(steer, lateral_acc)
        if sample_valid and abs(steer) > STEER_MIN_THRESHOLD and abs(lateral_acc) <= LAT_ACC_THRESHOLD:
          self.filtered_points.add_point(float(steer), float(lateral_acc))

  def get_msg(self, valid=True, with_points=False, frogpilot_toggles=None):
    msg = messaging.new_message('liveTorqueParameters')
    msg.valid = valid
    liveTorqueParameters = msg.liveTorqueParameters
    liveTorqueParameters.version = VERSION
    liveTorqueParameters.useParams = self.use_params

    # Calculate raw estimates when possible, only update filters when enough points are gathered
    if self.filtered_points.is_calculable():
      latAccelFactor, latAccelOffset, frictionCoeff = self.estimate_params()
      liveTorqueParameters.latAccelFactorRaw = float(latAccelFactor)
      liveTorqueParameters.latAccelOffsetRaw = float(latAccelOffset)
      liveTorqueParameters.frictionCoefficientRaw = float(frictionCoeff)

      if self.filtered_points.is_valid():
        invalid_values = any(val is None or not np.isfinite(val) for val in [latAccelFactor, latAccelOffset, frictionCoeff])
        invalid_offset = (not invalid_values and self.toadpilot_capable and
                          abs(latAccelOffset) > TOADPILOT_LAT_ACCEL_OFFSET_MAX)
        if invalid_values or invalid_offset:
          cloudlog.error("Live torque parameters are invalid or the TOADPILOT offset exceeds its sanity bound.")
          liveTorqueParameters.liveValid = False
          self.reset()
        else:
          liveTorqueParameters.liveValid = True
          latAccelFactor = np.clip(latAccelFactor, self.min_lataccel_factor, self.max_lataccel_factor)
          if self.toadpilot_capable:
            latAccelOffset = np.clip(latAccelOffset, -TOADPILOT_LAT_ACCEL_OFFSET_MAX, TOADPILOT_LAT_ACCEL_OFFSET_MAX)
          frictionCoeff = np.clip(frictionCoeff, self.min_friction, self.max_friction)
          self.update_params({'latAccelFactor': latAccelFactor, 'latAccelOffset': latAccelOffset, 'frictionCoefficient': frictionCoeff})

    if with_points:
      liveTorqueParameters.points = self.filtered_points.get_points()[:, [0, 2]].tolist()

    toadpilot_live_tune = (self.toadpilot_capable and getattr(frogpilot_toggles, "toadpilot_enabled", False) is True and
                           getattr(frogpilot_toggles, "toadpilot_profile", "observe") != "observe" and
                           getattr(frogpilot_toggles, "toadpilot_auto_tune", False) is True)
    use_custom_factor = frogpilot_toggles.use_custom_lat_accel_factor and not toadpilot_live_tune and not with_points
    use_custom_friction = frogpilot_toggles.use_custom_steer_friction and not toadpilot_live_tune and not with_points
    filtered_factor = float(self.filtered_params['latAccelFactor'].x)
    filtered_offset = float(self.filtered_params['latAccelOffset'].x)
    filtered_friction = float(self.filtered_params['frictionCoefficient'].x)
    if with_points and self.is_torque_tuning:
      # v2 cache values have one fixed contract, independent of the active estimator cadence.
      filtered_factor = float(np.clip(filtered_factor, self.cache_min_lataccel_factor, self.cache_max_lataccel_factor))
      filtered_friction = float(np.clip(filtered_friction, self.cache_min_friction, self.cache_max_friction))
      if self.toadpilot_capable:
        filtered_offset = float(np.clip(filtered_offset, -TOADPILOT_LAT_ACCEL_OFFSET_MAX, TOADPILOT_LAT_ACCEL_OFFSET_MAX))
    liveTorqueParameters.latAccelFactorFiltered = float(frogpilot_toggles.steer_lat_accel_factor if use_custom_factor
                                                         else filtered_factor)
    liveTorqueParameters.latAccelOffsetFiltered = filtered_offset
    liveTorqueParameters.frictionCoefficientFiltered = float(frogpilot_toggles.steer_friction if use_custom_friction
                                                              else filtered_friction)
    liveTorqueParameters.totalBucketPoints = len(self.filtered_points)
    liveTorqueParameters.decay = self.decay
    liveTorqueParameters.maxResets = self.resets
    return msg


def _consume_toadpilot_reset(estimator, params, status_params):
  if not params.get_bool(TOADPILOT_RESET_TUNE_KEY):
    return False
  estimator.reset_to_offline()
  params.remove("LiveTorqueParameters")
  params.remove(TOADPILOT_RESET_TUNE_KEY)
  status_params.remove(TOADPILOT_TUNE_STATUS_KEY)
  return True


def main(demo=False):
  config_realtime_process([0, 1, 2, 3], 5)

  pm = messaging.PubMaster(['liveTorqueParameters'])
  sm = messaging.SubMaster(['carControl', 'carOutput', 'carState', 'liveLocationKalman', 'frogpilotPlan'], poll='liveLocationKalman')

  params = Params()
  status_params = Params("/dev/shm/params")
  with car.CarParams.from_bytes(params.get("CarParams", block=True)) as CP:
    estimator = TorqueEstimator(CP)

  # FrogPilot variables
  frogpilot_toggles = get_frogpilot_toggles()

  if not frogpilot_toggles.liveValid:
    estimator = TorqueEstimator(CP, True)
  estimator.configure_toadpilot_observer(frogpilot_toggles)

  while True:
    sm.update()
    if sm.frame % 20 == 0:
      _consume_toadpilot_reset(estimator, params, status_params)

    if sm['frogpilotPlan'].togglesUpdated:
      frogpilot_toggles = get_frogpilot_toggles()
      estimator.configure_toadpilot_observer(frogpilot_toggles)

    if sm.all_checks():
      for which in sm.updated.keys():
        if sm.updated[which]:
          t = sm.logMonoTime[which] * 1e-9
          estimator.handle_log(t, which, sm[which])

    # 4Hz driven by liveLocationKalman
    if sm.frame % 5 == 0:
      pm.send('liveTorqueParameters', estimator.get_msg(valid=sm.all_checks(), with_points=False, frogpilot_toggles=frogpilot_toggles))

    # Cache points every 60 seconds while onroad
    if sm.frame % 240 == 0:
      msg = estimator.get_msg(valid=sm.all_checks(), with_points=True, frogpilot_toggles=frogpilot_toggles)
      params.put_nonblocking("LiveTorqueParameters", msg.to_bytes())

    if sm.frame % 20 == 0 and estimator.toadpilot_observe_enabled:
      # High-authority data is diagnostics-only and never enters filtered_points.
      collecting = bool(estimator.toadpilot_collecting and sm.all_checks())
      snapshot = estimator.toadpilot_observer.snapshot(collecting=collecting)
      status_params.put_nonblocking(TOADPILOT_TUNE_STATUS_KEY,
                                    json.dumps(snapshot, sort_keys=True, separators=(",", ":")))

if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='Process the --demo argument.')
  parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
  args = parser.parse_args()
  main(demo=args.demo)
