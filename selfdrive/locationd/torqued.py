#!/usr/bin/env python3
import json
import numpy as np
import time
import uuid
from collections import deque, defaultdict

import cereal.messaging as messaging
from cereal import car, log
from panda import Panda
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.car.toyota.values import TOYOTA_STEER_COMMAND_SCALE
from openpilot.selfdrive.locationd.helpers import PointBuckets, ParameterEstimator

from openpilot.selfdrive.frogpilot.frogpilot_variables import get_frogpilot_toggles
from openpilot.selfdrive.frogpilot.toadpilot import (TOADPILOT_FINGERPRINT, TOADPILOT_LAT_ACCEL_OFFSET_MAX,
                                                     TOADPILOT_RESET_TUNE_KEY, TOADPILOT_TORQUE_CACHE_VERSION,
                                                     TOADPILOT_STEERING_RESPONSE_BUILD,
                                                     TOADPILOT_STEERING_RESPONSE_PROFILE,
                                                     TOADPILOT_TORQUE_CHECKPOINT_KEY,
                                                     TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION,
                                                     TOADPILOT_TORQUE_STATUS_KEY,
                                                     TOADPILOT_TUNE_STATUS_KEY,
                                                     ToadPilotHighAuthorityObserver)

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
TOADPILOT_EXTENDED_DIAGNOSTIC_TAIL_SECONDS = 0.5

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


def toadpilot_extended_torque_sample(capable, steer_output_can):
  """Keep experimental above-stock commands out of normalized torque learning."""
  if not capable:
    return False
  try:
    steer_output_can = float(steer_output_can)
  except (TypeError, ValueError):
    return True
  return not np.isfinite(steer_output_can) or abs(steer_output_can) > TOYOTA_STEER_COMMAND_SCALE


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
    try:
      self.toadpilot_high_torque_authorized = self.toadpilot_capable and any(
        int(config.safetyParam) & Panda.FLAG_TOYOTA_TOADPILOT_HIGH_TORQUE
        for config in CP.safetyConfigs
      )
    except (AttributeError, TypeError):
      self.toadpilot_high_torque_authorized = False
    self.toadpilot_torque_output_valid = False
    self.toadpilot_torque_extension_active = False
    self.toadpilot_torque_current_output_can = None
    self.toadpilot_torque_peak_abs_output_can = 0.0
    self.toadpilot_extended_output_samples = 0
    self.toadpilot_last_extended_monotonic = None
    self.toadpilot_extended_window_until = None
    self.toadpilot_route_started_at_unix = time.time()
    self.toadpilot_route_id = f"diagnostic-session:{uuid.uuid4().hex}"
    self.toadpilot_car_state_valid = False
    self.toadpilot_current_eps_torque = None
    self.toadpilot_peak_abs_eps_torque = 0.0
    self.toadpilot_peak_abs_steering_rate_deg = 0.0
    self.toadpilot_driver_override_samples = 0
    self.toadpilot_temporary_fault_seen = False
    self.toadpilot_permanent_fault_seen = False
    self.toadpilot_extended_window_car_state_samples = 0
    self.toadpilot_extended_window_peak_abs_eps_torque = 0.0
    self.toadpilot_extended_window_peak_abs_steering_rate_deg = 0.0
    self.toadpilot_extended_window_driver_override_samples = 0
    self.toadpilot_extended_window_temporary_fault_seen = False
    self.toadpilot_extended_window_permanent_fault_seen = False
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

  def update_toadpilot_torque_diagnostic(self, t, steer_output_can):
    if not self.toadpilot_high_torque_authorized:
      return
    try:
      steer_output_can = float(steer_output_can)
      t = float(t)
    except (TypeError, ValueError):
      self.toadpilot_torque_output_valid = False
      self.toadpilot_torque_extension_active = False
      return
    if not np.isfinite(steer_output_can) or not np.isfinite(t):
      self.toadpilot_torque_output_valid = False
      self.toadpilot_torque_extension_active = False
      return

    self.toadpilot_torque_output_valid = True
    self.toadpilot_torque_current_output_can = steer_output_can
    self.toadpilot_torque_peak_abs_output_can = max(self.toadpilot_torque_peak_abs_output_can,
                                                    abs(steer_output_can))
    self.toadpilot_torque_extension_active = abs(steer_output_can) > TOYOTA_STEER_COMMAND_SCALE
    if self.toadpilot_torque_extension_active:
      self.toadpilot_extended_output_samples += 1
      self.toadpilot_last_extended_monotonic = t
      self.toadpilot_extended_window_until = t + TOADPILOT_EXTENDED_DIAGNOSTIC_TAIL_SECONDS

  def toadpilot_torque_snapshot(self, inputs_valid=True, route_id=None):
    inputs_valid = bool(inputs_valid)
    if isinstance(route_id, bytes):
      try:
        route_id = route_id.decode("utf-8")
      except UnicodeDecodeError:
        route_id = None
    if isinstance(route_id, str) and route_id.strip():
      self.toadpilot_route_id = route_id.strip()[:256]
    car_state_valid = bool(inputs_valid and self.toadpilot_car_state_valid)
    return {
      "schema_version": TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION,
      "published_at_unix": round(time.time(), 3),
      "fingerprint": self.toadpilot_observer.fingerprint,
      "steering_response_profile": TOADPILOT_STEERING_RESPONSE_PROFILE,
      "steering_response_build": TOADPILOT_STEERING_RESPONSE_BUILD,
      "route_id": self.toadpilot_route_id,
      "route_started_at_unix": round(self.toadpilot_route_started_at_unix, 3),
      "route_complete": False,
      "completed_at_unix": None,
      "summary_source": "torqued_live",
      "command_semantics": "controller_command_before_panda_or_eps",
      "route_metrics_scope": "whole_authorized_route",
      "extended_metrics_scope": "car_state_samples_during_extension_and_0.5s_post_extension_tail",
      "extended_window_tail_seconds": TOADPILOT_EXTENDED_DIAGNOSTIC_TAIL_SECONDS,
      "controller_authorized": bool(self.toadpilot_high_torque_authorized),
      "inputs_valid": inputs_valid,
      "output_valid": bool(self.toadpilot_torque_output_valid),
      "extension_active": bool(inputs_valid and self.toadpilot_torque_output_valid and
                               self.toadpilot_torque_extension_active),
      "current_output_can": (round(self.toadpilot_torque_current_output_can, 3)
                             if self.toadpilot_torque_output_valid else None),
      "peak_abs_output_can": round(self.toadpilot_torque_peak_abs_output_can, 3),
      "extended_output_samples": int(self.toadpilot_extended_output_samples),
      "last_extended_monotonic": (round(self.toadpilot_last_extended_monotonic, 3)
                                  if self.toadpilot_last_extended_monotonic is not None else None),
      "car_state_valid": car_state_valid,
      "current_eps_torque": (round(self.toadpilot_current_eps_torque, 3)
                             if car_state_valid else None),
      "peak_abs_eps_torque": round(self.toadpilot_peak_abs_eps_torque, 3),
      "peak_abs_steering_rate_deg": round(self.toadpilot_peak_abs_steering_rate_deg, 3),
      "driver_override_samples": int(self.toadpilot_driver_override_samples),
      "temporary_fault_seen": bool(self.toadpilot_temporary_fault_seen),
      "permanent_fault_seen": bool(self.toadpilot_permanent_fault_seen),
      "extended_window_car_state_samples": int(self.toadpilot_extended_window_car_state_samples),
      "extended_window_peak_abs_eps_torque": round(self.toadpilot_extended_window_peak_abs_eps_torque, 3),
      "extended_window_peak_abs_steering_rate_deg": round(
        self.toadpilot_extended_window_peak_abs_steering_rate_deg, 3,
      ),
      "extended_window_driver_override_samples": int(self.toadpilot_extended_window_driver_override_samples),
      "extended_window_temporary_fault_seen": bool(self.toadpilot_extended_window_temporary_fault_seen),
      "extended_window_permanent_fault_seen": bool(self.toadpilot_extended_window_permanent_fault_seen),
    }

  def update_toadpilot_car_state_diagnostic(self, t, msg):
    if not self.toadpilot_high_torque_authorized:
      return
    try:
      t = float(t)
      eps_torque = float(msg.steeringTorqueEps)
      steering_rate_deg = float(msg.steeringRateDeg)
      steering_pressed = bool(msg.steeringPressed)
      temporary_fault = bool(msg.steerFaultTemporary)
      permanent_fault = bool(msg.steerFaultPermanent)
    except (AttributeError, TypeError, ValueError):
      self.toadpilot_car_state_valid = False
      return
    if not np.isfinite(t) or not np.isfinite(eps_torque) or not np.isfinite(steering_rate_deg):
      self.toadpilot_car_state_valid = False
      return

    self.toadpilot_car_state_valid = True
    self.toadpilot_current_eps_torque = eps_torque
    self.toadpilot_peak_abs_eps_torque = max(self.toadpilot_peak_abs_eps_torque, abs(eps_torque))
    self.toadpilot_peak_abs_steering_rate_deg = max(self.toadpilot_peak_abs_steering_rate_deg,
                                                    abs(steering_rate_deg))
    self.toadpilot_driver_override_samples += int(steering_pressed)
    self.toadpilot_temporary_fault_seen |= temporary_fault
    self.toadpilot_permanent_fault_seen |= permanent_fault
    in_extended_window = (self.toadpilot_last_extended_monotonic is not None and
                          self.toadpilot_extended_window_until is not None and
                          self.toadpilot_last_extended_monotonic <= t <= self.toadpilot_extended_window_until)
    if in_extended_window:
      self.toadpilot_extended_window_car_state_samples += 1
      self.toadpilot_extended_window_peak_abs_eps_torque = max(
        self.toadpilot_extended_window_peak_abs_eps_torque, abs(eps_torque),
      )
      self.toadpilot_extended_window_peak_abs_steering_rate_deg = max(
        self.toadpilot_extended_window_peak_abs_steering_rate_deg, abs(steering_rate_deg),
      )
      self.toadpilot_extended_window_driver_override_samples += int(steering_pressed)
      self.toadpilot_extended_window_temporary_fault_seen |= temporary_fault
      self.toadpilot_extended_window_permanent_fault_seen |= permanent_fault

  def handle_log(self, t, which, msg):
    if which == "carControl":
      self.raw_points["carControl_t"].append(t + self.lag)
      self.raw_points["active"].append(msg.latActive)
    elif which == "carOutput":
      self.raw_points["carOutput_t"].append(t + self.lag)
      self.raw_points["steer_torque"].append(-msg.actuatorsOutput.steer)
      self.raw_points["steer_output_can"].append(msg.actuatorsOutput.steerOutputCan)
      self.update_toadpilot_torque_diagnostic(t, msg.actuatorsOutput.steerOutputCan)
    elif which == "carState":
      self.raw_points["carState_t"].append(t + self.lag)
      self.raw_points["vego"].append(msg.vEgo)
      self.raw_points["steer_override"].append(msg.steeringPressed)
      self.update_toadpilot_car_state_diagnostic(t, msg)
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
        output_can_available = len(self.raw_points['steer_output_can']) == self.hist_len
        steer_output_can = (np.interp(t, self.raw_points['carOutput_t'], self.raw_points['steer_output_can'])
                            if output_can_available else np.nan)
        extended_torque = toadpilot_extended_torque_sample(self.toadpilot_capable, steer_output_can)
        lateral_acc = (vego * yaw_rate) - (np.sin(roll) * ACCELERATION_DUE_TO_GRAVITY)
        sample_valid = all(active) and (not any(steer_override)) and vego > MIN_VEL and not extended_torque
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

  if not estimator.toadpilot_high_torque_authorized:
    status_params.remove(TOADPILOT_TORQUE_STATUS_KEY)

  last_torque_summary_write = 0.0
  last_persisted_extended_samples = 0
  last_persisted_temporary_fault = False
  last_persisted_permanent_fault = False

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

    if sm.frame % 20 == 0 and estimator.toadpilot_high_torque_authorized:
      torque_snapshot = estimator.toadpilot_torque_snapshot(
        inputs_valid=sm.all_checks(), route_id=params.get("CurrentRoute", encoding="utf-8"),
      )
      status_params.put_nonblocking(TOADPILOT_TORQUE_STATUS_KEY,
                                    json.dumps(torque_snapshot, sort_keys=True, separators=(",", ":")))
      route_has_diagnostic = (torque_snapshot["extended_output_samples"] > 0 or
                              torque_snapshot["temporary_fault_seen"] or
                              torque_snapshot["permanent_fault_seen"])
      fault_changed = (torque_snapshot["temporary_fault_seen"] != last_persisted_temporary_fault or
                       torque_snapshot["permanent_fault_seen"] != last_persisted_permanent_fault)
      first_extension = (torque_snapshot["extended_output_samples"] > 0 and
                         last_persisted_extended_samples == 0)
      # Persist a recoverable checkpoint without writing to flash at telemetry cadence.
      # The manager synchronously writes the final snapshot at the offroad transition.
      periodic_due = time.monotonic() - last_torque_summary_write >= 60.0
      if route_has_diagnostic and (fault_changed or first_extension or periodic_due):
        params.put_nonblocking(TOADPILOT_TORQUE_CHECKPOINT_KEY,
                               json.dumps(torque_snapshot, sort_keys=True, separators=(",", ":")))
        last_torque_summary_write = time.monotonic()
        last_persisted_extended_samples = torque_snapshot["extended_output_samples"]
        last_persisted_temporary_fault = torque_snapshot["temporary_fault_seen"]
        last_persisted_permanent_fault = torque_snapshot["permanent_fault_seen"]

if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='Process the --demo argument.')
  parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
  args = parser.parse_args()
  main(demo=args.demo)
