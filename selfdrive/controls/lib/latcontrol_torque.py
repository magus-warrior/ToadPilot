from collections import deque

import math
import numpy as np

from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.numpy_fast import interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.interfaces import LatControlInputs
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_friction
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.frogpilot.toadpilot import (TOADPILOT_FINGERPRINT, TOADPILOT_FRICTION_MAX_RATIO,
                                                     TOADPILOT_FRICTION_MIN_RATIO, TOADPILOT_LAT_ACCEL_FACTOR_MAX_RATIO,
                                                     TOADPILOT_LAT_ACCEL_FACTOR_MIN_RATIO, TOADPILOT_LAT_ACCEL_OFFSET_MAX,
                                                     ToadPilotLateralAssist, config_from_toggles)

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]
LOW_SPEED_Y_NN = [12, 3, 1, 0]

LAT_PLAN_MIN_IDX = 5

# Compatible pieces of upstream openpilot's 0.11.1 torque response. Keep these
# local to TOADPILOT until the branch can take the complete lateral-controller
# and opendbc units migration.
TOADPILOT_LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
TOADPILOT_JERK_LOOKAHEAD_SECONDS = 0.19
TOADPILOT_JERK_FILTER_CUTOFF_HZ = 1.2
TOADPILOT_JERK_FRICTION_GAIN = 0.3
TOADPILOT_FRICTION_THRESHOLD = 0.2


class DelayedLateralResponse:
  """Bounded request history used for delay-aligned feedback and jerk friction."""

  def __init__(self, lateral_delay, dt=DT_CTRL):
    try:
      self.dt = float(dt)
    except (TypeError, ValueError):
      self.dt = DT_CTRL
    if not math.isfinite(self.dt) or self.dt <= 0.0:
      self.dt = DT_CTRL
    self.buffer_len = max(3, int(TOADPILOT_LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt))
    try:
      lateral_delay = float(lateral_delay)
    except (TypeError, ValueError):
      lateral_delay = 0.0
    if not math.isfinite(lateral_delay):
      lateral_delay = 0.0
    self.delay_frames = int(np.clip(lateral_delay / self.dt + 1, 1, self.buffer_len))
    self.lookahead_frames = int(TOADPILOT_JERK_LOOKAHEAD_SECONDS / self.dt)
    self.lat_accel_requests = deque(maxlen=self.buffer_len)
    self.curvature_requests = deque(maxlen=self.buffer_len)
    self.jerk_filter = FirstOrderFilter(
      0.0,
      1.0 / (2.0 * np.pi * TOADPILOT_JERK_FILTER_CUTOFF_HZ),
      self.dt,
    )
    self.reset()

  def reset(self):
    self.lat_accel_requests.clear()
    self.lat_accel_requests.extend([0.0] * self.buffer_len)
    self.curvature_requests.clear()
    self.curvature_requests.extend([0.0] * self.buffer_len)
    self.jerk_filter.x = 0.0
    self.jerk_filter.initialized = True
    self.valid_samples = 0

  def update(self, lateral_accel, curvature):
    try:
      lateral_accel = float(lateral_accel)
      curvature = float(curvature)
    except (TypeError, ValueError):
      self.reset()
      return 0.0, 0.0, 0.0, False
    if not math.isfinite(lateral_accel) or not math.isfinite(curvature):
      self.reset()
      return 0.0, 0.0, 0.0, False

    self.lat_accel_requests.append(lateral_accel)
    self.curvature_requests.append(curvature)
    self.valid_samples = min(self.valid_samples + 1, self.buffer_len)

    expected_lateral_accel = self.lat_accel_requests[-self.delay_frames]
    expected_curvature = self.curvature_requests[-self.delay_frames]
    lookahead_idx = int(np.clip(-self.delay_frames + self.lookahead_frames, -self.buffer_len + 1, -2))
    raw_lateral_jerk = (
      self.lat_accel_requests[lookahead_idx + 1] - self.lat_accel_requests[lookahead_idx - 1]
    ) / (2.0 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    if not all(math.isfinite(value) for value in (expected_lateral_accel, expected_curvature, desired_lateral_jerk)):
      self.reset()
      return 0.0, 0.0, 0.0, False
    # A reset fills storage with neutral zeros. Stay on the established control
    # path until the delayed setpoint comes from a real request rather than that
    # padding. Inactive control cycles call update too, so normal engagement is
    # pre-warmed without introducing a special transition.
    response_ready = self.valid_samples >= self.delay_frames
    return expected_lateral_accel, expected_curvature, desired_lateral_jerk, response_ready


def kp_independent_low_speed_error(base_error, shaped_error, kp):
  """Keep only the low-speed contribution independent of proportional gain."""
  try:
    kp = float(kp)
  except (TypeError, ValueError):
    return float(shaped_error)
  if not math.isfinite(kp) or kp <= 1e-3:
    return float(shaped_error)
  return float(base_error + (shaped_error - base_error) / kp)


def use_toadpilot_upstream_response(config, capable, use_nnff, use_nnff_lite, response_ready):
  # Observe/disabled retain the exact branch baseline. NNFF already has its own
  # model-specific jerk and friction path, so do not stack this response on it.
  return bool(capable and config.enabled and config.profile in ("adaptive", "full") and
              not use_nnff and not use_nnff_lite and response_ready)


def torque_model_data_good(model_data):
  """Validate only the small model slice consumed by the existing NNFF path."""
  try:
    arrays = (model_data.orientation.x, model_data.orientation.y, model_data.acceleration.y)
    return all(len(values) >= CONTROL_N and
               all(math.isfinite(float(value)) for value in values[:CONTROL_N])
               for values in arrays)
  except (AttributeError, TypeError, ValueError):
    return False


def get_predicted_lateral_jerk(lat_accels, t_diffs):
  # compute finite difference between subsequent model_data.acceleration.y values
  # this is just two calls of np.diff followed by an element-wise division
  lat_accel_diffs = np.diff(lat_accels)
  lat_jerk = lat_accel_diffs / t_diffs
  # return as python list
  return lat_jerk.tolist()

def sign(x):
  return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)

def get_lookahead_value(future_vals, current_val):
  if len(future_vals) == 0:
    return current_val

  same_sign_vals = [v for v in future_vals if sign(v) == sign(current_val)]

  # if any future val has opposite sign of current val, return 0
  if len(same_sign_vals) < len(future_vals):
    return 0.0

  # otherwise return the value with minimum absolute value
  min_val = min(same_sign_vals + [current_val], key=lambda x: abs(x))
  return min_val

# At a given roll, if pitch magnitude increases, the
# gravitational acceleration component starts pointing
# in the longitudinal direction, decreasing the lateral
# acceleration component. Here we do the same thing
# to the roll value itself, then passed to nnff.
def roll_pitch_adjust(roll, pitch):
  return roll * math.cos(pitch)

class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

    # TOADPILOT shapes demand only inside the normalized stock controller cap.
    # Toyota CAN limits and Panda safety limits remain unchanged.
    self.toadpilot_capable = (CP.carName == "toyota" and CP.carFingerprint == TOADPILOT_FINGERPRINT and
                              CP.lateralTuning.which() == "torque")
    self.toadpilot_assist = ToadPilotLateralAssist()
    self.stock_lat_accel_factor = float(self.torque_params.latAccelFactor)
    self.stock_lat_accel_offset = float(self.torque_params.latAccelOffset)
    self.stock_friction = float(self.torque_params.friction)
    self.live_lat_accel_factor = self.stock_lat_accel_factor
    self.live_lat_accel_offset = self.stock_lat_accel_offset
    self.live_friction = self.stock_friction
    self.delayed_lateral_response = DelayedLateralResponse(CP.steerActuatorDelay)

    # Twilsonco's Lateral Neural Network Feedforward
    self.use_nnff = CI.use_nnff
    self.use_nnff_lite = CI.use_nnff_lite

    if self.use_nnff or self.use_nnff_lite:
      # Instantaneous lateral jerk changes very rapidly, making it not useful on its own,
      # however, we can "look ahead" to the future planned lateral jerk in order to guage
      # whether the current desired lateral jerk will persist into the future, i.e.
      # whether it's "deliberate" or not. This lets us simply ignore short-lived jerk.
      # Note that LAT_PLAN_MIN_IDX is defined above and is used in order to prevent
      # using a "future" value that is actually planned to occur before the "current" desired
      # value, which is offset by the steerActuatorDelay.
      self.friction_look_ahead_v = [1.4, 2.0] # how many seconds in the future to look ahead in [0, ~2.1] in 0.1 increments
      self.friction_look_ahead_bp = [9.0, 30.0] # corresponding speeds in m/s in [0, ~40] in 1.0 increments

      # Scaling the lateral acceleration "friction response" could be helpful for some.
      # Increase for a stronger response, decrease for a weaker response.
      self.lat_jerk_friction_factor = 0.4
      self.lat_accel_friction_factor = 0.7 # in [0, 3], in 0.05 increments. 3 is arbitrary safety limit

      # precompute time differences between ModelConstants.T_IDXS
      self.t_diffs = np.diff(ModelConstants.T_IDXS)
      self.desired_lat_jerk_time = CP.steerActuatorDelay + 0.3

    if self.use_nnff:
      self.pitch = FirstOrderFilter(0.0, 0.5, 0.01)
      # NN model takes current v_ego, lateral_accel, lat accel/jerk error, roll, and past/future/planned data
      # of lat accel and roll
      # Past value is computed using previous desired lat accel and observed roll
      self.torque_from_nn = CI.get_ff_nn
      self.nn_friction_override = CI.lat_torque_nn_model.friction_override

      # setup future time offsets
      self.nn_time_offset = CP.steerActuatorDelay + 0.2
      future_times = [0.3, 0.6, 1.0, 1.5] # seconds in the future
      self.nn_future_times = [i + self.nn_time_offset for i in future_times]
      self.nn_future_times_np = np.array(self.nn_future_times)

      # setup past time offsets
      self.past_times = [-0.3, -0.2, -0.1]
      history_check_frames = [int(abs(i)*100) for i in self.past_times]
      self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
      self.lateral_accel_desired_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])
      self.error_deque = deque(maxlen=history_check_frames[0])
      self.past_future_len = len(self.past_times) + len(self.nn_future_times)

  def reset(self):
    super().reset()
    self.toadpilot_assist.reset()

  def _apply_toadpilot_tune(self, config):
    use_stock_tune = (self.toadpilot_capable and config.enabled and config.profile != "observe" and
                      not config.auto_tune)
    use_live_tune = (self.toadpilot_capable and config.enabled and config.profile != "observe" and
                     config.auto_tune)
    if use_stock_tune:
      self.torque_params.latAccelFactor = self.stock_lat_accel_factor
      self.torque_params.latAccelOffset = self.stock_lat_accel_offset
      self.torque_params.friction = self.stock_friction
    elif use_live_tune:
      self.torque_params.latAccelFactor = self._bounded_live_value(
        self.live_lat_accel_factor, self.stock_lat_accel_factor,
        self.stock_lat_accel_factor * TOADPILOT_LAT_ACCEL_FACTOR_MIN_RATIO,
        self.stock_lat_accel_factor * TOADPILOT_LAT_ACCEL_FACTOR_MAX_RATIO,
      )
      self.torque_params.latAccelOffset = self._bounded_live_value(
        self.live_lat_accel_offset, self.stock_lat_accel_offset,
        -TOADPILOT_LAT_ACCEL_OFFSET_MAX, TOADPILOT_LAT_ACCEL_OFFSET_MAX,
      )
      self.torque_params.friction = self._bounded_live_value(
        self.live_friction, self.stock_friction, self.stock_friction * TOADPILOT_FRICTION_MIN_RATIO,
        self.stock_friction * TOADPILOT_FRICTION_MAX_RATIO,
      )
    else:
      self.torque_params.latAccelFactor = self.live_lat_accel_factor
      self.torque_params.latAccelOffset = self.live_lat_accel_offset
      self.torque_params.friction = self.live_friction

  def _toadpilot_tune_status(self, config, using_nnff):
    auto_tune_requested = (self.toadpilot_capable and config.enabled and config.profile != "observe" and
                           config.auto_tune)
    auto_tune_effective = auto_tune_requested and not using_nnff
    if auto_tune_effective:
      linear_tune_source = "live_filtered"
    elif auto_tune_requested:
      linear_tune_source = "suspended_by_nnff"
    elif self.toadpilot_capable and config.enabled and config.profile != "observe":
      linear_tune_source = "stock"
    else:
      linear_tune_source = "branch_baseline"

    if using_nnff:
      feedforward_source = "nnff"
    elif self.use_nnff_lite:
      feedforward_source = "linear_nnff_lite"
    else:
      feedforward_source = "linear"
    return {
      "auto_tune_requested": bool(auto_tune_requested),
      "auto_tune_effective": bool(auto_tune_effective),
      "live_filtered_selected": bool(auto_tune_effective),
      "linear_tune_source": linear_tune_source,
      "feedforward_source": feedforward_source,
    }

  @staticmethod
  def _bounded_live_value(value, stock, lower, upper):
    try:
      value = float(value)
    except (TypeError, ValueError):
      return float(stock)
    if not math.isfinite(value):
      return float(stock)
    return float(np.clip(value, min(lower, upper), max(lower, upper)))

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction, frogpilot_toggles=None):
    if self.toadpilot_capable:
      self.live_lat_accel_factor = self._bounded_live_value(latAccelFactor, self.stock_lat_accel_factor, -math.inf, math.inf)
      self.live_lat_accel_offset = self._bounded_live_value(latAccelOffset, self.stock_lat_accel_offset, -math.inf, math.inf)
      self.live_friction = self._bounded_live_value(friction, self.stock_friction, -math.inf, math.inf)
    else:
      self.live_lat_accel_factor = float(latAccelFactor)
      self.live_lat_accel_offset = float(latAccelOffset)
      self.live_friction = float(friction)
    self._apply_toadpilot_tune(config_from_toggles(frogpilot_toggles))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, llk, model_data=None,
             frogpilot_toggles=None, experimental_mode=False, toadpilot_centering_status=None,
             toadpilot_longitudinal_stopping=False):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    nn_log = None
    toadpilot_config = config_from_toggles(frogpilot_toggles)
    centering_status = toadpilot_centering_status or {
      "lane_centering_active": False,
      "lane_centering_gate_reason": "unavailable",
      "lane_offset_m": None,
      "path_lane_bias_m": None,
      "lane_width_m": None,
      "lane_centering_confidence": None,
      "lane_centering_confirm_samples": 0,
      "lane_centering_curvature_delta": 0.0,
    }
    self._apply_toadpilot_tune(toadpilot_config)

    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    expected_lateral_accel, expected_curvature, upstream_desired_lateral_jerk, response_ready = \
      self.delayed_lateral_response.update(future_desired_lateral_accel, desired_curvature)
    model_good = torque_model_data_good(model_data)
    upstream_response_active = use_toadpilot_upstream_response(
      toadpilot_config, self.toadpilot_capable, self.use_nnff, self.use_nnff_lite, response_ready,
    )

    if not active:
      self.toadpilot_assist.reset(desired_curvature)
      output_torque = 0.0
      pid_log.active = False
      if self.toadpilot_capable and self.toadpilot_assist.status_due(toadpilot_config):
        tune_status = self._toadpilot_tune_status(toadpilot_config, self.use_nnff)
        self.toadpilot_assist.publish_status(
          toadpilot_config,
          active=False,
          experimental_mode=bool(experimental_mode),
          requested_steer=0.0,
          active_lat_accel_factor=round(float(self.torque_params.latAccelFactor), 6),
          active_lat_accel_offset=round(float(self.torque_params.latAccelOffset), 6),
          active_friction=round(float(self.torque_params.friction), 6),
          **centering_status,
          **tune_status,
        )
    else:
      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      actual_lateral_jerk = 0.0
      if self.use_steering_angle:
        actual_curvature = actual_curvature_vm
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
        if self.use_nnff or self.use_nnff_lite:
          actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.0)
          actual_lateral_jerk = actual_curvature_rate * CS.vEgo ** 2
      else:
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      desired_lateral_accel = future_desired_lateral_accel
      feedback_lateral_accel = expected_lateral_accel if upstream_response_active else desired_lateral_accel
      feedback_curvature = expected_curvature if upstream_response_active else desired_curvature

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y if not self.use_nnff else LOW_SPEED_Y_NN)**2
      setpoint = feedback_lateral_accel + low_speed_factor * feedback_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature

      lateral_jerk_setpoint = 0
      lateral_jerk_measurement = 0
      lookahead_lateral_jerk = 0

      using_nnff = self.use_nnff and model_good
      if model_good and (self.use_nnff or self.use_nnff_lite):
        # prepare "look-ahead" desired lateral jerk
        lookahead = interp(CS.vEgo, self.friction_look_ahead_bp, self.friction_look_ahead_v)
        friction_upper_idx = next((i for i, val in enumerate(ModelConstants.T_IDXS) if val > lookahead), 16)
        predicted_lateral_jerk = get_predicted_lateral_jerk(model_data.acceleration.y, self.t_diffs)
        desired_lateral_jerk = (interp(self.desired_lat_jerk_time, ModelConstants.T_IDXS, model_data.acceleration.y) - desired_lateral_accel) / self.desired_lat_jerk_time
        lookahead_lateral_jerk = get_lookahead_value(predicted_lateral_jerk[LAT_PLAN_MIN_IDX:friction_upper_idx], desired_lateral_jerk)
        if not self.use_steering_angle or lookahead_lateral_jerk == 0.0:
          lookahead_lateral_jerk = 0.0
          actual_lateral_jerk = 0.0
          self.lat_accel_friction_factor = 1.0
        lateral_jerk_setpoint = self.lat_jerk_friction_factor * lookahead_lateral_jerk
        lateral_jerk_measurement = self.lat_jerk_friction_factor * actual_lateral_jerk

      if using_nnff:
        # update past data
        pitch = 0
        roll = params.roll
        if len(llk.calibratedOrientationNED.value) > 1:
          pitch = self.pitch.update(llk.calibratedOrientationNED.value[1])
          roll = roll_pitch_adjust(roll, pitch)
        self.roll_deque.append(roll)
        self.lateral_accel_desired_deque.append(desired_lateral_accel)

        # prepare past and future values
        # adjust future times to account for longitudinal acceleration
        adjusted_future_times = [t + 0.5*CS.aEgo*(t/max(CS.vEgo, 1.0)) for t in self.nn_future_times]
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        future_rolls = [roll_pitch_adjust(interp(t, ModelConstants.T_IDXS, model_data.orientation.x) + roll, interp(t, ModelConstants.T_IDXS, model_data.orientation.y) + pitch) for t in adjusted_future_times]
        past_lateral_accels_desired = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque)-1, i)] for i in self.history_frame_offsets]
        future_planned_lateral_accels = [interp(t, ModelConstants.T_IDXS[:CONTROL_N], model_data.acceleration.y) for t in adjusted_future_times]

        # compute NNFF error response
        nnff_setpoint_input = [CS.vEgo, setpoint, lateral_jerk_setpoint, roll] \
                              + [setpoint] * self.past_future_len \
                              + past_rolls + future_rolls
        # past lateral accel error shouldn't count, so use past desired like the setpoint input
        nnff_measurement_input = [CS.vEgo, measurement, lateral_jerk_measurement, roll] \
                                 + [measurement] * self.past_future_len \
                                 + past_rolls + future_rolls
        torque_from_setpoint = self.torque_from_nn(nnff_setpoint_input)
        torque_from_measurement = self.torque_from_nn(nnff_measurement_input)

        pid_log.error = torque_from_setpoint - torque_from_measurement
        error_blend_factor = interp(abs(desired_lateral_accel), [1.0, 2.0], [0.0, 1.0])
        if error_blend_factor > 0.0:  # blend in stronger error response when in high lat accel
          nnff_error_input = [CS.vEgo, setpoint - measurement, lateral_jerk_setpoint - lateral_jerk_measurement, 0.0]
          torque_from_error = self.torque_from_nn(nnff_error_input)
          if sign(pid_log.error) == sign(torque_from_error) and abs(pid_log.error) < abs(torque_from_error):
            pid_log.error = pid_log.error * (1.0 - error_blend_factor) + torque_from_error * error_blend_factor

        # compute feedforward (same as nn setpoint output)
        error = setpoint - measurement
        friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
        nn_input = [CS.vEgo, desired_lateral_accel, friction_input, roll] \
                   + past_lateral_accels_desired + future_planned_lateral_accels \
                   + past_rolls + future_rolls
        ff = self.torque_from_nn(nn_input)

        # apply friction override for cars with low NN friction response
        if self.nn_friction_override:
          pid_log.error += self.torque_from_lateral_accel(LatControlInputs(0.0, 0.0, CS.vEgo, CS.aEgo), self.torque_params,
                                                          friction_input, lateral_accel_deadzone, friction_compensation=True, gravity_adjusted=False)
        nn_log = nn_input + nnff_setpoint_input + nnff_measurement_input
      else:
        # Keep the high-speed learner's offset diagnostic-only until its behavior
        # across speed and road-crown transitions is validated.
        gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation
        torque_from_setpoint = self.torque_from_lateral_accel(LatControlInputs(setpoint, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                                              lateral_jerk_setpoint, lateral_accel_deadzone, friction_compensation=self.use_nnff_lite, gravity_adjusted=False)
        torque_from_measurement = self.torque_from_lateral_accel(LatControlInputs(measurement, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                                                 lateral_jerk_measurement, lateral_accel_deadzone, friction_compensation=self.use_nnff_lite, gravity_adjusted=False)
        pid_log.error = torque_from_setpoint - torque_from_measurement
        error = feedback_lateral_accel - actual_lateral_accel
        if upstream_response_active:
          # This branch controls in normalized torque space. Split the base and
          # low-speed portions so changing KP cannot silently retune low-speed
          # response, preserving the intent of upstream #36335 without changing
          # controller units or the stock output cap.
          base_torque_from_setpoint = self.torque_from_lateral_accel(
            LatControlInputs(feedback_lateral_accel, roll_compensation, CS.vEgo, CS.aEgo),
            self.torque_params, 0.0, lateral_accel_deadzone,
            friction_compensation=False, gravity_adjusted=False,
          )
          base_torque_from_measurement = self.torque_from_lateral_accel(
            LatControlInputs(actual_lateral_accel, roll_compensation, CS.vEgo, CS.aEgo),
            self.torque_params, 0.0, lateral_accel_deadzone,
            friction_compensation=False, gravity_adjusted=False,
          )
          base_pid_error = base_torque_from_setpoint - base_torque_from_measurement
          self.pid._k_p = frogpilot_toggles.steer_kp
          self.pid.speed = CS.vEgo
          pid_log.error = kp_independent_low_speed_error(base_pid_error, pid_log.error, self.pid.k_p)
        if self.use_nnff_lite:
          friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
        else:
          friction_input = error + (TOADPILOT_JERK_FRICTION_GAIN * upstream_desired_lateral_jerk if upstream_response_active else 0.0)
        if upstream_response_active:
          # Upstream #36619 lowered the friction transition to 0.2 and made its
          # input delay-independent by adding filtered desired jerk. Apply only
          # the friction term locally; Toyota's linear torque conversion and all
          # controller/Panda authority limits remain unchanged.
          ff = self.torque_from_lateral_accel(
            LatControlInputs(gravity_adjusted_lateral_accel, roll_compensation, CS.vEgo, CS.aEgo),
            self.torque_params, 0.0, lateral_accel_deadzone,
            friction_compensation=False, gravity_adjusted=True,
          )
          ff += get_friction(friction_input, lateral_accel_deadzone, TOADPILOT_FRICTION_THRESHOLD,
                             self.torque_params, friction_compensation=True)
        else:
          ff = self.torque_from_lateral_accel(LatControlInputs(gravity_adjusted_lateral_accel, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                              friction_input, lateral_accel_deadzone, friction_compensation=True,
                                              gravity_adjusted=True)

      ff_before_assist = ff
      if self.toadpilot_capable:
        if toadpilot_config.enabled:
          ff += self.toadpilot_assist.update(
            toadpilot_config, desired_curvature, desired_lateral_accel, actual_lateral_accel,
            CS.steeringPressed, CS.steeringRateDeg, steer_limited_by_safety, experimental_mode,
            v_ego=CS.vEgo, a_ego=CS.aEgo, brake_pressed=CS.brakePressed,
            longitudinal_stopping=toadpilot_longitudinal_stopping,
          )
        else:
          self.toadpilot_assist.reset(desired_curvature)
      else:
        self.toadpilot_assist.reset(desired_curvature)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      self.pid._k_p = frogpilot_toggles.steer_kp
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(
        self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited,
      )
      if self.toadpilot_capable and self.toadpilot_assist.status_due(toadpilot_config):
        tune_status = self._toadpilot_tune_status(toadpilot_config, using_nnff)
        self.toadpilot_assist.publish_status(
          toadpilot_config,
          active=True,
          experimental_mode=bool(experimental_mode),
          desired_lateral_accel=round(float(desired_lateral_accel), 6),
          actual_lateral_accel=round(float(actual_lateral_accel), 6),
          tracking_error=round(float(desired_lateral_accel - actual_lateral_accel), 6),
          response_setpoint_lateral_accel=round(float(feedback_lateral_accel), 6),
          response_desired_lateral_jerk=round(float(upstream_desired_lateral_jerk), 6),
          upstream_response_active=bool(upstream_response_active),
          feedforward_before_assist=round(float(ff_before_assist), 6),
          feedforward_after_assist=round(float(ff), 6),
          requested_steer=round(float(-output_torque), 6),
          controller_saturated=bool(pid_log.saturated),
          steer_limited=bool(steer_limited_by_safety),
          active_lat_accel_factor=round(float(self.torque_params.latAccelFactor), 6),
          active_lat_accel_offset=round(float(self.torque_params.latAccelOffset), 6),
          applied_lat_accel_offset=0.0,
          active_friction=round(float(self.torque_params.friction), 6),
          **centering_status,
          **tune_status,
        )
      if nn_log is not None:
        pid_log.nnLog = nn_log

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
