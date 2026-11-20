"""Linear Toyota steering port from FrogAi/FrogPilot adeb39e6."""
import math
import numpy as np
from collections import deque

from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.car.interfaces import FRICTION_THRESHOLD
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED, get_friction
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.frogpilot_torque_pid import PIDController
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

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

MAX_LAT_JERK_UP = 2.5            # m/s^3

class FrogPilotCamryTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.dt = DT_CTRL
    self.torque_params = CP.lateralTuning.torque
    # FrogPilot's linear Toyota tuning uses ki=0.3; the host's legacy controller uses 0.1.
    self.pid = PIDController(self.torque_params.kp, 0.3, rate=1/self.dt)
    self.driver_hold_frames = 0
    self.driver_direction = 0.
    self.stock_delay = CP.steerActuatorDelay + 0.2
    self.stock_factor = self.torque_params.latAccelFactor
    self.stock_friction = self.torque_params.friction
    self.using_adjusted_tune = False
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES = int(1 / self.dt)
    self.requested_lateral_accel_buffer = deque([0.] * self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES , maxlen=self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * (MAX_LAT_JERK_UP - 0.5)), self.dt)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    if not all(math.isfinite(value) for value in (latAccelFactor, latAccelOffset, friction)) or latAccelFactor <= 0. or friction < 0.:
      return
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    limit = self.steer_max * self.torque_params.latAccelFactor
    self.pid.set_limits(limit, -limit)

  def delay(self, sm):
    # The pinned models share FrogPilot's default 0.1 s output smoothing.
    estimate = sm['liveDelay'].lateralDelay
    if sm.seen['liveDelay'] and sm.alive['liveDelay'] and sm.valid['liveDelay'] and math.isfinite(estimate) and 0. < estimate <= 1.:
      return estimate + 0.1
    return self.stock_delay + 0.1

  def update_tune(self, sm, toggles):
    tune = sm['liveTorqueParameters']
    if sm.all_checks(['liveTorqueParameters']) and ((tune.useParams and not toggles.force_auto_tune_off) or toggles.force_auto_tune):
      self.update_live_torque_params(tune.latAccelFactorFiltered, tune.latAccelOffsetFiltered, tune.frictionCoefficientFiltered)
      self.using_adjusted_tune = True
    elif (not tune.useParams and self.using_adjusted_tune and not toggles.force_auto_tune) or toggles.use_custom_steer_friction or toggles.use_custom_lat_accel_factor:
      self.update_live_torque_params(toggles.steer_lat_accel_factor if toggles.use_custom_lat_accel_factor else self.stock_factor,
                                     0., toggles.steer_friction if toggles.use_custom_steer_friction else self.stock_friction)
      self.using_adjusted_tune = toggles.use_custom_steer_friction or toggles.use_custom_lat_accel_factor

  def yield_to_driver(self, steer, active, CS, delay):
    # Retain ToadPilot's Toyota driver-override direction and release hold.
    if not active:
      self.driver_hold_frames = 0
      return 0.
    if CS.steeringPressed:
      self.driver_direction = np.sign(CS.steeringTorque) if math.isfinite(CS.steeringTorque) else 0.
      self.driver_hold_frames = max(1, min(self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES, int(delay / self.dt)))
    if self.driver_hold_frames:
      if not CS.steeringPressed:
        self.driver_hold_frames -= 1
      if steer * self.driver_direction <= 0.:
        self.pid.i = 0.
        return 0.
    return steer

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay, llk, model_data, frogpilot_toggles):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES))
      expected_lateral_accel = self.requested_lateral_accel_buffer[-delay_frames]
      # TODO factor out lateral jerk from error to later replace it with delay independent alternative
      future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      self.requested_lateral_accel_buffer.append(future_desired_lateral_accel)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      desired_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / lat_delay

      measurement = measured_curvature * CS.vEgo ** 2
      measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
      self.previous_measurement = measurement

      low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) / max(CS.vEgo, MIN_SPEED)) ** 2
      setpoint = lat_delay * desired_lateral_jerk + expected_lateral_accel
      error = setpoint - measurement
      error_lsf = error + low_speed_factor / self.torque_params.kp * error

      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error_lsf)
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
      ff -= self.torque_params.latAccelOffset
      # TODO jerk is weighted by lat_delay for legacy reasons, but should be made independent of it
      ff += get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params, True) * self.torque_params.latAccelFactor

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error,
                                       -measurement_rate,
                                        feedforward=ff,
                                        speed=CS.vEgo,
                                        freeze_integrator=freeze_integrator)
      output_torque = output_lataccel / self.torque_params.latAccelFactor

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)  # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    steer = self.yield_to_driver(-output_torque, active, CS, lat_delay)
    pid_log.output = float(steer)
    return float(steer), 0.0, pid_log
