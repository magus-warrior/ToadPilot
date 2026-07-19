import math
import numpy as np

from cereal import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import carlog, apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, create_gas_interceptor_command, make_can_msg, rate_limit
from openpilot.selfdrive.car.secoc import add_mac, build_sync_mac
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.toyota import toyotacan
from openpilot.selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR, STOP_AND_GO_CAR
from openpilot.selfdrive.controls.lib.drive_helpers import CRUISE_LONG_PRESS
from openpilot.selfdrive.controls.lib.pid import PIDController
from opendbc.can.packer import CANPacker

from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_acceleration import get_max_allowed_accel

LongCtrlState = car.CarControl.Actuators.LongControlState
SteerControlType = car.CarParams.SteerControlType
VisualAlert = car.CarControl.HUDControl.VisualAlert

ACCELERATION_DUE_TO_GRAVITY = 9.81  # m/s^2

# The up limit allows the brakes/gas to unwind quickly leaving a stop,
# the down limit roughly matches the rate of ACCEL_NET, reducing PCM compensation windup
ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_PID_UNWIND = 0.03 * DT_CTRL * 3  # m/s^2 / frame

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# The research envelope is deliberately an extension of stock saturation, not
# a gain change. Normal requests retain Toyota's 1500-unit scale. A full-scale
# request must persist for this long before the target can extend toward the
# separately authorized hard ceiling.
TOADPILOT_HIGH_TORQUE_SATURATION_THRESHOLD = 0.999
TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES = 40  # 0.4 seconds at 100 Hz
TOADPILOT_HIGH_TORQUE_MAPPING_START = 0.90
TOADPILOT_HIGH_TORQUE_RELEASE_THRESHOLD = 0.85
TOADPILOT_HIGH_TORQUE_MIN_SPEED = 5.0
TOADPILOT_HIGH_TORQUE_MAX_SPEED = 15.0
TOADPILOT_HIGH_TORQUE_MIN_DESIRED_ACCEL = 1.0
TOADPILOT_HIGH_TORQUE_ARM_TRACKING_RATIO = 0.80
TOADPILOT_HIGH_TORQUE_RELEASE_TRACKING_RATIO = 0.95
TOADPILOT_HIGH_TORQUE_MAX_STEERING_RATE = 75.0

# LTA limits
# EPS ignores commands above this angle and causes PCS to fault
MAX_LTA_ANGLE = 94.9461  # deg
MAX_LTA_DRIVER_TORQUE_ALLOWANCE = 150  # slightly above steering pressed allows some resistance when changing lanes

# Lock / unlock door commands - Credit goes to AlexandreSato!
LOCK_CMD = b"\x40\x05\x30\x11\x00\x80\x00\x00"
UNLOCK_CMD = b"\x40\x05\x30\x11\x00\x40\x00\x00"

PARK = car.CarState.GearShifter.park


def update_high_torque_authority(requested_steer, lat_active, enabled, direction, frames, armed):
  """Arm on sustained saturation and release with conservative hysteresis."""
  if not lat_active or not enabled:
    return 0, 0, False

  requested_direction = 1 if requested_steer > 0.0 else -1 if requested_steer < 0.0 else 0
  requested_magnitude = abs(requested_steer)

  if armed:
    keep_armed = requested_direction == direction and requested_magnitude >= TOADPILOT_HIGH_TORQUE_RELEASE_THRESHOLD
    return (direction, frames, True) if keep_armed else (0, 0, False)

  saturated = requested_direction != 0 and requested_magnitude >= TOADPILOT_HIGH_TORQUE_SATURATION_THRESHOLD
  if not saturated:
    return 0, 0, False

  frames = frames + 1 if requested_direction == direction else 1
  frames = min(frames, TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES)
  return requested_direction, frames, frames >= TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES


def high_torque_tracking_shortfall(desired_curvature, actual_curvature, v_ego, actual_ratio):
  """Require meaningful, same-direction lateral undertracking in a test-speed window."""
  try:
    v_ego = float(v_ego)
    desired_lateral_accel = float(desired_curvature) * v_ego ** 2
    actual_lateral_accel = float(actual_curvature) * v_ego ** 2
    actual_ratio = float(actual_ratio)
  except (TypeError, ValueError):
    return False

  finite = all(math.isfinite(value) for value in (desired_lateral_accel, actual_lateral_accel, actual_ratio, v_ego))
  return (finite and 0.0 < actual_ratio <= 1.0 and
          TOADPILOT_HIGH_TORQUE_MIN_SPEED <= v_ego <= TOADPILOT_HIGH_TORQUE_MAX_SPEED and
          abs(desired_lateral_accel) >= TOADPILOT_HIGH_TORQUE_MIN_DESIRED_ACCEL and
          desired_lateral_accel * actual_lateral_accel > 0.0 and
          abs(actual_lateral_accel) < actual_ratio * abs(desired_lateral_accel))


def actual_curvature_from_angular_velocity(angular_velocity, v_ego):
  """Use calibrated yaw rate for a roll/steering-offset-independent response."""
  try:
    if len(angular_velocity) != 3:
      return math.nan
    v_ego = float(v_ego)
    yaw_rate = float(angular_velocity[2])
  except (TypeError, ValueError):
    return math.nan
  if not math.isfinite(v_ego) or not math.isfinite(yaw_rate) or v_ego <= 0.0:
    return math.nan
  return yaw_rate / v_ego


def high_torque_runtime_allowed(*, main_enabled, lat_active, steering_pressed, brake_pressed, stopping,
                                lane_change_active, steering_rate_deg, requested_steer, desired_curvature,
                                actual_curvature, v_ego, actual_ratio):
  """Fail-closed runtime gate for entering or retaining extended authority."""
  try:
    steering_rate_deg = float(steering_rate_deg)
    requested_steer = float(requested_steer)
    desired_curvature = float(desired_curvature)
  except (TypeError, ValueError):
    return False

  finite = all(math.isfinite(value) for value in (steering_rate_deg, requested_steer, desired_curvature))
  direction_matches = requested_steer * desired_curvature < 0.0
  return (finite and main_enabled and lat_active and not steering_pressed and not brake_pressed and not stopping and
          not lane_change_active and abs(steering_rate_deg) < TOADPILOT_HIGH_TORQUE_MAX_STEERING_RATE and
          direction_matches and
          high_torque_tracking_shortfall(desired_curvature, actual_curvature, v_ego, actual_ratio))


def steer_torque_target(requested_steer, params, high_torque_armed):
  """Preserve stock scaling, then add proportional headroom once armed."""
  requested_steer = float(clip(requested_steer, -1.0, 1.0))
  target = int(round(requested_steer * params.STEER_COMMAND_SCALE))
  if params.TOADPILOT_HIGH_TORQUE and high_torque_armed and \
      abs(requested_steer) > TOADPILOT_HIGH_TORQUE_MAPPING_START:
    direction = 1 if requested_steer > 0.0 else -1
    stock_mapping_start = TOADPILOT_HIGH_TORQUE_MAPPING_START * params.STEER_COMMAND_SCALE
    target_magnitude = interp(
      abs(requested_steer), [TOADPILOT_HIGH_TORQUE_MAPPING_START, 1.0],
      [stock_mapping_start, params.STEER_MAX],
    )
    target = int(round(direction * target_magnitude))
  return int(clip(target, -params.STEER_MAX, params.STEER_MAX))


def update_standstill_request(standstill_req, *, no_stop_timer, hybrid, gas_interceptor, sng_hack,
                              standstill, last_standstill, pcm_acc_status, cruise_standstill, requested_accel):
  """Update Toyota's ACC standstill bit without delaying hybrid launches.

  Newer NO_STOP_TIMER platforms can enter a special PCM standstill mode when
  cruise is engaged while the brake is held. Non-hybrids need that mode held
  until the planner requests positive acceleration to avoid a resume lurch.
  Hybrids can remain stopped after the bit is released, so holding it only adds
  delay. Older platforms (and legacy gas-interceptor setups) retain their
  original PCM-status behavior.
  """
  if not no_stop_timer or gas_interceptor:
    if standstill and not last_standstill:
      standstill_req = True
    if pcm_acc_status != 8 or sng_hack:
      standstill_req = False
    return standstill_req

  # FrogPilot's explicit SNG override retains its historical behavior.
  if sng_hack:
    return False

  # Toyota hybrids do not exhibit the non-hybrid resume lurch and can stay
  # stopped after RELEASE_STANDSTILL is sent.
  if hybrid:
    return False

  should_resume = requested_accel > 0.0
  if should_resume:
    standstill_req = False
  elif cruise_standstill:
    standstill_req = True
  return standstill_req


def get_long_tune(CP, params):
  kiBP = [2., 5.]
  kiV = [0.5, 0.25]

  return PIDController(0.0, (kiBP, kiV), k_f=1.0,
                       pos_limit=params.ACCEL_MAX, neg_limit=params.ACCEL_MIN,
                       rate=1 / (DT_CTRL * 3))


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.params = CarControllerParams(self.CP)
    self.frame = 0
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.permit_braking = True
    self.steer_rate_counter = 0
    self.distance_button = 0
    self.high_torque_saturation_direction = 0
    self.high_torque_saturation_frames = 0
    self.high_torque_armed = False

    # *** start long control state ***
    self.long_pid = get_long_tune(self.CP, self.params)

    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL * 3)
    self.pitch = FirstOrderFilter(0, 0.5, DT_CTRL)

    self.accel = 0
    self.prev_accel = 0
    # *** end long control state ***

    self.packer = CANPacker(dbc_name)

    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_prev_reset_counter = 0
    self.secoc_key: bytes = b"00" * 16

    # FrogPilot variables
    self.stock_max_accel = self.params.ACCEL_MAX

    self.doors_locked = False
    self.reverse_cruise_active = False

    self.cruise_timer = 0
    self.previous_set_speed = 0

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    if frogpilot_toggles.sport_plus:
      self.params.ACCEL_MAX = min(frogpilot_toggles.max_desired_acceleration, get_max_allowed_accel(CS.out.vEgo))
      self.long_pid.pos_limit = self.params.ACCEL_MAX
    else:
      self.params.ACCEL_MAX = min(frogpilot_toggles.max_desired_acceleration, self.stock_max_accel)
      self.long_pid.pos_limit = self.params.ACCEL_MAX

    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])

    # *** control msgs ***
    can_sends = []

    # *** handle secoc reset counter increase ***
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

        expected_mac = build_sync_mac(self.secoc_key, int(CS.secoc_synchronization['TRIP_CNT']), int(CS.secoc_synchronization['RESET_CNT']))
        if int(CS.secoc_synchronization['AUTHENTICATOR']) != expected_mac:
          carlog.error("SecOC synchronization MAC mismatch, wrong key?")

    # *** steer torque ***
    actual_curvature = actual_curvature_from_angular_velocity(CC.angularVelocity, CS.out.vEgo)
    tracking_ratio = (TOADPILOT_HIGH_TORQUE_RELEASE_TRACKING_RATIO if self.high_torque_armed else
                      TOADPILOT_HIGH_TORQUE_ARM_TRACKING_RATIO)
    high_torque_allowed = high_torque_runtime_allowed(
      main_enabled=CC.enabled,
      lat_active=lat_active,
      steering_pressed=CS.out.steeringPressed,
      brake_pressed=CS.out.brakePressed,
      stopping=stopping,
      lane_change_active=CC.leftBlinker or CC.rightBlinker,
      steering_rate_deg=CS.out.steeringRateDeg,
      requested_steer=actuators.steer,
      desired_curvature=actuators.curvature,
      actual_curvature=actual_curvature,
      v_ego=CS.out.vEgo,
      actual_ratio=tracking_ratio,
    )
    self.high_torque_saturation_direction, self.high_torque_saturation_frames, self.high_torque_armed = \
      update_high_torque_authority(
        actuators.steer, high_torque_allowed, self.params.TOADPILOT_HIGH_TORQUE,
        self.high_torque_saturation_direction, self.high_torque_saturation_frames, self.high_torque_armed,
      )
    new_steer = steer_torque_target(actuators.steer, self.params, self.high_torque_armed)
    # Runtime blockers remove the extended target immediately. The applied CAN
    # command still unwinds through Toyota/Panda's unchanged 25-unit/frame limit;
    # from 1700, returning to the stock 1500 envelope takes up to 0.08 seconds.
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_steer = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      apply_steer = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw, self.params)

        if not lat_active:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        self.last_angle = clip(apply_angle, -MAX_LTA_ANGLE, MAX_LTA_ANGLE)

    self.last_steer = apply_steer

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    steer_command = toyotacan.create_steer_command(self.packer, apply_steer, apply_steer_req)
    if self.CP.flags & ToyotaFlags.SECOC.value:
      # TODO: check if this slow and needs to be done by the CANPacker
      steer_command = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lka_message_counter,
                              steer_command)
      self.secoc_lka_message_counter += 1
    can_sends.append(steer_command)

    # STEERING_LTA does not seem to allow more rate by sending faster, and may wind up easier
    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      # cut steering torque with TORQUE_WIND_DOWN when either EPS torque or driver torque is above
      # the threshold, to limit max lateral acceleration and for driver torque blending respectively.
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

      if self.CP.flags & ToyotaFlags.SECOC.value:
        lta_steer_2 = toyotacan.create_lta_steer_command_2(self.packer, self.frame // 2)
        lta_steer_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lta_message_counter,
                              lta_steer_2)
        self.secoc_lta_message_counter += 1
        can_sends.append(lta_steer_2)

    # *** gas and brake ***
    if self.CP.enableGasInterceptor and CC.longActive and self.CP.carFingerprint not in STOP_AND_GO_CAR:
      MAX_INTERCEPTOR_GAS = 0.5
      # RAV4 has very sensitive gas pedal
      if self.CP.carFingerprint == CAR.TOYOTA_RAV4:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif self.CP.carFingerprint == CAR.TOYOTA_COROLLA:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      # offset for creep and windbrake
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (self.accel + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    elif self.CP.enableGasInterceptor and CC.longActive and self.CP.carFingerprint in STOP_AND_GO_CAR and self.accel > 0.0:
      interceptor_gas_cmd = 0.12 if CS.out.standstill else 0.
    else:
      interceptor_gas_cmd = 0.

    if self.frame % 2 == 0 and self.CP.enableGasInterceptor and self.CP.openpilotLongitudinalControl:
      # send exactly zero if gas cmd is zero. Interceptor will send the max between read value and gas cmd.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, self.frame // 2))

    # Older platforms use PCM status to manage standstill. Newer non-hybrids
    # hold the request until the planner wants to move; hybrids take the direct
    # resume path from upstream opendbc #3009.
    self.standstill_req = update_standstill_request(
      self.standstill_req,
      no_stop_timer=self.CP.carFingerprint in NO_STOP_TIMER_CAR,
      hybrid=bool(self.CP.flags & ToyotaFlags.HYBRID.value),
      gas_interceptor=self.CP.enableGasInterceptor,
      sng_hack=frogpilot_toggles.sng_hack,
      standstill=CS.out.standstill,
      last_standstill=self.last_standstill,
      pcm_acc_status=CS.pcm_acc_status,
      cruise_standstill=CS.out.cruiseState.standstill,
      requested_accel=actuators.accel,
    )

    self.last_standstill = CS.out.standstill

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 3 == 0:
        # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
        if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
          desired_distance = 4 - hud_control.leadDistanceBars
          if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
            self.distance_button = not self.distance_button
          else:
            self.distance_button = 0

        # internal PCM gas command can get stuck unwinding from negative accel so we apply a generous rate limit
        pcm_accel_cmd = actuators.accel
        if CC.longActive:
          pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
        self.prev_accel = pcm_accel_cmd

        # calculate amount of acceleration PCM should apply to reach target, given pitch.
        # clipped to only include downhill angles, avoids erroneously unsetting PERMIT_BRAKING when stopping on uphills
        accel_due_to_pitch = math.sin(min(self.pitch.x, 0.0)) * ACCELERATION_DUE_TO_GRAVITY
        # TODO: on uphills this sometimes sets PERMIT_BRAKING low not considering the creep force
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch

        # GVC does not overshoot ego acceleration when starting from stop, but still has a similar delay
        if not self.CP.flags & ToyotaFlags.SECOC.value:
          a_ego_blended = interp(CS.out.vEgo, [1.0, 2.0], [CS.gvc, CS.out.aEgo])
        else:
          a_ego_blended = CS.out.aEgo

        # wind down integral when approaching target for step changes and smooth ramps to reduce overshoot
        prev_aego = self.aego.x
        self.aego.update(a_ego_blended)
        j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 3)

        if frogpilot_toggles.frogsgomoo_tweak:
          future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.35, 1.0]))
        else:
          future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.25, 0.5]))
        a_ego_future = a_ego_blended + j_ego * future_t

        if CC.longActive:
          # constantly slowly unwind integral to recover from large temporary errors
          self.long_pid.i -= ACCEL_PID_UNWIND * float(np.sign(self.long_pid.i))

          error_future = pcm_accel_cmd - a_ego_future
          pcm_accel_cmd = self.long_pid.update(error_future,
                                               speed=CS.out.vEgo,
                                               feedforward=pcm_accel_cmd,
                                               freeze_integrator=actuators.longControlState != LongCtrlState.pid)
        else:
          self.long_pid.reset()

        # Along with rate limiting positive jerk above, this greatly improves gas response time
        # Consider the net acceleration request that the PCM should be applying (pitch included)
        net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        if net_acceleration_request_min < 0.2 or stopping or not CC.longActive:
          self.permit_braking = True
        elif net_acceleration_request_min > 0.3:
          self.permit_braking = False

        pcm_accel_cmd = clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX)

        can_sends.append(toyotacan.create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.permit_braking, self.standstill_req, lead,
                                                        CS.acc_type, fcw_alert, self.distance_button, self.reverse_cruise_active))
        self.accel = pcm_accel_cmd

    else:
      # we can spam can to cancel the system even if we are using lat only control
      if pcm_cancel_cmd:
        if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
          can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
        else:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead, CS.acc_type, False, self.distance_button, self.reverse_cruise_active))

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud, lat_active))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append([0x750, 0, b"\x0F\x02\x3E\x00\x00\x00\x00\x00", 0])

    new_actuators = actuators.as_builder()
    # Keep cereal's normalized steering contract and the torque learner's
    # established calibration on the stock 1500-unit scale. steerOutputCan is
    # the exact (possibly extended) command for diagnostics.
    new_actuators.steer = clip(apply_steer / self.params.STEER_COMMAND_SCALE, -1.0, 1.0)
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    # FrogPilot Toyota carcontroller functions
    if False: #self.previous_set_speed != CS.out.cruiseState.speedCluster:
      self.cruise_timer = CRUISE_LONG_PRESS
    elif self.cruise_timer > 0:
      self.cruise_timer -= 1
    else:
      self.previous_set_speed = CS.out.cruiseState.speedCluster

    # Lock doors when in drive / unlock doors when in park
    if not self.doors_locked and CS.out.gearShifter != PARK:
      if frogpilot_toggles.lock_doors:
        can_sends.append(make_can_msg(0x750, LOCK_CMD, 0))
      self.doors_locked = True
    elif self.doors_locked and CS.out.gearShifter == PARK:
      if frogpilot_toggles.unlock_doors:
        can_sends.append(make_can_msg(0x750, UNLOCK_CMD, 0))
      self.doors_locked = False

    self.reverse_cruise_active = frogpilot_toggles.reverse_cruise_increase
    self.reverse_cruise_active &= self.cruise_timer <= 0

    self.frame += 1
    return new_actuators, can_sends
