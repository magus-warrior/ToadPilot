import math
import unittest

from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from cereal import car
from panda import Panda

from openpilot.common.params import Params
from openpilot.selfdrive.car import apply_meas_steer_torque_limits, gen_empty_fingerprint
from openpilot.selfdrive.car.toyota.carcontroller import (TOADPILOT_HIGH_TORQUE_MAPPING_START,
                                                         TOADPILOT_HIGH_TORQUE_ARM_TRACKING_RATIO,
                                                         TOADPILOT_HIGH_TORQUE_FAULT_COOLDOWN_FRAMES,
                                                         TOADPILOT_HIGH_TORQUE_FULL_HEADROOM_RATE,
                                                         TOADPILOT_HIGH_TORQUE_MAX_STEERING_RATE,
                                                         TOADPILOT_HIGH_TORQUE_RAPID_RISE_DELTA,
                                                         TOADPILOT_HIGH_TORQUE_RAPID_RISE_MIN_RATE,
                                                         TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES,
                                                         TOADPILOT_STOP_RELEASE_WINDUP_LIMIT,
                                                         CarController,
                                                         actual_curvature_from_angular_velocity,
                                                         configured_high_torque_cap,
                                                         high_torque_headroom_scale,
                                                         high_torque_runtime_allowed,
                                                         high_torque_runtime_reason,
                                                         high_torque_tracking_shortfall,
                                                         limit_extra_torque_rise,
                                                         steer_torque_target,
                                                         update_high_torque_authority)
from openpilot.selfdrive.car.toyota.interface import CarInterface
from openpilot.selfdrive.car.toyota.values import (CAR, DBC, Ecu, TOADPILOT_HIGH_TORQUE_MAX,
                                                   TOYOTA_STEER_COMMAND_SCALE,
                                                   CarControllerParams)
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.selfdrive.frogpilot.toadpilot import TOADPILOT_EPS_FIRMWARE


def controller_params(*, fingerprint=CAR.TOYOTA_CAMRY_TSS2, torque=True, flagged=True):
  steer_type = car.CarParams.SteerControlType.torque if torque else car.CarParams.SteerControlType.angle
  safety_param = Panda.FLAG_TOYOTA_TOADPILOT_HIGH_TORQUE if flagged else 0
  CP = SimpleNamespace(
    flags=0,
    carFingerprint=fingerprint,
    steerControlType=steer_type,
    lateralTuning=SimpleNamespace(which=lambda: "torque" if torque else "pid"),
    safetyConfigs=[SimpleNamespace(safetyParam=safety_param)],
  )
  return CarControllerParams(CP)


class TestToadPilotHighTorqueController(unittest.TestCase):
  def test_authority_requires_flag_exact_camry_and_torque_control(self):
    enabled = controller_params()
    self.assertTrue(enabled.TOADPILOT_HIGH_TORQUE)
    self.assertEqual(enabled.STEER_MAX, TOADPILOT_HIGH_TORQUE_MAX)

    for params in (
      controller_params(flagged=False),
      controller_params(fingerprint=CAR.TOYOTA_COROLLA_TSS2),
      controller_params(torque=False),
    ):
      with self.subTest(params=params):
        self.assertFalse(params.TOADPILOT_HIGH_TORQUE)
        self.assertEqual(params.STEER_MAX, TOYOTA_STEER_COMMAND_SCALE)

  def test_normal_mapping_is_identical_below_stock_saturation(self):
    params = controller_params()
    for requested, expected in ((-0.8, -1200), (-0.5, -750), (0.0, 0), (0.5, 750), (0.8, 1200), (0.998, 1497)):
      with self.subTest(requested=requested):
        self.assertEqual(steer_torque_target(requested, params, False), expected)

    for requested, expected in ((-0.8, -1200), (0.0, 0), (0.8, 1200), (0.9, 1350)):
      with self.subTest(armed_requested=requested):
        self.assertEqual(steer_torque_target(requested, params, True), expected)

  def test_extension_is_proportional_after_saturation_handshake(self):
    params = controller_params()
    for requested, expected in ((1.0, 1700), (-1.0, -1700), (0.95, 1525), (-0.95, -1525)):
      with self.subTest(requested=requested):
        stock_target = int(round(requested * TOYOTA_STEER_COMMAND_SCALE))
        self.assertEqual(steer_torque_target(requested, params, False), stock_target)
        self.assertEqual(steer_torque_target(requested, params, True), expected)

    mapping_start_target = int(round(TOADPILOT_HIGH_TORQUE_MAPPING_START * TOYOTA_STEER_COMMAND_SCALE))
    self.assertEqual(steer_torque_target(TOADPILOT_HIGH_TORQUE_MAPPING_START, params, True), mapping_start_target)

    stock = controller_params(flagged=False)
    self.assertEqual(steer_torque_target(1.0, stock, True), 1500)
    self.assertEqual(steer_torque_target(-1.0, stock, True), -1500)

  def test_only_extra_headroom_tapers_near_high_or_rapidly_rising_rate(self):
    params = controller_params()
    self.assertEqual(steer_torque_target(1.0, params, True, 1.0), 1700)
    self.assertEqual(steer_torque_target(1.0, params, True, 0.5), 1600)
    self.assertEqual(steer_torque_target(1.0, params, True, 0.0), 1500)
    self.assertEqual(high_torque_headroom_scale(TOADPILOT_HIGH_TORQUE_FULL_HEADROOM_RATE, 0.0), 1.0)
    self.assertEqual(high_torque_headroom_scale(65.0, 0.0), 0.5)
    self.assertEqual(high_torque_headroom_scale(TOADPILOT_HIGH_TORQUE_MAX_STEERING_RATE, 0.0), 0.0)
    self.assertEqual(high_torque_headroom_scale(TOADPILOT_HIGH_TORQUE_RAPID_RISE_MIN_RATE,
                                                TOADPILOT_HIGH_TORQUE_RAPID_RISE_DELTA), 0.0)

  def test_selected_cap_is_bounded_and_malformed_values_fail_to_stock(self):
    params = controller_params()
    for configured in (1500, 1525, 1550, 1700):
      with self.subTest(configured=configured):
        self.assertEqual(configured_high_torque_cap(configured, params), configured)
        self.assertEqual(steer_torque_target(1.0, params, True, max_torque=configured), configured)

    for malformed in (None, True, 1499, 1510, 1701, 1550.0, "1550"):
      with self.subTest(malformed=malformed):
        self.assertEqual(configured_high_torque_cap(malformed, params), TOYOTA_STEER_COMMAND_SCALE)

  def test_extra_rise_is_five_per_frame_without_slowing_stock_or_release(self):
    self.assertEqual(limit_extra_torque_rise(1700, 1500, 1500), 1505)
    self.assertEqual(limit_extra_torque_rise(1700, 1505, 1500), 1510)
    self.assertEqual(limit_extra_torque_rise(-1700, -1550, 1500), -1555)
    self.assertEqual(limit_extra_torque_rise(1490, 1400, 1500), 1490)
    self.assertEqual(limit_extra_torque_rise(1500, 1560, 1500), 1500)

  def test_saturation_arming_is_continuous_with_release_hysteresis(self):
    direction = frames = 0
    armed = False
    for _ in range(TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES - 1):
      direction, frames, armed = update_high_torque_authority(1.0, True, True, direction, frames, armed)
      self.assertFalse(armed)

    direction, frames, armed = update_high_torque_authority(1.0, True, True, direction, frames, armed)
    self.assertTrue(armed)

    direction, frames, armed = update_high_torque_authority(0.90, True, True, direction, frames, armed)
    self.assertTrue(armed)
    self.assertEqual(direction, 1)

    for requested, active, enabled in ((0.84, True, True), (1.0, False, True), (1.0, True, False), (-1.0, True, True)):
      with self.subTest(requested=requested, active=active, enabled=enabled):
        self.assertEqual(update_high_torque_authority(requested, active, enabled, direction, frames, armed), (0, 0, False))

  def test_tracking_gate_requires_speed_curve_strength_direction_and_shortfall(self):
    ratio = TOADPILOT_HIGH_TORQUE_ARM_TRACKING_RATIO
    self.assertTrue(high_torque_tracking_shortfall(0.08, 0.05, 5.0, ratio))
    self.assertTrue(high_torque_tracking_shortfall(-0.08, -0.05, 5.0, ratio))

    rejected = (
      (0.08, 0.05, 4.99, ratio),   # too slow
      (0.005, 0.004, 10.0, ratio), # curve demand too small
      (0.08, -0.05, 5.0, ratio),  # wrong response direction
      (0.08, 0.07, 5.0, ratio),   # tracking within the arm threshold
      (float("nan"), 0.05, 5.0, ratio),
    )
    for args in rejected:
      with self.subTest(args=args):
        self.assertFalse(high_torque_tracking_shortfall(*args))

  def test_runtime_gate_requires_engagement_and_excludes_fault_and_lane_change_states(self):
    allowed = {
      "main_enabled": True,
      "lat_active": True,
      "steering_pressed": False,
      "brake_pressed": False,
      "stopping": False,
      "lane_change_active": False,
      "steering_rate_deg": 10.0,
      "requested_steer": -1.0,
      "desired_curvature": 0.02,
      "actual_curvature": 0.01,
      "v_ego": 10.0,
      "actual_ratio": TOADPILOT_HIGH_TORQUE_ARM_TRACKING_RATIO,
    }
    self.assertTrue(high_torque_runtime_allowed(**allowed))

    rejected = {
      "main_disabled": {"main_enabled": False},
      "lateral_inactive": {"lat_active": False},
      "driver_override": {"steering_pressed": True},
      "braking": {"brake_pressed": True},
      "stopping": {"stopping": True},
      "lane_change": {"lane_change_active": True},
      "preemptive_rate_boundary": {"steering_rate_deg": TOADPILOT_HIGH_TORQUE_MAX_STEERING_RATE},
      "nonfinite_rate": {"steering_rate_deg": float("nan")},
      "wrong_torque_direction": {"requested_steer": 1.0},
      "wrong_response_direction": {"actual_curvature": -0.01},
      "tracking_close_enough": {"actual_curvature": 0.019},
    }
    for name, overrides in rejected.items():
      with self.subTest(name=name):
        case = {**allowed, **overrides}
        self.assertFalse(high_torque_runtime_allowed(**case))
        self.assertNotEqual(high_torque_runtime_reason(**case), "eligible")

  def test_actual_curvature_uses_calibrated_yaw_rate_and_fails_closed(self):
    self.assertAlmostEqual(actual_curvature_from_angular_velocity([0.0, 0.0, 0.2], 10.0), 0.02)
    for angular_velocity, speed in (([], 10.0), ([0.0, 0.0, math.nan], 10.0), ([0.0, 0.0, 0.2], 0.0)):
      with self.subTest(angular_velocity=angular_velocity, speed=speed):
        self.assertTrue(math.isnan(actual_curvature_from_angular_velocity(angular_velocity, speed)))

  def test_existing_rate_and_measured_torque_guards_remain_effective(self):
    params = controller_params()
    command = 1500
    measured = 1500
    command = apply_meas_steer_torque_limits(1700, command, measured, params)
    self.assertEqual(command, 1515)

    for _ in range(100):
      command = apply_meas_steer_torque_limits(1700, command, measured, params)
    self.assertEqual(command, params.STEER_MAX)
    self.assertLessEqual(command, measured + params.STEER_ERROR_MAX)


class TestToadPilotHighTorqueControllerSequence(unittest.TestCase):
  def setUp(self):
    self.params_dir = TemporaryDirectory()
    params = Params(self.params_dir.name)
    eps_fw = car.CarParams.CarFw.new_message()
    eps_fw.ecu = Ecu.eps
    eps_fw.fwVersion = TOADPILOT_EPS_FIRMWARE
    with patch("openpilot.selfdrive.car.toyota.interface.read_toadpilot_steering_response_latch",
               return_value=True), \
         patch("openpilot.selfdrive.car.toyota.interface.toadpilot_steering_response_panda_matched",
               return_value=True):
      self.CP = CarInterface.get_params(
        CAR.TOYOTA_CAMRY_TSS2, gen_empty_fingerprint(), [eps_fw], False, False, params, False,
      )

    self.CP.openpilotLongitudinalControl = False
    self.CP.enableDsu = False
    self.controller = CarController(DBC[CAR.TOYOTA_CAMRY_TSS2]["pt"], self.CP, VehicleModel(self.CP))

    self.CC = car.CarControl.new_message()
    self.CC.enabled = True
    self.CC.latActive = True
    self.CC.longActive = False
    self.CC.actuators.steer = -1.0
    self.CC.actuators.curvature = 0.02
    self.CC.actuators.longControlState = car.CarControl.Actuators.LongControlState.pid
    self.CC.angularVelocity = [0.0, 0.0, 0.1]

    self.car_state = car.CarState.new_message()
    self.car_state.vEgo = 10.0
    self.car_state.vEgoRaw = 10.0
    self.car_state.gearShifter = car.CarState.GearShifter.drive
    self.CS = SimpleNamespace(
      out=self.car_state,
      secoc_synchronization={},
      pcm_acc_status=0,
      lkas_hud={},
      pcm_follow_distance=0,
      acc_type=0,
      gvc=0.0,
    )
    self.toggles = SimpleNamespace(
      sport_plus=False,
      max_desired_acceleration=2.0,
      sng_hack=False,
      frogsgomoo_tweak=False,
      lock_doors=False,
      unlock_doors=False,
      reverse_cruise_increase=False,
      toadpilot_improved_stop_and_go=True,
      toadpilot_experimental_max_torque=1700,
    )

  def tearDown(self):
    self.params_dir.cleanup()

  def update(self):
    # Model a responsive EPS so the unchanged measured-torque error guard does
    # not obscure the controller sequence under test.
    self.car_state.steeringTorqueEps = self.controller.last_steer
    return self.controller.update(self.CC.as_reader(), self.CS, 0, self.toggles)[0]

  def test_real_update_extends_only_after_handshake_and_resets_for_fault_rate(self):
    for _ in range(TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES - 1):
      output = self.update()
      self.assertFalse(self.controller.high_torque_armed)
      self.assertLessEqual(abs(output.steerOutputCan), TOYOTA_STEER_COMMAND_SCALE)

    self.update()
    self.assertTrue(self.controller.high_torque_armed)

    output = None
    while abs(self.controller.last_steer) <= TOYOTA_STEER_COMMAND_SCALE:
      output = self.update()
    self.assertIsNotNone(output)
    self.assertGreater(abs(output.steerOutputCan), TOYOTA_STEER_COMMAND_SCALE)

    # Toyota's existing request-bit fault avoidance begins at this rate. The
    # extension target must disarm immediately and stay disarmed through cuts.
    self.car_state.steeringRateDeg = 100.0
    for _ in range(TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES):
      output = self.update()
      self.assertFalse(self.controller.high_torque_armed)
      self.assertEqual(self.controller.high_torque_saturation_frames, 0)
    self.assertLessEqual(abs(output.steerOutputCan), TOYOTA_STEER_COMMAND_SCALE)

  def test_selected_cap_is_frozen_and_extra_command_uses_slow_rise(self):
    self.toggles.toadpilot_experimental_max_torque = 1550
    outputs = []
    for frame in range(180):
      if frame == 1:
        # A live toggle refresh cannot raise authority after initialization.
        self.toggles.toadpilot_experimental_max_torque = 1700
      outputs.append(self.update())

    self.assertEqual(self.controller.toadpilot_experimental_max_torque, 1550)
    self.assertEqual(max(abs(output.steerOutputCan) for output in outputs), 1550)
    extra_outputs = [output for output in outputs if abs(output.steerOutputCan) > TOYOTA_STEER_COMMAND_SCALE]
    extra = [abs(output.steerOutputCan) for output in extra_outputs]
    self.assertGreater(len(extra), 1)
    self.assertEqual(extra[:3], [1505, 1510, 1515])
    self.assertEqual(abs(extra_outputs[0].toadPilotHighTorqueTargetCan), 1550)

  def test_full_ceiling_unwinds_at_the_unchanged_rate_after_a_runtime_blocker(self):
    self.controller.last_steer = -TOADPILOT_HIGH_TORQUE_MAX
    self.controller.high_torque_saturation_direction = -1
    self.controller.high_torque_saturation_frames = TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES
    self.controller.high_torque_armed = True
    self.car_state.steeringRateDeg = TOADPILOT_HIGH_TORQUE_MAX_STEERING_RATE

    # Target authority disarms immediately, but the physical command must honor
    # the unchanged 25-unit/frame Toyota/Panda unwind limit. This explicitly
    # characterizes the bounded 0.08-second unwind from the 1700 ceiling.
    for _ in range(4):
      output = self.update()
      self.assertFalse(self.controller.high_torque_armed)
    self.assertEqual(abs(output.steerOutputCan), 1600)

    for _ in range(4):
      output = self.update()
    self.assertEqual(abs(output.steerOutputCan), TOYOTA_STEER_COMMAND_SCALE)

  def test_real_update_blocks_aol_lane_change_brake_and_driver_states(self):
    blockers = (
      ("main_disabled_aol", lambda: setattr(self.CC, "enabled", False)),
      ("lane_change", lambda: setattr(self.CC, "leftBlinker", True)),
      ("braking", lambda: setattr(self.car_state, "brakePressed", True)),
      ("driver_override", lambda: setattr(self.car_state, "steeringPressed", True)),
    )
    for name, block in blockers:
      with self.subTest(name=name):
        self.CC.enabled = True
        self.CC.leftBlinker = False
        self.car_state.brakePressed = False
        self.car_state.steeringPressed = False
        self.controller.high_torque_saturation_direction = -1
        self.controller.high_torque_saturation_frames = TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES
        self.controller.high_torque_armed = True
        block()
        self.update()
        self.assertFalse(self.controller.high_torque_armed)

  def test_rapid_rate_rise_pauses_only_extra_headroom_without_losing_arm(self):
    for _ in range(TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES):
      self.update()
    self.assertTrue(self.controller.high_torque_armed)
    for _ in range(10):
      self.car_state.steeringRateDeg = 40.0
      self.update()

    self.car_state.steeringRateDeg = 65.0
    output = self.update()
    self.assertTrue(self.controller.high_torque_armed)
    self.assertEqual(output.toadPilotHighTorqueReason, "extra_paused_rate_rising")
    self.assertLessEqual(abs(output.toadPilotHighTorqueTargetCan), TOYOTA_STEER_COMMAND_SCALE)

    for _ in range(TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES):
      self.car_state.steeringRateDeg = 40.0
      output = self.update()
    self.assertTrue(self.controller.high_torque_armed)
    self.assertEqual(output.toadPilotHighTorqueHeadroomScale, 1.0)
    self.assertGreater(abs(output.toadPilotHighTorqueTargetCan), TOYOTA_STEER_COMMAND_SCALE)

  def test_temporary_eps_fault_pauses_extra_authority_then_requires_normal_rearm(self):
    for _ in range(TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES):
      self.update()
    self.assertTrue(self.controller.high_torque_armed)

    self.car_state.steerFaultTemporary = True
    output = self.update()
    self.assertFalse(self.controller.high_torque_armed)
    self.assertEqual(output.toadPilotHighTorqueState, "eps_temporary_fault")
    self.assertLessEqual(abs(output.toadPilotHighTorqueTargetCan), TOYOTA_STEER_COMMAND_SCALE)

    self.car_state.steerFaultTemporary = False
    for frame in range(TOADPILOT_HIGH_TORQUE_FAULT_COOLDOWN_FRAMES):
      output = self.update()
      self.assertFalse(self.controller.high_torque_armed)
      self.assertLessEqual(abs(output.toadPilotHighTorqueTargetCan), TOYOTA_STEER_COMMAND_SCALE)
      self.assertEqual(output.toadPilotHighTorqueState, "eps_cooldown")
      if frame == TOADPILOT_HIGH_TORQUE_FAULT_COOLDOWN_FRAMES - 1:
        self.assertAlmostEqual(output.toadPilotFaultCooldown, 0.01)

    for _ in range(TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES):
      output = self.update()
    self.assertTrue(self.controller.high_torque_armed)
    self.assertGreater(abs(output.toadPilotHighTorqueTargetCan), TOYOTA_STEER_COMMAND_SCALE)

  def test_armed_partial_extension_reports_applied_reason_and_effective_scale(self):
    for _ in range(TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES):
      self.update()
    self.CC.actuators.steer = -0.95
    output = self.update()
    self.assertEqual(output.toadPilotHighTorqueState, "armed")
    self.assertEqual(output.toadPilotHighTorqueReason, "curve_undertracking")
    self.assertGreater(abs(output.toadPilotHighTorqueTargetCan), TOYOTA_STEER_COMMAND_SCALE)
    self.assertEqual(output.toadPilotHighTorqueHeadroomScale, 1.0)

  def test_final_status_matches_zeroed_lateral_command(self):
    self.controller.last_steer = -TOADPILOT_HIGH_TORQUE_MAX
    self.controller.high_torque_saturation_direction = -1
    self.controller.high_torque_saturation_frames = TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES
    self.controller.high_torque_armed = True
    self.CC.latActive = False

    output = self.update()
    self.assertEqual(output.steerOutputCan, 0)
    self.assertFalse(output.toadPilotSteerRequest)
    self.assertEqual(output.toadPilotHighTorqueState, "stock")
    self.assertEqual(output.toadPilotHighTorqueHeadroomScale, 0.0)

  def test_confirmed_camry_launch_clears_stale_integral_and_unwinds_only_negative_request_faster(self):
    self.CP.openpilotLongitudinalControl = True
    self.CC.longActive = True
    self.CC.actuators.longControlState = car.CarControl.Actuators.LongControlState.pid
    self.CC.actuators.accel = 0.6
    self.car_state.vEgo = 0.0
    self.car_state.vEgoRaw = 0.0
    self.controller.previous_long_control_state = car.CarControl.Actuators.LongControlState.stopping
    self.controller.prev_accel = -1.0
    self.controller.long_pid.i = -0.5
    self.controller.frame = 0

    output = self.update()
    self.assertAlmostEqual(self.controller.prev_accel, -1.0 + TOADPILOT_STOP_RELEASE_WINDUP_LIMIT)
    self.assertTrue(output.toadPilotStopGoReleaseActive)
    self.assertAlmostEqual(output.toadPilotLongAccelTarget, 0.6)
    self.assertGreater(output.accel, -1.0)

    # The faster release may reach zero, but it must not carry any part of the
    # larger step into positive acceleration.
    self.controller.prev_accel = -0.05
    self.controller.toadpilot_stop_go_release_active = True
    self.controller.frame = 3
    output = self.update()
    self.assertEqual(self.controller.prev_accel, 0.0)
    self.assertFalse(output.toadPilotStopGoReleaseActive)


if __name__ == "__main__":
  unittest.main()
