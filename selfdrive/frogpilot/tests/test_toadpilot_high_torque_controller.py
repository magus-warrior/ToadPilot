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
                                                         TOADPILOT_HIGH_TORQUE_MAX_STEERING_RATE,
                                                         TOADPILOT_HIGH_TORQUE_SATURATION_FRAMES,
                                                         CarController,
                                                         actual_curvature_from_angular_velocity,
                                                         high_torque_runtime_allowed,
                                                         high_torque_tracking_shortfall,
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


if __name__ == "__main__":
  unittest.main()
