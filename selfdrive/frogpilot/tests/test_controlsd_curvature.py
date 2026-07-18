import unittest
from types import SimpleNamespace

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.controlsd import curvature_cap_alert_required, get_lateral_target_curvature
from openpilot.selfdrive.controls.lib.drive_helpers import (
  MAX_CURVATURE,
  MAX_LATERAL_ACCEL_NO_ROLL,
  MAX_LATERAL_JERK,
  MIN_SPEED,
  clip_curvature,
)
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY


class ConcreteLatControl(LatControl):
  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, llk,
             model_data=None, frogpilot_toggles=None, experimental_mode=False, toadpilot_centering_status=None,
             toadpilot_longitudinal_stopping=False):
    raise NotImplementedError


class TestControlsCurvatureReset(unittest.TestCase):
  def test_inactive_control_tracks_measured_curvature_not_toadpilot_request(self):
    measured_curvature = 0.004
    toadpilot_requested_curvature = -0.006

    target = get_lateral_target_curvature(
      False, measured_curvature, toadpilot_requested_curvature,
    )

    self.assertEqual(target, measured_curvature)

  def test_active_control_preserves_toadpilot_requested_curvature(self):
    measured_curvature = 0.004
    toadpilot_requested_curvature = -0.006

    target = get_lateral_target_curvature(
      True, measured_curvature, toadpilot_requested_curvature,
    )

    self.assertEqual(target, toadpilot_requested_curvature)

  def test_engagement_starts_rate_limited_from_measured_curvature(self):
    v_ego = 20.0
    measured_curvature = 0.004
    requested_curvature = -0.006
    previous_desired_curvature = measured_curvature

    target = get_lateral_target_curvature(
      True, measured_curvature, requested_curvature,
    )
    engaged_curvature, curvature_limited = clip_curvature(
      v_ego, previous_desired_curvature, target, planner_curves=False, roll=0.0,
    )

    max_step = MAX_LATERAL_JERK / max(MIN_SPEED, v_ego) ** 2 * DT_CTRL
    self.assertAlmostEqual(engaged_curvature, measured_curvature - max_step)
    self.assertGreater(engaged_curvature, requested_curvature)
    self.assertFalse(curvature_limited)


class TestCurvatureAccelerationLimits(unittest.TestCase):
  def test_positive_and_negative_lateral_acceleration_are_limited(self):
    v_ego = 20.0

    positive, positive_limited = clip_curvature(v_ego, 0.1, 0.1, planner_curves=False, roll=0.0)
    negative, negative_limited = clip_curvature(v_ego, -0.1, -0.1, planner_curves=False, roll=0.0)

    self.assertAlmostEqual(positive * v_ego ** 2, MAX_LATERAL_ACCEL_NO_ROLL)
    self.assertAlmostEqual(negative * v_ego ** 2, -MAX_LATERAL_ACCEL_NO_ROLL)
    self.assertTrue(positive_limited)
    self.assertTrue(negative_limited)

  def test_roll_shifts_both_lateral_acceleration_bounds(self):
    v_ego = 20.0
    roll = 0.05
    roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY

    positive, positive_limited = clip_curvature(v_ego, 0.1, 0.1, planner_curves=False, roll=roll)
    negative, negative_limited = clip_curvature(v_ego, -0.1, -0.1, planner_curves=False, roll=roll)

    self.assertAlmostEqual(positive * v_ego ** 2, MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation)
    self.assertAlmostEqual(negative * v_ego ** 2, -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation)
    self.assertTrue(positive_limited)
    self.assertTrue(negative_limited)

  def test_jerk_rate_limiting_is_not_reported_as_curvature_saturation(self):
    v_ego = 20.0
    curvature, curvature_limited = clip_curvature(v_ego, 0.0, 0.01, planner_curves=False, roll=0.0)

    max_step = MAX_LATERAL_JERK / v_ego ** 2 * DT_CTRL
    self.assertAlmostEqual(curvature, max_step)
    self.assertFalse(curvature_limited)

  def test_low_speed_behavior_is_not_tightened_by_acceleration_limit(self):
    curvature, curvature_limited = clip_curvature(0.0, 0.1, 0.1, planner_curves=False, roll=0.0)
    planner_curvature, planner_curvature_limited = clip_curvature(0.0, 0.3, 0.3, planner_curves=True, roll=0.0)

    self.assertAlmostEqual(curvature, 0.1)
    self.assertAlmostEqual(planner_curvature, MAX_CURVATURE)
    self.assertFalse(curvature_limited)
    self.assertTrue(planner_curvature_limited)


class TestCurvatureSaturationSemantics(unittest.TestCase):
  def setUp(self):
    self.controller = ConcreteLatControl(SimpleNamespace(steerLimitTimer=3 * DT_CTRL), None)
    self.car_state = SimpleNamespace(vEgo=20.0, steeringPressed=False)

  def test_curvature_limit_is_not_masqueraded_as_safety_clipping(self):
    for _ in range(4):
      saturated = self.controller._check_saturation(
        False, self.car_state, steer_limited_by_safety=False, curvature_limited=True,
      )

    self.assertTrue(saturated)
    self.assertAlmostEqual(self.controller.sat_count, self.controller.sat_limit)

  def test_downstream_safety_clipping_suppresses_false_saturation(self):
    for _ in range(4):
      saturated = self.controller._check_saturation(
        True, self.car_state, steer_limited_by_safety=True, curvature_limited=False,
      )

    self.assertFalse(saturated)
    self.assertEqual(self.controller.sat_count, 0.0)

  def test_driver_input_suppresses_curvature_limit_saturation(self):
    self.car_state.steeringPressed = True
    for _ in range(4):
      saturated = self.controller._check_saturation(
        False, self.car_state, steer_limited_by_safety=False, curvature_limited=True,
      )

    self.assertFalse(saturated)
    self.assertEqual(self.controller.sat_count, 0.0)


class TestCurvatureCapAlerts(unittest.TestCase):
  def test_sustained_cap_uses_raw_preclip_request_for_shortfall(self):
    # At 20 m/s, the raw request is 4 m/s^2 while the capped controller can
    # track 3 m/s^2 exactly. Comparing only the clipped setpoint would miss it.
    self.assertTrue(curvature_cap_alert_required(
      curvature_limited=True,
      controller_saturated=True,
      requested_curvature=0.01,
      actual_lateral_accel=3.0,
      v_ego=20.0,
    ))

  def test_cap_and_controller_timer_are_both_required(self):
    common = {
      "requested_curvature": 0.01,
      "actual_lateral_accel": 3.0,
      "v_ego": 20.0,
    }

    self.assertFalse(curvature_cap_alert_required(False, True, **common))
    self.assertFalse(curvature_cap_alert_required(True, False, **common))

  def test_small_or_well_tracked_request_does_not_alert(self):
    self.assertFalse(curvature_cap_alert_required(True, True, 0.002, 0.5, 20.0))
    self.assertFalse(curvature_cap_alert_required(True, True, 0.01, 3.5, 20.0))

  def test_negative_turn_uses_same_absolute_shortfall_thresholds(self):
    self.assertTrue(curvature_cap_alert_required(True, True, -0.01, -3.0, 20.0))


if __name__ == "__main__":
  unittest.main()
