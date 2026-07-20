import json
import math
import unittest

from openpilot.selfdrive.frogpilot.controls.lib.mapped_stop_controller import (MapStopObservation,
                                                                               MapStopState,
                                                                               MapStopType,
                                                                               MappedStopController)


class TestMappedStopController(unittest.TestCase):
  @staticmethod
  def observation(distance, *, condition_id="route-a:step-2:stop-1", condition_type=MapStopType.STOP_SIGN,
                  confidence=1.0, age_s=0.0):
    return MapStopObservation(condition_id, condition_type, distance, confidence, age_s)

  @staticmethod
  def update(controller, observation, now, **overrides):
    values = {
      "v_ego": 10.0,
      "standstill": False,
      "gas_pressed": False,
      "now": now,
    }
    values.update(overrides)
    return controller.update(observation, **values)

  def test_approach_arms_in_comfortable_braking_window(self):
    controller = MappedStopController()

    constraint = self.update(controller, self.observation(100.0), 0.0)
    self.assertEqual(constraint.state, MapStopState.APPROACH)
    self.assertFalse(constraint.active)

    constraint = self.update(controller, self.observation(42.0), 5.8)
    self.assertEqual(constraint.state, MapStopState.ARMED)
    self.assertTrue(constraint.active)
    self.assertAlmostEqual(constraint.target_distance_m, 40.5)
    self.assertAlmostEqual(constraint.target_speed_mps, math.sqrt(2.0 * 1.6 * 40.5))
    self.assertAlmostEqual(constraint.required_decel_mps2, 100.0 / 81.0)
    self.assertFalse(constraint.late_stop)

  def test_stop_sign_holds_for_one_second_after_actual_standstill(self):
    controller = MappedStopController()
    condition = self.observation(2.0)

    constraint = self.update(controller, condition, 0.0, v_ego=0.05, standstill=False)
    self.assertEqual(constraint.state, MapStopState.APPROACH)
    self.assertEqual(constraint.reason, "awaiting_consistent_observation")
    self.assertFalse(constraint.active)

    constraint = self.update(controller, condition, 4.0, v_ego=0.05, standstill=False)
    self.assertEqual(constraint.state, MapStopState.ARMED)
    self.assertEqual(constraint.hold_elapsed_s, 0.0)

    constraint = self.update(controller, condition, 5.0, v_ego=0.0, standstill=True)
    self.assertEqual(constraint.state, MapStopState.HOLDING)
    self.assertTrue(constraint.active)
    self.assertEqual(constraint.hold_elapsed_s, 0.0)
    self.assertEqual(constraint.target_distance_m, 0.0)
    self.assertEqual(constraint.target_speed_mps, 0.0)

    constraint = self.update(controller, condition, 5.99, v_ego=0.0, standstill=True)
    self.assertEqual(constraint.state, MapStopState.HOLDING)
    self.assertTrue(constraint.active)

    constraint = self.update(controller, condition, 6.0, v_ego=0.0, standstill=True)
    self.assertEqual(constraint.state, MapStopState.RELEASED)
    self.assertEqual(constraint.reason, "stop_sign_hold_complete")
    self.assertFalse(constraint.active)
    self.assertEqual(constraint.hold_elapsed_s, 1.0)
    self.assertEqual(controller.diagnostic_snapshot()["hold_elapsed_s"], 1.0)

  def test_hold_must_be_continuous(self):
    controller = MappedStopController()
    condition = self.observation(2.0)

    self.update(controller, condition, 0.0, v_ego=0.0, standstill=True)
    self.update(controller, condition, 0.8, v_ego=0.0, standstill=True)
    constraint = self.update(controller, condition, 0.9, v_ego=0.2, standstill=False)
    self.assertEqual(constraint.state, MapStopState.ARMED)
    self.assertEqual(constraint.reason, "hold_interrupted")

    constraint = self.update(controller, condition, 1.0, v_ego=0.0, standstill=True)
    self.assertEqual(constraint.state, MapStopState.HOLDING)
    constraint = self.update(controller, condition, 1.9, v_ego=0.0, standstill=True)
    self.assertEqual(constraint.state, MapStopState.HOLDING)
    constraint = self.update(controller, condition, 2.0, v_ego=0.0, standstill=True)
    self.assertEqual(constraint.state, MapStopState.RELEASED)

  def test_released_stop_is_ignored_until_clearly_passed(self):
    controller = MappedStopController()
    condition_id = "stable-stop-id"

    self.update(controller, self.observation(1.0, condition_id=condition_id), 0.0, v_ego=0.0, standstill=True)
    self.update(controller, self.observation(1.0, condition_id=condition_id), 1.0, v_ego=0.0, standstill=True)
    self.update(controller, self.observation(1.0, condition_id=condition_id), 2.0, v_ego=0.0, standstill=True)

    constraint = self.update(controller, self.observation(3.0, condition_id=condition_id), 3.0,
                             v_ego=1.0, standstill=False)
    self.assertEqual(constraint.state, MapStopState.RELEASED)
    self.assertFalse(constraint.active)

    constraint = self.update(controller, self.observation(-1.0, condition_id=condition_id), 4.0,
                             v_ego=4.0, standstill=False)
    self.assertEqual(constraint.state, MapStopState.PASSED)
    self.assertFalse(constraint.active)

  def test_traffic_light_never_releases_from_elapsed_time(self):
    controller = MappedStopController()
    light = self.observation(1.0, condition_type=MapStopType.TRAFFIC_LIGHT)

    self.update(controller, light, 0.0, v_ego=0.0, standstill=True)
    constraint = self.update(controller, light, 30.0, v_ego=0.0, standstill=True)
    self.assertEqual(constraint.state, MapStopState.HOLDING)
    self.assertEqual(constraint.reason, "traffic_light_requires_clear")
    self.assertTrue(constraint.active)

    constraint = self.update(controller, light, 30.1, v_ego=0.0, standstill=True,
                             traffic_signal_clear=True)
    self.assertEqual(constraint.state, MapStopState.RELEASED)
    self.assertEqual(constraint.reason, "traffic_signal_clear")
    self.assertFalse(constraint.active)

  def test_gas_override_cancels_only_map_constraint(self):
    controller = MappedStopController()
    constraint = self.update(controller, self.observation(100.0), 0.0, gas_pressed=True)
    self.assertEqual(constraint.state, MapStopState.APPROACH)
    self.assertFalse(constraint.active)

    constraint = self.update(controller, self.observation(90.0), 1.0, gas_pressed=True)
    self.assertEqual(constraint.state, MapStopState.APPROACH)
    self.assertEqual(constraint.reason, "condition_ahead")
    self.assertFalse(constraint.active)
    self.assertIsNone(controller.released_condition_id)

    constraint = self.update(controller, self.observation(42.0), 5.8)
    self.assertEqual(constraint.state, MapStopState.ARMED)
    self.assertTrue(constraint.active)

    constraint = self.update(controller, self.observation(42.0, age_s=0.05), 5.85, gas_pressed=True)
    self.assertEqual(constraint.state, MapStopState.RELEASED)
    self.assertEqual(constraint.reason, "driver_gas_override")
    self.assertFalse(constraint.active)

  def test_rejects_stale_low_confidence_malformed_and_jumping_data(self):
    invalid_observations = (
      self.observation(20.0, age_s=2.51),
      self.observation(20.0, confidence=0.74),
      self.observation(math.nan),
      self.observation(20.0, condition_id=""),
    )
    for observation in invalid_observations:
      with self.subTest(observation=observation):
        controller = MappedStopController()
        constraint = self.update(controller, observation, 0.0)
        self.assertEqual(constraint.state, MapStopState.CANCELLED)
        self.assertFalse(constraint.active)
        self.assertEqual(controller.rejection_count, 1)

    controller = MappedStopController()
    self.update(controller, self.observation(50.0), 0.0)
    constraint = self.update(controller, self.observation(5.0), 1.0)
    self.assertEqual(constraint.state, MapStopState.CANCELLED)
    self.assertEqual(constraint.reason, "jumping_along_route_distance")
    self.assertFalse(constraint.active)
    self.assertEqual(controller.diagnostic_snapshot()["last_rejection_reason"],
                     "jumping_along_route_distance")

  def test_cached_20hz_samples_do_not_corrupt_1hz_jump_baseline(self):
    controller = MappedStopController()
    first_source_time = 10.0
    first_distance = 160.0

    constraint = self.update(controller, self.observation(first_distance), first_source_time, v_ego=20.0)
    self.assertFalse(constraint.active)
    self.assertEqual(constraint.reason, "awaiting_consistent_observation")

    for frame in range(1, 20):
      now = first_source_time + frame * 0.05
      cached_age = frame * 0.05
      constraint = self.update(controller, self.observation(first_distance, age_s=cached_age), now,
                               v_ego=20.0)
      self.assertFalse(constraint.active)
      self.assertEqual(controller.confirmation_count, 1)
      self.assertNotEqual(constraint.state, MapStopState.CANCELLED)

    constraint = self.update(controller, self.observation(140.0), 11.0, v_ego=20.0)
    self.assertTrue(constraint.active)
    self.assertEqual(constraint.state, MapStopState.ARMED)
    self.assertEqual(controller.rejection_count, 0)
    self.assertAlmostEqual(controller.diagnostic_snapshot()["observation_source_time_s"], 11.0)

  def test_cached_navigation_distance_progresses_smoothly_between_source_updates(self):
    controller = MappedStopController()
    self.update(controller, self.observation(50.0), 0.0, v_ego=10.0)
    constraint = self.update(controller, self.observation(40.0), 1.0, v_ego=10.0)
    self.assertEqual(constraint.state, MapStopState.ARMED)
    initial_distance = constraint.along_route_distance_m
    initial_target = constraint.target_speed_mps

    constraint = self.update(controller, self.observation(40.0, age_s=0.5), 1.5, v_ego=8.0)

    self.assertAlmostEqual(constraint.along_route_distance_m, initial_distance - 4.0)
    self.assertLess(constraint.target_speed_mps, initial_target)
    self.assertEqual(controller.confirmation_count, 2)
    self.assertEqual(controller.rejection_count, 0)

  def test_source_timestamp_jitter_does_not_fake_second_confirmation(self):
    controller = MappedStopController()
    condition = self.observation(50.0)

    constraint = self.update(controller, condition, 10.0, observation_source_time_s=10.0)
    self.assertFalse(constraint.active)
    self.assertEqual(controller.confirmation_count, 1)

    constraint = self.update(controller, self.observation(50.0, age_s=0.05), 10.06,
                             observation_source_time_s=10.01)
    self.assertFalse(constraint.active)
    self.assertEqual(controller.confirmation_count, 1)

    constraint = self.update(controller, self.observation(40.0), 11.0,
                             observation_source_time_s=11.0)
    self.assertTrue(constraint.active)
    self.assertEqual(controller.confirmation_count, 2)

  def test_condition_switch_cannot_replace_an_active_stop_in_one_frame(self):
    controller = MappedStopController()
    self.update(controller, self.observation(50.0, condition_id="first"), 0.0)
    self.update(controller, self.observation(40.0, condition_id="first"), 1.0)

    constraint = self.update(controller, self.observation(39.0, condition_id="wrong-next-stop"), 1.1)
    self.assertEqual(constraint.state, MapStopState.CANCELLED)
    self.assertEqual(constraint.reason, "condition_changed_while_active")
    self.assertFalse(constraint.active)

    constraint = self.update(controller,
                             self.observation(39.0, condition_id="wrong-next-stop", age_s=0.05), 1.15)
    self.assertEqual(constraint.state, MapStopState.CANCELLED)
    self.assertEqual(constraint.reason, "quarantined_active_discontinuity")
    self.assertFalse(constraint.active)
    snapshot = controller.diagnostic_snapshot()
    self.assertEqual(snapshot["quarantined_condition_id"], "wrong-next-stop")
    self.assertEqual(snapshot["quarantine_reason"], "condition_changed_while_active")

    constraint = self.update(controller, self.observation(50.0, condition_id="actual-later-stop"), 2.0)
    self.assertEqual(constraint.state, MapStopState.APPROACH)
    self.assertFalse(constraint.active)
    constraint = self.update(controller, self.observation(40.0, condition_id="actual-later-stop"), 3.0)
    self.assertEqual(constraint.state, MapStopState.ARMED)
    self.assertTrue(constraint.active)
    self.assertIsNone(controller.diagnostic_snapshot()["quarantined_condition_id"])
    self.assertEqual(controller.diagnostic_snapshot()["last_quarantined_condition_id"], "wrong-next-stop")

  def test_active_distance_jump_is_quarantined_across_cached_frames(self):
    controller = MappedStopController()
    condition_id = "jumping-stop"
    self.update(controller, self.observation(50.0, condition_id=condition_id), 0.0, v_ego=10.0)
    self.update(controller, self.observation(40.0, condition_id=condition_id), 1.0, v_ego=10.0)

    constraint = self.update(controller, self.observation(2.0, condition_id=condition_id), 2.0,
                             v_ego=10.0)
    self.assertEqual(constraint.state, MapStopState.CANCELLED)
    self.assertEqual(constraint.reason, "jumping_along_route_distance")
    self.assertFalse(constraint.active)

    constraint = self.update(controller, self.observation(2.0, condition_id=condition_id, age_s=0.05),
                             2.05, v_ego=10.0)
    self.assertEqual(constraint.state, MapStopState.CANCELLED)
    self.assertEqual(constraint.reason, "quarantined_active_discontinuity")
    self.assertFalse(constraint.active)
    self.assertEqual(controller.diagnostic_snapshot()["quarantined_condition_id"], condition_id)

    constraint = self.update(controller, self.observation(-1.0, condition_id=condition_id), 3.0,
                             v_ego=7.0)
    self.assertEqual(constraint.state, MapStopState.PASSED)
    self.assertFalse(constraint.active)
    self.assertIsNone(controller.diagnostic_snapshot()["quarantined_condition_id"])
    self.assertEqual(controller.diagnostic_snapshot()["last_quarantined_condition_id"], condition_id)

  def test_idle_stale_observation_can_recover_without_quarantine(self):
    controller = MappedStopController()
    stale = self.observation(20.0, condition_id="recoverable", age_s=2.51)
    constraint = self.update(controller, stale, 3.0)
    self.assertEqual(constraint.state, MapStopState.CANCELLED)
    self.assertIsNone(controller.diagnostic_snapshot()["quarantined_condition_id"])

    constraint = self.update(controller, self.observation(50.0, condition_id="recoverable"), 3.1)
    self.assertEqual(constraint.state, MapStopState.APPROACH)
    self.assertFalse(constraint.active)
    constraint = self.update(controller, self.observation(40.0, condition_id="recoverable"), 4.1)
    self.assertEqual(constraint.state, MapStopState.ARMED)
    self.assertTrue(constraint.active)

  def test_new_close_condition_is_confirmed_then_quarantined_as_unsafe(self):
    controller = MappedStopController()
    condition_id = "late-map-stop"

    constraint = self.update(controller, self.observation(3.0, condition_id=condition_id), 0.0,
                             v_ego=20.0)
    self.assertEqual(constraint.state, MapStopState.APPROACH)
    self.assertFalse(constraint.active)

    constraint = self.update(controller, self.observation(2.0, condition_id=condition_id), 1.0,
                             v_ego=20.0)
    self.assertEqual(constraint.state, MapStopState.CANCELLED)
    self.assertEqual(constraint.reason, "unsafe_initial_required_decel")
    self.assertFalse(constraint.active)
    snapshot = controller.diagnostic_snapshot()
    self.assertEqual(snapshot["quarantined_condition_id"], condition_id)
    self.assertEqual(snapshot["last_rejection_reason"], "unsafe_initial_required_decel")

  def test_released_id_clears_only_after_two_distinct_fresh_absences(self):
    controller = MappedStopController()
    condition_id = "loop-stop"
    condition = self.observation(1.0, condition_id=condition_id)

    self.update(controller, condition, 0.0, v_ego=0.0, standstill=True)
    self.update(controller, condition, 1.0, v_ego=0.0, standstill=True)
    constraint = self.update(controller, condition, 2.0, v_ego=0.0, standstill=True)
    self.assertEqual(constraint.state, MapStopState.RELEASED)

    constraint = self.update(controller, None, 3.0, v_ego=0.0, standstill=True,
                             explicit_absence=True, observation_source_time_s=3.0)
    self.assertEqual(constraint.state, MapStopState.RELEASED)
    self.assertEqual(controller.released_condition_id, condition_id)
    self.assertEqual(controller.absence_count, 1)

    constraint = self.update(controller, None, 3.1, v_ego=0.0, standstill=True,
                             explicit_absence=True, observation_source_time_s=3.0)
    self.assertEqual(constraint.state, MapStopState.RELEASED)
    self.assertEqual(controller.absence_count, 1)

    constraint = self.update(controller, None, 4.0, v_ego=0.0, standstill=True,
                             explicit_absence=True, observation_source_time_s=4.0)
    self.assertEqual(constraint.state, MapStopState.IDLE)
    self.assertIsNone(controller.released_condition_id)

    constraint = self.update(controller, self.observation(50.0, condition_id=condition_id), 5.0)
    self.assertEqual(constraint.reason, "awaiting_consistent_observation")
    self.assertFalse(constraint.active)
    constraint = self.update(controller, self.observation(40.0, condition_id=condition_id), 6.0)
    self.assertEqual(constraint.state, MapStopState.ARMED)
    self.assertTrue(constraint.active)

  def test_diagnostic_snapshot_is_json_safe_and_exposes_reasons(self):
    controller = MappedStopController()
    self.update(controller, self.observation(50.0), 0.0)
    self.update(controller, self.observation(40.0), 1.0)

    snapshot = controller.diagnostic_snapshot()
    self.assertEqual(snapshot["schema_version"], 1)
    self.assertEqual(snapshot["state"], "armed")
    self.assertEqual(snapshot["reason"], "comfortable_braking_window")
    self.assertEqual(snapshot["condition_type"], "stop_sign")
    self.assertEqual(snapshot["condition_id"], "route-a:step-2:stop-1")
    self.assertEqual(snapshot["confirmation_count"], 2)
    self.assertIn("target_distance_m", snapshot)
    json.dumps(snapshot, allow_nan=False)

  def test_constructor_and_vehicle_inputs_fail_safely(self):
    for value in (0.0, -1.0, math.nan, True):
      with self.subTest(value=value):
        with self.assertRaises(ValueError):
          MappedStopController(comfortable_decel_mps2=value)

    controller = MappedStopController()
    constraint = self.update(controller, self.observation(20.0), 0.0, v_ego=math.nan)
    self.assertEqual(constraint.state, MapStopState.CANCELLED)
    self.assertFalse(constraint.active)


if __name__ == "__main__":
  unittest.main()
