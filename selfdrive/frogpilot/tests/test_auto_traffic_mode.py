import math
import unittest

from cereal import custom
from openpilot.selfdrive.frogpilot.controls.lib.auto_traffic_mode import (AutoTrafficMode,
                                                                         AutoTrafficModeConfig,
                                                                         blend_traffic_mode_value,
                                                                         valid_auto_traffic_lead)


DT = 0.05


class TestAutoTrafficMode(unittest.TestCase):
  def test_radarless_ghost_lead_cannot_become_traffic_evidence(self):
    self.assertFalse(valid_auto_traffic_lead(True, radarless_model=True, model_probability=0.0,
                                            probability_threshold=0.5))
    self.assertFalse(valid_auto_traffic_lead(True, radarless_model=True, model_probability=0.5,
                                            probability_threshold=0.5))
    self.assertTrue(valid_auto_traffic_lead(True, radarless_model=True, model_probability=0.51,
                                           probability_threshold=0.5))
    self.assertTrue(valid_auto_traffic_lead(True, radarless_model=False, model_probability=0.0,
                                           probability_threshold=0.5))

  def test_profile_value_blend_is_gradual_and_invalid_input_stays_normal(self):
    self.assertEqual(blend_traffic_mode_value(8.0, 0.0, 0.0), 8.0)
    self.assertEqual(blend_traffic_mode_value(8.0, 0.0, 0.5), 4.0)
    self.assertEqual(blend_traffic_mode_value(8.0, 0.0, 1.0), 0.0)
    self.assertEqual(blend_traffic_mode_value(8.0, 0.0, float("nan")), 8.0)

  def test_plan_schema_carries_the_shared_profile_blend(self):
    plan = custom.FrogPilotPlan.new_message()
    plan.trafficModeBlend = 0.375
    self.assertAlmostEqual(plan.trafficModeBlend, 0.375)

  @staticmethod
  def update(controller, duration=DT, **overrides):
    values = {
      "v_ego": 5.0,
      "lead_present": False,
      "lead_distance": math.inf,
      "lead_speed": None,
      "approaching_intersection": False,
    }
    values.update(overrides)
    status = None
    for _ in range(round(duration / DT)):
      status = controller.update(dt=DT, **values)
    return status

  def test_sustained_slow_close_lead_activates(self):
    controller = AutoTrafficMode()
    status = self.update(controller, duration=5.95, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertFalse(status.active)
    self.assertEqual(status.reason, "building_slow_close_lead")

    status = self.update(controller, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertTrue(status.active)
    self.assertTrue(status.changed)
    self.assertEqual(status.reason, "slow_close_lead")

  def test_configured_30_mph_threshold_and_entry_time_are_applied(self):
    controller = AutoTrafficMode()
    controller.set_thresholds(max_speed_mph=30, entry_time=1.0)

    status = self.update(controller, duration=0.95, v_ego=13.0,
                         lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertFalse(status.active)
    status = self.update(controller, v_ego=13.0,
                         lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertTrue(status.active)

    controller = AutoTrafficMode()
    controller.set_thresholds(max_speed_mph=30, entry_time=1.0)
    status = self.update(controller, duration=2.0, v_ego=13.5,
                         lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertFalse(status.active)

  def test_configured_30_mph_threshold_controls_exit(self):
    controller = AutoTrafficMode()
    controller.set_thresholds(max_speed_mph=30, entry_time=1.0)
    self.update(controller, duration=1.0, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertTrue(controller.active)

    status = self.update(controller, duration=2.0, v_ego=13.5)
    self.assertFalse(status.active)
    self.assertTrue(status.changed)
    self.assertEqual(status.reason, "free_flow")

  def test_intersection_only_supports_real_congestion_evidence(self):
    controller = AutoTrafficMode()
    status = self.update(controller, duration=30.0, approaching_intersection=True)
    self.assertFalse(status.active)
    self.assertEqual(status.confidence, 0.0)
    self.assertEqual(status.reason, "monitoring")

    controller = AutoTrafficMode()
    status = self.update(controller, duration=4.45, lead_present=True, lead_distance=18.0,
                         lead_speed=4.0, approaching_intersection=True)
    self.assertFalse(status.active)
    status = self.update(controller, lead_present=True, lead_distance=18.0,
                         lead_speed=4.0, approaching_intersection=True)
    self.assertTrue(status.active)
    self.assertEqual(status.reason, "slow_close_lead_near_intersection")

  def test_brief_lead_dropout_does_not_discard_all_evidence(self):
    controller = AutoTrafficMode()
    self.update(controller, duration=4.0, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    status = self.update(controller, duration=0.5)
    self.assertAlmostEqual(status.congestion_time, 3.75)

    status = self.update(controller, duration=2.25, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertTrue(status.active)

  def test_two_real_stop_launch_cycles_activate(self):
    controller = AutoTrafficMode()
    for cycle in range(2):
      self.update(controller, duration=1.0, v_ego=0.0)
      status = self.update(controller, duration=DT, v_ego=2.5,
                           lead_present=True, lead_distance=10.0, lead_speed=2.5)
      if cycle == 0:
        self.assertFalse(status.active)
        self.assertEqual(status.stop_launch_cycles, 1)

    self.assertTrue(status.active)
    self.assertTrue(status.changed)
    self.assertEqual(status.reason, "repeated_stop_and_go")

  def test_speed_jitter_is_not_a_stop_launch_cycle(self):
    controller = AutoTrafficMode()
    for _ in range(10):
      self.update(controller, duration=0.25, v_ego=0.0)
      status = self.update(controller, duration=0.25, v_ego=0.8)
    self.assertFalse(status.active)
    self.assertEqual(status.stop_launch_cycles, 0)

  def test_old_stop_launch_expires(self):
    config = AutoTrafficModeConfig(stop_launch_window=10.0)
    controller = AutoTrafficMode(config)
    self.update(controller, duration=1.0, v_ego=0.0)
    status = self.update(controller, duration=0.5, v_ego=2.5)
    self.assertEqual(status.stop_launch_cycles, 1)

    status = self.update(controller, duration=11.0, v_ego=8.0)
    self.assertEqual(status.stop_launch_cycles, 0)
    self.update(controller, duration=1.0, v_ego=0.0)
    status = self.update(controller, duration=0.5, v_ego=2.5)
    self.assertFalse(status.active)
    self.assertEqual(status.stop_launch_cycles, 1)

  def test_exit_dwell_resets_when_low_speed_traffic_returns(self):
    controller = AutoTrafficMode()
    self.update(controller, duration=6.0, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertTrue(controller.active)

    status = self.update(controller, duration=1.5, v_ego=19.0)
    self.assertTrue(status.active)
    self.assertGreater(status.free_flow_time, 1.0)

    # Even a brief nearby slow lead restarts the exit timer.
    status = self.update(controller, duration=0.5, v_ego=5.0, lead_present=True,
                         lead_distance=20.0, lead_speed=10.0)
    self.assertTrue(status.active)
    self.assertEqual(status.free_flow_time, 0.0)

    status = self.update(controller, duration=2.0, v_ego=19.0)
    self.assertFalse(status.active)
    self.assertTrue(status.changed)
    self.assertEqual(status.reason, "free_flow")
    self.assertEqual(status.stop_launch_cycles, 0)

  def test_nearby_fast_lead_does_not_block_free_flow_exit(self):
    controller = AutoTrafficMode()
    self.update(controller, duration=6.0, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertTrue(controller.active)

    status = self.update(controller, duration=2.0, v_ego=19.0, lead_present=True,
                         lead_distance=20.0, lead_speed=18.0)
    self.assertFalse(status.active)
    self.assertTrue(status.changed)
    self.assertEqual(status.reason, "free_flow")

  def test_empty_road_releases_below_speed_threshold(self):
    for speed in (0.0, 2.0, 10.0):
      controller = AutoTrafficMode()
      self.update(controller, duration=6.0, lead_present=True, lead_distance=18.0, lead_speed=4.0)
      self.assertTrue(self.update(controller, duration=1.95, v_ego=speed).active)
      status = self.update(controller, v_ego=speed)
      self.assertFalse(status.active)
      self.assertTrue(status.changed)
      self.assertFalse(self.update(controller, duration=10.0, v_ego=speed).active)

  def test_speed_exit_takes_precedence_over_slow_lead(self):
    controller = AutoTrafficMode()
    controller.set_thresholds(max_speed_mph=30, entry_time=1.0)
    self.update(controller, duration=1.0, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    status = self.update(controller, duration=2.0, v_ego=13.5,
                         lead_present=True, lead_distance=20.0, lead_speed=10.0)
    self.assertFalse(status.active)
    self.assertTrue(status.changed)

  def test_exact_speed_cutoff_cannot_reactivate_with_close_lead(self):
    controller = AutoTrafficMode()
    controller.set_thresholds(max_speed_mph=30, entry_time=1.0)
    self.update(controller, duration=1.0, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    for _ in range(3):
      status = self.update(controller, duration=2.0, v_ego=controller.config.free_flow_speed,
                           lead_present=True, lead_distance=20.0, lead_speed=10.0)
      self.assertFalse(status.active)

  def test_short_lead_dropout_does_not_release_active_mode(self):
    controller = AutoTrafficMode()
    self.update(controller, duration=6.0, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertTrue(self.update(controller, duration=1.0).active)
    status = self.update(controller, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    self.assertTrue(status.active)
    self.assertEqual(status.free_flow_time, 0.0)

  def test_empty_road_stop_cycles_do_not_activate(self):
    controller = AutoTrafficMode()
    for _ in range(4):
      self.update(controller, duration=1.0, v_ego=0.0)
      self.assertFalse(self.update(controller, v_ego=2.5).active)

  def test_invalid_speed_cannot_release_active_traffic(self):
    controller = AutoTrafficMode()
    self.update(controller, duration=6.0, lead_present=True, lead_distance=18.0, lead_speed=4.0)
    for speed in (math.nan, math.inf, True):
      self.assertTrue(self.update(controller, duration=3.0, v_ego=speed).active)

  def test_invalid_sensor_values_cannot_activate(self):
    controller = AutoTrafficMode()
    for invalid_speed in (math.nan, math.inf, True):
      status = self.update(controller, duration=10.0, v_ego=invalid_speed,
                           lead_present=True, lead_distance=1.0, lead_speed=0.0,
                           approaching_intersection=True)
      self.assertFalse(status.active)

    for invalid_dt in (-0.1, math.nan, math.inf, True, None):
      with self.subTest(dt=invalid_dt), self.assertRaises(ValueError):
        controller.update(v_ego=0.0, lead_present=False, dt=invalid_dt)

  def test_delayed_update_cannot_fake_sustained_observation(self):
    controller = AutoTrafficMode()
    status = controller.update(v_ego=0.0, lead_present=True, lead_distance=5.0,
                               lead_speed=0.0, approaching_intersection=True, dt=30.0)
    self.assertFalse(status.active)
    self.assertEqual(status.congestion_time, controller.config.max_observation_dt)
    self.assertFalse(controller.stop_armed)


if __name__ == "__main__":
  unittest.main()
