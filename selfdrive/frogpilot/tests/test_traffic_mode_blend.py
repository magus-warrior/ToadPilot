import unittest

from types import SimpleNamespace
from unittest.mock import patch

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_following import (TRAFFIC_MODE_BLEND_TIME,
                                                                            FrogPilotFollowing)


class TestTrafficModeBlend(unittest.TestCase):
  def setUp(self):
    parent = SimpleNamespace(
      lead_one=SimpleNamespace(vLead=0.0),
      tracking_lead=False,
    )
    self.following = FrogPilotFollowing(parent)
    self.controls_state = SimpleNamespace(enabled=True, personality=1)
    self.car_state = SimpleNamespace(trafficMode=False)
    self.toggles = SimpleNamespace(
      aggressive_follow=1.0,
      standard_follow=1.0,
      relaxed_follow=1.0,
      custom_personalities=False,
      aggressive_jerk_acceleration=1.0,
      aggressive_jerk_danger=1.0,
      aggressive_jerk_speed=1.0,
      standard_jerk_acceleration=1.0,
      standard_jerk_danger=1.0,
      standard_jerk_speed=1.0,
      relaxed_jerk_acceleration=1.0,
      relaxed_jerk_danger=1.0,
      relaxed_jerk_speed=1.0,
      traffic_mode_follow=[8.0, 8.0],
      traffic_mode_jerk_acceleration=[5.0, 5.0],
      traffic_mode_jerk_danger=[6.0, 6.0],
      traffic_mode_jerk_speed=[7.0, 7.0],
    )

  def update(self):
    self.following.update(0.1, self.controls_state, self.car_state, 100.0, 5.0, self.toggles)

  @patch("openpilot.selfdrive.frogpilot.controls.lib.frogpilot_following.get_T_FOLLOW", return_value=4.0)
  @patch("openpilot.selfdrive.frogpilot.controls.lib.frogpilot_following.get_jerk_factor", return_value=(1.0, 2.0, 3.0))
  def test_traffic_profile_blends_in_and_out_over_two_seconds(self, _jerk, _follow):
    self.update()
    self.assertEqual(self.following.traffic_mode_blend, 0.0)
    self.assertEqual(self.following.base_acceleration_jerk, 1.0)

    self.car_state.trafficMode = True
    self.update()
    blend_step = DT_MDL / TRAFFIC_MODE_BLEND_TIME
    self.assertAlmostEqual(self.following.traffic_mode_blend, blend_step)
    self.assertGreater(self.following.base_acceleration_jerk, 1.0)
    self.assertLess(self.following.base_acceleration_jerk, 5.0)

    for _ in range(round(TRAFFIC_MODE_BLEND_TIME / DT_MDL) - 1):
      self.update()
    self.assertAlmostEqual(self.following.traffic_mode_blend, 1.0)
    self.assertAlmostEqual(self.following.base_acceleration_jerk, 5.0)
    self.assertAlmostEqual(self.following.base_danger_jerk, 6.0)
    self.assertAlmostEqual(self.following.base_speed_jerk, 7.0)
    self.assertAlmostEqual(self.following.t_follow, 8.0)

    self.car_state.trafficMode = False
    self.update()
    self.assertAlmostEqual(self.following.traffic_mode_blend, 1.0 - blend_step)
    self.assertGreater(self.following.base_acceleration_jerk, 1.0)
    self.assertLess(self.following.base_acceleration_jerk, 5.0)

  @patch("openpilot.selfdrive.frogpilot.controls.lib.frogpilot_following.get_T_FOLLOW", return_value=4.0)
  @patch("openpilot.selfdrive.frogpilot.controls.lib.frogpilot_following.get_jerk_factor", return_value=(1.0, 2.0, 3.0))
  def test_disengaging_resets_the_blend(self, _jerk, _follow):
    self.car_state.trafficMode = True
    self.update()
    self.controls_state.enabled = False
    self.update()
    self.assertEqual(self.following.traffic_mode_blend, 0.0)
    self.assertEqual(self.following.t_follow, 0.0)


if __name__ == "__main__":
  unittest.main()
