import unittest

from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longitudinal_planner import get_max_accel
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_acceleration import (FrogPilotAcceleration,
                                                                             get_max_accel_eco)


class TestTrafficModeAccelerationBlend(unittest.TestCase):
  def setUp(self):
    self.parent = SimpleNamespace(
      frogpilot_following=SimpleNamespace(following_lead=False, traffic_mode_blend=0.0),
      lead_one=SimpleNamespace(aLeadK=0.0),
      v_cruise=20.0,
    )
    self.acceleration = FrogPilotAcceleration(self.parent)
    self.car_state = SimpleNamespace(ecoGear=False, sportGear=False, forceCoast=False, trafficMode=False)
    self.toggles = SimpleNamespace(
      acceleration_profile=1,
      deceleration_profile=0,
      human_acceleration=False,
      map_acceleration=False,
      map_deceleration=False,
      sport_plus=False,
    )

  def test_max_acceleration_uses_the_same_gradual_profile_blend(self):
    v_ego = 10.0
    normal = get_max_accel_eco(v_ego)
    traffic = get_max_accel(v_ego)

    self.acceleration.update(self.car_state, v_ego, self.toggles)
    self.assertAlmostEqual(self.acceleration.max_accel, normal)

    self.parent.frogpilot_following.traffic_mode_blend = 0.5
    self.acceleration.update(self.car_state, v_ego, self.toggles)
    self.assertAlmostEqual(self.acceleration.max_accel, (normal + traffic) / 2.0)

    self.parent.frogpilot_following.traffic_mode_blend = 1.0
    self.acceleration.update(self.car_state, v_ego, self.toggles)
    self.assertAlmostEqual(self.acceleration.max_accel, traffic)


if __name__ == "__main__":
  unittest.main()
