import unittest

from types import SimpleNamespace

from cereal import custom
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_vcruise import FrogPilotVCruise
from openpilot.selfdrive.frogpilot.controls.lib.mapped_stop_controller import MapStopState


class TestMappedStopVCruiseIntegration(unittest.TestCase):
  def setUp(self):
    parent = SimpleNamespace(
      cem=SimpleNamespace(stop_light_detected=False),
      frogpilot_following=SimpleNamespace(following_lead=False),
      lead_one=SimpleNamespace(vLead=0.0, dRel=100.0),
      model_length=100.0,
      model_stopped=False,
      road_curvature=0.0,
      road_curvature_detected=False,
      tracking_lead=False,
      v_cruise=20.0,
    )
    self.controller = FrogPilotVCruise(parent)
    self.car_state = SimpleNamespace(gasPressed=False, standstill=False, vEgoCluster=0.0)
    self.controls_state = SimpleNamespace(enabled=True, vCruiseCluster=72.0)
    self.frogpilot_car_state = SimpleNamespace(accelPressed=False, decelPressed=False, dashboardSpeedLimit=0.0)
    self.toggles = SimpleNamespace(
      force_standstill=False,
      force_stops=False,
      human_following=False,
      map_turn_speed_controller=False,
      show_speed_limits=False,
      speed_limit_controller=False,
      toadpilot_map_stop_assist=True,
      vision_turn_speed_controller=False,
    )

  @staticmethod
  def navigation(distance=0.0, condition_type="stopSign", condition_id=123, condition_valid=True):
    navigation = custom.FrogPilotNavigation.new_message()
    navigation.intersectionConditionValid = condition_valid
    if condition_valid:
      navigation.intersectionConditionType = condition_type
      navigation.intersectionConditionDistance = distance
      navigation.intersectionConditionId = condition_id
    return navigation

  def update(self, distance, speed, now, *, age=0.0, standstill=False, gas_pressed=False,
             condition_type="stopSign", condition_valid=True):
    self.car_state.standstill = standstill
    self.car_state.vEgoCluster = speed
    self.car_state.gasPressed = gas_pressed
    return self.controller.update(
      self.car_state, self.controls_state, self.frogpilot_car_state,
      self.navigation(distance, condition_type, condition_valid=condition_valid), None, 20.0, speed, self.toggles,
      navigation_age_s=age, navigation_valid=True, now=now,
    )

  def test_approach_caps_speed_then_holds_exactly_after_standstill(self):
    self.assertEqual(self.update(50.0, 10.0, 0.0), 20.0)
    target = self.update(40.0, 10.0, 1.0)
    self.assertLess(target, 20.0)
    self.assertFalse(self.controller.forcing_stop)
    self.assertEqual(self.controller.map_stop_constraint.state, MapStopState.ARMED)

    for now, distance, speed in (
      (2.0, 31.0, 8.0),
      (3.0, 24.0, 7.0),
      (4.0, 18.0, 6.0),
      (5.0, 13.0, 5.0),
      (6.0, 9.0, 4.0),
      (7.0, 6.0, 3.0),
      (8.0, 3.5, 2.0),
    ):
      self.update(distance, speed, now)

    self.assertEqual(self.update(1.5, 0.0, 9.0, standstill=True), 0.0)
    self.assertTrue(self.controller.forcing_stop)
    self.assertEqual(self.controller.map_stop_constraint.state, MapStopState.HOLDING)

    self.assertEqual(self.update(1.5, 0.0, 9.5, age=0.5, standstill=True), 0.0)
    self.assertTrue(self.controller.forcing_stop)

    released_target = self.update(1.5, 0.0, 10.0, standstill=True)
    self.assertEqual(self.controller.map_stop_constraint.state, MapStopState.RELEASED)
    self.assertAlmostEqual(self.controller.map_stop_constraint.hold_elapsed_s, 1.0)
    self.assertFalse(self.controller.forcing_stop)
    self.assertEqual(released_target, 20.0)

  def test_static_traffic_signal_never_creates_map_braking_constraint(self):
    target = self.update(20.0, 8.0, 0.0, condition_type="trafficSignal")
    self.assertEqual(target, 20.0)
    self.assertFalse(self.controller.forcing_stop)
    self.assertIsNone(self.controller.map_stop_constraint)

  def test_release_latch_clears_after_two_distinct_nav_absences(self):
    self.assertEqual(self.update(1.0, 0.0, 0.0, standstill=True), 20.0)
    self.assertEqual(self.update(1.0, 0.0, 1.0, standstill=True), 0.0)
    self.assertEqual(self.update(1.0, 0.0, 2.0, standstill=True), 20.0)
    self.assertEqual(self.controller.map_stop_constraint.state, MapStopState.RELEASED)

    self.update(0.0, 0.0, 3.0, standstill=True, condition_valid=False)
    self.assertEqual(self.controller.map_stop_constraint.state, MapStopState.RELEASED)
    self.assertEqual(self.controller.map_stop_controller.absence_count, 1)

    self.update(0.0, 0.0, 3.1, age=0.1, standstill=True, condition_valid=False)
    self.assertEqual(self.controller.map_stop_constraint.state, MapStopState.RELEASED)
    self.assertEqual(self.controller.map_stop_controller.absence_count, 1)

    self.update(0.0, 0.0, 4.0, standstill=True, condition_valid=False)
    self.assertEqual(self.controller.map_stop_constraint.state, MapStopState.IDLE)
    self.assertIsNone(self.controller.map_stop_controller.released_condition_id)

    self.assertEqual(self.update(50.0, 10.0, 5.0), 20.0)
    target = self.update(40.0, 10.0, 6.0)
    self.assertLess(target, 20.0)
    self.assertEqual(self.controller.map_stop_constraint.state, MapStopState.ARMED)

  def test_new_abrupt_map_stop_fails_off_without_a_speed_cap(self):
    self.assertEqual(self.update(3.0, 20.0, 0.0), 20.0)
    self.assertEqual(self.controller.map_stop_constraint.reason, "awaiting_consistent_observation")

    self.assertEqual(self.update(2.0, 20.0, 1.0), 20.0)
    self.assertEqual(self.controller.map_stop_constraint.state, MapStopState.CANCELLED)
    self.assertEqual(self.controller.map_stop_constraint.reason, "unsafe_initial_required_decel")
    self.assertFalse(self.controller.forcing_stop)


if __name__ == "__main__":
  unittest.main()
