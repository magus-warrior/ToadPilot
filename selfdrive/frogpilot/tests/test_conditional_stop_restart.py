import unittest
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.selfdrive.frogpilot.controls.lib.conditional_experimental_mode import ConditionalExperimentalMode


class TestConditionalStopRestart(unittest.TestCase):
  def setUp(self):
    self.planner = SimpleNamespace(
      model_length=0.0, model_stopped=True, tracking_lead=True,
      road_curvature=1e-6, road_curvature_detected=False,
      lane_width_left=3.0, lane_width_right=3.0,
      lead_one=SimpleNamespace(vLead=0.0),
      frogpilot_following=SimpleNamespace(following_lead=True, slower_lead=False),
      frogpilot_vcruise=SimpleNamespace(forcing_stop=False, slc=SimpleNamespace(experimental_mode=False)),
    )
    self.toggles = SimpleNamespace(
      experimental_mode_via_press=True, conditional_limit=0, conditional_limit_lead=0,
      lane_detection_width=2.5, conditional_signal_lane_detection=False, conditional_signal=0,
      conditional_navigation=False, conditional_navigation_lead=False,
      conditional_curves=False, conditional_curves_lead=False,
      conditional_lead=True, conditional_slower_lead=False, conditional_stopped_lead=True,
      conditional_model_stop_time=9,
    )
    self.car = SimpleNamespace(standstill=False, leftBlinker=False, rightBlinker=False)
    self.frogcar = SimpleNamespace(trafficMode=True)
    self.nav = SimpleNamespace(approachingIntersection=False, approachingTurn=False)
    self.detector = ConditionalExperimentalMode(self.planner)
    self.params = self.enterContext(patch(
      'openpilot.selfdrive.frogpilot.controls.lib.conditional_experimental_mode.params_memory'))
    self.params.get_int.return_value = 0

  def update(self, frames=1, speed=0.0):
    for _ in range(frames):
      self.detector.update(self.car, self.frogcar, self.nav, speed, self.toggles)

  def approach_stopped_lead(self):
    self.update(60, 2.0)
    self.assertTrue(self.detector.experimental_mode)
    self.assertTrue(self.detector.slow_lead_detected)
    self.car.standstill = True

  def test_departed_lead_does_not_reenter_experimental_mode_on_launch(self):
    self.approach_stopped_lead()
    self.planner.lead_one.vLead = 3.0
    self.planner.model_stopped = False
    self.planner.model_length = 50.0
    self.update(30)
    self.assertFalse(self.detector.experimental_mode)
    self.car.standstill = False
    self.update(speed=0.4)
    self.assertFalse(self.detector.experimental_mode)
    self.assertFalse(self.detector.slow_lead_detected)

  def test_moving_lead_does_not_cancel_model_stop(self):
    self.approach_stopped_lead()
    self.planner.lead_one.vLead = 3.0
    self.update(60)
    self.assertTrue(self.detector.experimental_mode)
    self.assertFalse(self.detector.slow_lead_detected)

  def test_forced_stop_remains_held(self):
    self.approach_stopped_lead()
    self.planner.frogpilot_vcruise.forcing_stop = True
    self.planner.lead_one.vLead = 3.0
    self.update(60)
    self.assertTrue(self.detector.experimental_mode)

  def test_lost_lead_does_not_cancel_stop_and_model_detection_stays_current(self):
    self.approach_stopped_lead()
    self.planner.tracking_lead = False
    self.update(60)
    self.assertTrue(self.detector.experimental_mode)
    self.assertTrue(self.detector.stop_light_detected)
    self.assertFalse(self.detector.slow_lead_detected)

  def test_new_model_stop_does_not_auto_engage_at_standstill(self):
    self.car.standstill = True
    self.planner.tracking_lead = False
    self.update(60)
    self.assertFalse(self.detector.experimental_mode)

  def test_stopped_lead_condition_still_engages_during_approach(self):
    self.approach_stopped_lead()

  def test_explicit_off_takes_precedence_over_standstill_hold(self):
    self.approach_stopped_lead()
    self.params.get_int.return_value = 1
    self.update()
    self.assertFalse(self.detector.experimental_mode)

  def test_explicit_on_survives_model_release(self):
    self.car.standstill = True
    self.planner.model_stopped = False
    self.params.get_int.return_value = 2
    self.update()
    self.assertTrue(self.detector.experimental_mode)


if __name__ == '__main__':
  unittest.main()
