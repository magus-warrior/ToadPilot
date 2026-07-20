import unittest

from types import SimpleNamespace

from openpilot.selfdrive.frogpilot.controls.lib.conditional_experimental_mode import ConditionalExperimentalMode


class TestTrafficModeStopDetection(unittest.TestCase):
  @staticmethod
  def detector(traffic_mode):
    planner = SimpleNamespace(model_length=2.0, model_stopped=True, tracking_lead=False)
    detector = ConditionalExperimentalMode(planner)
    state = SimpleNamespace(trafficMode=traffic_mode)
    toggles = SimpleNamespace(conditional_model_stop_time=9.0)
    for _ in range(40):
      detector.stop_sign_and_light(state, 5.0, toggles)
    return detector

  def test_traffic_profile_does_not_disable_independent_stop_detection(self):
    self.assertTrue(self.detector(False).stop_light_detected)
    self.assertTrue(self.detector(True).stop_light_detected)

  def test_curve_gate_still_suppresses_false_stop_detection(self):
    detector = self.detector(True)
    detector.curve_detected = True
    detector.stop_sign_and_light(SimpleNamespace(trafficMode=True), 5.0,
                                 SimpleNamespace(conditional_model_stop_time=9.0))
    self.assertFalse(detector.stop_light_detected)


if __name__ == "__main__":
  unittest.main()
