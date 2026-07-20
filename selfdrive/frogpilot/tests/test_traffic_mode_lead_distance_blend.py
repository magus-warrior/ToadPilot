import unittest

from types import SimpleNamespace

from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA, get_lead


class TestTrafficModeLeadDistanceBlend(unittest.TestCase):
  def test_radar_lead_offset_blends_without_a_distance_jump(self):
    lead = SimpleNamespace(
      x=[30.0],
      y=[0.0],
      v=[5.0],
      a=[0.0],
      prob=0.99,
    )
    toggles = SimpleNamespace(
      allow_far_lead_tracking=False,
      increased_stopped_distance=4.0,
      lead_detection_probability=0.5,
    )

    def distance(blend):
      result = get_lead(
        5.0, True, {}, lead, 5.0, SimpleNamespace(), False, toggles, blend,
        low_speed_override=False,
      )
      return result["dRel"]

    unadjusted = 30.0 - RADAR_TO_CAMERA
    self.assertAlmostEqual(distance(0.0), unadjusted - 4.0)
    self.assertAlmostEqual(distance(0.5), unadjusted - 2.0)
    self.assertAlmostEqual(distance(1.0), unadjusted)


if __name__ == "__main__":
  unittest.main()
