import unittest

from openpilot.selfdrive.car.toyota.carcontroller import update_standstill_request
from openpilot.selfdrive.car.toyota.values import CAR, NO_STOP_TIMER_CAR


class TestToyotaStandstillRequest(unittest.TestCase):
  @staticmethod
  def update(current=False, **overrides):
    state = {
      "no_stop_timer": True,
      "hybrid": False,
      "gas_interceptor": False,
      "sng_hack": False,
      "standstill": True,
      "last_standstill": False,
      "pcm_acc_status": 8,
      "cruise_standstill": True,
      "requested_accel": 0.0,
    }
    state.update(overrides)
    return update_standstill_request(current, **state)

  def test_camry_tss2_uses_no_stop_timer_path(self):
    self.assertIn(CAR.TOYOTA_CAMRY_TSS2, NO_STOP_TIMER_CAR)

  def test_hybrid_releases_immediately_and_stays_released(self):
    self.assertFalse(self.update(hybrid=True))
    self.assertFalse(self.update(True, hybrid=True, last_standstill=True, requested_accel=-0.5))

  def test_non_hybrid_holds_until_planner_requests_positive_accel(self):
    self.assertTrue(self.update())
    self.assertTrue(self.update(True, last_standstill=True, cruise_standstill=False, requested_accel=0.0))
    self.assertFalse(self.update(True, last_standstill=True, cruise_standstill=False, requested_accel=0.01))

  def test_legacy_pcm_status_behavior_is_preserved(self):
    self.assertTrue(self.update(no_stop_timer=False))
    self.assertFalse(self.update(True, no_stop_timer=False, last_standstill=True, pcm_acc_status=7))

  def test_gas_interceptor_keeps_legacy_behavior(self):
    self.assertTrue(self.update(gas_interceptor=True))
    self.assertFalse(self.update(True, gas_interceptor=True, last_standstill=True, pcm_acc_status=7))

  def test_explicit_sng_override_still_releases(self):
    self.assertFalse(self.update(True, sng_hack=True, last_standstill=True))


if __name__ == "__main__":
  unittest.main()
