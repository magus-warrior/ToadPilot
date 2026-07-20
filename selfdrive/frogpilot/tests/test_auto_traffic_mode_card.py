import unittest

from types import SimpleNamespace
from unittest.mock import patch

from openpilot.selfdrive.frogpilot.controls.frogpilot_card import (AUTO_TRAFFIC_PLAN_MAX_AGE_S,
                                                                 FrogPilotCard,
                                                                 get_auto_traffic_recommendation)


class FakeSubMaster:
  def __init__(self, auto_active, *, alive=True, valid=True, log_mono_time=1):
    self.data = {"frogpilotPlan": SimpleNamespace(autoTrafficMode=auto_active)}
    self.alive = {"frogpilotPlan": alive}
    self.valid = {"frogpilotPlan": valid}
    self.logMonoTime = {"frogpilotPlan": log_mono_time}

  def __getitem__(self, service):
    return self.data[service]


class TestAutomaticTrafficModeCard(unittest.TestCase):
  @staticmethod
  def card(automatic=True):
    car = SimpleNamespace(
      CP=SimpleNamespace(carName="toyota"),
      frogpilot_toggles=SimpleNamespace(toadpilot_automatic_traffic_mode=automatic),
    )
    return FrogPilotCard(car)

  @staticmethod
  def sm(auto_active):
    return {"frogpilotPlan": SimpleNamespace(autoTrafficMode=auto_active)}

  def test_button_forces_opposite_then_returns_to_auto(self):
    card = self.card()

    active = self.sm(True)
    self.assertTrue(card.get_traffic_mode(active))
    card.toggle_traffic_mode(active)
    self.assertEqual(card.traffic_mode_override, -1)
    self.assertFalse(card.get_traffic_mode(active))
    card.toggle_traffic_mode(active)
    self.assertEqual(card.traffic_mode_override, 0)
    self.assertTrue(card.get_traffic_mode(active))

    inactive = self.sm(False)
    card.toggle_traffic_mode(inactive)
    self.assertEqual(card.traffic_mode_override, 1)
    self.assertTrue(card.get_traffic_mode(inactive))
    card.toggle_traffic_mode(inactive)
    self.assertEqual(card.traffic_mode_override, 0)
    self.assertFalse(card.get_traffic_mode(inactive))

  def test_disabled_automatic_mode_preserves_manual_toggle(self):
    card = self.card(automatic=False)
    sm = self.sm(True)
    self.assertFalse(card.get_traffic_mode(sm))
    card.toggle_traffic_mode(sm)
    self.assertTrue(card.get_traffic_mode(sm))
    self.assertEqual(card.traffic_mode_override, 0)

  def test_live_recommendation_requires_alive_valid_and_fresh_plan(self):
    now_ns = 10_000_000_000
    fresh_time = now_ns - int(AUTO_TRAFFIC_PLAN_MAX_AGE_S * 1e9)
    stale_time = fresh_time - 1

    with patch("openpilot.selfdrive.frogpilot.controls.frogpilot_card.time.monotonic_ns", return_value=now_ns):
      self.assertTrue(get_auto_traffic_recommendation(FakeSubMaster(True, log_mono_time=fresh_time)))
      self.assertFalse(get_auto_traffic_recommendation(FakeSubMaster(True, alive=False, log_mono_time=now_ns)))
      self.assertFalse(get_auto_traffic_recommendation(FakeSubMaster(True, valid=False, log_mono_time=now_ns)))
      self.assertFalse(get_auto_traffic_recommendation(FakeSubMaster(True, log_mono_time=stale_time)))
      self.assertFalse(get_auto_traffic_recommendation(FakeSubMaster(True, log_mono_time=0)))

  def test_manual_forced_on_overrides_a_stale_auto_plan(self):
    card = self.card()
    stale_active = FakeSubMaster(True, log_mono_time=1)

    with patch("openpilot.selfdrive.frogpilot.controls.frogpilot_card.time.monotonic_ns", return_value=10_000_000_000):
      self.assertFalse(card.get_traffic_mode(stale_active))

      # A button press against a failed-safe recommendation explicitly forces on.
      card.toggle_traffic_mode(stale_active)
      self.assertEqual(card.traffic_mode_override, 1)
      self.assertTrue(card.get_traffic_mode(stale_active))


if __name__ == "__main__":
  unittest.main()
