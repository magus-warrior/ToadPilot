import json
import math
import unittest
from dataclasses import asdict
from types import SimpleNamespace as NS
from unittest.mock import patch

from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_acceleration import FrogPilotAcceleration
from openpilot.selfdrive.frogpilot.controls.lib.longitudinal_features import StopReleaseLatch, smooth_stop_request
from openpilot.selfdrive.frogpilot.toadpilot import ToadPilotConfig, decode_toadpilot_config


class TestLongitudinalFeatureConfig(unittest.TestCase):
  def test_v8_migrates_with_new_features_on_and_preserves_choices(self):
    old = asdict(ToadPilotConfig(profile="observe", auto_tune=False, enabled=False))
    old.pop("responsive_takeoff")
    old.pop("smooth_stop_finish")
    old["version"] = 8
    new = decode_toadpilot_config(json.dumps(old))
    self.assertTrue(new.responsive_takeoff)
    self.assertTrue(new.smooth_stop_finish)
    self.assertFalse(new.enabled)
    self.assertFalse(new.auto_tune)
    self.assertEqual(new.profile, "observe")
    for key, value in old.items():
      if key != "version":
        self.assertEqual(getattr(new, key), value)

  def test_explicit_off_persists_and_malformed_fails_off(self):
    saved = asdict(ToadPilotConfig(responsive_takeoff=False, smooth_stop_finish=False))
    self.assertEqual(decode_toadpilot_config(json.dumps(saved)), ToadPilotConfig(**saved))
    for key in ("responsive_takeoff", "smooth_stop_finish"):
      for bad in (1, None, "true"):
        cfg = decode_toadpilot_config(json.dumps({**saved, key: bad}))
        self.assertFalse(cfg.enabled)
        self.assertFalse(cfg.responsive_takeoff)
        self.assertFalse(cfg.smooth_stop_finish)


class TestStopReleaseLatch(unittest.TestCase):
  def step(self, latch, **changes):
    args = dict(enabled=True, active=True, stopping=False, brake_pressed=False, speed=0.0, accel=0.0)
    return latch.update(**{**args, **changes})

  def test_zero_accel_transition_does_not_lose_release(self):
    latch = StopReleaseLatch()
    self.assertFalse(self.step(latch, stopping=True, accel=-2.0))
    for _ in range(30):
      self.assertFalse(self.step(latch))
    self.assertTrue(self.step(latch, accel=0.12))
    self.assertFalse(self.step(latch, accel=0.12))

  def test_requires_stop_and_cancels_on_override_invalid_or_expiry(self):
    self.assertFalse(self.step(StopReleaseLatch(), accel=0.12))
    for changes in (dict(enabled=False), dict(active=False), dict(brake_pressed=True),
                    dict(speed=2.0), dict(speed=-0.1), dict(speed=math.nan), dict(accel=math.nan)):
      latch = StopReleaseLatch()
      self.step(latch, stopping=True)
      self.assertFalse(self.step(latch, **changes))
      self.assertFalse(self.step(latch, accel=0.12))
    latch = StopReleaseLatch()
    self.step(latch, stopping=True)
    for _ in range(201):
      self.step(latch)
    self.assertFalse(self.step(latch, accel=0.12))

  def test_positive_accel_with_stop_cannot_release(self):
    latch = StopReleaseLatch()
    for _ in range(300):
      self.assertFalse(self.step(latch, stopping=True, accel=0.12))


class TestResponsiveAcceleration(unittest.TestCase):
  def test_traffic_retains_human_acceleration_ceiling(self):
    planner = NS(frogpilot_following=NS(traffic_mode_blend=1.0, following_lead=True),
                 lead_one=NS(aLeadK=0.7), v_cruise=25.0)
    toggles = NS(map_acceleration=False, acceleration_profile=2, sport_plus=False,
                 human_acceleration=True, map_deceleration=False, deceleration_profile=0,
                 toadpilot_responsive_takeoff=True)
    car = NS(ecoGear=False, sportGear=False, forceCoast=False)
    accel = FrogPilotAcceleration(planner)
    accel.update(car, 0.0, toggles)
    self.assertEqual(accel.max_accel, 3.0)
    before_min = accel.min_accel
    toggles.toadpilot_responsive_takeoff = False
    accel.update(car, 0.0, toggles)
    self.assertEqual(accel.max_accel, 1.6)
    self.assertEqual(accel.min_accel, before_min)


class TestSmoothStopFinish(unittest.TestCase):
  def test_relief_is_bounded_and_never_weakens_stronger_plan(self):
    for stock in (-2.0, -1.0, -0.6):
      for speed in (0.0, 0.05, 0.1, 0.25, 0.49, 0.5, 1.0):
        for planner in (-3.0, -0.9, -0.7, -0.05, 0.0, 0.1):
          out = smooth_stop_request(stock, stock, planner, speed, -0.6, allowed=True)
          self.assertGreaterEqual(out, stock)
          self.assertLessEqual(out, stock + 0.3)
          self.assertLessEqual(out, -0.5)
          if planner < stock:
            self.assertEqual(out, stock)
          else:
            self.assertLessEqual(out, planner)

  def test_hard_gates_preserve_stock(self):
    for speed, accel, planner, allowed in (
      (0.2, -0.6, -0.05, False), (0.2, -2.0, -0.05, True),
      (0.2, 0.0, -0.05, True), (math.nan, -0.6, -0.05, True),
      (0.2, math.nan, -0.05, True), (0.2, -0.6, math.nan, True),
    ):
      self.assertEqual(smooth_stop_request(-0.9, -0.9, planner, speed, accel, allowed=allowed), -0.9)

  @staticmethod
  def setup_control(enabled=True, fingerprint="TOYOTA_CAMRY_TSS2"):
    cp = NS(carFingerprint=fingerprint, openpilotLongitudinalControl=True, enableGasInterceptor=False,
            autoResumeSng=False, startingState=False, stopAccel=-2.0, startAccel=0.0,
            longitudinalTuning=NS(kpBP=[0.0], kpV=[0.0], kiBP=[0.0], kiV=[0.0], kf=1.0))
    toggles = NS(vEgoStarting=0.25, stoppingDecelRate=0.3, human_acceleration=True,
                 start_accel_ramp=0.0, toadpilot_smooth_stop_finish=enabled)
    cs = NS(vEgo=0.4, aEgo=-0.6, brakePressed=False, standstill=False, cruiseState=NS(standstill=False))
    control = LongControl(cp)
    control.long_control_state = LongCtrlState.pid
    control.last_output_accel = -0.6
    return control, toggles, cs

  def test_final_approach_then_hold_has_no_relief_removal_step(self):
    control, toggles, cs = self.setup_control()
    baseline, off, cs_off = self.setup_control(False)
    with patch.object(cloudlog, "event"):
      for i in range(80):
        cs.vEgo = cs_off.vEgo = 0.4 - i * 0.004
        output = control.update(True, cs, -0.05, True, (-3.5, 2.0), toggles)
        stock = baseline.update(True, cs_off, -0.05, True, (-3.5, 2.0), off)
        self.assertGreaterEqual(output, stock - 1e-9)
        self.assertLessEqual(output - stock, 0.3)
      self.assertGreater(output, stock)
      before_hold = output
      cs.vEgo = 0.0
      cs.aEgo = 0.0
      cs.standstill = True
      output = control.update(True, cs, 0.0, True, (-3.5, 2.0), toggles)
      self.assertAlmostEqual(output, before_hold - 0.3 * DT_CTRL)
      for _ in range(800):
        output = control.update(True, cs, 0.0, True, (-3.5, 2.0), toggles)
      self.assertAlmostEqual(output, -2.0, delta=0.004)

  def test_other_car_and_forced_stop_keep_original_braking(self):
    for fingerprint, forced in (("TOYOTA_RAV4_TSS2", False), ("TOYOTA_CAMRY_TSS2", True)):
      control, toggles, cs = self.setup_control(fingerprint=fingerprint)
      with patch.object(cloudlog, "event"):
        for i in range(100):
          out = control.update(True, cs, -0.05, True, (-3.5, 2.0), toggles, forcing_stop=forced)
          self.assertAlmostEqual(out, -0.6 - (i + 1) * 0.3 * DT_CTRL)

  def test_retired_ramp_cannot_override_planner_or_stop_with_either_toggle_choice(self):
    for feature in (True, False):
      control, toggles, cs = self.setup_control()
      toggles.toadpilot_responsive_takeoff = feature
      toggles.start_accel_ramp = 1.0
      cs.vEgo = 0.0
      cs.aEgo = 0.0
      with patch.object(cloudlog, "event"):
        stopped = control.update(True, cs, 0.12, True, (-3.5, 2.0), toggles)
        self.assertLessEqual(stopped, 0.0)
        launched = control.update(True, cs, 0.12, False, (-3.5, 2.0), toggles)
      self.assertAlmostEqual(launched, 0.12)
      self.assertFalse(control.start_accel_ramp.armed)


if __name__ == "__main__":
  unittest.main()
