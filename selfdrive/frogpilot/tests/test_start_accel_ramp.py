import math
import unittest

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState, should_stop_for_long_control
from openpilot.selfdrive.frogpilot.controls.lib.start_accel_ramp import (START_ACCEL_RAMP_DEFAULT,
                                                                        START_ACCEL_BREAKAWAY_DURATION,
                                                                        START_ACCEL_BREAKAWAY_END_SPEED,
                                                                        START_ACCEL_BREAKAWAY_FLOOR,
                                                                        START_ACCEL_RAMP_END_SPEED,
                                                                        START_ACCEL_RAMP_MAX_DURATION,
                                                                        StartAccelRamp,
                                                                        parse_start_accel_ramp,
                                                                        validate_start_accel_ramp)


OPENPILOT_ROOT = Path(__file__).resolve().parents[3]


class TestLongControlStopRequest(unittest.TestCase):
  def test_only_real_stop_flags_hold_long_control(self):
    self.assertFalse(should_stop_for_long_control(False, False))
    self.assertTrue(should_stop_for_long_control(True, False))
    self.assertTrue(should_stop_for_long_control(False, True))
    self.assertTrue(should_stop_for_long_control(True, True))

  def test_model_distance_is_not_used_as_a_boolean_stop_hold(self):
    source = (OPENPILOT_ROOT / "selfdrive/controls/controlsd.py").read_text()
    self.assertNotIn("forcingStopLength < 1", source)
    self.assertIn("should_stop_for_long_control(long_plan.shouldStop, self.sm['frogpilotPlan'].forcingStop)", source)


class TestStartAccelRampSettings(unittest.TestCase):
  def test_default_and_malformed_values_fail_disabled(self):
    self.assertEqual(START_ACCEL_RAMP_DEFAULT, 0.0)
    for value in (None, True, b"bad", b"\xff", "nan", math.inf, -0.05, 1.05, 0.33):
      with self.subTest(value=value):
        self.assertEqual(parse_start_accel_ramp(value), START_ACCEL_RAMP_DEFAULT)

  def test_valid_values_are_canonical(self):
    self.assertEqual(parse_start_accel_ramp(b"0.35"), 0.35)
    self.assertEqual(validate_start_accel_ramp(1), 1.0)
    with self.assertRaisesRegex(ValueError, "number"):
      validate_start_accel_ramp("0.35")


class TestStartAccelRampControl(unittest.TestCase):
  @staticmethod
  def update(ramp, **overrides):
    values = {
      "output_accel": -0.1,
      "configured_accel": 0.5,
      "v_ego": 0.0,
      "planner_accel": 0.3,
      "released_from_stop": False,
      "control_active": True,
      "should_stop": False,
      "brake_pressed": False,
      "cruise_standstill": False,
    }
    values.update(overrides)
    return ramp.update(**values)

  def test_zero_setting_preserves_output_exactly(self):
    ramp = StartAccelRamp()
    output = self.update(ramp, output_accel=-0.42, configured_accel=0.0, released_from_stop=True)
    self.assertEqual(output, -0.42)
    self.assertFalse(ramp.armed)

  def test_requires_a_real_stop_release_and_positive_plan(self):
    ramp = StartAccelRamp()
    self.assertEqual(self.update(ramp), -0.1)

    self.assertEqual(self.update(ramp, planner_accel=0.0, released_from_stop=True), -0.1)
    self.assertTrue(ramp.armed)
    self.assertFalse(ramp.started)

    output = self.update(ramp, planner_accel=0.3)
    self.assertGreater(output, 0.0)
    self.assertTrue(ramp.started)

  def test_floor_is_bounded_and_fades_with_speed(self):
    ramp = StartAccelRamp()
    output = self.update(ramp, output_accel=0.0, configured_accel=0.8, released_from_stop=True)
    for _ in range(100):
      output = self.update(ramp, output_accel=0.0, configured_accel=0.8)
    self.assertAlmostEqual(output, 0.8)

    output = self.update(ramp, output_accel=0.0, configured_accel=0.8, v_ego=START_ACCEL_RAMP_END_SPEED / 2)
    self.assertAlmostEqual(output, 0.4)
    self.assertEqual(self.update(ramp, output_accel=-0.2, v_ego=START_ACCEL_RAMP_END_SPEED), -0.2)
    self.assertFalse(ramp.armed)

  def test_brief_breakaway_boost_hands_back_to_configured_ramp(self):
    ramp = StartAccelRamp()
    output = self.update(ramp, output_accel=0.0, configured_accel=0.4, released_from_stop=True)
    while ramp.elapsed + DT_CTRL < START_ACCEL_BREAKAWAY_DURATION:
      output = self.update(ramp, output_accel=0.0, configured_accel=0.4)
    self.assertAlmostEqual(output, START_ACCEL_BREAKAWAY_FLOOR)

    while ramp.elapsed <= START_ACCEL_BREAKAWAY_DURATION:
      output = self.update(ramp, output_accel=0.0, configured_accel=0.4)
    self.assertAlmostEqual(output, 0.4)

    ramp = StartAccelRamp()
    self.update(ramp, output_accel=0.0, configured_accel=0.4, released_from_stop=True)
    ramp.accel = START_ACCEL_BREAKAWAY_FLOOR
    output = self.update(ramp, output_accel=0.0, configured_accel=0.4, v_ego=START_ACCEL_BREAKAWAY_END_SPEED)
    expected = 0.4 * (1.0 - START_ACCEL_BREAKAWAY_END_SPEED / START_ACCEL_RAMP_END_SPEED)
    self.assertAlmostEqual(output, expected)

  def test_breakaway_boost_never_enables_a_disabled_ramp(self):
    ramp = StartAccelRamp()
    output = self.update(ramp, output_accel=-0.2, configured_accel=0.0, released_from_stop=True)
    self.assertEqual(output, -0.2)
    self.assertFalse(ramp.armed)

  def test_stop_or_brake_clears_ramp_without_delayed_retrigger(self):
    for gate in ({"should_stop": True}, {"brake_pressed": True}, {"cruise_standstill": True}, {"control_active": False}):
      with self.subTest(gate=gate):
        ramp = StartAccelRamp()
        self.assertGreater(self.update(ramp, released_from_stop=True), 0.0)
        self.assertEqual(self.update(ramp, output_accel=-0.2, **gate), -0.2)
        self.assertFalse(ramp.armed)
        self.assertEqual(self.update(ramp, output_accel=-0.2), -0.2)

  def test_armed_ramp_expires_if_planner_never_requests_motion(self):
    ramp = StartAccelRamp()
    self.update(ramp, planner_accel=0.0, released_from_stop=True)
    for _ in range(math.ceil(START_ACCEL_RAMP_MAX_DURATION / DT_CTRL) + 1):
      self.update(ramp, planner_accel=0.0)
    self.assertFalse(ramp.armed)
    self.assertEqual(self.update(ramp, output_accel=-0.2, planner_accel=0.3), -0.2)

  def test_long_control_applies_ramp_on_stopping_to_pid_transition(self):
    CP = SimpleNamespace(
      enableGasInterceptor=False,
      startingState=False,
      stopAccel=-2.0,
      startAccel=0.0,
      longitudinalTuning=SimpleNamespace(kpBP=[0.0], kpV=[0.0], kiBP=[0.0], kiV=[0.0], kf=0.0),
    )
    toggles = SimpleNamespace(
      vEgoStarting=0.25,
      stoppingDecelRate=0.3,
      human_acceleration=False,
      start_accel_ramp=0.5,
    )
    CS = SimpleNamespace(
      vEgo=0.0,
      aEgo=0.0,
      brakePressed=False,
      cruiseState=SimpleNamespace(standstill=True),
    )
    control = LongControl(CP)

    control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles)
    self.assertEqual(control.long_control_state, LongCtrlState.stopping)

    CS.cruiseState.standstill = False
    output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)
    self.assertEqual(control.long_control_state, LongCtrlState.pid)
    self.assertGreater(output, 0.0)

  def test_logs_release_start_and_brake_intervention(self):
    ramp = StartAccelRamp()
    with patch.object(ramp, "_log") as log:
      self.update(ramp, released_from_stop=True)
      self.update(ramp, brake_pressed=True)

    events = [call.args[0] for call in log.call_args_list]
    self.assertEqual(events, ["armed", "started", "brake_intervention"])

  def test_logs_why_a_release_was_blocked(self):
    ramp = StartAccelRamp()
    with patch.object(ramp, "_log") as log:
      self.update(ramp, released_from_stop=True, should_stop=True, cruise_standstill=True)

    self.assertEqual(log.call_args.args[0], "release_blocked")
    self.assertEqual(log.call_args.kwargs["blocked_by"], ["planner_stop", "cruise_standstill"])


if __name__ == "__main__":
  unittest.main()
