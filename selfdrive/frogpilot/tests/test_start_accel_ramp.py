import math
import unittest

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import (LongControl, LongCtrlState,
                                                          STOP_GO_PLANNER_STOP_GRACE_FRAMES,
                                                          cruise_standstill_for_long_control,
                                                          should_stop_for_long_control)
from openpilot.selfdrive.controls.controlsd import longitudinal_controls_inactive_reason
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
    self.assertIn("should_stop_for_long_control(long_plan.shouldStop, forcing_stop)", source)

  def test_improved_stop_and_go_ignores_only_auto_resume_pcm_standstill(self):
    launch = {"active": True, "should_stop": False, "brake_pressed": False, "planner_accel": 0.3}
    auto_resume = SimpleNamespace(enableGasInterceptor=False, autoResumeSng=True,
                                  carName="toyota", carFingerprint="TOYOTA_CAMRY_TSS2")
    legacy = SimpleNamespace(enableGasInterceptor=False, autoResumeSng=False,
                             carName="toyota", carFingerprint="TOYOTA_CAMRY_TSS2")
    wrong_car = SimpleNamespace(enableGasInterceptor=False, autoResumeSng=True,
                                carName="toyota", carFingerprint="TOYOTA_RAV4_TSS2")
    interceptor = SimpleNamespace(enableGasInterceptor=True, autoResumeSng=False,
                                  carName="toyota", carFingerprint="TOYOTA_CAMRY_TSS2")

    self.assertFalse(cruise_standstill_for_long_control(auto_resume, True, True, **launch))
    self.assertTrue(cruise_standstill_for_long_control(auto_resume, True, False, **launch))
    self.assertTrue(cruise_standstill_for_long_control(legacy, True, True, **launch))
    self.assertTrue(cruise_standstill_for_long_control(wrong_car, True, True, **launch))
    self.assertFalse(cruise_standstill_for_long_control(interceptor, True, False, **launch))

    for gates in (
      {**launch, "active": False},
      {**launch, "should_stop": True},
      {**launch, "brake_pressed": True},
      {**launch, "planner_accel": 0.0},
      {**launch, "planner_accel": math.nan},
      {**launch, "planner_accel": None},
    ):
      with self.subTest(gates=gates):
        self.assertTrue(cruise_standstill_for_long_control(auto_resume, True, True, **gates))

  def test_longitudinal_inactive_reason_is_specific(self):
    self.assertEqual(longitudinal_controls_inactive_reason(False, False, False, True), "openpilot_disabled")
    self.assertEqual(longitudinal_controls_inactive_reason(True, True, False, True, True),
                     "driver_accelerator_override")
    self.assertEqual(longitudinal_controls_inactive_reason(True, True, False, True, False),
                     "longitudinal_override")
    self.assertEqual(longitudinal_controls_inactive_reason(True, False, True, True), "frogpilot_pause")
    self.assertEqual(longitudinal_controls_inactive_reason(True, False, False, False),
                     "openpilot_longitudinal_unavailable")
    self.assertEqual(longitudinal_controls_inactive_reason(True, False, False, True), "active")


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

  def test_traffic_mode_honors_configured_start_accel_ramp(self):
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
      start_accel_ramp=1.0,
    )
    CS = SimpleNamespace(
      vEgo=0.0,
      aEgo=0.0,
      brakePressed=False,
      cruiseState=SimpleNamespace(standstill=True),
    )
    for traffic_mode in (False, True):
      for configured in (0.0, 0.55, 1.0):
        with self.subTest(traffic_mode=traffic_mode, configured=configured):
          control = LongControl(CP)
          toggles.start_accel_ramp = configured
          CS.vEgo = 0.0
          CS.brakePressed = False
          CS.cruiseState.standstill = True
          control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles, traffic_mode=traffic_mode)
          CS.cruiseState.standstill = False
          output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles, traffic_mode=traffic_mode)
          self.assertEqual(control.long_control_state, LongCtrlState.pid)
          self.assertEqual(control.start_accel_ramp.started, configured > 0.0)
          self.assertAlmostEqual(output, configured)

          # At half the assist's speed range the configured floor has halved.
          CS.vEgo = 1.0
          output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles, traffic_mode=traffic_mode)
          self.assertAlmostEqual(output, configured / 2)

          # A new planner stop cancels even the higher Traffic Mode floor.
          output = control.update(True, CS, 0.3, True, (-3.0, 2.0), toggles, traffic_mode=traffic_mode)
          self.assertLessEqual(output, 0.0)
          self.assertFalse(control.start_accel_ramp.armed)

  def test_improved_stop_and_go_breaks_stale_camry_standstill_loop(self):
    CP = SimpleNamespace(
      enableGasInterceptor=False,
      autoResumeSng=True,
      carName="toyota",
      carFingerprint="TOYOTA_CAMRY_TSS2",
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
      toadpilot_improved_stop_and_go=True,
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
    self.assertTrue(control.start_accel_ramp.started)

    CS.cruiseState.standstill = True
    output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)
    self.assertEqual(control.long_control_state, LongCtrlState.pid)
    self.assertGreater(output, 0.0)
    self.assertTrue(control.start_accel_ramp.started)
    self.assertTrue(control.stop_go_handoff_active)

  def test_stop_and_go_release_is_independent_of_optional_accel_floor(self):
    CP = SimpleNamespace(
      enableGasInterceptor=False,
      autoResumeSng=True,
      carName="toyota",
      carFingerprint="TOYOTA_CAMRY_TSS2",
      startingState=False,
      stopAccel=-2.0,
      startAccel=0.0,
      longitudinalTuning=SimpleNamespace(kpBP=[0.0], kpV=[0.0], kiBP=[0.0], kiV=[0.0], kf=1.0),
    )
    toggles = SimpleNamespace(
      vEgoStarting=0.25,
      stoppingDecelRate=0.3,
      human_acceleration=False,
      start_accel_ramp=0.0,
      toadpilot_improved_stop_and_go=True,
    )
    CS = SimpleNamespace(vEgo=0.0, aEgo=0.0, brakePressed=False,
                         cruiseState=SimpleNamespace(standstill=True))
    control = LongControl(CP)

    control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles)
    output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)

    self.assertEqual(control.long_control_state, LongCtrlState.pid)
    self.assertAlmostEqual(output, 0.3)
    self.assertTrue(control.stop_go_handoff_active)
    self.assertFalse(control.start_accel_ramp.armed)

  def test_stop_and_go_handoff_is_time_bounded(self):
    CP = SimpleNamespace(
      enableGasInterceptor=False,
      autoResumeSng=True,
      carName="toyota",
      carFingerprint="TOYOTA_CAMRY_TSS2",
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
      toadpilot_improved_stop_and_go=True,
    )
    CS = SimpleNamespace(vEgo=0.0, aEgo=0.0, brakePressed=False,
                         cruiseState=SimpleNamespace(standstill=True))
    control = LongControl(CP)

    control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles)
    control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)
    for _ in range(math.ceil(START_ACCEL_RAMP_MAX_DURATION / DT_CTRL) + 1):
      output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)

    self.assertFalse(control.stop_go_handoff_active)
    self.assertEqual(control.stop_go_handoff_last_event, "timeout")
    self.assertFalse(control.start_accel_ramp.armed)
    self.assertEqual(output, 0.0)

  def test_improved_stop_and_go_does_not_override_safety_gates(self):
    CP = SimpleNamespace(
      enableGasInterceptor=False,
      autoResumeSng=True,
      carName="toyota",
      carFingerprint="TOYOTA_CAMRY_TSS2",
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
      toadpilot_improved_stop_and_go=True,
    )

    for active, should_stop, brake_pressed, expected_event in (
      (True, True, False, "planner_stop_confirmed"),
      (True, False, True, "brake"),
      (False, False, False, "controls_inactive"),
    ):
      with self.subTest(active=active, should_stop=should_stop, brake_pressed=brake_pressed):
        CS = SimpleNamespace(
          vEgo=0.0,
          aEgo=0.0,
          brakePressed=False,
          cruiseState=SimpleNamespace(standstill=True),
        )
        control = LongControl(CP)
        control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles)
        self.assertGreater(control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles), 0.0)
        self.assertTrue(control.start_accel_ramp.started)

        CS.brakePressed = brake_pressed
        output = control.update(active, CS, 0.3, should_stop, (-3.0, 2.0), toggles)
        if should_stop:
          for _ in range(math.ceil(0.10 / DT_CTRL) + 1):
            output = control.update(active, CS, 0.3, should_stop, (-3.0, 2.0), toggles)
        self.assertLessEqual(output, 0.0)
        self.assertFalse(control.stop_go_handoff_active)
        self.assertEqual(control.stop_go_handoff_last_event, expected_event)
        self.assertFalse(control.start_accel_ramp.armed)
        self.assertFalse(control.start_accel_ramp.started)

        if expected_event != "planner_stop_confirmed":
          CS.brakePressed = False
          output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)
          self.assertFalse(control.start_accel_ramp.started)
          self.assertLessEqual(output, 0.0)

  def test_brief_contradictory_planner_stop_pauses_without_accelerating_or_rearming(self):
    CP = SimpleNamespace(
      enableGasInterceptor=False, autoResumeSng=True, carName="toyota",
      carFingerprint="TOYOTA_CAMRY_TSS2", startingState=False, stopAccel=-2.0, startAccel=0.0,
      longitudinalTuning=SimpleNamespace(kpBP=[0.0], kpV=[0.0], kiBP=[0.0], kiV=[0.0], kf=0.0),
    )
    toggles = SimpleNamespace(vEgoStarting=0.25, stoppingDecelRate=0.3, human_acceleration=False,
                              start_accel_ramp=0.4, toadpilot_improved_stop_and_go=True)
    CS = SimpleNamespace(vEgo=0.0, aEgo=0.0, brakePressed=False,
                         cruiseState=SimpleNamespace(standstill=True))
    control = LongControl(CP)
    control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles)
    self.assertEqual(control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles),
                     START_ACCEL_BREAKAWAY_FLOOR)
    started_count = control.stop_go_event_counts["started"]

    for _ in range(5):
      output = control.update(True, CS, 0.3, True, (-3.0, 2.0), toggles)
      self.assertLessEqual(output, 0.0)
      self.assertEqual(control.long_control_state, LongCtrlState.stopping)
      self.assertTrue(control.stop_go_handoff_active)
      self.assertTrue(control.start_accel_ramp.armed)
    self.assertTrue(control.stop_go_planner_stop_pending)

    output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)
    self.assertGreater(output, 0.0)
    self.assertFalse(control.stop_go_planner_stop_pending)
    self.assertEqual(control.stop_go_event_counts["started"], started_count)
    self.assertEqual(control.stop_go_handoff_last_event, "planner_stop_cleared")

  def test_contradictory_planner_stop_confirms_on_exact_100ms_boundary(self):
    CP = SimpleNamespace(
      enableGasInterceptor=False, autoResumeSng=True, carName="toyota",
      carFingerprint="TOYOTA_CAMRY_TSS2", startingState=False, stopAccel=-2.0, startAccel=0.0,
      longitudinalTuning=SimpleNamespace(kpBP=[0.0], kpV=[0.0], kiBP=[0.0], kiV=[0.0], kf=0.0),
    )
    toggles = SimpleNamespace(vEgoStarting=0.25, stoppingDecelRate=0.3, human_acceleration=False,
                              start_accel_ramp=0.4, toadpilot_improved_stop_and_go=True)
    CS = SimpleNamespace(vEgo=0.0, aEgo=0.0, brakePressed=False,
                         cruiseState=SimpleNamespace(standstill=True))
    control = LongControl(CP)
    control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles)
    control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)

    for _ in range(STOP_GO_PLANNER_STOP_GRACE_FRAMES - 1):
      output = control.update(True, CS, 0.3, True, (-3.0, 2.0), toggles)
      self.assertLessEqual(output, 0.0)
      self.assertTrue(control.stop_go_handoff_active)
      self.assertTrue(control.stop_go_planner_stop_pending)

    output = control.update(True, CS, 0.3, True, (-3.0, 2.0), toggles)
    self.assertLessEqual(output, 0.0)
    self.assertFalse(control.stop_go_handoff_active)
    self.assertFalse(control.stop_go_planner_stop_pending)
    self.assertEqual(control.stop_go_handoff_last_event, "planner_stop_confirmed")
    self.assertAlmostEqual(control.stop_go_last_planner_stop_duration, 0.10)
    self.assertGreater(control.stop_go_last_handoff_duration, 0.10)

  def test_forced_stop_bypasses_planner_pulse_grace_and_records_inactive_detail(self):
    CP = SimpleNamespace(
      enableGasInterceptor=False, autoResumeSng=True, carName="toyota",
      carFingerprint="TOYOTA_CAMRY_TSS2", startingState=False, stopAccel=-2.0, startAccel=0.0,
      longitudinalTuning=SimpleNamespace(kpBP=[0.0], kpV=[0.0], kiBP=[0.0], kiV=[0.0], kf=0.0),
    )
    toggles = SimpleNamespace(vEgoStarting=0.25, stoppingDecelRate=0.3, human_acceleration=False,
                              start_accel_ramp=0.4, toadpilot_improved_stop_and_go=True)
    CS = SimpleNamespace(vEgo=0.0, aEgo=0.0, brakePressed=False,
                         cruiseState=SimpleNamespace(standstill=True))
    control = LongControl(CP)
    control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles)
    control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)
    output = control.update(True, CS, 0.3, True, (-3.0, 2.0), toggles, forcing_stop=True)
    self.assertLessEqual(output, 0.0)
    self.assertFalse(control.stop_go_handoff_active)
    self.assertEqual(control.stop_go_handoff_last_event, "forced_stop")

    control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)
    control.update(False, CS, 0.3, False, (-3.0, 2.0), toggles,
                   inactive_reason="driver_accelerator_override")
    self.assertEqual(control.stop_go_handoff_last_event, "controls_inactive")
    self.assertEqual(control.stop_go_handoff_last_detail, "driver_accelerator_override")
    self.assertIn("controls_inactive:driver_accelerator_override", control.stop_go_event_counts)

  def test_disabled_improved_stop_and_go_preserves_pcm_standstill_hold(self):
    CP = SimpleNamespace(
      enableGasInterceptor=False,
      autoResumeSng=True,
      carName="toyota",
      carFingerprint="TOYOTA_CAMRY_TSS2",
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
      toadpilot_improved_stop_and_go=False,
    )
    CS = SimpleNamespace(vEgo=0.0, aEgo=0.0, brakePressed=False,
                         cruiseState=SimpleNamespace(standstill=True))
    control = LongControl(CP)

    control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles)
    output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)

    self.assertEqual(control.long_control_state, LongCtrlState.stopping)
    self.assertLessEqual(output, 0.0)
    self.assertFalse(control.start_accel_ramp.started)

    del toggles.toadpilot_improved_stop_and_go
    control = LongControl(CP)
    control.update(True, CS, 0.0, True, (-3.0, 2.0), toggles)
    output = control.update(True, CS, 0.3, False, (-3.0, 2.0), toggles)
    self.assertEqual(control.long_control_state, LongCtrlState.stopping)
    self.assertLessEqual(output, 0.0)

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
