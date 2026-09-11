import json
import math
import unittest

from dataclasses import replace
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import cereal.messaging as messaging
import numpy as np

from cereal import car
from openpilot.common.params import Params
from openpilot.selfdrive.car import gen_empty_fingerprint
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.hyundai.interface import CarInterface as HyundaiCarInterface
from openpilot.selfdrive.car.hyundai.values import CAR as HYUNDAI_CAR
from openpilot.selfdrive.car.toyota.interface import CarInterface
from openpilot.selfdrive.car.toyota.values import CAR
from openpilot.selfdrive.controls.lib.events import forcing_stop_alert, steer_temp_unavailable_silent_alert
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.frogpilot.toadpilot import (DEFAULT_TOADPILOT_CONFIG, TOADPILOT_CONFIG_VERSION,
                                                     TOADPILOT_LANE_CENTER_CONFIRM_SAMPLES,
                                                     TOADPILOT_LANE_SOFT_FADE_MAX_UPDATES,
                                                     TOADPILOT_LAT_ACCEL_OFFSET_MAX, TOADPILOT_PROFILES,
                                                     TOADPILOT_RESET_TUNE_KEY, TOADPILOT_TUNE_STATUS_KEY,
                                                     ToadPilotConfig, ToadPilotHighAuthorityObserver,
                                                     ToadPilotLaneCentering, config_from_toggles,
                                                     decode_toadpilot_config, extract_toadpilot_lane_context,
                                                     toadpilot_steer_saturated)
from openpilot.selfdrive.locationd.torqued import (FACTOR_SANITY, FACTOR_SANITY_QLOG,
                                                   FRICTION_SANITY, FRICTION_SANITY_QLOG, MAX_FILTER_DECAY,
                                                   MIN_FILTER_DECAY, VERSION, TorqueEstimator,
                                                   _consume_toadpilot_reset)
from openpilot.system.manager.process_config import managed_processes


class LinearTorqueInterface:
  use_nnff = False
  use_nnff_lite = False

  @staticmethod
  def torque_from_lateral_accel():
    return CarInterfaceBase.torque_from_lateral_accel_linear


class TestToadPilotSteeringFaultMessage(unittest.TestCase):
  def test_confirmed_toyota_high_rate_fault_explains_1700_pause(self):
    CP = SimpleNamespace(carFingerprint=CAR.TOYOTA_CAMRY_TSS2)
    CS = car.CarState.new_message()
    CS.steerFaultStatus = 9
    CS.steerFaultStatusInitial = 21
    output = car.CarOutput.new_message()
    output.actuatorsOutput.toadPilotHighTorqueAuthorized = True
    alert = steer_temp_unavailable_silent_alert(
      CP, CS, {"carOutput": output}, False, 0, SimpleNamespace(),
    )
    self.assertEqual(alert.alert_text_1, "Steering Temporarily Unavailable")
    self.assertIn("high-rate state 21", alert.alert_text_2)
    self.assertIn("1700 headroom paused", alert.alert_text_2)

  def test_stock_authority_keeps_generic_temporary_fault_message(self):
    CP = SimpleNamespace(carFingerprint=CAR.TOYOTA_CAMRY_TSS2)
    CS = car.CarState.new_message()
    CS.steerFaultStatusInitial = 21
    output = car.CarOutput.new_message()
    alert = steer_temp_unavailable_silent_alert(
      CP, CS, {"carOutput": output}, False, 0, SimpleNamespace(),
    )
    self.assertEqual(alert.alert_text_2, "")


def toad_toggles(*, enabled=True, profile="adaptive", auto_tune=True):
  return SimpleNamespace(
    toadpilot_enabled=enabled,
    toadpilot_profile=profile,
    toadpilot_auto_tune=auto_tune,
    toadpilot_lane_centering_assist=True,
    toadpilot_smooth_lane_transitions=True,
    toadpilot_straight_line_smoothing=True,
    toadpilot_experimental_max_torque=1500,
    toadpilot_improved_stop_and_go=True,
    toadpilot_automatic_traffic_mode=True,
    toadpilot_map_stop_assist=True,
    toadpilot_curve_assist_in_experimental=False,
    toadpilot_smart_saturation_warning=True,
    toadpilot_telemetry=False,
    use_custom_lat_accel_factor=False,
    use_custom_steer_friction=False,
    steer_lat_accel_factor=0.0,
    steer_friction=0.0,
  )


def lane_model(*, frame_id=1, lane_offset_m=0.30, path_lane_bias_m=0.0,
               lane_probability=0.95, lane_std=0.10, lane_width_m=3.60,
               camera_offset_m=0.04, center_slope=0.0):
  """Construct the subset of modelV2 used by TOADPILOT lane centering."""
  x = np.linspace(0.0, 50.0, 21).tolist()
  raw_center = [lane_offset_m - camera_offset_m + center_slope * distance for distance in x]
  left_y = [center - lane_width_m / 2.0 for center in raw_center]
  right_y = [center + lane_width_m / 2.0 for center in raw_center]
  path_y = [center + path_lane_bias_m for center in raw_center]
  unused_line = SimpleNamespace(x=x, y=[0.0] * len(x))
  return SimpleNamespace(
    frameId=frame_id,
    laneLines=(unused_line, SimpleNamespace(x=x, y=left_y), SimpleNamespace(x=x, y=right_y), unused_line),
    laneLineProbs=(0.0, lane_probability, lane_probability, 0.0),
    laneLineStds=(1.0, lane_std, lane_std, 1.0),
    position=SimpleNamespace(x=x, y=path_y),
  )


def centering_config(*, profile="adaptive", experimental_opt_in=False):
  return ToadPilotConfig(
    enabled=True,
    profile=profile,
    lane_centering_assist=True,
    curve_assist_in_experimental=experimental_opt_in,
  )


class TestToadPilotConfigV7(unittest.TestCase):
  def test_route_logger_stays_disabled_after_low_memory_failure(self):
    self.assertNotIn("loggerd", managed_processes)

  def test_v1_config_migrates_to_new_defaults_and_preserves_other_choices(self):
    legacy = {
      "version": 1,
      "enabled": False,
      "profile": "observe",
      "auto_tune": False,
      "curve_assist_in_experimental": True,
      "smart_saturation_warning": False,
      "telemetry": False,
    }

    migrated = decode_toadpilot_config(json.dumps(legacy))

    self.assertEqual(TOADPILOT_CONFIG_VERSION, 8)
    self.assertEqual(migrated.version, TOADPILOT_CONFIG_VERSION)
    self.assertTrue(migrated.enabled)
    self.assertEqual(migrated.profile, "adaptive")
    self.assertTrue(migrated.lane_centering_assist)
    self.assertTrue(migrated.smooth_lane_transitions)
    self.assertTrue(migrated.straight_line_smoothing)
    self.assertEqual(migrated.experimental_max_torque, 1500)
    self.assertTrue(migrated.improved_stop_and_go)
    self.assertTrue(migrated.automatic_traffic_mode)
    self.assertTrue(migrated.map_stop_assist)
    for field in ("auto_tune", "curve_assist_in_experimental", "smart_saturation_warning", "telemetry"):
      self.assertEqual(getattr(migrated, field), legacy[field])

  def test_malformed_v1_and_future_versions_fail_closed(self):
    valid_v1 = {
      "version": 1,
      "enabled": True,
      "profile": "adaptive",
      "auto_tune": False,
      "curve_assist_in_experimental": True,
      "smart_saturation_warning": False,
      "telemetry": False,
    }
    malformed_cases = (
      {key: value for key, value in valid_v1.items() if key != "telemetry"},
      {**valid_v1, "unexpected": True},
      {**valid_v1, "version": 8},
      {**valid_v1, "version": True},
      {**valid_v1, "version": 1.0},
    )

    for raw_config in malformed_cases:
      with self.subTest(raw_config=raw_config):
        config = decode_toadpilot_config(json.dumps(raw_config))
        self.assertFalse(config.enabled)
        self.assertEqual(config.profile, "observe")
        self.assertFalse(config.lane_centering_assist)
        self.assertFalse(config.smooth_lane_transitions)
        self.assertFalse(config.straight_line_smoothing)
        self.assertFalse(config.improved_stop_and_go)

  def test_lane_centering_default_is_on_but_runtime_toggle_is_strict_bool(self):
    base = toad_toggles()
    self.assertTrue(DEFAULT_TOADPILOT_CONFIG.lane_centering_assist)
    self.assertTrue(config_from_toggles(base).lane_centering_assist)

    base.toadpilot_lane_centering_assist = 1
    self.assertFalse(config_from_toggles(base).lane_centering_assist)

    base.toadpilot_lane_centering_assist = True
    self.assertTrue(config_from_toggles(base).lane_centering_assist)

  def test_smooth_lane_transitions_default_is_on_but_runtime_toggle_is_strict_bool(self):
    base = toad_toggles()
    self.assertTrue(DEFAULT_TOADPILOT_CONFIG.smooth_lane_transitions)
    self.assertTrue(config_from_toggles(base).smooth_lane_transitions)

    base.toadpilot_smooth_lane_transitions = 1
    self.assertFalse(config_from_toggles(base).smooth_lane_transitions)

  def test_straight_line_smoothing_default_is_on_but_runtime_toggle_is_strict_bool(self):
    base = toad_toggles()
    self.assertTrue(DEFAULT_TOADPILOT_CONFIG.straight_line_smoothing)
    self.assertTrue(config_from_toggles(base).straight_line_smoothing)

    base.toadpilot_straight_line_smoothing = 1
    self.assertFalse(config_from_toggles(base).straight_line_smoothing)

    base.toadpilot_straight_line_smoothing = False
    self.assertFalse(config_from_toggles(base).straight_line_smoothing)

    base.toadpilot_smooth_lane_transitions = False
    self.assertFalse(config_from_toggles(base).smooth_lane_transitions)


class TestToadPilotLaneContext(unittest.TestCase):
  def test_extracts_camera_corrected_lane_offset_and_path_bias(self):
    context = extract_toadpilot_lane_context(
      lane_model(lane_offset_m=0.27, path_lane_bias_m=0.08, center_slope=0.004),
      v_ego=20.0,
      camera_offset=0.04,
    )

    self.assertTrue(context.valid)
    self.assertAlmostEqual(context.lane_offset_m, 0.27, places=3)
    self.assertAlmostEqual(context.path_lane_bias_m, 0.08, places=3)
    self.assertAlmostEqual(context.lane_width_m, 3.60, places=3)
    self.assertGreaterEqual(context.confidence, 0.80)

  def test_rejects_bad_confidence_and_implausible_lane_width(self):
    cases = (
      lane_model(lane_probability=0.20),
      lane_model(lane_probability=1.20),
      lane_model(lane_std=0.80),
      lane_model(lane_std=-0.10),
      lane_model(lane_width_m=2.40),
      lane_model(lane_width_m=4.80),
    )
    for model in cases:
      with self.subTest(model=model):
        self.assertFalse(extract_toadpilot_lane_context(model, v_ego=20.0).valid)

  def test_rejects_implausibly_displaced_lane_pair_in_both_directions(self):
    for lane_offset in (-1.21, 1.21):
      with self.subTest(lane_offset=lane_offset):
        context = extract_toadpilot_lane_context(
          lane_model(lane_offset_m=lane_offset),
          v_ego=20.0,
        )
        self.assertFalse(context.valid)
        self.assertEqual(context.reason, "lane_offset_implausible")


class TestToadPilotLaneCentering(unittest.TestCase):
  @staticmethod
  def update(controller, config, context, *, base_curvature=0.0, v_ego=20.0, **gate_overrides):
    gates = {
      "lateral_active": True,
      "steering_pressed": False,
      "steering_rate_deg": 0.0,
      "steer_limited": False,
      "experimental_mode": False,
      "lane_change_active": False,
      "blinker_active": False,
    }
    gates.update(gate_overrides)
    return controller.update(config, base_curvature, v_ego, context, **gates)

  def run_frames(self, controller, *, lane_offset_m=0.30, path_lane_bias_m=0.0,
                 config=None, frames=None, start_frame=1, v_ego=20.0, **gates):
    config = config or centering_config()
    frames = frames or TOADPILOT_LANE_CENTER_CONFIRM_SAMPLES + 20
    result = 0.0
    for frame_id in range(start_frame, start_frame + frames):
      context = extract_toadpilot_lane_context(
        lane_model(frame_id=frame_id, lane_offset_m=lane_offset_m, path_lane_bias_m=path_lane_bias_m),
        v_ego=v_ego,
      )
      result = self.update(controller, config, context, v_ego=v_ego, **gates)
    return result

  def test_high_confidence_left_and_right_corrections_are_symmetric(self):
    outputs = []
    for offset in (-0.30, 0.30):
      controller = ToadPilotLaneCentering()
      outputs.append(self.run_frames(controller, lane_offset_m=offset))
      self.assertTrue(controller.snapshot()["lane_centering_active"])

    self.assertLess(outputs[0] * outputs[1], 0.0)
    self.assertAlmostEqual(abs(outputs[0]), abs(outputs[1]), places=9)

  def test_persistence_advances_only_on_new_model_frames(self):
    controller = ToadPilotLaneCentering()
    config = centering_config()
    same_context = extract_toadpilot_lane_context(lane_model(frame_id=77), v_ego=20.0)

    for _ in range(TOADPILOT_LANE_CENTER_CONFIRM_SAMPLES * 3):
      self.assertEqual(self.update(controller, config, same_context), 0.0)
    self.assertFalse(controller.snapshot()["lane_centering_active"])

    result = self.run_frames(
      controller,
      config=config,
      start_frame=78,
      frames=TOADPILOT_LANE_CENTER_CONFIRM_SAMPLES + 2,
    )
    self.assertNotEqual(result, 0.0)
    self.assertTrue(controller.snapshot()["lane_centering_active"])

  def test_invalid_lane_evidence_and_path_bias_never_activate(self):
    bad_confidence = ToadPilotLaneCentering()
    config = centering_config()
    for frame_id in range(1, TOADPILOT_LANE_CENTER_CONFIRM_SAMPLES + 20):
      context = extract_toadpilot_lane_context(
        lane_model(frame_id=frame_id, lane_probability=0.20),
        v_ego=20.0,
      )
      self.assertEqual(self.update(bad_confidence, config, context), 0.0)
    self.assertFalse(bad_confidence.snapshot()["lane_centering_active"])

    path_biased = ToadPilotLaneCentering()
    self.assertEqual(self.run_frames(path_biased, path_lane_bias_m=0.30), 0.0)
    self.assertFalse(path_biased.snapshot()["lane_centering_active"])

  def test_model_path_and_existing_curve_must_agree_with_recovery_direction(self):
    path_opposes_recovery = ToadPilotLaneCentering()
    self.assertEqual(
      self.run_frames(path_opposes_recovery, lane_offset_m=0.12, path_lane_bias_m=-0.15),
      0.0,
    )
    self.assertFalse(path_opposes_recovery.snapshot()["lane_centering_active"])

    base_curve_opposes_recovery = ToadPilotLaneCentering()
    opposing_base_curvature = -0.0005  # -0.20 m/s^2 at 20 m/s: meaningful but still gentle.
    self.assertEqual(
      self.run_frames(
        base_curve_opposes_recovery,
        lane_offset_m=0.30,
        base_curvature=opposing_base_curvature,
      ),
      opposing_base_curvature,
    )
    self.assertFalse(base_curve_opposes_recovery.snapshot()["lane_centering_active"])

  def test_soft_planner_gate_monotonically_fades_then_clears_active_trim(self):
    controller = ToadPilotLaneCentering()
    config = centering_config()
    active_output = self.run_frames(controller, lane_offset_m=0.30, frames=120)
    active_correction = controller.correction_curvature
    self.assertGreater(active_output, 0.0)
    self.assertGreater(active_correction, 0.0)

    context = extract_toadpilot_lane_context(
      lane_model(frame_id=10_000, lane_offset_m=0.12, path_lane_bias_m=-0.15),
      v_ego=20.0,
    )
    first_output = self.update(
      controller,
      config,
      context,
    )
    self.assertGreater(controller.correction_curvature, 0.0)
    self.assertLess(controller.correction_curvature, active_correction)
    self.assertGreater(first_output, 0.0)
    self.assertFalse(controller.snapshot()["lane_centering_active"])
    self.assertEqual(controller.snapshot()["lane_centering_gate_reason"], "fading_planner_direction")

    previous = controller.correction_curvature
    for _ in range(TOADPILOT_LANE_SOFT_FADE_MAX_UPDATES):
      output = self.update(
        controller,
        config,
        context,
      )
      self.assertLessEqual(abs(controller.correction_curvature), abs(previous))
      previous = controller.correction_curvature

    self.assertEqual(output, 0.0)
    self.assertEqual(controller.correction_curvature, 0.0)

  def test_transition_smoothing_toggle_restores_immediate_soft_gate(self):
    controller = ToadPilotLaneCentering()
    config = replace(centering_config(), smooth_lane_transitions=False)
    self.assertGreater(self.run_frames(controller, config=config, frames=120), 0.0)

    context = extract_toadpilot_lane_context(
      lane_model(frame_id=10_000, lane_offset_m=0.12, path_lane_bias_m=-0.15),
      v_ego=20.0,
    )
    self.assertEqual(
      self.update(controller, config, context),
      0.0,
    )
    self.assertEqual(controller.correction_curvature, 0.0)

  def test_alternating_valid_and_soft_frames_cannot_rearm_fade_budget(self):
    controller = ToadPilotLaneCentering()
    config = centering_config()
    self.assertGreater(self.run_frames(controller, frames=120), 0.0)
    previous = controller.correction_curvature

    for update_index in range(TOADPILOT_LANE_SOFT_FADE_MAX_UPDATES):
      if update_index % 2:
        model = lane_model(frame_id=20_000 + update_index, lane_offset_m=0.30)
      else:
        model = lane_model(frame_id=20_000 + update_index, lane_offset_m=0.12, path_lane_bias_m=-0.15)
      context = extract_toadpilot_lane_context(model, v_ego=20.0)
      self.update(controller, config, context)
      self.assertLessEqual(abs(controller.correction_curvature), abs(previous))
      previous = controller.correction_curvature

    self.assertEqual(controller.correction_curvature, 0.0)
    self.assertEqual(controller.confirm_samples, 0)

  def test_curve_direction_disagreement_is_immediate_and_cannot_reverse_base_request(self):
    controller = ToadPilotLaneCentering()
    self.assertLess(self.run_frames(controller, lane_offset_m=-0.30, frames=120), 0.0)
    context = extract_toadpilot_lane_context(
      lane_model(frame_id=30_000, lane_offset_m=-0.30),
      v_ego=20.0,
    )
    base_curvature = 0.00017  # +0.068 m/s^2: meaningful and opposite the old trim.

    output = self.update(controller, centering_config(), context, base_curvature=base_curvature)

    self.assertEqual(output, base_curvature)
    self.assertGreater(output, 0.0)
    self.assertEqual(controller.correction_curvature, 0.0)
    self.assertEqual(controller.snapshot()["lane_centering_gate_reason"], "curve_direction")

  def test_hard_gate_interrupting_invalid_evidence_fade_flushes_stale_geometry(self):
    controller = ToadPilotLaneCentering()
    config = centering_config()
    self.assertGreater(self.run_frames(controller, frames=120), 0.0)
    invalid_context = extract_toadpilot_lane_context(
      lane_model(frame_id=40_000, lane_probability=0.20),
      v_ego=20.0,
    )
    self.assertGreater(self.update(controller, config, invalid_context), 0.0)
    self.assertTrue(controller.soft_gate_clear_filter)

    opposing_context = extract_toadpilot_lane_context(
      lane_model(frame_id=40_001, lane_offset_m=0.30),
      v_ego=20.0,
    )
    base_curvature = -0.00017
    self.assertEqual(
      self.update(controller, config, opposing_context, base_curvature=base_curvature),
      base_curvature,
    )
    self.assertFalse(controller.filter_initialized)
    self.assertIsNone(controller.snapshot()["lane_offset_m"])

  def test_low_confidence_can_only_fade_an_existing_trim(self):
    controller = ToadPilotLaneCentering()
    config = centering_config()
    self.assertGreater(self.run_frames(controller, frames=120), 0.0)
    previous = controller.correction_curvature
    bad_context = extract_toadpilot_lane_context(
      lane_model(frame_id=10_000, lane_probability=0.20),
      v_ego=20.0,
    )

    output = self.update(controller, config, bad_context)

    self.assertGreater(output, 0.0)
    self.assertGreater(controller.correction_curvature, 0.0)
    self.assertLess(controller.correction_curvature, previous)
    self.assertEqual(controller.snapshot()["lane_centering_gate_reason"], "fading_lane_confidence_low")

  def test_planner_bias_and_strong_curve_remain_immediate_gates(self):
    cases = (
      (0.0, 0.30, "planner_bias"),
      (0.002, 0.0, "curve_too_strong"),
    )
    for base_curvature, path_lane_bias, expected_reason in cases:
      with self.subTest(expected_reason=expected_reason):
        controller = ToadPilotLaneCentering()
        self.assertGreater(self.run_frames(controller, frames=120), 0.0)
        context = extract_toadpilot_lane_context(
          lane_model(frame_id=10_000, path_lane_bias_m=path_lane_bias),
          v_ego=20.0,
        )

        output = self.update(controller, centering_config(), context, base_curvature=base_curvature)

        self.assertEqual(output, base_curvature)
        self.assertEqual(controller.correction_curvature, 0.0)
        self.assertEqual(controller.snapshot()["lane_centering_gate_reason"], expected_reason)

  def test_offset_reversal_never_carries_old_direction_correction(self):
    controller = ToadPilotLaneCentering()
    self.assertGreater(self.run_frames(controller, lane_offset_m=0.30), 0.0)

    reversed_context = extract_toadpilot_lane_context(
      lane_model(frame_id=20_000, lane_offset_m=-0.30),
      v_ego=20.0,
    )
    self.assertEqual(self.update(controller, centering_config(), reversed_context), 0.0)
    self.assertFalse(controller.snapshot()["lane_centering_active"])

    self.assertLess(
      self.run_frames(controller, lane_offset_m=-0.30, start_frame=20_001),
      0.0,
    )

  def test_dynamic_safety_gates_drop_active_correction_immediately(self):
    gate_cases = (
      {"lane_change_active": True},
      {"blinker_active": True},
      {"steering_pressed": True},
      {"steer_limited": True},
      {"steering_rate_deg": 80.0},
      {"lateral_active": False},
      {"a_ego": -0.76},
      {"brake_pressed": True},
      {"longitudinal_stopping": True},
    )

    for gates in gate_cases:
      with self.subTest(gates=gates):
        controller = ToadPilotLaneCentering()
        self.assertNotEqual(self.run_frames(controller), 0.0)
        context = extract_toadpilot_lane_context(lane_model(frame_id=10_000), v_ego=20.0)
        self.assertEqual(self.update(controller, centering_config(), context, **gates), 0.0)
        self.assertFalse(controller.snapshot()["lane_centering_active"])

  def test_forced_stop_alert_does_not_show_stale_distance_at_standstill(self):
    sm = {"frogpilotPlan": SimpleNamespace(forcingStopLength=6.18744)}
    moving = forcing_stop_alert(None, SimpleNamespace(standstill=False), sm, False, 0, SimpleNamespace())
    stopped = forcing_stop_alert(None, SimpleNamespace(standstill=True), sm, False, 0, SimpleNamespace())
    self.assertEqual(moving.alert_text_1, "Forcing the car to stop in 20.3 feet")
    self.assertEqual(stopped.alert_text_1, "Holding at forced stop")

  def test_transition_gates_require_fresh_geometry_before_reentry(self):
    for gate in ({"lane_change_active": True}, {"longitudinal_stopping": True}):
      with self.subTest(gate=gate):
        controller = ToadPilotLaneCentering()
        self.assertGreater(self.run_frames(controller), 0.0)

        gated_context = extract_toadpilot_lane_context(lane_model(frame_id=10_000), v_ego=20.0)
        self.assertEqual(self.update(controller, centering_config(), gated_context, **gate), 0.0)

        for sample in range(TOADPILOT_LANE_CENTER_CONFIRM_SAMPLES - 1):
          context = extract_toadpilot_lane_context(lane_model(frame_id=10_001 + sample), v_ego=20.0)
          self.assertEqual(self.update(controller, centering_config(), context), 0.0)
          self.assertFalse(controller.snapshot()["lane_centering_active"])

        context = extract_toadpilot_lane_context(
          lane_model(frame_id=10_000 + TOADPILOT_LANE_CENTER_CONFIRM_SAMPLES),
          v_ego=20.0,
        )
        self.assertGreater(self.update(controller, centering_config(), context), 0.0)
        self.assertTrue(controller.snapshot()["lane_centering_active"])

  def test_slowing_below_operating_range_drops_centering_and_clears_history(self):
    controller = ToadPilotLaneCentering()
    self.assertNotEqual(self.run_frames(controller), 0.0)
    context = extract_toadpilot_lane_context(lane_model(frame_id=10_001), v_ego=4.9)

    self.assertEqual(
      self.update(controller, centering_config(), context, v_ego=4.9),
      0.0,
    )
    status = controller.snapshot()
    self.assertFalse(status["lane_centering_active"])
    self.assertEqual(status["lane_centering_gate_reason"], "speed_low")
    self.assertIsNone(status["lane_offset_m"])

  def test_low_speed_curvature_cap_does_not_grow_while_slowing(self):
    speed = 12.5
    config = centering_config(profile="full")
    controller = ToadPilotLaneCentering()
    correction = self.run_frames(
      controller,
      config=config,
      lane_offset_m=0.90,
      frames=250,
      v_ego=speed,
    )
    fixed_cap = TOADPILOT_PROFILES["full"].centering_max_accel / 15.0 ** 2
    self.assertLessEqual(abs(correction), fixed_cap + 1e-12)

  def test_low_speed_launch_blend_is_smooth_and_bounded(self):
    outputs = []
    for speed in (5.0, 8.5, 12.0):
      controller = ToadPilotLaneCentering()
      outputs.append(self.run_frames(controller, v_ego=speed, frames=120))

    self.assertEqual(outputs[0], 0.0)
    self.assertGreater(outputs[1], 0.0)
    self.assertGreater(outputs[2], outputs[1])

  def test_normal_rate_limit_does_not_masquerade_as_saturation(self):
    self.assertFalse(toadpilot_steer_saturated(True, 0.25))
    self.assertTrue(toadpilot_steer_saturated(True, 0.80))
    self.assertTrue(toadpilot_steer_saturated(True, -0.95))
    self.assertFalse(toadpilot_steer_saturated(False, 1.0))

  def test_experimental_mode_centering_is_independent_of_curve_opt_in(self):
    default_config = ToadPilotLaneCentering()
    default_result = self.run_frames(default_config, experimental_mode=True)

    opted_in = ToadPilotLaneCentering()
    opted_in_result = self.run_frames(
      opted_in,
      config=centering_config(experimental_opt_in=True),
      experimental_mode=True,
    )
    self.assertNotEqual(default_result, 0.0)
    self.assertAlmostEqual(default_result, opted_in_result)
    self.assertTrue(default_config.snapshot()["lane_centering_active"])

  def test_observe_disabled_and_lane_toggle_off_are_output_neutral(self):
    configs = (
      centering_config(profile="observe"),
      replace(centering_config(), enabled=False),
      replace(centering_config(), lane_centering_assist=False),
    )
    for config in configs:
      with self.subTest(config=config):
        controller = ToadPilotLaneCentering()
        self.assertEqual(self.run_frames(controller, config=config), 0.0)
        self.assertFalse(controller.snapshot()["lane_centering_active"])

  def test_curvature_correction_stays_inside_profile_lateral_accel_cap(self):
    v_ego = 20.0
    config = centering_config(profile="full")
    controller = ToadPilotLaneCentering()
    correction = self.run_frames(
      controller,
      config=config,
      lane_offset_m=0.90,
      frames=250,
      v_ego=v_ego,
    )
    curvature_cap = TOADPILOT_PROFILES["full"].centering_max_accel / v_ego ** 2

    self.assertLessEqual(abs(correction), curvature_cap + 1e-12)
    self.assertGreater(abs(correction), 0.95 * curvature_cap)


class CamryFixture(unittest.TestCase):
  def setUp(self):
    self.params_dir = TemporaryDirectory()
    self.params = Params(self.params_dir.name)
    self.CP = CarInterface.get_params(
      CAR.TOYOTA_CAMRY_TSS2,
      gen_empty_fingerprint(),
      [],
      False,
      False,
      self.params,
      False,
    )

  def tearDown(self):
    self.params_dir.cleanup()


class TestCamryLiveTorqueSafety(CamryFixture):
  def setUp(self):
    super().setUp()
    self.controller = LatControlTorque(self.CP, LinearTorqueInterface())
    self.stock_factor = self.controller.stock_lat_accel_factor
    self.stock_offset = self.controller.stock_lat_accel_offset
    self.stock_friction = self.controller.stock_friction

  def assert_tune(self, factor, offset, friction):
    torque = self.controller.torque_params
    self.assertAlmostEqual(torque.latAccelFactor, factor, places=6)
    self.assertAlmostEqual(torque.latAccelOffset, offset, places=6)
    self.assertAlmostEqual(torque.friction, friction, places=6)

  def test_auto_tune_switches_atomically_between_live_and_stock(self):
    live_factor = self.stock_factor * 1.1
    live_offset = 0.2
    live_friction = self.stock_friction * 1.1

    self.controller.update_live_torque_params(
      live_factor, live_offset, live_friction, toad_toggles(auto_tune=True),
    )
    self.assert_tune(live_factor, live_offset, live_friction)

    self.controller.update_live_torque_params(
      live_factor, live_offset, live_friction, toad_toggles(auto_tune=False),
    )
    self.assert_tune(self.stock_factor, self.stock_offset, self.stock_friction)

    # Observe and disabled retain the branch baseline tune; only residual demand
    # shaping is output-neutral in Observe.
    for toggles in (
      toad_toggles(profile="observe", auto_tune=False),
      toad_toggles(enabled=False, auto_tune=False),
    ):
      with self.subTest(toggles=toggles):
        self.controller.update_live_torque_params(live_factor, live_offset, live_friction, toggles)
        self.assert_tune(live_factor, live_offset, live_friction)

  def test_live_parameters_are_sanitized_to_camry_learner_bounds(self):
    min_factor = self.stock_factor * (1.0 - FACTOR_SANITY)
    max_factor = self.stock_factor * (1.0 + FACTOR_SANITY)
    min_friction = self.stock_friction * (1.0 - FRICTION_SANITY)
    max_friction = self.stock_friction * (1.0 + FRICTION_SANITY)

    cases = (
      (-1e6, -1e6, -1e6, min_factor, -TOADPILOT_LAT_ACCEL_OFFSET_MAX, min_friction),
      (1e6, 1e6, 1e6, max_factor, TOADPILOT_LAT_ACCEL_OFFSET_MAX, max_friction),
      (math.nan, math.nan, math.nan, self.stock_factor, self.stock_offset, self.stock_friction),
      (-math.inf, -math.inf, -math.inf, self.stock_factor, self.stock_offset, self.stock_friction),
      (math.inf, math.inf, math.inf, self.stock_factor, self.stock_offset, self.stock_friction),
    )
    for factor, offset, friction, expected_factor, expected_offset, expected_friction in cases:
      with self.subTest(factor=factor, offset=offset, friction=friction):
        self.controller.update_live_torque_params(
          factor, offset, friction, toad_toggles(auto_tune=True),
        )
        self.assert_tune(expected_factor, expected_offset, expected_friction)
        self.assertTrue(math.isfinite(self.controller.live_lat_accel_factor))
        self.assertTrue(math.isfinite(self.controller.live_lat_accel_offset))
        self.assertTrue(math.isfinite(self.controller.live_friction))

  def test_branch_baseline_preserves_manual_tune_until_adaptive_auto_tune(self):
    manual_factor = self.stock_factor * 0.55
    manual_offset = 0.3
    manual_friction = self.stock_friction * 1.7
    adaptive_factor = self.stock_factor * (1.0 - FACTOR_SANITY)
    adaptive_friction = self.stock_friction * (1.0 + FRICTION_SANITY)

    for baseline_toggles in (
      toad_toggles(enabled=False, auto_tune=False),
      toad_toggles(profile="observe", auto_tune=False),
    ):
      with self.subTest(toggles=baseline_toggles):
        fresh_cp = CarInterface.get_params(
          CAR.TOYOTA_CAMRY_TSS2,
          gen_empty_fingerprint(),
          [],
          False,
          False,
          self.params,
          False,
        )
        controller = LatControlTorque(fresh_cp, LinearTorqueInterface())
        controller.update_live_torque_params(
          manual_factor, manual_offset, manual_friction, baseline_toggles,
        )
        torque = controller.torque_params
        self.assertAlmostEqual(torque.latAccelFactor, manual_factor, places=6)
        self.assertAlmostEqual(torque.latAccelOffset, manual_offset, places=6)
        self.assertAlmostEqual(torque.friction, manual_friction, places=6)

        controller.update_live_torque_params(
          manual_factor, manual_offset, manual_friction, toad_toggles(auto_tune=True),
        )
        torque = controller.torque_params
        self.assertAlmostEqual(torque.latAccelFactor, adaptive_factor, places=6)
        self.assertAlmostEqual(torque.latAccelOffset, manual_offset, places=6)
        self.assertAlmostEqual(torque.friction, adaptive_friction, places=6)


class TestTorqueEstimatorReset(CamryFixture):
  def test_reset_to_offline_clears_learned_state_and_observer(self):
    with patch("openpilot.selfdrive.locationd.torqued.Params", return_value=self.params):
      estimator = TorqueEstimator(self.CP)

    estimator.filtered_params["latAccelFactor"].x = estimator.offline_latAccelFactor * 1.2
    estimator.filtered_params["latAccelOffset"].x = 0.4
    estimator.filtered_params["frictionCoefficient"].x = estimator.offline_friction * 1.2
    estimator.filtered_points.add_point(0.2, 0.4)
    estimator.raw_points["active"].append(True)
    self.assertTrue(estimator.toadpilot_observer.add_sample(0.75, 1.5))

    estimator.reset_to_offline()

    self.assertAlmostEqual(estimator.filtered_params["latAccelFactor"].x, estimator.offline_latAccelFactor)
    self.assertEqual(estimator.filtered_params["latAccelOffset"].x, 0.0)
    self.assertAlmostEqual(estimator.filtered_params["frictionCoefficient"].x, estimator.offline_friction)
    self.assertEqual(len(estimator.filtered_points), 0)
    self.assertEqual(sum(len(values) for values in estimator.raw_points.values()), 0)
    self.assertEqual(estimator.toadpilot_observer.accepted_samples, 0)

  def test_invalid_v2_cache_decay_removes_cache_and_falls_back_offline(self):
    self.assertEqual(VERSION, 2)
    cache = messaging.new_message("liveTorqueParameters")
    cache.valid = True
    cache_ltp = cache.liveTorqueParameters
    cache_ltp.version = 2
    cache_ltp.liveValid = True
    cache_ltp.latAccelFactorFiltered = self.CP.lateralTuning.torque.latAccelFactor * 1.1
    cache_ltp.latAccelOffsetFiltered = 0.2
    cache_ltp.frictionCoefficientFiltered = self.CP.lateralTuning.torque.friction * 1.1
    cache_ltp.points = [[0.1, 0.2], [-0.1, -0.2]]
    cache_ltp.decay = MAX_FILTER_DECAY + 1.0
    self.params.put("CarParamsPrevRoute", self.CP.to_bytes())
    self.params.put("LiveTorqueParameters", cache.to_bytes())

    with patch("openpilot.selfdrive.locationd.torqued.Params", return_value=self.params), \
         patch("openpilot.selfdrive.locationd.torqued.cloudlog.exception") as log_exception:
      estimator = TorqueEstimator(self.CP)
    log_exception.assert_called_once()

    self.assertIsNone(self.params.get("LiveTorqueParameters"))
    self.assertAlmostEqual(estimator.filtered_params["latAccelFactor"].x, estimator.offline_latAccelFactor)
    self.assertEqual(estimator.filtered_params["latAccelOffset"].x, 0.0)
    self.assertAlmostEqual(estimator.filtered_params["frictionCoefficient"].x, estimator.offline_friction)
    self.assertEqual(estimator.decay, MIN_FILTER_DECAY)
    self.assertEqual(len(estimator.filtered_points), 0)

  def test_consume_reset_clears_durable_state_and_resets_estimator(self):
    with patch("openpilot.selfdrive.locationd.torqued.Params", return_value=self.params):
      estimator = TorqueEstimator(self.CP)

    estimator.filtered_params["latAccelFactor"].x = estimator.offline_latAccelFactor * 1.2
    estimator.filtered_params["latAccelOffset"].x = 0.4
    estimator.filtered_params["frictionCoefficient"].x = estimator.offline_friction * 1.2
    estimator.filtered_points.add_point(0.2, 0.4)
    self.params.put("LiveTorqueParameters", b"cached")
    self.params.put_bool(TOADPILOT_RESET_TUNE_KEY, True)

    with TemporaryDirectory() as status_path:
      status_params = Params(status_path)
      status_params.put(TOADPILOT_TUNE_STATUS_KEY, b'{"stale":true}')
      with patch.object(estimator, "reset_to_offline", wraps=estimator.reset_to_offline) as reset:
        self.assertTrue(_consume_toadpilot_reset(estimator, self.params, status_params))
      reset.assert_called_once_with()
      self.assertIsNone(status_params.get(TOADPILOT_TUNE_STATUS_KEY))

    self.assertIsNone(self.params.get("LiveTorqueParameters"))
    self.assertFalse(self.params.get_bool(TOADPILOT_RESET_TUNE_KEY))
    self.assertAlmostEqual(estimator.filtered_params["latAccelFactor"].x, estimator.offline_latAccelFactor)
    self.assertEqual(estimator.filtered_params["latAccelOffset"].x, 0.0)
    self.assertAlmostEqual(estimator.filtered_params["frictionCoefficient"].x, estimator.offline_friction)
    self.assertEqual(estimator.decay, MIN_FILTER_DECAY)
    self.assertEqual(len(estimator.filtered_points), 0)

  def test_v2_cache_restores_float32_min_and_max_bounds(self):
    self.assertEqual(VERSION, 2)
    offline_factor = float(self.CP.lateralTuning.torque.latAccelFactor)
    offline_friction = float(self.CP.lateralTuning.torque.friction)
    min_factor = (1.0 - FACTOR_SANITY) * offline_factor
    max_factor = (1.0 + FACTOR_SANITY) * offline_factor
    min_friction = (1.0 - FRICTION_SANITY) * offline_friction
    max_friction = (1.0 + FRICTION_SANITY) * offline_friction
    car_params_bytes = self.CP.to_bytes()
    cases = (
      ("minimum", min_factor, min_friction),
      ("maximum", max_factor, max_friction),
    )

    for name, factor_bound, friction_bound in cases:
      with self.subTest(bound=name):
        cache = messaging.new_message("liveTorqueParameters")
        cache.valid = True
        cache_ltp = cache.liveTorqueParameters
        cache_ltp.version = VERSION
        cache_ltp.liveValid = True
        cache_ltp.latAccelFactorFiltered = factor_bound
        cache_ltp.latAccelOffsetFiltered = 0.2
        cache_ltp.frictionCoefficientFiltered = friction_bound
        cache_ltp.points = [[0.1, 0.2], [-0.1, -0.2]]
        cache_ltp.decay = MIN_FILTER_DECAY + 25.0

        wire_factor = float(cache_ltp.latAccelFactorFiltered)
        wire_friction = float(cache_ltp.frictionCoefficientFiltered)
        expected_factor = float(np.clip(wire_factor, min_factor, max_factor))
        expected_friction = float(np.clip(wire_friction, min_friction, max_friction))
        if name == "minimum":
          self.assertLess(wire_factor, factor_bound)
        else:
          self.assertGreater(wire_factor, factor_bound)

        self.params.put("CarParamsPrevRoute", car_params_bytes)
        self.params.put("LiveTorqueParameters", cache.to_bytes())
        with patch("openpilot.selfdrive.locationd.torqued.Params", return_value=self.params), \
             patch("openpilot.selfdrive.locationd.torqued.cloudlog.exception") as log_exception:
          estimator = TorqueEstimator(self.CP)
        log_exception.assert_not_called()

        self.assertIsNotNone(self.params.get("LiveTorqueParameters"))
        self.assertAlmostEqual(estimator.filtered_params["latAccelFactor"].x, expected_factor, places=12)
        self.assertAlmostEqual(estimator.filtered_params["latAccelOffset"].x, 0.2, places=6)
        self.assertAlmostEqual(estimator.filtered_params["frictionCoefficient"].x, expected_friction, places=12)
        self.assertEqual(estimator.decay, MIN_FILTER_DECAY + 25.0)
        self.assertEqual(len(estimator.filtered_points), 2)

  def test_non_toad_decimated_cache_roundtrip_uses_full_bounds(self):
    with patch("openpilot.selfdrive.car.hyundai.interface.params", self.params):
      hyundai_cp = HyundaiCarInterface.get_params(
        HYUNDAI_CAR.HYUNDAI_ELANTRA_2021,
        gen_empty_fingerprint(),
        [],
        False,
        False,
        self.params,
        False,
      )

    with patch("openpilot.selfdrive.locationd.torqued.Params", return_value=self.params):
      source = TorqueEstimator(hyundai_cp, decimated=True)

    self.assertFalse(source.toadpilot_capable)
    self.assertEqual(source.factor_sanity, FACTOR_SANITY_QLOG)
    self.assertEqual(source.friction_sanity, FRICTION_SANITY_QLOG)
    full_min_factor = source.offline_latAccelFactor * (1.0 - FACTOR_SANITY)
    full_max_factor = source.offline_latAccelFactor * (1.0 + FACTOR_SANITY)
    full_min_friction = source.offline_friction * (1.0 - FRICTION_SANITY)
    full_max_friction = source.offline_friction * (1.0 + FRICTION_SANITY)
    qlog_max_factor = source.offline_latAccelFactor * (1.0 + FACTOR_SANITY_QLOG)
    qlog_max_friction = source.offline_friction * (1.0 + FRICTION_SANITY_QLOG)
    self.assertAlmostEqual(source.cache_min_lataccel_factor, full_min_factor, places=12)
    self.assertAlmostEqual(source.cache_max_lataccel_factor, full_max_factor, places=12)
    self.assertAlmostEqual(source.cache_min_friction, full_min_friction, places=12)
    self.assertAlmostEqual(source.cache_max_friction, full_max_friction, places=12)
    self.assertGreater(qlog_max_factor, full_max_factor)
    self.assertGreater(qlog_max_friction, full_max_friction)

    source.filtered_params["latAccelFactor"].x = qlog_max_factor
    source.filtered_params["latAccelOffset"].x = 0.2
    source.filtered_params["frictionCoefficient"].x = qlog_max_friction
    cache = source.get_msg(with_points=True, frogpilot_toggles=toad_toggles(enabled=False))
    cache_ltp = cache.liveTorqueParameters
    cache_ltp.liveValid = True
    published_factor = float(cache_ltp.latAccelFactorFiltered)
    published_friction = float(cache_ltp.frictionCoefficientFiltered)

    self.assertLess(published_factor, qlog_max_factor)
    self.assertLess(published_friction, qlog_max_friction)
    self.assertAlmostEqual(published_factor, full_max_factor, places=6)
    self.assertAlmostEqual(published_friction, full_max_friction, places=6)

    self.params.put("CarParamsPrevRoute", hyundai_cp.to_bytes())
    self.params.put("LiveTorqueParameters", cache.to_bytes())
    with patch("openpilot.selfdrive.locationd.torqued.Params", return_value=self.params), \
         patch("openpilot.selfdrive.locationd.torqued.cloudlog.exception") as log_exception:
      restored = TorqueEstimator(hyundai_cp)
    log_exception.assert_not_called()

    expected_factor = float(np.clip(published_factor, full_min_factor, full_max_factor))
    expected_friction = float(np.clip(published_friction, full_min_friction, full_max_friction))
    self.assertIsNotNone(self.params.get("LiveTorqueParameters"))
    self.assertEqual(restored.factor_sanity, FACTOR_SANITY)
    self.assertEqual(restored.friction_sanity, FRICTION_SANITY)
    self.assertAlmostEqual(restored.cache_min_lataccel_factor, full_min_factor, places=12)
    self.assertAlmostEqual(restored.cache_max_lataccel_factor, full_max_factor, places=12)
    self.assertAlmostEqual(restored.cache_min_friction, full_min_friction, places=12)
    self.assertAlmostEqual(restored.cache_max_friction, full_max_friction, places=12)
    self.assertAlmostEqual(restored.filtered_params["latAccelFactor"].x, expected_factor, places=12)
    self.assertAlmostEqual(restored.filtered_params["latAccelOffset"].x, 0.2, places=6)
    self.assertAlmostEqual(restored.filtered_params["frictionCoefficient"].x, expected_friction, places=12)


class TestObserverExactAuthorityBounds(unittest.TestCase):
  def test_exact_authority_endpoints_match_published_contract(self):
    observer = ToadPilotHighAuthorityObserver()

    self.assertFalse(observer.add_sample(0.5, 1.0))
    self.assertTrue(observer.add_sample(np.nextafter(0.5, math.inf), 1.0))
    self.assertTrue(observer.add_sample(1.0, 2.0))
    self.assertFalse(observer.add_sample(np.nextafter(1.0, math.inf), 2.0))

    snapshot = observer.snapshot()
    self.assertEqual(snapshot["acceptance"]["absolute_steer"], "0.50 < steer <= 1.00")
    self.assertEqual(snapshot["candidate_samples"], 2)
    self.assertEqual(snapshot["accepted_samples"], 2)
    self.assertEqual(snapshot["rejected_samples"], 0)


if __name__ == "__main__":
  unittest.main()
