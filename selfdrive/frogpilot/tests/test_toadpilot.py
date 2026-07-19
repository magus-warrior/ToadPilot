import json
import unittest

from dataclasses import replace
from tempfile import TemporaryDirectory
from types import SimpleNamespace

from openpilot.common.params import Params, UnknownKeyName
from openpilot.selfdrive.frogpilot.toadpilot import (DEFAULT_TOADPILOT_CONFIG, TOADPILOT_CONFIG_KEY,
                                                     TOADPILOT_PROFILES, TOADPILOT_STEERING_RESPONSE_LATCH_ENV,
                                                     TOADPILOT_STEERING_RESPONSE_BUILD,
                                                     TOADPILOT_STEERING_RESPONSE_PANDA_SIGNATURES,
                                                     TOADPILOT_STEERING_RESPONSE_PROFILE,
                                                     TOADPILOT_STEERING_RESPONSE_REQUEST_KEY,
                                                     TOADPILOT_TORQUE_CHECKPOINT_KEY,
                                                     TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION,
                                                     TOADPILOT_TORQUE_LAST_ROUTE_KEY,
                                                     TOADPILOT_TORQUE_STATUS_KEY,
                                                     ToadPilotConfig,
                                                     ToadPilotHighAuthorityObserver, ToadPilotLateralAssist,
                                                     config_from_toggles, decode_toadpilot_config,
                                                     latch_toadpilot_steering_response_request,
                                                     normalize_toadpilot_config, read_toadpilot_config,
                                                     read_toadpilot_steering_response_latch,
                                                     read_toadpilot_steering_response_profile,
                                                     read_toadpilot_steering_response_request, write_toadpilot_config,
                                                     write_toadpilot_steering_response_request,
                                                     persist_toadpilot_torque_last_route,
                                                     toadpilot_steering_response_panda_matched,
                                                     valid_toadpilot_torque_summary)


def run_assist(assist, config, **overrides):
  inputs = {
    "desired_curvature": 0.002,
    "desired_lateral_accel": 2.0,
    "actual_lateral_accel": 0.0,
    "steering_pressed": False,
    "steering_rate_deg": 0.0,
    "steer_limited": False,
    "experimental_mode": False,
  }
  inputs.update(overrides)
  return assist.update(config, **inputs)


class TestToadPilotConfig(unittest.TestCase):
  def test_new_install_defaults_enable_guarded_assists(self):
    self.assertEqual(DEFAULT_TOADPILOT_CONFIG, ToadPilotConfig(
      enabled=True,
      profile="adaptive",
      auto_tune=True,
      lane_centering_assist=True,
      smooth_lane_transitions=True,
      curve_assist_in_experimental=False,
      smart_saturation_warning=True,
      telemetry=True,
    ))

  def test_normalizes_valid_config(self):
    config = normalize_toadpilot_config({
      "version": 3,
      "enabled": True,
      "profile": "full",
      "auto_tune": False,
      "lane_centering_assist": True,
      "smooth_lane_transitions": False,
      "curve_assist_in_experimental": True,
      "smart_saturation_warning": False,
      "telemetry": False,
    })

    self.assertEqual(config, ToadPilotConfig(
      enabled=True,
      profile="full",
      auto_tune=False,
      lane_centering_assist=True,
      smooth_lane_transitions=False,
      curve_assist_in_experimental=True,
      smart_saturation_warning=False,
      telemetry=False,
    ))

  def test_partial_update_preserves_base(self):
    base = ToadPilotConfig(enabled=True, profile="adaptive", auto_tune=False, telemetry=False)
    config = normalize_toadpilot_config({"profile": "full"}, base=base, partial=True)

    self.assertEqual(config, replace(base, profile="full"))

  def test_rejects_unknown_or_mistyped_values(self):
    invalid_configs = (
      {"not_a_setting": True},
      {"profile": "maximum"},
      {"profile": []},
      {"profile": {}},
      {"enabled": 1},
      {"auto_tune": "true"},
      {"smooth_lane_transitions": 1},
      {"version": 4},
    )

    for config in invalid_configs:
      with self.subTest(config=config), self.assertRaises(ValueError):
        normalize_toadpilot_config(config)

  def test_invalid_encoded_config_falls_back_to_safe_default(self):
    for raw_config in (b"not-json", b'{"enabled":true,"profile":"unknown"}'):
      with self.subTest(raw_config=raw_config):
        config = decode_toadpilot_config(raw_config)
        self.assertFalse(config.enabled)
        self.assertEqual(config.profile, "observe")
        self.assertFalse(config.lane_centering_assist)

  def test_exact_v1_migration_enables_new_defaults_and_preserves_other_choices(self):
    legacy = {
      "version": 1,
      "enabled": False,
      "profile": "observe",
      "auto_tune": False,
      "curve_assist_in_experimental": True,
      "smart_saturation_warning": False,
      "telemetry": False,
    }

    config = decode_toadpilot_config(json.dumps(legacy))

    self.assertEqual(config.version, 3)
    self.assertTrue(config.enabled)
    self.assertEqual(config.profile, "adaptive")
    self.assertTrue(config.lane_centering_assist)
    self.assertTrue(config.smooth_lane_transitions)
    for field in ("auto_tune", "curve_assist_in_experimental", "smart_saturation_warning", "telemetry"):
      self.assertEqual(getattr(config, field), legacy[field])

  def test_exact_v2_config_migrates_without_resetting_saved_choices(self):
    explicit = {
      "version": 2,
      "enabled": False,
      "profile": "full",
      "auto_tune": False,
      "lane_centering_assist": False,
      "curve_assist_in_experimental": True,
      "smart_saturation_warning": False,
      "telemetry": False,
    }

    expected = {
      **explicit,
      "version": 3,
      "smooth_lane_transitions": True,
    }
    self.assertEqual(decode_toadpilot_config(json.dumps(explicit)), ToadPilotConfig(**expected))

  def test_explicit_v3_config_is_preserved_exactly(self):
    explicit = {
      "version": 3,
      "enabled": False,
      "profile": "full",
      "auto_tune": False,
      "lane_centering_assist": False,
      "smooth_lane_transitions": False,
      "curve_assist_in_experimental": True,
      "smart_saturation_warning": False,
      "telemetry": False,
    }

    self.assertEqual(decode_toadpilot_config(json.dumps(explicit)), ToadPilotConfig(**explicit))

  def test_malformed_or_hybrid_v2_config_fails_closed(self):
    valid_v2 = {
      "version": 2,
      "enabled": True,
      "profile": "adaptive",
      "auto_tune": True,
      "lane_centering_assist": True,
      "curve_assist_in_experimental": False,
      "smart_saturation_warning": True,
      "telemetry": True,
    }
    malformed = (
      {key: value for key, value in valid_v2.items() if key != "telemetry"},
      {**valid_v2, "smooth_lane_transitions": True},
      {**valid_v2, "unexpected": False},
    )

    for raw_config in malformed:
      with self.subTest(raw_config=raw_config):
        config = decode_toadpilot_config(json.dumps(raw_config))
        self.assertFalse(config.enabled)
        self.assertEqual(config.profile, "observe")
        self.assertFalse(config.lane_centering_assist)
        self.assertFalse(config.smooth_lane_transitions)

  def test_malformed_shared_toggles_fall_back_safely(self):
    toggles = SimpleNamespace(
      toadpilot_enabled="false",
      toadpilot_profile=[],
      toadpilot_auto_tune="false",
      toadpilot_curve_assist_in_experimental=1,
    )
    config = config_from_toggles(toggles)
    self.assertFalse(config.enabled)
    self.assertEqual(config.profile, "observe")
    self.assertTrue(config.auto_tune)
    self.assertFalse(config.lane_centering_assist)
    self.assertFalse(config.smooth_lane_transitions)
    self.assertFalse(config.curve_assist_in_experimental)

  def test_incomplete_shared_toggle_snapshot_fails_closed_atomically(self):
    config = config_from_toggles(SimpleNamespace(
      toadpilot_enabled=True,
      toadpilot_profile="adaptive",
    ))

    self.assertFalse(config.enabled)
    self.assertEqual(config.profile, "observe")
    self.assertFalse(config.lane_centering_assist)
    self.assertFalse(config.smooth_lane_transitions)

  def test_params_roundtrip(self):
    expected = ToadPilotConfig(enabled=True, profile="adaptive", auto_tune=False, telemetry=False)

    with TemporaryDirectory() as params_path:
      params = Params(params_path)
      written = write_toadpilot_config(expected, params)

      self.assertEqual(written, expected)
      self.assertIsNotNone(params.get(TOADPILOT_CONFIG_KEY))
      self.assertEqual(read_toadpilot_config(params), expected)

      params.remove(TOADPILOT_CONFIG_KEY)
      self.assertEqual(read_toadpilot_config(params), DEFAULT_TOADPILOT_CONFIG)

  def test_steering_response_request_defaults_off_and_round_trips_separately(self):
    with TemporaryDirectory() as params_path:
      params = Params(params_path)
      self.assertFalse(read_toadpilot_steering_response_request(params))

      self.assertTrue(write_toadpilot_steering_response_request(True, params))
      self.assertTrue(read_toadpilot_steering_response_request(params))
      self.assertEqual(params.get(TOADPILOT_STEERING_RESPONSE_REQUEST_KEY, encoding="utf-8"),
                       TOADPILOT_STEERING_RESPONSE_PROFILE)
      self.assertEqual(read_toadpilot_config(params), DEFAULT_TOADPILOT_CONFIG)

      self.assertFalse(write_toadpilot_steering_response_request(False, params))
      self.assertFalse(read_toadpilot_steering_response_request(params))
      self.assertIsNone(params.get(TOADPILOT_STEERING_RESPONSE_REQUEST_KEY))

  def test_steering_response_request_rejects_non_boolean_writes_and_malformed_storage(self):
    with TemporaryDirectory() as params_path:
      params = Params(params_path)
      with self.assertRaisesRegex(ValueError, "must be a boolean"):
        write_toadpilot_steering_response_request(1, params)

      for malformed in ("true", "1", "camry_tss2_response_research_v0"):
        with self.subTest(malformed=malformed):
          params.put(TOADPILOT_STEERING_RESPONSE_REQUEST_KEY, malformed)
          self.assertFalse(read_toadpilot_steering_response_request(params))

  def test_steering_response_boot_latch_requires_exact_profile_and_is_immutable_until_relatch(self):
    with TemporaryDirectory() as params_path:
      params = Params(params_path)
      environ = {}

      params.put(TOADPILOT_STEERING_RESPONSE_REQUEST_KEY, "camry_tss2_response_research_v1")
      self.assertIsNone(latch_toadpilot_steering_response_request(params, environ))
      self.assertNotIn(TOADPILOT_STEERING_RESPONSE_LATCH_ENV, environ)
      self.assertFalse(read_toadpilot_steering_response_latch(environ))

      write_toadpilot_steering_response_request(True, params)
      self.assertEqual(read_toadpilot_steering_response_profile(params), TOADPILOT_STEERING_RESPONSE_PROFILE)
      # A persistent write alone cannot change the manager-lifetime latch.
      self.assertFalse(read_toadpilot_steering_response_latch(environ))

      self.assertEqual(latch_toadpilot_steering_response_request(params, environ),
                       TOADPILOT_STEERING_RESPONSE_PROFILE)
      self.assertTrue(read_toadpilot_steering_response_latch(environ))

      write_toadpilot_steering_response_request(False, params)
      self.assertTrue(read_toadpilot_steering_response_latch(environ))
      self.assertIsNone(read_toadpilot_steering_response_profile(params))

  def test_steering_response_requires_exact_single_panda_firmware_signature(self):
    with TemporaryDirectory() as params_path:
      params = Params(params_path)
      expected = next(iter(TOADPILOT_STEERING_RESPONSE_PANDA_SIGNATURES))

      self.assertFalse(toadpilot_steering_response_panda_matched(params))
      params.put("PandaSignatures", b"wrong")
      self.assertFalse(toadpilot_steering_response_panda_matched(params))
      params.put("PandaSignatures", expected + b"," + expected)
      self.assertFalse(toadpilot_steering_response_panda_matched(params))
      params.put("PandaSignatures", expected)
      self.assertTrue(toadpilot_steering_response_panda_matched(params))

  def test_internal_params_allowlist_rejects_malformed_and_unknown_keys(self):
    with TemporaryDirectory() as params_path:
      params = Params(params_path)
      self.assertEqual(params.check_key(TOADPILOT_CONFIG_KEY), TOADPILOT_CONFIG_KEY.encode())
      self.assertEqual(params.check_key(TOADPILOT_STEERING_RESPONSE_REQUEST_KEY),
                       TOADPILOT_STEERING_RESPONSE_REQUEST_KEY.encode())

      for key in (b"\xff", b"ToadPilotConfig\x00", "ToadPilotConfig\x00", "ToadPilotUnknown"):
        with self.subTest(key=key), self.assertRaises(UnknownKeyName):
          params.check_key(key)

  def test_completed_torque_summary_is_validated_and_persisted(self):
    summary = {
      "schema_version": TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION,
      "published_at_unix": 110.0,
      "fingerprint": "TOYOTA_CAMRY_TSS2",
      "steering_response_profile": TOADPILOT_STEERING_RESPONSE_PROFILE,
      "steering_response_build": TOADPILOT_STEERING_RESPONSE_BUILD,
      "route_id": "00000001--2026-07-18--12-00-00",
      "route_started_at_unix": 100.0,
      "route_complete": False,
      "completed_at_unix": None,
      "summary_source": "torqued_live",
      "command_semantics": "controller_command_before_panda_or_eps",
      "controller_authorized": True,
      "extended_output_samples": 4,
      "temporary_fault_seen": False,
      "permanent_fault_seen": False,
    }
    self.assertTrue(valid_toadpilot_torque_summary(summary))

    with TemporaryDirectory() as params_path, TemporaryDirectory() as status_path:
      params = Params(params_path)
      status_params = Params(status_path)
      status_params.put(TOADPILOT_TORQUE_STATUS_KEY, json.dumps(summary))

      self.assertFalse(persist_toadpilot_torque_last_route(params, status_params, completed_at_unix=109.0))
      self.assertIsNone(params.get(TOADPILOT_TORQUE_LAST_ROUTE_KEY))
      self.assertTrue(persist_toadpilot_torque_last_route(params, status_params, completed_at_unix=111.0))
      saved = json.loads(params.get(TOADPILOT_TORQUE_LAST_ROUTE_KEY))
      self.assertTrue(saved["route_complete"])
      self.assertEqual(saved["completed_at_unix"], 111.0)
      self.assertEqual(saved["summary_source"], "manager_offroad_transition")
      self.assertTrue(valid_toadpilot_torque_summary(saved))

  def test_torque_summary_persistence_rejects_stale_provenance_and_empty_routes(self):
    base = {
      "schema_version": TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION,
      "published_at_unix": 110.0,
      "fingerprint": "TOYOTA_CAMRY_TSS2",
      "steering_response_profile": TOADPILOT_STEERING_RESPONSE_PROFILE,
      "steering_response_build": TOADPILOT_STEERING_RESPONSE_BUILD,
      "route_id": "route",
      "route_started_at_unix": 100.0,
      "route_complete": False,
      "completed_at_unix": None,
      "summary_source": "torqued_live",
      "command_semantics": "controller_command_before_panda_or_eps",
      "controller_authorized": True,
      "extended_output_samples": 0,
      "temporary_fault_seen": False,
      "permanent_fault_seen": False,
    }
    self.assertTrue(valid_toadpilot_torque_summary(base))
    for override in ({"schema_version": 1}, {"fingerprint": "TOYOTA_PRIUS"},
                     {"steering_response_profile": "old"}, {"steering_response_build": "old"},
                     {"route_id": ""}, {"route_id": " route "}, {"published_at_unix": "110.0"},
                     {"published_at_unix": True}, {"route_started_at_unix": "100.0"},
                     {"route_started_at_unix": 111.0}, {"summary_source": "unknown"},
                     {"route_complete": True, "completed_at_unix": 109.0,
                      "summary_source": "manager_offroad_transition"}):
      with self.subTest(override=override):
        self.assertFalse(valid_toadpilot_torque_summary({**base, **override}))

    with TemporaryDirectory() as params_path, TemporaryDirectory() as status_path:
      params = Params(params_path)
      status_params = Params(status_path)
      status_params.put(TOADPILOT_TORQUE_STATUS_KEY, json.dumps(base))
      self.assertFalse(persist_toadpilot_torque_last_route(params, status_params, completed_at_unix=111.0))
      self.assertIsNone(params.get(TOADPILOT_TORQUE_LAST_ROUTE_KEY))
      self.assertIsNone(params.get(TOADPILOT_TORQUE_CHECKPOINT_KEY))


class TestToadPilotLateralAssist(unittest.TestCase):
  def test_disabled_and_observe_profiles_never_add_assist(self):
    disabled = ToadPilotConfig(enabled=False, profile="full")
    observe = ToadPilotConfig(enabled=True, profile="observe")

    for config in (disabled, observe):
      assist = ToadPilotLateralAssist()
      outputs = [run_assist(assist, config) for _ in range(100)]
      self.assertEqual(outputs, [0.0] * len(outputs))

  def test_adaptive_and_full_are_bounded_and_full_is_stronger(self):
    adaptive_assist = ToadPilotLateralAssist()
    full_assist = ToadPilotLateralAssist()
    adaptive_config = ToadPilotConfig(enabled=True, profile="adaptive")
    full_config = ToadPilotConfig(enabled=True, profile="full")

    adaptive_outputs = [run_assist(adaptive_assist, adaptive_config) for _ in range(400)]
    full_outputs = [run_assist(full_assist, full_config) for _ in range(400)]

    adaptive_bound = TOADPILOT_PROFILES["adaptive"].entry_assist + TOADPILOT_PROFILES["adaptive"].tracking_assist
    full_bound = TOADPILOT_PROFILES["full"].entry_assist + TOADPILOT_PROFILES["full"].tracking_assist
    self.assertTrue(all(0.0 <= output <= adaptive_bound for output in adaptive_outputs))
    self.assertTrue(all(0.0 <= output <= full_bound for output in full_outputs))
    self.assertGreater(adaptive_outputs[-1], 0.0)
    self.assertGreater(full_outputs[-1], adaptive_outputs[-1])

  def test_driver_input_and_steering_limits_gate_assist(self):
    config = ToadPilotConfig(enabled=True, profile="full")
    gated_inputs = (
      {"steering_pressed": True},
      {"steer_limited": True},
      {"steering_rate_deg": 75.0},
      {"steering_rate_deg": -80.0},
    )

    for inputs in gated_inputs:
      with self.subTest(inputs=inputs):
        self.assertEqual(run_assist(ToadPilotLateralAssist(), config, **inputs), 0.0)

  def test_experimental_mode_requires_explicit_opt_in(self):
    default_config = ToadPilotConfig(enabled=True, profile="adaptive")
    opted_in_config = replace(default_config, curve_assist_in_experimental=True)

    self.assertEqual(run_assist(ToadPilotLateralAssist(), default_config, experimental_mode=True), 0.0)
    self.assertGreater(run_assist(ToadPilotLateralAssist(), opted_in_config, experimental_mode=True), 0.0)

  def test_warmed_assist_drops_immediately_on_hard_gates(self):
    full = ToadPilotConfig(enabled=True, profile="full")
    cases = (
      (full, {"steering_pressed": True}),
      (full, {"steer_limited": True}),
      (full, {"steering_rate_deg": 75.0}),
      (full, {"experimental_mode": True}),
      (full, {"v_ego": 11.9}),
      (full, {"a_ego": -0.76}),
      (full, {"brake_pressed": True}),
      (full, {"longitudinal_stopping": True}),
      (replace(full, profile="observe"), {}),
      (full, {"desired_curvature": 0.001}),
    )

    for gated_config, inputs in cases:
      with self.subTest(config=gated_config, inputs=inputs):
        assist = ToadPilotLateralAssist()
        self.assertGreater([run_assist(assist, full) for _ in range(100)][-1], 0.0)
        self.assertEqual(run_assist(assist, gated_config, **inputs), 0.0)
        self.assertEqual(assist.residual, 0.0)

  def test_direction_reversal_never_carries_old_direction(self):
    config = ToadPilotConfig(enabled=True, profile="full")
    assist = ToadPilotLateralAssist()
    self.assertGreater([run_assist(assist, config) for _ in range(100)][-1], 0.0)

    reversed_output = run_assist(
      assist, config,
      desired_curvature=-0.002,
      desired_lateral_accel=-2.0,
      actual_lateral_accel=0.0,
    )
    self.assertLessEqual(reversed_output, 0.0)

  def test_profile_change_clips_existing_residual(self):
    full = ToadPilotConfig(enabled=True, profile="full")
    adaptive = replace(full, profile="adaptive")
    assist = ToadPilotLateralAssist()
    [run_assist(assist, full) for _ in range(400)]

    output = run_assist(assist, adaptive)
    adaptive_bound = TOADPILOT_PROFILES["adaptive"].entry_assist + TOADPILOT_PROFILES["adaptive"].tracking_assist
    self.assertLessEqual(abs(output), adaptive_bound)

  def test_disabled_telemetry_does_not_write(self):
    class FailOnWrite:
      def put_nonblocking(self, *args, **kwargs):
        raise AssertionError("telemetry write should be gated")

    assist = ToadPilotLateralAssist()
    assist.status_params = FailOnWrite()
    assist.publish_status(ToadPilotConfig(enabled=False, telemetry=True), active=False)
    assist.publish_status(ToadPilotConfig(enabled=True, telemetry=False), active=True)


class TestToadPilotHighAuthorityObserver(unittest.TestCase):
  def test_observer_splits_directions_and_bins_without_applying_tune(self):
    observer = ToadPilotHighAuthorityObserver()
    self.assertTrue(observer.add_sample(0.55, 1.10))
    self.assertTrue(observer.add_sample(-0.70, -1.40))
    self.assertTrue(observer.add_sample(0.90, 1.80))
    self.assertFalse(observer.add_sample(0.90, 0.10))
    self.assertFalse(observer.add_sample(0.40, 0.80))

    snapshot = observer.snapshot()
    self.assertTrue(snapshot["observe_only"])
    self.assertFalse(snapshot["applied_to_tune"])
    self.assertEqual(snapshot["candidate_samples"], 4)
    self.assertEqual(snapshot["accepted_samples"], 3)
    self.assertEqual(snapshot["rejected_samples"], 1)
    self.assertEqual(snapshot["directions"]["left"]["samples"], 2)
    self.assertEqual(snapshot["directions"]["right"]["samples"], 1)
    self.assertEqual(snapshot["bins"]["left"]["0.50-0.65"]["samples"], 1)
    self.assertEqual(snapshot["bins"]["right"]["0.65-0.80"]["samples"], 1)
    self.assertEqual(snapshot["bins"]["left"]["0.80-1.00"]["samples"], 1)



if __name__ == "__main__":
  unittest.main()
