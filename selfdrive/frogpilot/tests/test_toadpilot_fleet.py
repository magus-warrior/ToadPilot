import json
import math
import unittest

from unittest.mock import Mock, patch

from openpilot.selfdrive.frogpilot.fleetmanager import helpers
from openpilot.selfdrive.frogpilot.controls.lib.start_accel_ramp import START_ACCEL_RAMP_PARAM
from openpilot.selfdrive.frogpilot.toadpilot import (TOADPILOT_CONFIG_KEY, TOADPILOT_CONFIG_VERSION, TOADPILOT_RESET_TUNE_KEY,
                                                     TOADPILOT_NAVIGATION_STATUS_KEY,
                                                     TOADPILOT_STEERING_RESPONSE_BUILD,
                                                     TOADPILOT_STATUS_KEY, TOADPILOT_STEERING_RESPONSE_PROFILE,
                                                     TOADPILOT_STEERING_RESPONSE_REQUEST_KEY,
                                                     TOADPILOT_STEERING_STATUS_KEY,
                                                     TOADPILOT_STOP_GO_DIAGNOSTIC_SCHEMA_VERSION,
                                                     TOADPILOT_STOP_GO_LAST_ROUTE_KEY,
                                                     TOADPILOT_STOP_GO_STATUS_KEY,
                                                     TOADPILOT_TORQUE_CHECKPOINT_KEY,
                                                     TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION,
                                                     TOADPILOT_TORQUE_LAST_ROUTE_KEY, TOADPILOT_TORQUE_STATUS_KEY,
                                                     TOADPILOT_TUNE_STATUS_KEY)


VALID_CONFIG = {
  "version": TOADPILOT_CONFIG_VERSION,
  "enabled": True,
  "profile": "adaptive",
  "auto_tune": True,
  "lane_centering_assist": True,
  "smooth_lane_transitions": True,
  "straight_line_smoothing": True,
  "experimental_max_torque": 1500,
  "improved_stop_and_go": True,
  "responsive_takeoff": True,
  "smooth_stop_finish": True,
  "automatic_traffic_mode": True,
  "traffic_mode_entry_time": 1.0,
  "traffic_mode_max_speed_mph": 30,
  "map_stop_assist": True,
  "curve_assist_in_experimental": False,
  "smart_saturation_warning": True,
  "telemetry": True,
}


class MemoryParams:
  def __init__(self, values=None):
    self.values = dict(values or {})
    self.put_calls = []
    self.remove_calls = []

  def get(self, key, **kwargs):
    value = self.values.get(key)
    if value is None or isinstance(value, bytes):
      return value
    if isinstance(value, str):
      return value.encode()
    return value

  def get_bool(self, key):
    value = self.values.get(key)
    return value is True or value in (b"1", "1")

  def put(self, key, value):
    self.put_calls.append((key, value))
    self.values[key] = value

  def put_bool(self, key, value):
    self.put_calls.append((key, value))
    self.values[key] = bool(value)

  def remove(self, key):
    self.remove_calls.append(key)
    self.values.pop(key, None)


class TestToadPilotFleetStatus(unittest.TestCase):
  def test_steering_experiment_reports_pending_until_boot_latch_and_controller_flag_match(self):
    persistent = MemoryParams({TOADPILOT_STEERING_RESPONSE_REQUEST_KEY: TOADPILOT_STEERING_RESPONSE_PROFILE})
    road_state = {"offroad_confirmed": True}

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "read_toadpilot_steering_response_latch", return_value=False):
      result = helpers._get_toadpilot_steering_experiment(
        {"supported": True, "steering_authority_flag": False, "panda_firmware_matched": False}, road_state,
      )

    self.assertEqual(result["profile"], TOADPILOT_STEERING_RESPONSE_PROFILE)
    self.assertTrue(result["requested"])
    self.assertFalse(result["boot_latched"])
    self.assertTrue(result["available"])
    self.assertFalse(result["effective"])
    self.assertTrue(result["pending_reboot"])
    self.assertTrue(result["requires_reboot"])
    self.assertTrue(result["request_writable"])
    self.assertEqual(result["state"], "pending_enable")

  def test_steering_experiment_effective_requires_supported_vehicle_latch_and_controller_flag(self):
    persistent = MemoryParams({TOADPILOT_STEERING_RESPONSE_REQUEST_KEY: TOADPILOT_STEERING_RESPONSE_PROFILE})
    road_state = {"offroad_confirmed": False}

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "read_toadpilot_steering_response_latch", return_value=True):
      result = helpers._get_toadpilot_steering_experiment(
        {"supported": True, "steering_authority_flag": True, "panda_firmware_matched": True}, road_state,
      )

    self.assertTrue(result["boot_latched"])
    self.assertTrue(result["controller_flag_present"])
    self.assertTrue(result["effective"])
    self.assertFalse(result["pending_reboot"])
    self.assertEqual(result["state"], "effective")

  def test_boot_latch_with_old_panda_firmware_fails_closed(self):
    persistent = MemoryParams({TOADPILOT_STEERING_RESPONSE_REQUEST_KEY: TOADPILOT_STEERING_RESPONSE_PROFILE})

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "read_toadpilot_steering_response_latch", return_value=True):
      result = helpers._get_toadpilot_steering_experiment(
        {"supported": True, "steering_authority_flag": False, "panda_firmware_matched": False},
        {"offroad_confirmed": True},
      )

    self.assertFalse(result["effective"])
    self.assertEqual(result["state"], "armed_waiting_for_firmware")
    self.assertIn("exact paired firmware", result["reason"])

  def test_disabling_request_does_not_change_effective_boot_latch(self):
    persistent = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "read_toadpilot_steering_response_latch", return_value=True):
        result = helpers._get_toadpilot_steering_experiment(
          {"supported": True, "steering_authority_flag": True, "panda_firmware_matched": True},
          {"offroad_confirmed": True},
        )

    self.assertFalse(result["requested"])
    self.assertTrue(result["boot_latched"])
    self.assertTrue(result["effective"])
    self.assertTrue(result["pending_reboot"])
    self.assertEqual(result["state"], "pending_disable")

  def test_old_steering_profile_is_stale_and_never_requested(self):
    old_profile = "camry_tss2_response_research_v1"
    persistent = MemoryParams({TOADPILOT_STEERING_RESPONSE_REQUEST_KEY: old_profile})

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "read_toadpilot_steering_response_latch", return_value=False):
      result = helpers._get_toadpilot_steering_experiment(
        {"supported": True, "steering_authority_flag": False, "panda_firmware_matched": False}, {"offroad_confirmed": True},
      )

    self.assertFalse(result["requested"])
    self.assertEqual(result["stored_profile"], old_profile)
    self.assertEqual(result["stale_profile"], old_profile)
    self.assertTrue(result["legacy_profile"])
    self.assertFalse(result["effective"])
    self.assertEqual(result["state"], "stale_profile")

  def test_status_freshness_uses_server_time(self):
    now = 1_000.0
    cases = (
      ({"published_at_unix": now - 2.25}, True, 2.25),
      ({"published_at_unix": now - 5.01}, False, 5.01),
    )

    for status, expected_fresh, expected_age in cases:
      with self.subTest(status=status), \
           patch.object(helpers, "read_json_status", return_value=status.copy()), \
           patch.object(helpers.time, "time", return_value=now):
        result = helpers._get_toadpilot_status("status")

        self.assertIs(result["fresh"], expected_fresh)
        self.assertEqual(result["age_seconds"], expected_age)

  def test_malformed_statuses_fail_closed(self):
    malformed_statuses = (
      ["not", "an", "object"],
      "not an object",
      {"published_at_unix": True},
      {"published_at_unix": "1000"},
      {"published_at_unix": None},
      {"published_at_unix": math.nan},
      {"published_at_unix": math.inf},
      {"published_at_unix": -math.inf},
    )

    for status in malformed_statuses:
      with self.subTest(status=status), \
           patch.object(helpers, "read_json_status", return_value=status), \
           patch.object(helpers.time, "time", return_value=1_000.0):
        result = helpers._get_toadpilot_status("status")

        if isinstance(result, dict) and result:
          self.assertIs(result["fresh"], False)
          self.assertIsNone(result["age_seconds"])
        else:
          self.assertEqual(result, {})

  def test_future_status_fails_closed(self):
    with patch.object(helpers, "read_json_status", return_value={"published_at_unix": 1_010.0}), \
         patch.object(helpers.time, "time", return_value=1_000.0):
      result = helpers._get_toadpilot_status("status")

    self.assertIs(result["fresh"], False)

  def test_last_torque_route_requires_exact_provenance_and_reports_age(self):
    summary = {
      "schema_version": TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION,
      "published_at_unix": 1_010.0,
      "fingerprint": "TOYOTA_CAMRY_TSS2",
      "steering_response_profile": TOADPILOT_STEERING_RESPONSE_PROFILE,
      "steering_response_build": TOADPILOT_STEERING_RESPONSE_BUILD,
      "route_id": "route",
      "route_started_at_unix": 1_000.0,
      "route_complete": True,
      "completed_at_unix": 1_011.0,
      "summary_source": "manager_offroad_transition",
      "command_semantics": "controller_command_before_panda_or_eps",
      "controller_authorized": True,
      "extended_output_samples": 5,
    }
    with patch.object(helpers, "read_json_status", side_effect=lambda key, _: (
           summary if key == TOADPILOT_TORQUE_LAST_ROUTE_KEY else {})), \
         patch.object(helpers.time, "time", return_value=1_020.0):
      result = helpers._get_toadpilot_last_torque_route()

    self.assertTrue(result["provenance_valid"])
    self.assertEqual(result["age_seconds"], 9.0)

    with patch.object(helpers, "read_json_status", return_value={**summary, "steering_response_build": "old"}):
      self.assertEqual(helpers._get_toadpilot_last_torque_route(), {})

    future = {**summary, "completed_at_unix": 1_030.0}
    with patch.object(helpers, "read_json_status", side_effect=lambda key, _: (
           future if key == TOADPILOT_TORQUE_LAST_ROUTE_KEY else {})), \
         patch.object(helpers.time, "time", return_value=1_020.0):
      self.assertEqual(helpers._get_toadpilot_last_torque_route(), {})

  def test_newer_interrupted_checkpoint_does_not_downgrade_same_completed_route(self):
    common = {
      "schema_version": TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION,
      "fingerprint": "TOYOTA_CAMRY_TSS2",
      "steering_response_profile": TOADPILOT_STEERING_RESPONSE_PROFILE,
      "steering_response_build": TOADPILOT_STEERING_RESPONSE_BUILD,
      "route_id": "route",
      "route_started_at_unix": 1_000.0,
      "command_semantics": "controller_command_before_panda_or_eps",
      "controller_authorized": True,
      "extended_output_samples": 5,
    }
    completed = {**common, "published_at_unix": 1_010.0, "route_complete": True,
                 "completed_at_unix": 1_011.0, "summary_source": "manager_offroad_transition"}
    late_checkpoint = {**common, "published_at_unix": 1_012.0, "route_complete": False,
                       "completed_at_unix": None, "summary_source": "torqued_live"}

    def status_for_key(key, _):
      return completed if key == TOADPILOT_TORQUE_LAST_ROUTE_KEY else late_checkpoint

    with patch.object(helpers, "read_json_status", side_effect=status_for_key), \
         patch.object(helpers.time, "time", return_value=1_020.0):
      result = helpers._get_toadpilot_last_torque_route()

    self.assertTrue(result["route_complete"])
    self.assertEqual(result["completed_at_unix"], 1_011.0)

  def test_last_stop_go_route_requires_valid_completed_provenance(self):
    summary = {
      "schema_version": TOADPILOT_STOP_GO_DIAGNOSTIC_SCHEMA_VERSION,
      "published_at_unix": 1_010.0,
      "route_started_at_unix": 1_000.0,
      "route_id": "route",
      "route_complete": True,
      "completed_at_unix": 1_011.0,
      "summary_source": "manager_offroad_transition",
      "fingerprint": "TOYOTA_CAMRY_TSS2",
      "event_sequence": 4,
      "last_event": "controls_inactive",
    }
    with patch.object(helpers, "read_json_status", return_value=summary), \
         patch.object(helpers.time, "time", return_value=1_020.0):
      result = helpers._get_toadpilot_last_stop_go_route()
    self.assertTrue(result["provenance_valid"])
    self.assertEqual(result["age_seconds"], 9.0)

    with patch.object(helpers, "read_json_status", return_value={**summary, "summary_source": "controlsd_live"}):
      self.assertEqual(helpers._get_toadpilot_last_stop_go_route(), {})


class TestToadPilotFleetMutations(unittest.TestCase):
  def test_steering_experiment_request_is_default_off_and_can_only_change_offroad(self):
    requested = {**VALID_CONFIG, "experimental_steering_response_requested": True}
    persistent = MemoryParams()
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
      with self.assertRaisesRegex(ValueError, "confirmed offroad"):
        helpers.set_toadpilot_config(requested)

    self.assertEqual(persistent.put_calls, [])
    toggles_updated.assert_not_called()

  def test_steering_experiment_request_persists_offroad_without_live_activation(self):
    requested = {**VALID_CONFIG, "experimental_steering_response_requested": True}
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated, \
         patch.object(helpers, "get_toadpilot_state", return_value={"state": "saved"}):
      result = helpers.set_toadpilot_config(requested)

    self.assertEqual(persistent.values[TOADPILOT_STEERING_RESPONSE_REQUEST_KEY], TOADPILOT_STEERING_RESPONSE_PROFILE)
    self.assertNotIn("experimental_steering_response_requested",
                     json.loads(persistent.values[TOADPILOT_CONFIG_KEY]))
    self.assertEqual(result, {"state": "saved"})
    toggles_updated.assert_called_once_with()

  def test_explicit_off_request_clears_stale_profile_only_when_confirmed_offroad(self):
    requested = {**VALID_CONFIG, "experimental_steering_response_requested": False}
    old_profile = "camry_tss2_response_research_v1"
    persistent = MemoryParams({"IsOffroad": True, TOADPILOT_STEERING_RESPONSE_REQUEST_KEY: old_profile})
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated, \
         patch.object(helpers, "get_toadpilot_state", return_value={"state": "saved"}):
      helpers.set_toadpilot_config(requested)

    self.assertNotIn(TOADPILOT_STEERING_RESPONSE_REQUEST_KEY, persistent.values)
    toggles_updated.assert_called_once_with()

  def test_steering_experiment_request_rejects_non_boolean_before_writes(self):
    requested = {**VALID_CONFIG, "experimental_steering_response_requested": 1}
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
      with self.assertRaisesRegex(ValueError, "must be a boolean"):
        helpers.set_toadpilot_config(requested)

    self.assertEqual(persistent.put_calls, [])
    toggles_updated.assert_not_called()

  def test_config_round_trip_preserves_explicit_disabled_choices(self):
    requested = {
      **VALID_CONFIG,
      "enabled": False,
      "profile": "observe",
      "auto_tune": False,
      "lane_centering_assist": False,
      "smooth_lane_transitions": False,
      "straight_line_smoothing": False,
      "improved_stop_and_go": False,
      "automatic_traffic_mode": False,
      "map_stop_assist": False,
      "curve_assist_in_experimental": False,
      "smart_saturation_warning": False,
      "telemetry": False,
    }
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles"), \
         patch.object(helpers, "get_toadpilot_state", return_value={"state": "saved"}):
      helpers.set_toadpilot_config(requested)

    self.assertEqual(json.loads(persistent.values[TOADPILOT_CONFIG_KEY]), requested)

  def test_config_requires_smooth_transition_choice_before_write(self):
    requested = dict(VALID_CONFIG)
    requested.pop("smooth_lane_transitions")
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
      with self.assertRaisesRegex(ValueError, "smooth_lane_transitions"):
        helpers.set_toadpilot_config(requested)

    self.assertEqual(persistent.put_calls, [])
    toggles_updated.assert_not_called()

  def test_config_requires_stop_and_go_choice_before_write(self):
    requested = dict(VALID_CONFIG)
    requested.pop("improved_stop_and_go")
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
      with self.assertRaisesRegex(ValueError, "improved_stop_and_go"):
        helpers.set_toadpilot_config(requested)

    self.assertEqual(persistent.put_calls, [])
    toggles_updated.assert_not_called()

  def test_config_requires_straight_line_smoothing_choice_before_write(self):
    requested = dict(VALID_CONFIG)
    requested.pop("straight_line_smoothing")
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
      with self.assertRaisesRegex(ValueError, "straight_line_smoothing"):
        helpers.set_toadpilot_config(requested)

    self.assertEqual(persistent.put_calls, [])
    toggles_updated.assert_not_called()

  def test_config_requires_experimental_max_torque_choice_before_write(self):
    requested = dict(VALID_CONFIG)
    requested.pop("experimental_max_torque")
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
      with self.assertRaisesRegex(ValueError, "experimental_max_torque"):
        helpers.set_toadpilot_config(requested)

    self.assertEqual(persistent.put_calls, [])
    toggles_updated.assert_not_called()

  def test_config_change_is_rejected_onroad_before_writes(self):
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams({"IsOnroad": True})
    toggles_updated = Mock()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles", toggles_updated):
      with self.assertRaisesRegex(ValueError, "offroad"):
        helpers.set_toadpilot_config(VALID_CONFIG)

    self.assertEqual(persistent.put_calls, [])
    self.assertEqual(persistent.remove_calls, [])
    toggles_updated.assert_not_called()

  def test_config_change_is_allowed_when_device_is_offline_without_car_state(self):
    persistent = MemoryParams()
    memory = MemoryParams()

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated, \
         patch.object(helpers, "get_toadpilot_state", return_value={"state": "saved"}):
      result = helpers.set_toadpilot_config(VALID_CONFIG)

    self.assertIn(TOADPILOT_CONFIG_KEY, persistent.values)
    self.assertEqual(json.loads(persistent.values[TOADPILOT_CONFIG_KEY]), VALID_CONFIG)
    self.assertEqual(result, {"state": "saved"})
    toggles_updated.assert_called_once_with()

  def test_config_change_rechecks_onroad_state_before_write(self):
    persistent = MemoryParams()
    memory = MemoryParams()
    onroad_states = iter((False, True))

    def get_bool(key):
      return next(onroad_states) if key == "IsOnroad" else False

    memory.get_bool = Mock(side_effect=get_bool)
    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
      with self.assertRaisesRegex(ValueError, "offroad"):
        helpers.set_toadpilot_config(VALID_CONFIG)

    self.assertEqual(persistent.put_calls, [])
    toggles_updated.assert_not_called()

  def test_reset_is_rejected_onroad_before_writes(self):
    persistent = MemoryParams({"LiveTorqueParameters": b"cached"})
    memory = MemoryParams({
      "IsOnroad": True,
      TOADPILOT_STATUS_KEY: b"runtime",
      TOADPILOT_TORQUE_STATUS_KEY: b"torque",
      TOADPILOT_TUNE_STATUS_KEY: b"tune",
    })

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory):
      with self.assertRaisesRegex(ValueError, "offroad"):
        helpers.reset_toadpilot_tune()

    self.assertEqual(persistent.put_calls, [])
    self.assertEqual(persistent.remove_calls, [])
    self.assertEqual(memory.remove_calls, [])

  def test_reset_still_requires_confirmed_offroad_when_vehicle_state_is_unknown(self):
    persistent = MemoryParams({"LiveTorqueParameters": b"cached"})
    memory = MemoryParams({
      TOADPILOT_STATUS_KEY: b"runtime",
      TOADPILOT_TUNE_STATUS_KEY: b"tune",
    })

    with patch.object(helpers, "params", persistent), patch.object(helpers, "params_memory", memory):
      with self.assertRaisesRegex(ValueError, "confirmed offroad"):
        helpers.reset_toadpilot_tune()

    self.assertEqual(persistent.put_calls, [])
    self.assertEqual(persistent.remove_calls, [])
    self.assertEqual(memory.remove_calls, [])

  def test_offroad_reset_clears_cache_and_status_and_queues_reset(self):
    persistent = MemoryParams({"IsOffroad": True, "LiveTorqueParameters": b"cached",
                               TOADPILOT_TORQUE_CHECKPOINT_KEY: b"checkpoint",
                               TOADPILOT_TORQUE_LAST_ROUTE_KEY: b"summary"})
    memory = MemoryParams({
      "IsOnroad": False,
      TOADPILOT_STATUS_KEY: b"runtime",
      TOADPILOT_TORQUE_STATUS_KEY: b"torque",
      TOADPILOT_TUNE_STATUS_KEY: b"tune",
    })

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "get_toadpilot_state", return_value={"state": "fresh"}):
      result = helpers.reset_toadpilot_tune()

    self.assertNotIn("LiveTorqueParameters", persistent.values)
    self.assertNotIn(TOADPILOT_TORQUE_CHECKPOINT_KEY, persistent.values)
    self.assertNotIn(TOADPILOT_TORQUE_LAST_ROUTE_KEY, persistent.values)
    self.assertIs(persistent.values[TOADPILOT_RESET_TUNE_KEY], True)
    self.assertNotIn(TOADPILOT_STATUS_KEY, memory.values)
    self.assertNotIn(TOADPILOT_TORQUE_STATUS_KEY, memory.values)
    self.assertNotIn(TOADPILOT_TUNE_STATUS_KEY, memory.values)
    self.assertEqual(result["state"], {"state": "fresh"})

  def test_disabling_config_or_telemetry_clears_status(self):
    cases = (
      {**VALID_CONFIG, "enabled": False},
      {**VALID_CONFIG, "telemetry": False},
    )

    for config in cases:
      with self.subTest(config=config):
        persistent = MemoryParams({"IsOffroad": True})
        memory = MemoryParams({
          "IsOnroad": False,
          TOADPILOT_STATUS_KEY: b"runtime",
          TOADPILOT_STEERING_STATUS_KEY: b"steering",
          TOADPILOT_STOP_GO_STATUS_KEY: b"stop_go",
          TOADPILOT_TUNE_STATUS_KEY: b"tune",
        })

        with patch.object(helpers, "params", persistent), \
             patch.object(helpers, "params_memory", memory), \
             patch.object(helpers, "update_frogpilot_toggles"), \
             patch.object(helpers, "get_toadpilot_state", return_value={"state": "fresh"}):
          helpers.set_toadpilot_config(config)

        self.assertIn(TOADPILOT_CONFIG_KEY, persistent.values)
        self.assertNotIn(TOADPILOT_STATUS_KEY, memory.values)
        self.assertNotIn(TOADPILOT_STEERING_STATUS_KEY, memory.values)
        self.assertNotIn(TOADPILOT_STOP_GO_STATUS_KEY, memory.values)
        self.assertNotIn(TOADPILOT_TUNE_STATUS_KEY, memory.values)

  def test_enabled_telemetry_preserves_current_status(self):
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams({
      "IsOnroad": False,
      TOADPILOT_STATUS_KEY: b"runtime",
      TOADPILOT_TUNE_STATUS_KEY: b"tune",
    })

    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles"), \
         patch.object(helpers, "get_toadpilot_state", return_value={"state": "fresh"}):
      helpers.set_toadpilot_config(VALID_CONFIG)

    self.assertIn(TOADPILOT_STATUS_KEY, memory.values)
    self.assertIn(TOADPILOT_TUNE_STATUS_KEY, memory.values)


class TestToadPilotResetRoute(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    from openpilot.selfdrive.frogpilot.fleetmanager import fleet_manager
    cls.fleet_manager = fleet_manager
    cls.client = fleet_manager.app.test_client()

  def test_reset_requires_exact_boolean_confirmation(self):
    invalid_payloads = (
      None,
      {},
      {"confirm": False},
      {"confirm": 1},
      {"confirm": True, "extra": True},
    )

    reset = Mock(return_value={"message": "reset", "state": {}})
    with patch.object(self.fleet_manager.fleet, "reset_toadpilot_tune", reset):
      for payload in invalid_payloads:
        with self.subTest(payload=payload):
          kwargs = {} if payload is None else {"data": json.dumps(payload), "content_type": "application/json"}
          response = self.client.post("/toadpilot/reset_tune", **kwargs)
          self.assertEqual(response.status_code, 400)

    reset.assert_not_called()

  def test_read_only_route_returns_current_state(self):
    state = {"config": {"enabled": False}, "effective_enabled": False}
    with patch.object(self.fleet_manager.fleet, "get_toadpilot_state", return_value=state):
      response = self.client.get("/toadpilot")

    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.get_json(), state)

  def test_tools_template_renders_toadpilot_panel(self):
    response = self.client.get("/tools")

    self.assertEqual(response.status_code, 200)
    self.assertIn(b"TOADPILOT", response.data)
    self.assertIn(b"Lane centering assist", response.data)
    self.assertIn(b'id="toadEnabled" checked', response.data)
    self.assertIn(b'value="adaptive" selected', response.data)
    self.assertIn(b'id="toadLaneCentering" checked', response.data)
    self.assertIn(b'id="toadSmoothLaneTransitions" checked', response.data)
    self.assertIn(b'id="toadStraightLineSmoothing" checked', response.data)
    self.assertIn(b'straight_line_smoothing: toadStraightLineSmoothing.checked', response.data)
    self.assertIn(b'role="tablist"', response.data)
    self.assertIn(b'id="toadResponsiveTakeoff" checked', response.data)
    self.assertIn(b'id="toadSmoothStopFinish" checked', response.data)
    self.assertIn(b'responsive_takeoff: toadResponsiveTakeoff.checked', response.data)
    self.assertIn(b'smooth_stop_finish: toadSmoothStopFinish.checked', response.data)
    self.assertNotIn(b'toadImprovedStopAndGo', response.data)
    self.assertNotIn(b'startRampSlider', response.data)
    self.assertNotIn(b'loadStartAccelRamp', response.data)
    self.assertNotIn(b'saved ramp', response.data)
    self.assertNotIn(b'toadObserverStatus', response.data)
    self.assertNotIn(b'renderToadObserver', response.data)
    self.assertIn(b'Steering profile only', response.data)
    self.assertNotIn(b'Observe keeps stock controller output unchanged', response.data)
    self.assertIn(b'id="toadLongitudinalReadiness"', response.data)
    self.assertIn(b'liveStopGo.responsive_takeoff_enabled === true', response.data)
    self.assertIn(b'liveStopGo.smooth_stop_finish_enabled === true', response.data)
    self.assertIn(b'Number(config.traffic_mode_max_speed_mph || 30)', response.data)
    self.assertIn(b'id="toadAutomaticTrafficMode" checked', response.data)
    self.assertIn(b'automatic_traffic_mode: toadAutomaticTrafficMode.checked', response.data)
    self.assertIn(b'id="toadMapStopAssist" checked', response.data)
    self.assertIn(b'map_stop_assist: toadMapStopAssist.checked', response.data)
    self.assertIn(b'id="toadAutomaticTrafficState"', response.data)
    self.assertIn(b'id="toadMapStopState"', response.data)
    self.assertIn(b'id="toadStopAndGoState"', response.data)
    self.assertIn(b'id="toadStopGoLiveState"', response.data)
    self.assertIn(b'id="toadStopGoLastEvent"', response.data)
    self.assertIn(b'id="toadSteeringLiveState"', response.data)
    self.assertIn(b'id="toadPandaSafetyState"', response.data)
    self.assertIn(b'20 Hz diagnostic samples above 1500', response.data)
    self.assertIn(b'guarded planner-confirmed launch handoff enabled', response.data)
    self.assertIn(b'Experimental Camry authority', response.data)
    self.assertIn(b'id="toadExperimentalMaxTorque"', response.data)
    self.assertIn(b'experimental_max_torque: Number(toadExperimentalMaxTorque.value)', response.data)
    self.assertIn(b'id="toadAuthoritySummary"', response.data)
    self.assertIn(b'id="toadAuthorityDetail"', response.data)
    self.assertIn(b'1700 hard authority is active.', response.data)
    self.assertNotIn(b'Existing authority by default.', response.data)
    self.assertIn(b'id="toadExperimentalSteeringResponse"', response.data)
    self.assertNotIn(b'id="toadExperimentalSteeringResponse" checked', response.data)
    self.assertIn(b'requestPayload.experimental_steering_response_requested = steeringRequest', response.data)
    self.assertIn(b'toadExperimentalSteeringResponse.disabled = busy || toadOnroad || !toadSteeringRequestAllowed', response.data)
    self.assertIn(b'toadSteeringRequestAllowed = false', response.data)
    self.assertEqual(response.headers["Cache-Control"], "no-store")
    self.assertIn(b'requestOptions.cache = "no-store"', response.data)
    self.assertIn(b'renderToadPilot(state, populateConfig === true)', response.data)
    self.assertIn(b'version: toadConfigVersion', response.data)
    self.assertNotIn(b'Camry Start Acceleration Ramp', response.data)

  def test_config_route_disables_caching_and_returns_saved_state(self):
    state = {"config": VALID_CONFIG, "effective_enabled": True}
    with patch.object(self.fleet_manager.fleet, "set_toadpilot_config", return_value=state) as save:
      response = self.client.post("/toadpilot", json=VALID_CONFIG)

    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.get_json(), state)
    self.assertEqual(response.headers["Cache-Control"], "no-store")
    save.assert_called_once_with(VALID_CONFIG)

  def test_config_read_disables_caching(self):
    state = {"config": VALID_CONFIG, "effective_enabled": True}
    with patch.object(self.fleet_manager.fleet, "get_toadpilot_state", return_value=state):
      response = self.client.get("/toadpilot")

    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.get_json(), state)
    self.assertEqual(response.headers["Cache-Control"], "no-store")

  def test_reset_with_literal_confirmation_calls_helper(self):
    reset = Mock(return_value={"message": "reset", "state": {"ok": True}})
    with patch.object(self.fleet_manager.fleet, "reset_toadpilot_tune", reset):
      response = self.client.post("/toadpilot/reset_tune", json={"confirm": True})

    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.get_json()["state"], {"ok": True})
    reset.assert_called_once_with()

  def test_route_reports_offroad_guard_as_client_error(self):
    reset = Mock(side_effect=ValueError("TOADPILOT learning data can only be reset while the device is offroad."))
    with patch.object(self.fleet_manager.fleet, "reset_toadpilot_tune", reset):
      response = self.client.post("/toadpilot/reset_tune", json={"confirm": True})

    self.assertEqual(response.status_code, 400)
    self.assertIn("offroad", response.get_json()["error"])

  def test_mapbox_token_route_rejects_blank_and_non_text_values(self):
    with patch.object(self.fleet_manager.fleet, "set_public_token") as set_public, \
         patch.object(self.fleet_manager.fleet, "set_secret_token") as set_secret:
      blank_response = self.client.post("/mapbox_token", json={"public_token": "  ", "secret_token": ""})
      non_text_response = self.client.post("/mapbox_token", json={"public_token": 123})

    self.assertEqual(blank_response.status_code, 400)
    self.assertIn("Enter", blank_response.get_json()["error"])
    self.assertEqual(non_text_response.status_code, 400)
    self.assertIn("text", non_text_response.get_json()["error"])
    set_public.assert_not_called()
    set_secret.assert_not_called()


class TestStartAccelRampFleet(unittest.TestCase):
  def test_default_is_zero_and_disabled(self):
    persistent = MemoryParams()
    memory = MemoryParams()
    with patch.object(helpers, "params", persistent), patch.object(helpers, "params_memory", memory):
      state = helpers.get_start_accel_ramp_state()

    self.assertEqual(state["value"], 0.0)
    self.assertEqual(state["default"], 0.0)
    self.assertFalse(state["enabled"])

  def test_offroad_save_round_trips_and_updates_toggles(self):
    persistent = MemoryParams({"IsOffroad": True})
    memory = MemoryParams()
    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
      state = helpers.set_start_accel_ramp({"value": 0.35})

    self.assertEqual(persistent.values[START_ACCEL_RAMP_PARAM], "0.35")
    self.assertEqual(state["value"], 0.35)
    toggles_updated.assert_called_once_with()

  def test_onroad_save_is_rejected_before_writes(self):
    persistent = MemoryParams()
    memory = MemoryParams({"IsOnroad": True})
    with patch.object(helpers, "params", persistent), \
         patch.object(helpers, "params_memory", memory), \
         patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
      with self.assertRaisesRegex(ValueError, "onroad"):
        helpers.set_start_accel_ramp({"value": 0.35})

    self.assertEqual(persistent.put_calls, [])
    toggles_updated.assert_not_called()

  def test_invalid_values_are_rejected_before_writes(self):
    invalid_values = (True, "0.35", math.nan, math.inf, -0.05, 1.05, 0.33)
    for value in invalid_values:
      with self.subTest(value=value):
        persistent = MemoryParams({"IsOffroad": True})
        memory = MemoryParams()
        with patch.object(helpers, "params", persistent), \
             patch.object(helpers, "params_memory", memory), \
             patch.object(helpers, "update_frogpilot_toggles") as toggles_updated:
          with self.assertRaises(ValueError):
            helpers.set_start_accel_ramp({"value": value})
        self.assertEqual(persistent.put_calls, [])
        toggles_updated.assert_not_called()

  def test_route_validates_payload_and_disables_caching(self):
    from openpilot.selfdrive.frogpilot.fleetmanager import fleet_manager
    client = fleet_manager.app.test_client()
    state = {"value": 0.35, "default": 0.0}

    with patch.object(fleet_manager.fleet, "get_start_accel_ramp_state", return_value=state):
      response = client.get("/start_accel_ramp")
    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.get_json(), state)
    self.assertEqual(response.headers["Cache-Control"], "no-store")

    with patch.object(fleet_manager.fleet, "set_start_accel_ramp", return_value=state) as save:
      response = client.post("/start_accel_ramp", json={"value": 0.35})
    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.headers["Cache-Control"], "no-store")
    save.assert_called_once_with({"value": 0.35})

    response = client.post("/start_accel_ramp", data="[]", content_type="application/json")
    self.assertEqual(response.status_code, 400)


if __name__ == "__main__":
  unittest.main()
