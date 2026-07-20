import json
import math
import os
import stat
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from openpilot.selfdrive.frogpilot.controls.lib.mapped_stop_controller import MappedStopController
from openpilot.selfdrive.frogpilot.controls.lib.navigation_trial_recorder import NavigationTrialRecorder


class TestNavigationTrialRecorder(unittest.TestCase):
  def test_default_lookahead_matches_map_controller_input_window(self):
    recorder = NavigationTrialRecorder(session_id="session")

    self.assertEqual(recorder.lookahead_distance, MappedStopController.MAX_LOOKAHEAD_M)

  @staticmethod
  def condition(condition_id="map-stop-1", distance=50.0, **overrides):
    value = {"valid": True, "id": condition_id, "type": "stop_sign", "distance_m": distance}
    value.update(overrides)
    return value

  @staticmethod
  def load(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))

  def test_release_lifecycle_is_bounded_and_has_stop_hold_summary(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="route-secret", sample_interval=0.2, max_samples=4)
      for index in range(12):
        distance = 30.0 - index * 2.5
        recorder.update(
          self.condition(distance=distance),
          {"state": "holding" if index >= 9 else "braking", "holding": index >= 9,
           "output_accel": -0.4},
          vehicle={"v_ego": 0.0 if index >= 9 else 5.0, "a_ego": -0.2,
                   "standstill": index >= 9, "brake_pressed": False},
          model={"stop_detected": True, "model_length": distance + 1.0},
          lead={"present": False},
          planner={"should_stop": True, "requested_accel": -0.5},
          traffic={"active": True, "reason": "slow_close_lead", "confidence": 0.9},
          monotonic_time=index * 0.1,
          wall_time=1000.0 + index * 0.1,
        )

      path = recorder.update(
        self.condition(distance=1.5), {"state": "released", "released": True},
        vehicle={"v_ego": 0.0, "standstill": True}, monotonic_time=1.2, wall_time=1001.2,
      )
      self.assertIsNotNone(path)
      payload = self.load(path)
      self.assertEqual(payload["schema_version"], 1)
      self.assertEqual(payload["outcome"], "released")
      self.assertTrue(payload["summary"]["stopped"])
      self.assertTrue(payload["summary"]["hold_observed"])
      self.assertAlmostEqual(payload["summary"]["hold_duration_s"], 0.3)
      self.assertFalse(payload["summary"]["late_stop"])
      self.assertLessEqual(len(payload["samples"]), 4)
      self.assertGreater(payload["sampling"]["dropped_samples"], 0)
      self.assertEqual(stat.S_IMODE(os.stat(path).st_mode), 0o600)
      health = recorder.health_snapshot()
      self.assertTrue(health["healthy"])
      self.assertEqual(health["write_attempts"], 1)
      self.assertEqual(health["write_successes"], 1)
      self.assertEqual(health["writes_in_flight"], 0)

  def test_stop_summary_requires_true_standstill_at_controller_speed(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session", sample_interval=0.01)
      recorder.update(self.condition(distance=8.0), {},
                      vehicle={"v_ego": 0.05, "standstill": False},
                      monotonic_time=0.0, wall_time=100.0)
      recorder.update(self.condition(distance=6.0), {},
                      vehicle={"v_ego": recorder.standstill_speed_mps + 0.001, "standstill": True},
                      monotonic_time=0.1, wall_time=100.1)
      recorder.update(self.condition(distance=4.0), {"state": "holding", "holding": True},
                      vehicle={"v_ego": recorder.standstill_speed_mps, "standstill": True},
                      monotonic_time=0.2, wall_time=100.2)
      path = recorder.update(self.condition(distance=4.0), {"state": "released", "released": True},
                             vehicle={"v_ego": 0.0, "standstill": True},
                             monotonic_time=1.2, wall_time=101.2)

      summary = self.load(path)["summary"]
      self.assertTrue(summary["stopped"])
      self.assertEqual(summary["stop_distance_m"], 4.0)
      self.assertEqual(summary["stop_time_s"], 0.2)

  def test_provenance_is_frozen_and_navigation_freshness_is_whitelisted(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session", sample_interval=0.01)
      initial_provenance = {
        "build": {"version": "2026.7", "commit": "abc123", "branch": "test", "latitude": 37.0},
        "config": {"version": 7, "enabled": True, "map_stop_assist": True, "telemetry": True},
        "settings": {"traffic_follow": 0.5, "increased_stopped_distance": 1.25,
                     "start_accel_ramp": 0.4, "destination": "location-secret"},
        "controller": {"comfortable_decel": 1.6, "hold_s": 1.0,
                       "standstill_speed": recorder.standstill_speed_mps, "longitude": -122.0},
        "route": {"geometry": "location-secret"},
      }
      recorder.update(
        self.condition(distance=20.0), {}, provenance=initial_provenance,
        navigation={"navigation_age_s": 0.75, "fresh": True, "updated": True,
                    "valid": True, "alive": True, "latitude": 37.0},
        monotonic_time=0.0, wall_time=100.0,
      )
      path = recorder.update(
        self.condition(distance=10.0), {"released": True},
        provenance={"build": {"commit": "changed-after-start"}},
        navigation={"navigation_age_s": 1.25, "fresh": True, "updated": False,
                    "valid": True, "alive": True},
        monotonic_time=1.0, wall_time=101.0,
      )

      raw_payload = Path(path).read_text()
      self.assertNotIn("location-secret", raw_payload)
      payload = self.load(path)
      self.assertEqual(payload["provenance"]["build"]["git_revision"], "abc123")
      self.assertEqual(payload["provenance"]["config"]["config_version"], 7)
      self.assertTrue(payload["provenance"]["config"]["map_stop_assist_enabled"])
      self.assertEqual(payload["provenance"]["settings"]["traffic_follow_s"], 0.5)
      self.assertEqual(payload["provenance"]["controller_constants"]["comfortable_decel_mps2"], 1.6)
      self.assertNotIn("route", payload["provenance"])
      self.assertNotIn("latitude", payload["provenance"]["build"])
      self.assertNotIn("longitude", payload["provenance"]["controller_constants"])

      first_navigation = payload["samples"][0]["navigation"]
      self.assertEqual(first_navigation, {
        "observation_age_s": 0.75,
        "service_alive": True,
        "service_valid": True,
        "source_fresh": True,
        "source_updated": True,
      })
      self.assertNotIn("latitude", first_navigation)

  def test_passed_trial_reports_overshoot_and_late_stop(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session")
      recorder.update(self.condition(distance=5.0), {"state": "braking"},
                      vehicle={"v_ego": 1.0}, monotonic_time=0.0, wall_time=100.0)
      path = recorder.update(self.condition(distance=-1.25), {"state": "holding", "holding": True},
                             vehicle={"v_ego": 0.0, "standstill": True},
                             monotonic_time=1.0, wall_time=101.0)
      payload = self.load(path)
      self.assertEqual(payload["outcome"], "passed")
      self.assertEqual(payload["summary"]["overshoot_m"], 1.25)
      self.assertTrue(payload["summary"]["late_stop"])
      self.assertEqual(payload["summary"]["stop_distance_m"], -1.25)

  def test_invalid_context_never_begins_and_condition_loss_finishes(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session", lookahead_distance=100.0)
      invalid_conditions = (
        self.condition(distance=101.0),
        self.condition(distance=math.nan),
        self.condition(type="turn"),
        self.condition(valid=False),
        {"valid": True, "type": "stop_sign", "distance_m": 10.0},
      )
      for index, condition in enumerate(invalid_conditions):
        recorder.update(condition, {}, monotonic_time=float(index), wall_time=100.0 + index)
        self.assertFalse(recorder.active)

      recorder.update(self.condition(distance=30.0), {}, monotonic_time=10.0, wall_time=110.0)
      self.assertTrue(recorder.active)
      path = recorder.update({"valid": False}, {}, monotonic_time=11.0, wall_time=111.0)
      self.assertEqual(self.load(path)["outcome"], "condition_lost")

  def test_switch_writes_old_trial_and_starts_new_one(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session")
      recorder.update(self.condition("first", 30.0), {}, monotonic_time=0.0, wall_time=100.0)
      path = recorder.update(self.condition("second", 40.0), {}, monotonic_time=1.0, wall_time=101.0)
      first = self.load(path)
      self.assertEqual(first["outcome"], "condition_switched")
      self.assertTrue(recorder.active)
      self.assertNotEqual(first["condition"]["id"], recorder._condition_id)

      path = recorder.update(self.condition("second", 39.0), {"cancelled": True, "cancel_reason": "driver_gas"},
                             monotonic_time=2.0, wall_time=102.0)
      second = self.load(path)
      self.assertEqual(second["outcome"], "cancelled")
      self.assertEqual(second["cancel_reason"], "driver_gas")

  def test_payload_is_gps_free_and_raw_ids_are_hashed(self):
    with tempfile.TemporaryDirectory() as directory:
      raw_id = "raw-stop-id-37.123,-122.456"
      recorder = NavigationTrialRecorder(directory, session_id="raw-route-secret")
      recorder.update(
        self.condition(raw_id, 20.0, latitude=37.123, longitude=-122.456), {},
        vehicle={"v_ego": 3.0, "latitude": 37.123, "longitude": -122.456, "position": [1, 2]},
        model={"model_length": 19.0, "lat": 37.123},
        monotonic_time=0.0, wall_time=100.0,
      )
      path = recorder.update(self.condition(raw_id, 10.0), {"released": True},
                             monotonic_time=1.0, wall_time=101.0)
      raw_payload = Path(path).read_text()
      self.assertNotIn(raw_id, raw_payload)
      self.assertNotIn("raw-route-secret", raw_payload)
      payload = self.load(path)
      self.assertNotEqual(payload["condition"]["id"], payload["condition"]["stable_id"])

      second = NavigationTrialRecorder(directory, session_id="different-session")
      second.update(self.condition(raw_id, 20.0), {}, monotonic_time=2.0, wall_time=102.0)
      second_path = second.update(self.condition(raw_id, 10.0), {"released": True},
                                  monotonic_time=3.0, wall_time=103.0)
      second_payload = self.load(second_path)
      self.assertNotEqual(payload["condition"]["id"], second_payload["condition"]["id"])
      self.assertEqual(payload["condition"]["stable_id"], second_payload["condition"]["stable_id"])

      forbidden_keys = {"lat", "lon", "latitude", "longitude", "position", "coordinates", "geometry"}
      def inspect(value):
        if isinstance(value, dict):
          self.assertTrue(forbidden_keys.isdisjoint(key.lower() for key in value))
          for child in value.values():
            inspect(child)
        elif isinstance(value, list):
          for child in value:
            inspect(child)
      inspect(payload)

  def test_non_finite_diagnostics_are_omitted_from_strict_json(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session")
      recorder.update(self.condition(distance=20.0), {},
                      vehicle={"v_ego": math.nan, "a_ego": math.inf},
                      model={"model_length": -math.inf},
                      lead={"distance": math.nan},
                      planner={"target_accel": math.inf},
                      traffic={"confidence": math.nan},
                      monotonic_time=0.0, wall_time=100.0)
      path = recorder.update(self.condition(distance=10.0), {"released": True},
                             monotonic_time=1.0, wall_time=101.0)
      payload = self.load(path)
      sample = payload["samples"][0]
      for section in ("vehicle", "model", "lead", "planner", "traffic"):
        self.assertNotIn(section, sample)

  def test_failed_replace_leaves_no_partial_artifact(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session")
      recorder.update(self.condition(distance=20.0), {}, monotonic_time=0.0, wall_time=100.0)
      with patch("openpilot.selfdrive.frogpilot.controls.lib.navigation_trial_recorder.os.replace",
                 side_effect=OSError("simulated rename failure")):
        path = recorder.update(self.condition(distance=10.0), {"released": True},
                               monotonic_time=1.0, wall_time=101.0)
      self.assertIsNone(path)
      self.assertEqual(list(Path(directory).iterdir()), [])
      health = recorder.health_snapshot()
      self.assertFalse(health["healthy"])
      self.assertEqual(health["write_attempts"], 1)
      self.assertEqual(health["write_failures"], 1)
      self.assertIn("simulated rename failure", health["last_error"])

  def test_async_writer_is_single_bounded_visible_and_close_is_nonblocking(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session", async_writes=True,
                                         max_pending_writes=1)
      original_atomic_write = recorder._atomic_write
      write_started = threading.Event()
      release_write = threading.Event()
      worker_ids = []

      def slow_atomic_write(payload, sequence):
        worker_ids.append(threading.get_ident())
        write_started.set()
        if not release_write.wait(2.0):
          raise TimeoutError("test writer was not released")
        return original_atomic_write(payload, sequence)

      with patch.object(recorder, "_atomic_write", side_effect=slow_atomic_write):
        for index in range(3):
          started = float(index * 2)
          recorder.update(self.condition(f"async-{index}", 20.0), {},
                          monotonic_time=started, wall_time=100.0 + started)
          self.assertIsNone(recorder.update(
            self.condition(f"async-{index}", 10.0), {"released": True},
            monotonic_time=started + 1.0, wall_time=101.0 + started,
          ))
          if index == 0:
            self.assertTrue(write_started.wait(1.0))

        queued_health = recorder.health_snapshot()
        self.assertEqual(queued_health["write_attempts"], 3)
        self.assertEqual(queued_health["write_dropped"], 1)
        self.assertEqual(queued_health["writes_in_flight"], 2)
        self.assertEqual(queued_health["maximum_pending_writes"], 1)

        close_started = time.monotonic()
        recorder.close()
        self.assertLess(time.monotonic() - close_started, 0.2)
        self.assertTrue(recorder.health_snapshot()["closed"])

        release_write.set()
        self.assertTrue(recorder.wait_for_pending_writes(2.0))

      health = recorder.health_snapshot()
      self.assertFalse(health["healthy"])
      self.assertEqual(health["write_successes"], 2)
      self.assertEqual(health["write_failures"], 0)
      self.assertEqual(health["write_dropped"], 1)
      self.assertEqual(health["last_error"], "write_queue_full")
      self.assertEqual(len(set(worker_ids)), 1)
      self.assertEqual(len(list(Path(directory).glob("navigation-trial-v1-*.json"))), 2)

  def test_close_persists_an_open_trial_before_draining(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session", async_writes=True)
      recorder.update(self.condition(distance=20.0), {}, monotonic_time=0.0, wall_time=100.0)

      recorder.close()
      self.assertTrue(recorder.wait_for_pending_writes(2.0))

      artifacts = list(Path(directory).glob("navigation-trial-v1-*.json"))
      self.assertEqual(len(artifacts), 1)
      self.assertEqual(self.load(artifacts[0])["outcome"], "session_ended")

  def test_retention_removes_only_owned_old_artifacts(self):
    with tempfile.TemporaryDirectory() as directory:
      directory_path = Path(directory)
      unrelated = directory_path / "keep-me.json"
      unrelated.write_text("important")
      recorder = NavigationTrialRecorder(directory, session_id="session", max_files=2)
      for index in range(3):
        condition_id = f"stop-{index}"
        start = float(index * 10)
        recorder.update(self.condition(condition_id, 20.0), {},
                        monotonic_time=start, wall_time=100.0 + start)
        recorder.update(self.condition(condition_id, 10.0), {"released": True},
                        monotonic_time=start + 1.0, wall_time=101.0 + start)

      artifacts = [path for path in directory_path.glob("navigation-trial-v1-*.json")]
      self.assertEqual(len(artifacts), 2)
      self.assertEqual(unrelated.read_text(), "important")

  def test_time_must_be_finite_and_monotonic(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = NavigationTrialRecorder(directory, session_id="session")
      for invalid in (math.nan, math.inf, -1.0, True):
        recorder.update(self.condition(), {}, monotonic_time=invalid, wall_time=100.0)
        self.assertFalse(recorder.active)
      recorder.update(self.condition(), {}, monotonic_time=1.0, wall_time=-1.0)
      self.assertFalse(recorder.active)
      recorder.update(self.condition(), {}, monotonic_time=5.0, wall_time=100.0)
      recorder.update({"valid": False}, {}, monotonic_time=4.0, wall_time=101.0)
      self.assertTrue(recorder.active)


if __name__ == "__main__":
  unittest.main()
