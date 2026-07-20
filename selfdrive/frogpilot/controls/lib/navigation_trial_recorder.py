import hashlib
import json
import math
import os
import queue
import re
import secrets
import threading
import time
from collections import deque

from openpilot.selfdrive.frogpilot.controls.lib.mapped_stop_controller import MappedStopController


NAVIGATION_TRIAL_SCHEMA_VERSION = 1
DEFAULT_NAVIGATION_TRIAL_DIR = "/data/toadpilot/navigation_trials"
_ARTIFACT_PREFIX = "navigation-trial-v1"
_OWNED_ARTIFACT_PATTERN = re.compile(
  rf"^{_ARTIFACT_PREFIX}-[0-9]+-[0-9a-f]{{16}}-[0-9]+\.json$"
)
_STOP_TYPES = {
  "stop": "stop_sign",
  "stop_sign": "stop_sign",
  "traffic_light": "traffic_signal",
  "traffic_signal": "traffic_signal",
}

_PROVENANCE_FIELDS = {
  "build": (("build",), {
    "software_version": ("software_version", "version"),
    "build_id": ("build_id",),
    "git_revision": ("git_revision", "revision", "commit"),
    "git_branch": ("git_branch", "branch"),
    "release_channel": ("release_channel", "channel"),
    "dirty": ("dirty",),
  }),
  "config": (("config",), {
    "config_version": ("config_version", "version"),
    "toadpilot_enabled": ("toadpilot_enabled", "enabled"),
    "profile": ("profile",),
    "map_stop_assist_enabled": ("map_stop_assist_enabled", "map_stop_assist"),
    "automatic_traffic_mode_enabled": ("automatic_traffic_mode_enabled", "automatic_traffic_mode"),
    "telemetry_enabled": ("telemetry_enabled", "telemetry"),
  }),
  "settings": (("settings",), {
    "traffic_follow_s": ("traffic_follow_s", "traffic_follow"),
    "traffic_stopping_distance_m": ("traffic_stopping_distance_m", "traffic_stopping_distance"),
    "increased_stopped_distance_m": ("increased_stopped_distance_m", "increased_stopped_distance"),
    "start_accel_ramp_mps2": ("start_accel_ramp_mps2", "start_accel_ramp"),
    "experimental_mode": ("experimental_mode",),
    "longitudinal_personality": ("longitudinal_personality",),
  }),
  "controller_constants": (("controller_constants", "controller"), {
    "comfortable_decel_mps2": ("comfortable_decel_mps2", "comfortable_decel"),
    "stop_sign_hold_s": ("stop_sign_hold_s", "hold_s"),
    "stop_line_buffer_m": ("stop_line_buffer_m", "stop_line_buffer"),
    "hold_capture_distance_m": ("hold_capture_distance_m", "hold_capture_distance"),
    "standstill_speed_mps": ("standstill_speed_mps", "standstill_speed"),
    "max_observation_age_s": ("max_observation_age_s", "max_observation_age"),
    "passed_distance_m": ("passed_distance_m", "passed_distance"),
    "arming_margin_m": ("arming_margin_m", "arming_margin"),
    "max_initial_required_decel_mps2": ("max_initial_required_decel_mps2", "max_initial_required_decel"),
    "required_confirmation_samples": ("required_confirmation_samples",),
    "required_absence_samples": ("required_absence_samples",),
  }),
}

# Only these diagnostic names can reach disk. In particular, position,
# latitude, longitude, bearings, route geometry, and raw map identifiers are
# intentionally not accepted.
_SECTION_FIELDS = {
  "vehicle": {
    "speed_mps": ("speed_mps", "speed", "v_ego"),
    "accel_mps2": ("accel_mps2", "accel", "a_ego"),
    "standstill": ("standstill",),
    "brake_pressed": ("brake_pressed",),
    "gas_pressed": ("gas_pressed",),
    "cruise_standstill": ("cruise_standstill",),
  },
  "model": {
    "stop_detected": ("stop_detected",),
    "stop_distance_m": ("stop_distance_m", "stop_distance"),
    "model_length_m": ("model_length_m", "model_length"),
    "predicted_speed_mps": ("predicted_speed_mps", "predicted_speed"),
  },
  "lead": {
    "present": ("present", "status"),
    "distance_m": ("distance_m", "distance", "d_rel"),
    "speed_mps": ("speed_mps", "speed", "v_lead"),
    "relative_speed_mps": ("relative_speed_mps", "relative_speed", "v_rel"),
  },
  "planner": {
    "should_stop": ("should_stop",),
    "forcing_stop": ("forcing_stop", "force_stop"),
    "target_speed_mps": ("target_speed_mps", "target_speed"),
    "target_accel_mps2": ("target_accel_mps2", "target_accel"),
    "requested_accel_mps2": ("requested_accel_mps2", "requested_accel"),
    "stop_distance_m": ("stop_distance_m", "stop_distance"),
  },
  "controller": {
    "state": ("state",),
    "active": ("active",),
    "holding": ("holding",),
    "released": ("released",),
    "cancelled": ("cancelled", "canceled"),
    "cancel_reason": ("cancel_reason",),
    "forcing_stop": ("forcing_stop", "force_stop"),
    "requested_accel_mps2": ("requested_accel_mps2", "requested_accel"),
    "output_accel_mps2": ("output_accel_mps2", "output_accel"),
    "required_decel_mps2": ("required_decel_mps2",),
    "along_route_distance_m": ("along_route_distance_m",),
    "hold_elapsed_s": ("hold_elapsed_s",),
    "late_stop": ("late_stop",),
    "confirmation_count": ("confirmation_count",),
    "required_confirmation_samples": ("required_confirmation_samples",),
    "absence_count": ("absence_count",),
    "rejection_count": ("rejection_count",),
    "last_rejection_reason": ("last_rejection_reason",),
    "quarantine_active": ("quarantine_active",),
    "quarantine_reason": ("quarantine_reason",),
  },
  "traffic": {
    "active": ("active", "mode_active"),
    "reason": ("reason",),
    "confidence": ("confidence",),
    "manual_override": ("manual_override",),
  },
  "navigation": {
    "observation_age_s": ("observation_age_s", "navigation_age_s", "age_s"),
    "source_fresh": ("source_fresh", "fresh"),
    "source_updated": ("source_updated", "updated"),
    "service_valid": ("service_valid", "valid"),
    "service_alive": ("service_alive", "alive"),
    "condition_valid": ("condition_valid",),
    "gps_ok": ("gps_ok",),
    "localizer_valid": ("localizer_valid",),
  },
}


def _finite_number(value):
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    return False
  try:
    return math.isfinite(float(value))
  except (OverflowError, TypeError, ValueError):
    return False


def _safe_scalar(value):
  if isinstance(value, bool):
    return value
  if _finite_number(value):
    return round(float(value), 5)
  if isinstance(value, str):
    value = value.strip()
    if value:
      return value[:80]
  return None


def _safe_provenance_scalar(value):
  if isinstance(value, bool):
    return value
  if type(value) is int:
    return value
  return _safe_scalar(value)


def _first_value(source, aliases):
  if not isinstance(source, dict):
    return None
  for alias in aliases:
    if alias in source:
      return source[alias]
  return None


def _safe_section(name, source):
  result = {}
  for output_name, aliases in _SECTION_FIELDS[name].items():
    value = _safe_scalar(_first_value(source, aliases))
    if value is not None:
      result[output_name] = value
  return result


def _safe_provenance(source):
  if not isinstance(source, dict):
    return {}

  result = {}
  for output_group, (group_aliases, fields) in _PROVENANCE_FIELDS.items():
    group_source = next((source.get(alias) for alias in group_aliases if isinstance(source.get(alias), dict)), None)
    if group_source is None:
      continue
    safe_group = {}
    for output_name, aliases in fields.items():
      value = _safe_provenance_scalar(_first_value(group_source, aliases))
      if value is not None:
        safe_group[output_name] = value
    if safe_group:
      result[output_group] = safe_group
  return result


class NavigationTrialRecorder:
  """Persist bounded, GPS-free diagnostics for mapped stopping conditions.

  ``update`` accepts caller snapshots as dictionaries. The recorder only
  serializes the explicitly whitelisted fields above. A caller may provide
  ``monotonic_time`` and ``wall_time`` for deterministic replay; production
  callers can omit both to use the local clocks.
  """
  def __init__(self, directory=DEFAULT_NAVIGATION_TRIAL_DIR, *, session_id=None,
               lookahead_distance=MappedStopController.MAX_LOOKAHEAD_M, sample_interval=0.2, max_samples=900,
               max_files=100, pass_distance=1.0, late_stop_tolerance=0.5,
               standstill_speed_mps=MappedStopController.STANDSTILL_SPEED_MPS,
               async_writes=False, max_pending_writes=4):
    if (not _finite_number(lookahead_distance) or lookahead_distance <= 0.0 or
        not _finite_number(sample_interval) or sample_interval <= 0.0 or
        type(max_samples) is not int or max_samples <= 0 or
        type(max_files) is not int or max_files <= 0 or
        not _finite_number(pass_distance) or pass_distance < 0.0 or
        not _finite_number(late_stop_tolerance) or late_stop_tolerance < 0.0 or
        not _finite_number(standstill_speed_mps) or standstill_speed_mps < 0.0 or
        type(async_writes) is not bool or
        type(max_pending_writes) is not int or max_pending_writes <= 0):
      raise ValueError("invalid NavigationTrialRecorder bounds")

    self.directory = os.path.abspath(os.fspath(directory))
    self.lookahead_distance = float(lookahead_distance)
    self.sample_interval = float(sample_interval)
    self.max_samples = max_samples
    self.max_files = max_files
    self.pass_distance = float(pass_distance)
    self.late_stop_tolerance = float(late_stop_tolerance)
    self.standstill_speed_mps = float(standstill_speed_mps)
    self.async_writes = async_writes
    self.max_pending_writes = max_pending_writes

    raw_session_id = secrets.token_hex(16) if session_id is None else str(session_id)
    self._session_secret = raw_session_id
    self.session_id = self._digest("session", raw_session_id)[:16]
    self._sequence = 0
    self._last_monotonic_time = None
    self.active = False
    self.last_written_path = None
    self._closed = False
    self._close_event = threading.Event()
    self._health_condition = threading.Condition()
    self._write_attempts = 0
    self._write_successes = 0
    self._write_failures = 0
    self._write_dropped = 0
    self._retention_failures = 0
    self._writes_in_flight = 0
    self._last_error = None
    self._last_error_at_unix = None
    self._last_success_at_unix = None
    self._write_queue = queue.Queue(maxsize=max_pending_writes) if async_writes else None
    self._write_worker_thread = None
    self._clear_trial()
    if async_writes:
      self._write_worker_thread = threading.Thread(
        target=self._write_worker, name="navigation_trial_writer", daemon=True,
      )
      self._write_worker_thread.start()

  @staticmethod
  def _digest(namespace, value):
    return hashlib.sha256(f"toadpilot-navigation-v1:{namespace}:{value}".encode("utf-8")).hexdigest()

  @staticmethod
  def _condition_values(condition):
    if not isinstance(condition, dict) or condition.get("valid") is not True:
      return None
    condition_type = condition.get("type", condition.get("condition_type"))
    if not isinstance(condition_type, str):
      return None
    condition_type = _STOP_TYPES.get(condition_type.strip().lower())
    condition_id = condition.get("id", condition.get("condition_id"))
    if condition_type is None or isinstance(condition_id, bool) or not isinstance(condition_id, (str, int)):
      return None
    condition_id = str(condition_id).strip()
    distance = condition.get("distance_m", condition.get("distance"))
    if not condition_id or not _finite_number(distance):
      return None
    return condition_id, condition_type, float(distance)

  @staticmethod
  def _controller_state(controller):
    if not isinstance(controller, dict):
      return ""
    state = controller.get("state")
    return state.strip().lower() if isinstance(state, str) else ""

  @classmethod
  def _terminal_controller_state(cls, controller):
    state = cls._controller_state(controller)
    cancelled = (isinstance(controller, dict) and
                 (controller.get("cancelled") is True or controller.get("canceled") is True))
    released = isinstance(controller, dict) and controller.get("released") is True
    if cancelled or state in {"cancelled", "canceled"}:
      return "cancelled"
    if released or state == "released":
      return "released"
    return None

  def _condition_hash(self, raw_condition_id):
    # Session salting makes a condition stable for one drive/replay while
    # preventing artifacts from becoming a cross-route location database.
    return self._digest("condition", f"{self._session_secret}:{raw_condition_id}")[:16]

  def _clear_trial(self):
    self._raw_condition_id = None
    self._condition_id = None
    self._stable_condition_id = None
    self._condition_type = None
    self._last_condition_distance = None
    self._provenance = {}
    self._trial_id = None
    self._started_monotonic = None
    self._started_wall = None
    self._last_sample_monotonic = None
    self._samples = deque(maxlen=getattr(self, "max_samples", 1))
    self._samples_dropped = 0
    self._minimum_distance = math.inf
    self._first_stop_distance = None
    self._first_stop_monotonic = None
    self._hold_started_monotonic = None
    self._hold_duration = 0.0
    self._hold_observed = False

  def _begin(self, raw_condition_id, condition_type, distance, monotonic_time, wall_time, provenance=None):
    self._sequence += 1
    self._raw_condition_id = raw_condition_id
    self._condition_id = self._condition_hash(raw_condition_id)
    self._stable_condition_id = self._digest("stable-condition", raw_condition_id)[:16]
    self._condition_type = condition_type
    self._last_condition_distance = distance
    # Freeze a narrow, GPS-free settings snapshot at trial entry. Later config
    # updates cannot rewrite the provenance of an already-active trial.
    self._provenance = _safe_provenance(provenance)
    self._trial_id = self._digest(
      "trial", f"{self._session_secret}:{raw_condition_id}:{self._sequence}:{monotonic_time}"
    )[:16]
    self._started_monotonic = monotonic_time
    self._started_wall = wall_time
    self._minimum_distance = distance
    self.active = True

  def _update_summary(self, distance, vehicle, controller, monotonic_time):
    self._minimum_distance = min(self._minimum_distance, distance)
    raw_speed = _first_value(vehicle, _SECTION_FIELDS["vehicle"]["speed_mps"])
    standstill = (isinstance(vehicle, dict) and vehicle.get("standstill") is True and
                  _finite_number(raw_speed) and max(float(raw_speed), 0.0) <= self.standstill_speed_mps)
    if standstill and self._first_stop_distance is None:
      self._first_stop_distance = distance
      self._first_stop_monotonic = monotonic_time

    controller_state = self._controller_state(controller)
    holding = (isinstance(controller, dict) and controller.get("holding") is True) or controller_state == "holding"
    if holding and self._hold_started_monotonic is None:
      self._hold_started_monotonic = monotonic_time
      self._hold_observed = True
    elif not holding and self._hold_started_monotonic is not None:
      self._hold_duration += max(0.0, monotonic_time - self._hold_started_monotonic)
      self._hold_started_monotonic = None

  def _make_sample(self, condition_type, distance, monotonic_time, wall_time, **sections):
    sample = {
      "elapsed_s": round(max(0.0, monotonic_time - self._started_monotonic), 3),
      "wall_time_unix": round(wall_time, 3),
      "condition": {
        "type": condition_type,
        "distance_m": round(distance, 3),
      },
    }
    for section_name in _SECTION_FIELDS:
      safe_values = _safe_section(section_name, sections.get(section_name))
      if safe_values:
        sample[section_name] = safe_values
    return sample

  def _append_sample(self, condition_type, distance, monotonic_time, wall_time, terminal=False, **sections):
    sample_due = (self._last_sample_monotonic is None or terminal or
                  monotonic_time - self._last_sample_monotonic + 1e-9 >= self.sample_interval)
    if not sample_due:
      return
    if len(self._samples) == self.max_samples:
      self._samples_dropped += 1
    self._samples.append(self._make_sample(condition_type, distance, monotonic_time, wall_time, **sections))
    self._last_sample_monotonic = monotonic_time

  def _build_payload(self, outcome, monotonic_time, wall_time, controller):
    hold_duration = self._hold_duration
    if self._hold_started_monotonic is not None:
      hold_duration += max(0.0, monotonic_time - self._hold_started_monotonic)

    terminal_stop_outcome = outcome in {"released", "passed"}
    stopped = self._first_stop_distance is not None
    late_stop = terminal_stop_outcome and (
      not stopped or self._first_stop_distance < -self.late_stop_tolerance
    )
    cancel_reason = _safe_scalar(_first_value(controller, ("cancel_reason",)))
    payload = {
      "schema_version": NAVIGATION_TRIAL_SCHEMA_VERSION,
      "artifact_type": "mapped_navigation_stop_trial",
      "privacy": "gps_free_hashed_condition_ids",
      "session_id": self.session_id,
      "trial_id": self._trial_id,
      "condition": {
        "id": self._condition_id,
        "stable_id": self._stable_condition_id,
        "type": self._condition_type,
      },
      "provenance": self._provenance,
      "started_at_unix": round(self._started_wall, 3),
      "completed_at_unix": round(max(self._started_wall, wall_time), 3),
      "duration_s": round(max(0.0, monotonic_time - self._started_monotonic), 3),
      "outcome": outcome,
      "summary": {
        "stopped": stopped,
        "stop_distance_m": round(self._first_stop_distance, 3) if stopped else None,
        "stop_time_s": (round(self._first_stop_monotonic - self._started_monotonic, 3)
                        if stopped else None),
        "hold_observed": self._hold_observed,
        "hold_duration_s": round(hold_duration, 3),
        "minimum_distance_m": round(self._minimum_distance, 3),
        "overshoot_m": round(max(0.0, -self._minimum_distance), 3),
        "late_stop_evaluable": terminal_stop_outcome,
        "late_stop": late_stop,
      },
      "sampling": {
        "minimum_interval_s": self.sample_interval,
        "retained_samples": len(self._samples),
        "dropped_samples": self._samples_dropped,
        "maximum_samples": self.max_samples,
      },
      "samples": list(self._samples),
    }
    if outcome == "cancelled" and isinstance(cancel_reason, str):
      payload["cancel_reason"] = cancel_reason
    return payload

  @staticmethod
  def _error_text(error):
    detail = str(error).strip()
    return f"{type(error).__name__}: {detail[:160]}" if detail else type(error).__name__

  def _atomic_write(self, payload, sequence):
    temp_path = None
    try:
      # Serialization happens before creating a temporary file. allow_nan=False
      # is the final guard against invalid diagnostic numbers reaching disk.
      encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
      os.makedirs(self.directory, mode=0o700, exist_ok=True)
      completed_ms = int(round(payload["completed_at_unix"] * 1000.0))
      filename = f"{_ARTIFACT_PREFIX}-{completed_ms}-{payload['trial_id']}-{sequence}.json"
      final_path = os.path.join(self.directory, filename)
      temp_name = f".{filename}.{os.getpid()}.{secrets.token_hex(6)}.tmp"
      temp_path = os.path.join(self.directory, temp_name)

      file_descriptor = os.open(temp_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
      with os.fdopen(file_descriptor, "wb") as stream:
        stream.write(encoded)
        stream.flush()
        os.fsync(stream.fileno())
      os.chmod(temp_path, 0o600)
      os.replace(temp_path, final_path)
      temp_path = None

      # Best-effort directory sync completes the durable atomic rename on
      # filesystems that support fsync on directories.
      try:
        directory_fd = os.open(self.directory, os.O_RDONLY)
        try:
          os.fsync(directory_fd)
        finally:
          os.close(directory_fd)
      except OSError:
        pass
      retention_error = self._enforce_retention()
      return final_path, None, retention_error
    except (OSError, TypeError, ValueError, OverflowError) as error:
      if temp_path is not None:
        try:
          os.unlink(temp_path)
        except OSError:
          pass
      return None, self._error_text(error), None

  def _enforce_retention(self):
    errors = []
    try:
      owned_files = []
      with os.scandir(self.directory) as entries:
        for entry in entries:
          if _OWNED_ARTIFACT_PATTERN.fullmatch(entry.name) and entry.is_file(follow_symlinks=False):
            owned_files.append((entry.stat(follow_symlinks=False).st_mtime_ns, entry.name, entry.path))
      owned_files.sort()
      for _, _, path in owned_files[:-self.max_files]:
        try:
          os.unlink(path)
        except OSError as error:
          errors.append(self._error_text(error))
    except OSError as error:
      errors.append(self._error_text(error))
    return errors[0] if errors else None

  def _complete_write(self, path, error, retention_error):
    completed_at = time.time()
    with self._health_condition:
      self._writes_in_flight = max(0, self._writes_in_flight - 1)
      if path is not None:
        self._write_successes += 1
        self._last_success_at_unix = completed_at
        self.last_written_path = path
      else:
        self._write_failures += 1
      if retention_error is not None:
        self._retention_failures += 1
      visible_error = error or retention_error
      if visible_error is not None:
        self._last_error = visible_error
        self._last_error_at_unix = completed_at
      self._health_condition.notify_all()

  def _write_worker(self):
    while True:
      if self._close_event.is_set() and self._write_queue.empty():
        break
      try:
        payload, sequence = self._write_queue.get(timeout=0.1)
      except queue.Empty:
        continue

      try:
        path, error, retention_error = self._atomic_write(payload, sequence)
      except Exception as error:  # Keep diagnostics failures from killing the single worker.
        path, error, retention_error = None, self._error_text(error), None
      finally:
        self._write_queue.task_done()
      self._complete_write(path, error, retention_error)

    with self._health_condition:
      self._health_condition.notify_all()

  def _submit_async_write(self, payload, sequence):
    with self._health_condition:
      self._write_attempts += 1
      if self._closed:
        self._write_dropped += 1
        self._last_error = "writer_closed"
        self._last_error_at_unix = time.time()
        self._health_condition.notify_all()
        return
      try:
        # Holding the health lock until the in-flight counter is incremented
        # prevents the worker from completing a just-submitted job first.
        self._write_queue.put_nowait((payload, sequence))
      except queue.Full:
        self._write_dropped += 1
        self._last_error = "write_queue_full"
        self._last_error_at_unix = time.time()
        self._health_condition.notify_all()
        return
      self._writes_in_flight += 1

  def _write_sync(self, payload, sequence):
    with self._health_condition:
      self._write_attempts += 1
      self._writes_in_flight += 1
    try:
      path, error, retention_error = self._atomic_write(payload, sequence)
    except Exception as error:
      path, error, retention_error = None, self._error_text(error), None
    self._complete_write(path, error, retention_error)
    return path

  def health_snapshot(self):
    """Return JSON-safe writer health without blocking on pending I/O."""
    with self._health_condition:
      worker_alive = bool(self._write_worker_thread and self._write_worker_thread.is_alive())
      healthy = not (self._write_failures or self._write_dropped or self._retention_failures)
      if self.async_writes and not self._closed and not worker_alive:
        healthy = False
      return {
        "schema_version": 1,
        "async_writes": self.async_writes,
        "closed": self._closed,
        "healthy": healthy,
        "worker_alive": worker_alive,
        "write_attempts": self._write_attempts,
        "write_successes": self._write_successes,
        "write_failures": self._write_failures,
        "write_dropped": self._write_dropped,
        "retention_failures": self._retention_failures,
        "writes_in_flight": self._writes_in_flight,
        "queued_writes": self._write_queue.qsize() if self._write_queue is not None else 0,
        "maximum_pending_writes": self.max_pending_writes if self.async_writes else 0,
        "last_written_path": self.last_written_path,
        "last_success_at_unix": self._last_success_at_unix,
        "last_error": self._last_error,
        "last_error_at_unix": self._last_error_at_unix,
      }

  def wait_for_pending_writes(self, timeout=None):
    """Wait for deterministic tests/shutdown orchestration; normal updates never call this."""
    if timeout is not None and (not _finite_number(timeout) or timeout < 0.0):
      raise ValueError("timeout must be finite and non-negative")
    deadline = None if timeout is None else time.monotonic() + float(timeout)
    with self._health_condition:
      while self._writes_in_flight:
        remaining = None if deadline is None else deadline - time.monotonic()
        if remaining is not None and remaining <= 0.0:
          return False
        self._health_condition.wait(remaining)
      return True

  def close(self):
    """Finish any open trial, then ask the daemon writer to drain without waiting."""
    if self._closed:
      return
    if self.active:
      monotonic_time = max(time.monotonic(), self._last_monotonic_time or 0.0)
      wall_time = max(time.time(), self._started_wall or 0.0)
      self._finish("session_ended", monotonic_time, wall_time, {})
    with self._health_condition:
      self._closed = True
      self._close_event.set()
      self._health_condition.notify_all()

  def _finish(self, outcome, monotonic_time, wall_time, controller):
    payload = self._build_payload(outcome, monotonic_time, wall_time, controller)
    sequence = self._sequence
    if self.async_writes:
      self._submit_async_write(payload, sequence)
      path = None
    else:
      path = self._write_sync(payload, sequence)
    self.active = False
    self._clear_trial()
    return path

  def update(self, condition, controller=None, *, vehicle=None, model=None, lead=None,
             planner=None, traffic=None, navigation=None, provenance=None,
             monotonic_time=None, wall_time=None):
    """Consume one stopping-context snapshot and return a completed artifact path.

    Returns ``None`` while a trial remains active, when no valid trial can
    begin, if persistence fails, or when an async write was queued. A valid
    beginning requires an explicitly valid stop condition with an ID and a
    finite 0..lookahead distance. ``provenance`` is frozen only at that start.
    """
    if self._closed:
      return None
    monotonic_time = time.monotonic() if monotonic_time is None else monotonic_time
    wall_time = time.time() if wall_time is None else wall_time
    if (not _finite_number(monotonic_time) or float(monotonic_time) < 0.0 or
        not _finite_number(wall_time) or float(wall_time) < 0.0):
      return None
    monotonic_time = float(monotonic_time)
    wall_time = float(wall_time)
    if self._last_monotonic_time is not None and monotonic_time < self._last_monotonic_time:
      return None
    self._last_monotonic_time = monotonic_time

    controller = controller if isinstance(controller, dict) else {}
    terminal_controller_state = self._terminal_controller_state(controller)
    values = self._condition_values(condition)

    if self.active:
      if terminal_controller_state is not None:
        if values is not None and values[0] == self._raw_condition_id:
          _, condition_type, distance = values
          self._last_condition_distance = distance
          self._update_summary(distance, vehicle, controller, monotonic_time)
          self._append_sample(condition_type, distance, monotonic_time, wall_time, terminal=True,
                              vehicle=vehicle, model=model, lead=lead, planner=planner,
                              controller=controller, traffic=traffic, navigation=navigation)
        elif self._last_condition_distance is not None:
          self._append_sample(self._condition_type, self._last_condition_distance, monotonic_time, wall_time,
                              terminal=True, vehicle=vehicle, model=model, lead=lead, planner=planner,
                              controller=controller, traffic=traffic, navigation=navigation)
        return self._finish(terminal_controller_state, monotonic_time, wall_time, controller)

      if values is None:
        if self._last_condition_distance is not None:
          self._append_sample(self._condition_type, self._last_condition_distance, monotonic_time, wall_time,
                              terminal=True, vehicle=vehicle, model=model, lead=lead, planner=planner,
                              controller=controller, traffic=traffic, navigation=navigation)
        return self._finish("condition_lost", monotonic_time, wall_time, controller)

      raw_condition_id, condition_type, distance = values
      if raw_condition_id != self._raw_condition_id or condition_type != self._condition_type:
        path = self._finish("condition_switched", monotonic_time, wall_time, controller)
        if 0.0 <= distance <= self.lookahead_distance:
          self._begin(raw_condition_id, condition_type, distance, monotonic_time, wall_time, provenance)
          self._update_summary(distance, vehicle, controller, monotonic_time)
          self._append_sample(condition_type, distance, monotonic_time, wall_time,
                              vehicle=vehicle, model=model, lead=lead, planner=planner,
                              controller=controller, traffic=traffic, navigation=navigation)
        return path

      if distance > self.lookahead_distance:
        return self._finish("condition_lost", monotonic_time, wall_time, controller)

      passed = ((isinstance(condition, dict) and condition.get("passed") is True) or
                distance <= -self.pass_distance)
      self._last_condition_distance = distance
      self._update_summary(distance, vehicle, controller, monotonic_time)
      self._append_sample(condition_type, distance, monotonic_time, wall_time, terminal=passed,
                          vehicle=vehicle, model=model, lead=lead, planner=planner,
                          controller=controller, traffic=traffic, navigation=navigation)
      if passed:
        return self._finish("passed", monotonic_time, wall_time, controller)
      return None

    if values is None or terminal_controller_state is not None:
      return None
    raw_condition_id, condition_type, distance = values
    if not 0.0 <= distance <= self.lookahead_distance:
      return None

    self._begin(raw_condition_id, condition_type, distance, monotonic_time, wall_time, provenance)
    self._update_summary(distance, vehicle, controller, monotonic_time)
    self._append_sample(condition_type, distance, monotonic_time, wall_time,
                        vehicle=vehicle, model=model, lead=lead, planner=planner,
                        controller=controller, traffic=traffic, navigation=navigation)
    return None
