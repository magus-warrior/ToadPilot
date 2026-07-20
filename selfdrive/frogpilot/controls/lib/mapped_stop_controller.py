"""Guarded state machine for stop conditions supplied by navigation data.

This module deliberately does not command acceleration.  It produces a
distance and speed constraint that the longitudinal planner can combine with
its model, lead, and driver constraints.  Releasing a mapped stop therefore
only removes the map constraint; it can never force the vehicle to move.
"""

import math
import time

from dataclasses import dataclass
from enum import Enum
from typing import Optional


MAP_STOP_DIAGNOSTIC_SCHEMA_VERSION = 1


class MapStopType(str, Enum):
  STOP_SIGN = "stop_sign"
  TRAFFIC_LIGHT = "traffic_light"


class MapStopState(str, Enum):
  IDLE = "idle"
  APPROACH = "approach"
  ARMED = "armed"
  HOLDING = "holding"
  RELEASED = "released"
  PASSED = "passed"
  CANCELLED = "cancelled"


@dataclass(frozen=True)
class MapStopObservation:
  """One navigation stop condition expressed along the active route."""

  condition_id: str
  condition_type: MapStopType
  along_route_distance_m: float
  confidence: float = 1.0
  age_s: float = 0.0


@dataclass(frozen=True)
class MapStopConstraint:
  """Planner-facing output and a JSON-safe diagnostic representation."""

  active: bool
  state: MapStopState
  reason: str
  condition_id: Optional[str]
  condition_type: Optional[MapStopType]
  along_route_distance_m: Optional[float]
  target_distance_m: Optional[float]
  target_speed_mps: Optional[float]
  comfortable_decel_mps2: float
  required_decel_mps2: Optional[float]
  hold_elapsed_s: float
  late_stop: bool

  def as_dict(self):
    return {
      "active": self.active,
      "state": self.state.value,
      "reason": self.reason,
      "condition_id": self.condition_id,
      "condition_type": self.condition_type.value if self.condition_type is not None else None,
      "along_route_distance_m": self.along_route_distance_m,
      "target_distance_m": self.target_distance_m,
      "target_speed_mps": self.target_speed_mps,
      "comfortable_decel_mps2": self.comfortable_decel_mps2,
      "required_decel_mps2": self.required_decel_mps2,
      "hold_elapsed_s": self.hold_elapsed_s,
      "late_stop": self.late_stop,
    }


class MappedStopController:
  """Validate, approach, stop, hold, and release one mapped condition.

  Stop signs release after a continuous one-second genuine standstill.
  Traffic lights have no timed release because route data does not contain the
  signal color; they require ``traffic_signal_clear`` or a driver gas override.
  """

  COMFORTABLE_DECEL_MPS2 = 1.6
  STOP_LINE_BUFFER_M = 1.5
  ARMING_PREVIEW_S = 1.0
  ARMING_MARGIN_M = 2.0
  HOLD_CAPTURE_DISTANCE_M = 6.0
  STANDSTILL_SPEED_MPS = 0.12
  STOP_SIGN_HOLD_S = 1.0
  # navd preserves signed distance until 2 m past a condition. Marking the
  # condition passed halfway through that window leaves room for GPS noise.
  PASSED_DISTANCE_M = -1.0

  REQUIRED_CONFIRMATION_SAMPLES = 2
  REQUIRED_ABSENCE_SAMPLES = 2
  # frogpilotNavigation runs at 1 Hz. Samples closer together than this are
  # cached planner reads, even if scheduling jitter makes their derived source
  # timestamps differ by a few milliseconds.
  DISTINCT_SOURCE_INTERVAL_S = 0.25
  # Static map data must never introduce emergency-style braking. 2.5 m/s^2 is
  # a moderate initial-deceleration ceiling above the 1.6 m/s^2 comfort target.
  MAX_INITIAL_REQUIRED_DECEL_MPS2 = 2.5

  MIN_CONFIDENCE = 0.75
  # navd normally updates around 1 Hz. Allow one delayed cycle plus modest
  # scheduling jitter, while still failing off when the feed is genuinely old.
  MAX_OBSERVATION_AGE_S = 2.5
  MAX_LOOKAHEAD_M = 1000.0
  MIN_DISTANCE_M = -100.0
  MAX_CONDITION_ID_LENGTH = 128
  DISTANCE_JUMP_BASE_M = 6.0
  DISTANCE_JUMP_PROGRESS_RATIO = 0.75
  SOURCE_TIME_EPSILON_S = 1e-3
  CACHED_DISTANCE_EPSILON_M = 1e-3
  ACTIVE_DISCONTINUITY_REASONS = frozenset({
    "changed_cached_observation",
    "jumping_along_route_distance",
    "out_of_order_observation",
  })

  def __init__(self, comfortable_decel_mps2=COMFORTABLE_DECEL_MPS2, stop_sign_hold_s=STOP_SIGN_HOLD_S):
    if not self._finite_number(comfortable_decel_mps2) or comfortable_decel_mps2 <= 0:
      raise ValueError("comfortable_decel_mps2 must be finite and positive")
    if not self._finite_number(stop_sign_hold_s) or stop_sign_hold_s < 0:
      raise ValueError("stop_sign_hold_s must be finite and non-negative")

    self.comfortable_decel_mps2 = float(comfortable_decel_mps2)
    self.stop_sign_hold_s = float(stop_sign_hold_s)
    self.rejection_count = 0
    self.last_rejection_reason = None
    self.transition_count = 0
    self._last_constraint = None
    self.reset()

  @staticmethod
  def _finite_number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)

  def reset(self):
    self.state = MapStopState.IDLE
    self.reason = "idle"
    self.condition_id = None
    self.condition_type = None
    self.distance_m = None
    self.confidence = None
    self.observation_age_s = None
    self.last_observation_source_time_s = None
    self.last_observation_distance_m = None
    self.last_runtime_update_time_s = None
    self.confirmation_count = 0
    self.last_confirmation_source_time_s = None
    self.absence_count = 0
    self.last_absence_source_time_s = None
    self.hold_started_at_s = None
    self.completed_hold_duration_s = 0.0
    self.released_condition_id = None
    self.quarantined_condition_id = None
    self.quarantined_distance_m = None
    self.quarantine_reason = None
    self.quarantined_at_s = None
    self.last_quarantined_condition_id = None
    self.last_quarantine_reason = None
    self.last_transition_time_s = None
    self._last_constraint = self._constraint(False, 0.0)

  def _transition(self, state, reason, now):
    if state != self.state or reason != self.reason:
      self.transition_count += 1
      self.last_transition_time_s = now
    self.state = state
    self.reason = reason

  @staticmethod
  def _normalize_type(condition_type):
    if isinstance(condition_type, MapStopType):
      return condition_type
    try:
      return MapStopType(condition_type)
    except (TypeError, ValueError):
      return None

  def _validate_observation(self, observation, v_ego, now, observation_source_time_s=None):
    if not isinstance(observation, MapStopObservation):
      return None, None, "missing_or_malformed_condition"

    condition_id = observation.condition_id
    if not isinstance(condition_id, str) or not condition_id.strip():
      return None, None, "missing_condition_id"
    condition_id = condition_id.strip()
    if len(condition_id) > self.MAX_CONDITION_ID_LENGTH:
      return None, None, "condition_id_too_long"

    condition_type = self._normalize_type(observation.condition_type)
    if condition_type is None:
      return None, None, "unsupported_condition_type"

    distance = observation.along_route_distance_m
    if not self._finite_number(distance) or not self.MIN_DISTANCE_M <= distance <= self.MAX_LOOKAHEAD_M:
      return None, None, "invalid_along_route_distance"

    confidence = observation.confidence
    if not self._finite_number(confidence) or not 0.0 <= confidence <= 1.0:
      return None, None, "invalid_confidence"
    if confidence < self.MIN_CONFIDENCE:
      return None, None, "low_confidence"

    age_s = observation.age_s
    if not self._finite_number(age_s) or age_s < 0.0:
      return None, None, "invalid_observation_age"
    if age_s > self.MAX_OBSERVATION_AGE_S:
      return None, None, "stale_observation"

    source_time_s = now - float(age_s) if observation_source_time_s is None else observation_source_time_s
    if (not self._finite_number(source_time_s) or source_time_s > now + self.SOURCE_TIME_EPSILON_S or
        now - source_time_s > self.MAX_OBSERVATION_AGE_S + self.SOURCE_TIME_EPSILON_S):
      return None, None, "invalid_observation_source_time"
    source_time_s = float(source_time_s)
    if (condition_id == self.condition_id and self.last_observation_distance_m is not None and
        self.last_observation_source_time_s is not None):
      source_elapsed_s = source_time_s - self.last_observation_source_time_s
      if source_elapsed_s < -self.SOURCE_TIME_EPSILON_S:
        return None, None, "out_of_order_observation"

      if source_elapsed_s < self.DISTINCT_SOURCE_INTERVAL_S:
        # The same 1 Hz nav sample is normally consumed by about twenty 20 Hz
        # planner updates. It must neither advance the continuity baseline nor
        # appear to show a stopped route distance while the car is moving.
        if abs(self.last_observation_distance_m - float(distance)) > self.CACHED_DISTANCE_EPSILON_M:
          return None, None, "changed_cached_observation"
      else:
        expected_progress_m = max(v_ego, 0.0) * source_elapsed_s
        observed_progress_m = self.last_observation_distance_m - float(distance)
        jump_tolerance_m = self.DISTANCE_JUMP_BASE_M + self.DISTANCE_JUMP_PROGRESS_RATIO * expected_progress_m
        if abs(observed_progress_m - expected_progress_m) > jump_tolerance_m:
          return None, None, "jumping_along_route_distance"

    normalized = MapStopObservation(condition_id, condition_type, float(distance), float(confidence), float(age_s))
    return normalized, source_time_s, None

  def _clear_active_condition(self, reason, now):
    self._transition(MapStopState.CANCELLED, reason, now)
    self.condition_id = None
    self.condition_type = None
    self.distance_m = None
    self.confidence = None
    self.observation_age_s = None
    self.last_observation_source_time_s = None
    self.last_observation_distance_m = None
    self.last_runtime_update_time_s = None
    self.confirmation_count = 0
    self.last_confirmation_source_time_s = None
    self.hold_started_at_s = None
    self.completed_hold_duration_s = 0.0

  def _quarantine_condition(self, condition_id, distance_m, reason, now):
    self.quarantined_condition_id = condition_id
    self.quarantined_distance_m = distance_m
    self.quarantine_reason = reason
    self.quarantined_at_s = now
    self.last_quarantined_condition_id = condition_id
    self.last_quarantine_reason = reason

  def _clear_quarantine(self):
    self.quarantined_condition_id = None
    self.quarantined_distance_m = None
    self.quarantine_reason = None
    self.quarantined_at_s = None

  def _accept_observation(self, observation, source_time_s, now, v_ego):
    condition_changed = observation.condition_id != self.condition_id
    distinct_source = (condition_changed or self.last_observation_source_time_s is None or
                       source_time_s - self.last_observation_source_time_s >= self.DISTINCT_SOURCE_INTERVAL_S)
    newly_confirmed = False
    if condition_changed:
      self.confirmation_count = 1
      self.last_confirmation_source_time_s = source_time_s
      self.hold_started_at_s = None
      self.completed_hold_duration_s = 0.0
    elif (self.last_confirmation_source_time_s is None or
          source_time_s - self.last_confirmation_source_time_s >= self.DISTINCT_SOURCE_INTERVAL_S):
      previous_count = self.confirmation_count
      self.confirmation_count = min(self.confirmation_count + 1, self.REQUIRED_CONFIRMATION_SAMPLES)
      self.last_confirmation_source_time_s = source_time_s
      newly_confirmed = (previous_count < self.REQUIRED_CONFIRMATION_SAMPLES and
                         self.confirmation_count >= self.REQUIRED_CONFIRMATION_SAMPLES)

    self.condition_id = observation.condition_id
    self.condition_type = observation.condition_type
    if distinct_source or self.distance_m is None:
      # Correct to the current planner instant when a navigation packet already
      # has measurable transport age. New packets remain the authoritative
      # baseline; cached reads are progressed smoothly below.
      self.distance_m = max(observation.along_route_distance_m - max(v_ego, 0.0) * observation.age_s,
                            self.MIN_DISTANCE_M)
    elif self.last_runtime_update_time_s is not None:
      runtime_elapsed_s = max(now - self.last_runtime_update_time_s, 0.0)
      self.distance_m = max(self.distance_m - max(v_ego, 0.0) * runtime_elapsed_s, self.MIN_DISTANCE_M)
    self.last_runtime_update_time_s = now
    self.confidence = observation.confidence
    self.observation_age_s = observation.age_s
    self.absence_count = 0
    self.last_absence_source_time_s = None
    if distinct_source:
      self.last_observation_source_time_s = source_time_s
      self.last_observation_distance_m = observation.along_route_distance_m
    return newly_confirmed

  def _valid_explicit_absence_source(self, source_time_s, now):
    return (self._finite_number(source_time_s) and
            source_time_s <= now + self.SOURCE_TIME_EPSILON_S and
            now - source_time_s <= self.MAX_OBSERVATION_AGE_S + self.SOURCE_TIME_EPSILON_S)

  def _handle_explicit_absence(self, source_time_s, v_ego, now):
    if not self._valid_explicit_absence_source(source_time_s, now):
      self.rejection_count += 1
      self.last_rejection_reason = "invalid_absence_source_time"
      if self.state not in (MapStopState.RELEASED, MapStopState.PASSED):
        self._clear_active_condition("invalid_absence_source_time", now)
      return self._constraint(False, v_ego, now)

    source_time_s = float(source_time_s)
    if (self.last_absence_source_time_s is None or
        source_time_s - self.last_absence_source_time_s >= self.DISTINCT_SOURCE_INTERVAL_S):
      self.absence_count += 1
      self.last_absence_source_time_s = source_time_s

    # A fresh explicit absence is proof that a quarantined map entry left the
    # source feed. It may be reconsidered only if it later earns confirmation
    # again from two new source samples.
    if self.quarantined_condition_id is not None:
      self._clear_quarantine()

    if self.state in (MapStopState.APPROACH, MapStopState.ARMED, MapStopState.HOLDING):
      self._clear_active_condition("condition_absent", now)
      return self._constraint(False, v_ego, now)

    if (self.state in (MapStopState.RELEASED, MapStopState.PASSED) and
        self.absence_count >= self.REQUIRED_ABSENCE_SAMPLES):
      self.released_condition_id = None
      self.condition_id = None
      self.condition_type = None
      self.distance_m = None
      self.confidence = None
      self.observation_age_s = None
      self.last_observation_source_time_s = None
      self.last_observation_distance_m = None
      self.last_runtime_update_time_s = None
      self.confirmation_count = 0
      self.last_confirmation_source_time_s = None
      self._transition(MapStopState.IDLE, "released_condition_absent_confirmed", now)

    return self._constraint(False, v_ego, now)

  def _initial_required_decel(self, v_ego):
    target_distance_m = max((self.distance_m or 0.0) - self.STOP_LINE_BUFFER_M, 0.0)
    if v_ego <= self.STANDSTILL_SPEED_MPS:
      return 0.0
    if target_distance_m <= 0.0:
      return math.inf
    return v_ego ** 2 / (2.0 * target_distance_m)

  def _arming_distance(self, v_ego):
    braking_distance_m = v_ego ** 2 / (2.0 * self.comfortable_decel_mps2)
    return (self.STOP_LINE_BUFFER_M + self.ARMING_MARGIN_M +
            v_ego * self.ARMING_PREVIEW_S + braking_distance_m)

  def _hold_elapsed(self, now):
    if self.hold_started_at_s is None:
      return self.completed_hold_duration_s
    return max(now - self.hold_started_at_s, 0.0)

  def _finish_hold(self, now):
    self.completed_hold_duration_s = self._hold_elapsed(now)
    self.hold_started_at_s = None

  def _constraint(self, active, v_ego, now=None):
    hold_elapsed_s = self._hold_elapsed(now) if now is not None else 0.0
    target_distance_m = None
    target_speed_mps = None
    required_decel_mps2 = None
    late_stop = False

    if active and self.distance_m is not None:
      target_distance_m = (0.0 if self.state == MapStopState.HOLDING else
                           max(self.distance_m - self.STOP_LINE_BUFFER_M, 0.0))
      target_speed_mps = math.sqrt(2.0 * self.comfortable_decel_mps2 * target_distance_m)
      if target_distance_m > 0.0:
        required_decel_mps2 = v_ego ** 2 / (2.0 * target_distance_m)
      elif v_ego > self.STANDSTILL_SPEED_MPS:
        # Keep diagnostics finite/JSON-safe when already at the stop line.
        required_decel_mps2 = v_ego ** 2 / 0.2
      else:
        required_decel_mps2 = 0.0
      late_stop = required_decel_mps2 > self.comfortable_decel_mps2

    constraint = MapStopConstraint(
      active=active,
      state=self.state,
      reason=self.reason,
      condition_id=self.condition_id,
      condition_type=self.condition_type,
      along_route_distance_m=self.distance_m,
      target_distance_m=target_distance_m,
      target_speed_mps=target_speed_mps,
      comfortable_decel_mps2=self.comfortable_decel_mps2,
      required_decel_mps2=required_decel_mps2,
      hold_elapsed_s=hold_elapsed_s,
      late_stop=late_stop,
    )
    self._last_constraint = constraint
    return constraint

  def update(self, observation, *, v_ego, standstill, gas_pressed, control_active=True,
             traffic_signal_clear=False, observation_source_time_s=None,
             explicit_absence=False, now=None):
    """Return the current map-only stop constraint.

    ``standstill`` must be the vehicle's actual standstill signal.  A low speed
    alone never starts the hold timer.  ``traffic_signal_clear`` must come from
    a source that knows the live signal state, never from static map timing.
    ``observation_source_time_s`` identifies the underlying 1 Hz navigation
    sample so cached planner reads cannot satisfy confirmation counters.
    ``explicit_absence`` is reserved for a fresh, valid navigation sample that
    explicitly contains no mapped stop sign.
    """
    now = time.monotonic() if now is None else now
    if not self._finite_number(now) or not self._finite_number(v_ego) or v_ego < 0.0:
      self.rejection_count += 1
      self.last_rejection_reason = "invalid_vehicle_or_time_data"
      self._clear_active_condition("invalid_vehicle_or_time_data", now if self._finite_number(now) else None)
      return self._constraint(False, 0.0)

    now = float(now)
    v_ego = float(v_ego)
    if not control_active:
      self._clear_active_condition("controls_inactive", now)
      self._clear_quarantine()
      self.released_condition_id = None
      self.absence_count = 0
      self.last_absence_source_time_s = None
      return self._constraint(False, v_ego, now)

    if observation is None and explicit_absence:
      return self._handle_explicit_absence(observation_source_time_s, v_ego, now)

    normalized, source_time_s, invalid_reason = self._validate_observation(
      observation, v_ego, now, observation_source_time_s,
    )
    if invalid_reason is not None:
      self.rejection_count += 1
      self.last_rejection_reason = invalid_reason
      if (self.state in (MapStopState.ARMED, MapStopState.HOLDING) and
          invalid_reason in self.ACTIVE_DISCONTINUITY_REASONS and
          isinstance(observation, MapStopObservation) and isinstance(observation.condition_id, str)):
        self._quarantine_condition(observation.condition_id.strip(), observation.along_route_distance_m,
                                   invalid_reason, now)
      if self.state not in (MapStopState.RELEASED, MapStopState.PASSED):
        self._clear_active_condition(invalid_reason, now)
      # Bad map updates must not make a released condition eligible again.
      # Preserve the release state/reason and record rejection separately.
      return self._constraint(False, v_ego, now)
    observation = normalized

    if self.quarantined_condition_id is not None:
      if observation.condition_id == self.quarantined_condition_id:
        self.quarantined_distance_m = observation.along_route_distance_m
        if self.state == MapStopState.PASSED or observation.along_route_distance_m <= self.PASSED_DISTANCE_M:
          self.released_condition_id = observation.condition_id
          self._transition(MapStopState.PASSED, "quarantined_condition_passed", now)
          self._clear_quarantine()
        else:
          self._transition(MapStopState.CANCELLED, "quarantined_active_discontinuity", now)
        return self._constraint(False, v_ego, now)

      # A valid, different ID proves navigation advanced to another condition.
      self._clear_quarantine()

    if observation.condition_id == self.released_condition_id:
      self._accept_observation(observation, source_time_s, now, v_ego)
      if self.state == MapStopState.PASSED or observation.along_route_distance_m <= self.PASSED_DISTANCE_M:
        self._transition(MapStopState.PASSED, "condition_passed", now)
      else:
        self._transition(MapStopState.RELEASED, self.reason, now)
      return self._constraint(False, v_ego, now)

    if self.condition_id is not None and observation.condition_id != self.condition_id:
      if self.state in (MapStopState.ARMED, MapStopState.HOLDING):
        self.rejection_count += 1
        self.last_rejection_reason = "condition_changed_while_active"
        self._quarantine_condition(observation.condition_id, observation.along_route_distance_m,
                                   "condition_changed_while_active", now)
        self._clear_active_condition("condition_changed_while_active", now)
        return self._constraint(False, v_ego, now)
      self.hold_started_at_s = None
      self.completed_hold_duration_s = 0.0

    newly_confirmed = self._accept_observation(observation, source_time_s, now, v_ego)

    if self.confirmation_count < self.REQUIRED_CONFIRMATION_SAMPLES:
      self._transition(MapStopState.APPROACH, "awaiting_consistent_observation", now)
      return self._constraint(False, v_ego, now)

    if observation.along_route_distance_m <= self.PASSED_DISTANCE_M:
      self.released_condition_id = observation.condition_id
      self._transition(MapStopState.PASSED, "condition_passed", now)
      return self._constraint(False, v_ego, now)

    if newly_confirmed:
      initial_required_decel = self._initial_required_decel(v_ego)
      if initial_required_decel > self.MAX_INITIAL_REQUIRED_DECEL_MPS2:
        self.rejection_count += 1
        self.last_rejection_reason = "unsafe_initial_required_decel"
        self._quarantine_condition(observation.condition_id, observation.along_route_distance_m,
                                   "unsafe_initial_required_decel", now)
        self._clear_active_condition("unsafe_initial_required_decel", now)
        return self._constraint(False, v_ego, now)

    if observation.condition_type == MapStopType.TRAFFIC_LIGHT and traffic_signal_clear:
      self.released_condition_id = observation.condition_id
      self._finish_hold(now)
      self._transition(MapStopState.RELEASED, "traffic_signal_clear", now)
      return self._constraint(False, v_ego, now)

    actual_standstill = bool(standstill) and v_ego <= self.STANDSTILL_SPEED_MPS
    close_to_stop = self.distance_m <= self.HOLD_CAPTURE_DISTANCE_M

    if self.state == MapStopState.HOLDING:
      if gas_pressed:
        self.released_condition_id = observation.condition_id
        self._finish_hold(now)
        self._transition(MapStopState.RELEASED, "driver_gas_override", now)
        return self._constraint(False, v_ego, now)

      if not actual_standstill:
        self.hold_started_at_s = None
        self.completed_hold_duration_s = 0.0
        self._transition(MapStopState.ARMED, "hold_interrupted", now)
        return self._constraint(True, v_ego, now)

      hold_elapsed_s = self._hold_elapsed(now)
      if observation.condition_type == MapStopType.STOP_SIGN and hold_elapsed_s >= self.stop_sign_hold_s:
        self.released_condition_id = observation.condition_id
        self._finish_hold(now)
        self._transition(MapStopState.RELEASED, "stop_sign_hold_complete", now)
        return self._constraint(False, v_ego, now)

      reason = "holding_stop_sign" if observation.condition_type == MapStopType.STOP_SIGN else "traffic_light_requires_clear"
      self._transition(MapStopState.HOLDING, reason, now)
      return self._constraint(True, v_ego, now)

    if actual_standstill and close_to_stop:
      self.completed_hold_duration_s = 0.0
      self.hold_started_at_s = now
      reason = "holding_stop_sign" if observation.condition_type == MapStopType.STOP_SIGN else "traffic_light_requires_clear"
      self._transition(MapStopState.HOLDING, reason, now)
      if gas_pressed:
        self.released_condition_id = observation.condition_id
        self._finish_hold(now)
        self._transition(MapStopState.RELEASED, "driver_gas_override", now)
        return self._constraint(False, v_ego, now)
      return self._constraint(True, v_ego, now)

    if self.state == MapStopState.ARMED or self.distance_m <= self._arming_distance(v_ego):
      self._transition(MapStopState.ARMED, "comfortable_braking_window", now)
      if gas_pressed:
        self.released_condition_id = observation.condition_id
        self._finish_hold(now)
        self._transition(MapStopState.RELEASED, "driver_gas_override", now)
        return self._constraint(False, v_ego, now)
      return self._constraint(True, v_ego, now)

    self._transition(MapStopState.APPROACH, "condition_ahead", now)
    return self._constraint(False, v_ego, now)

  def diagnostic_snapshot(self):
    snapshot = self._last_constraint.as_dict()
    snapshot.update({
      "schema_version": MAP_STOP_DIAGNOSTIC_SCHEMA_VERSION,
      "confidence": self.confidence,
      "observation_age_s": self.observation_age_s,
      "observation_source_time_s": self.last_observation_source_time_s,
      "confirmation_count": self.confirmation_count,
      "required_confirmation_samples": self.REQUIRED_CONFIRMATION_SAMPLES,
      "absence_count": self.absence_count,
      "required_absence_samples": self.REQUIRED_ABSENCE_SAMPLES,
      "rejection_count": self.rejection_count,
      "last_rejection_reason": self.last_rejection_reason,
      "transition_count": self.transition_count,
      "last_transition_time_s": self.last_transition_time_s,
      "released_condition_id": self.released_condition_id,
      "quarantine_active": self.quarantined_condition_id is not None,
      "quarantined_condition_id": self.quarantined_condition_id,
      "quarantined_distance_m": self.quarantined_distance_m,
      "quarantine_reason": self.quarantine_reason,
      "quarantined_at_s": self.quarantined_at_s,
      "last_quarantined_condition_id": self.last_quarantined_condition_id,
      "last_quarantine_reason": self.last_quarantine_reason,
      "max_initial_required_decel_mps2": self.MAX_INITIAL_REQUIRED_DECEL_MPS2,
    })
    return snapshot
