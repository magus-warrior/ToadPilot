import math
from collections import deque
from dataclasses import dataclass


def blend_traffic_mode_value(normal_value, traffic_value, blend):
  """Linearly blend one finite setting without allowing a bad blend input to jump modes."""
  safe_blend = (max(0.0, min(float(blend), 1.0))
                if isinstance(blend, (int, float)) and not isinstance(blend, bool) and math.isfinite(blend)
                else 0.0)
  return float(normal_value + (traffic_value - normal_value) * safe_blend)


def valid_auto_traffic_lead(lead_present, *, radarless_model, model_probability=0.0,
                            probability_threshold=0.5):
  """Reject the always-present low-probability model lead slots on radarless cars."""
  if not lead_present:
    return False
  if not radarless_model:
    return True
  return (isinstance(model_probability, (int, float)) and not isinstance(model_probability, bool) and
          math.isfinite(model_probability) and isinstance(probability_threshold, (int, float)) and
          not isinstance(probability_threshold, bool) and math.isfinite(probability_threshold) and
          model_probability > probability_threshold)


@dataclass(frozen=True)
class AutoTrafficModeConfig:
  """Thresholds for the automatic Traffic Mode recommendation."""
  low_speed: float = 13.0                 # m/s (29 mph)
  free_flow_speed: float = 17.0           # m/s (38 mph)
  slow_lead_speed: float = 13.0           # m/s
  close_lead_min_distance: float = 12.0   # m
  close_lead_max_distance: float = 40.0   # m
  close_lead_time_gap: float = 2.5        # s
  close_lead_distance_offset: float = 8.0  # m

  lead_entry_time: float = 6.0            # s
  intersection_entry_time: float = 4.5    # s
  free_flow_exit_time: float = 2.0       # s
  evidence_decay_rate: float = 0.5        # evidence seconds lost per second

  stopped_speed: float = 0.3              # m/s
  launched_speed: float = 2.0             # m/s
  stopped_dwell_time: float = 0.75        # s
  stop_launches_required: int = 2
  stop_launch_window: float = 90.0         # s

  # A single delayed planner update must not count an unobserved gap as
  # sustained congestion or standstill. Wall time still advances so old stop
  # and launch events expire normally.
  max_observation_dt: float = 0.5          # s


@dataclass(frozen=True)
class AutoTrafficModeStatus:
  active: bool
  reason: str
  confidence: float
  changed: bool
  close_lead: bool
  close_slow_lead: bool
  congestion_time: float
  free_flow_time: float
  stop_launch_cycles: int


class AutoTrafficMode:
  """Hysteretic Traffic Mode recommendation based on observed road behavior.

  ``dt`` passed to :meth:`update` must be elapsed monotonic time. The class
  intentionally owns no clock, which makes both planner use and replay tests
  deterministic.

  A nearby, slow lead at low ego speed must persist before activation. Two
  genuine stop-to-launch cycles within a rolling window can strengthen
  congestion evidence only while a slow nearby lead is currently present. Navigation intersection data can only strengthen the
  first signal; it cannot activate Traffic Mode on its own.
  """
  def __init__(self, config=None):
    self.config = config or AutoTrafficModeConfig()
    self.reset()

  def reset(self):
    self.active = False
    self.activation_reason = "monitoring"
    self.elapsed_time = 0.0
    self.congestion_time = 0.0
    self.free_flow_time = 0.0

    self.stopped_time = 0.0
    self.stop_armed = False
    self.stop_launches = deque()

  def set_thresholds(self, *, max_speed_mph, entry_time):
    """Apply validated user thresholds without discarding accumulated evidence."""
    max_speed_mps = float(max_speed_mph) * 0.44704
    self.config = AutoTrafficModeConfig(
      **{**self.config.__dict__, "low_speed": max_speed_mps, "free_flow_speed": max_speed_mps,
         "lead_entry_time": float(entry_time)}
    )

  def _close_distance(self, v_ego):
    config = self.config
    return min(config.close_lead_max_distance,
               max(config.close_lead_min_distance,
                   v_ego * config.close_lead_time_gap + config.close_lead_distance_offset))

  def _update_stop_launches(self, v_ego, observation_dt):
    config = self.config
    if v_ego <= config.stopped_speed:
      self.stopped_time += observation_dt
      if self.stopped_time >= config.stopped_dwell_time:
        self.stop_armed = True
    elif self.stop_armed and v_ego >= config.launched_speed:
      self.stop_launches.append(self.elapsed_time)
      self.stop_armed = False
      self.stopped_time = 0.0
    elif not self.stop_armed:
      self.stopped_time = 0.0

    while self.stop_launches and self.elapsed_time - self.stop_launches[0] > config.stop_launch_window:
      self.stop_launches.popleft()

  def _inactive_confidence(self, approaching_intersection):
    config = self.config
    required_time = config.intersection_entry_time if approaching_intersection else config.lead_entry_time
    lead_confidence = self.congestion_time / max(required_time, 1e-3)
    cycle_confidence = len(self.stop_launches) / max(config.stop_launches_required, 1)
    return min(1.0, max(lead_confidence, cycle_confidence))

  def update(self, *, v_ego, lead_present, lead_distance=math.inf, lead_speed=None,
             approaching_intersection=False, dt):
    """Update and return an immutable status snapshot.

    Invalid sensor values fail conservatively: they cannot create congestion
    evidence. Invalid ``dt`` is a caller error because time going backwards
    would invalidate every hysteresis timer.
    """
    if isinstance(dt, bool) or not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt < 0.0:
      raise ValueError("dt must be a finite, non-negative monotonic duration")

    dt = float(dt)
    self.elapsed_time += dt
    observation_dt = min(dt, self.config.max_observation_dt)

    valid_ego_speed = isinstance(v_ego, (int, float)) and not isinstance(v_ego, bool) and math.isfinite(v_ego)
    safe_v_ego = max(0.0, float(v_ego)) if valid_ego_speed else math.inf
    valid_lead_distance = (isinstance(lead_distance, (int, float)) and not isinstance(lead_distance, bool) and
                           math.isfinite(lead_distance) and lead_distance >= 0.0)
    valid_lead_speed = (isinstance(lead_speed, (int, float)) and not isinstance(lead_speed, bool) and
                        math.isfinite(lead_speed))

    close_lead = bool(lead_present and valid_ego_speed and valid_lead_distance and
                      lead_distance <= self._close_distance(safe_v_ego))
    close_slow_lead = bool(close_lead and valid_lead_speed and lead_speed <= self.config.slow_lead_speed)
    congestion_observed = (safe_v_ego <= self.config.low_speed and
                           safe_v_ego < self.config.free_flow_speed and close_slow_lead)

    if congestion_observed:
      self.congestion_time = min(self.config.lead_entry_time, self.congestion_time + observation_dt)
    else:
      self.congestion_time = max(0.0, self.congestion_time - observation_dt * self.config.evidence_decay_rate)

    if valid_ego_speed:
      self._update_stop_launches(safe_v_ego, observation_dt)
    else:
      # An invalid speed cannot complete a stop/launch transition.
      self.stopped_time = 0.0

    changed = False
    transition_reason = None
    if not self.active:
      required_time = (self.config.intersection_entry_time if approaching_intersection
                       else self.config.lead_entry_time)
      repeated_stop_and_go = len(self.stop_launches) >= self.config.stop_launches_required
      if repeated_stop_and_go and congestion_observed:
        self.active = True
        self.activation_reason = "repeated_stop_and_go"
        changed = True
      elif congestion_observed and self.congestion_time + 1e-9 >= required_time:
        self.active = True
        self.activation_reason = ("slow_close_lead_near_intersection" if approaching_intersection
                                  else "slow_close_lead")
        changed = True

      self.free_flow_time = 0.0
    else:
      # Speed takes precedence above the configured threshold. Below it,
      # loss of a nearby lead also releases the profile after a short dwell.
      # Invalid speed must never count as observed free flow.
      free_flow_observed = valid_ego_speed and (safe_v_ego >= self.config.free_flow_speed or not close_lead)
      if free_flow_observed:
        self.free_flow_time += observation_dt
      else:
        # A brief lead dropout or speed spike must not change profiles.
        self.free_flow_time = 0.0

      if self.free_flow_time + 1e-9 >= self.config.free_flow_exit_time:
        self.active = False
        self.activation_reason = "monitoring"
        self.congestion_time = 0.0
        self.free_flow_time = 0.0
        self.stop_launches.clear()
        self.stop_armed = False
        self.stopped_time = 0.0
        changed = True
        transition_reason = "free_flow"

    if self.active:
      confidence = max(0.0, 1.0 - self.free_flow_time / max(self.config.free_flow_exit_time, 1e-3))
      reason = self.activation_reason
    else:
      confidence = self._inactive_confidence(approaching_intersection)
      if transition_reason is not None:
        reason = transition_reason
      elif self.congestion_time > 0.0:
        reason = "building_slow_close_lead"
      elif self.stop_launches:
        reason = "building_stop_and_go"
      else:
        reason = "monitoring"

    return AutoTrafficModeStatus(
      active=self.active,
      reason=reason,
      confidence=confidence,
      changed=changed,
      close_lead=close_lead,
      close_slow_lead=close_slow_lead,
      congestion_time=self.congestion_time,
      free_flow_time=self.free_flow_time,
      stop_launch_cycles=len(self.stop_launches),
    )
