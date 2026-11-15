"""Bounded recovery from a persistent model-path bias in the active steering flow.
Positive offset means the car is left of the lane center; positive curvature
means a right turn in model coordinates. No fixed directional offset is used.
"""
from dataclasses import dataclass
import math
from bisect import bisect_right

from openpilot.selfdrive.toad_modeld.starpilot_response import response_enabled


# Disabled by request for model baseline testing; this is read at process start.
PATH_RECOVERY_ENABLED = False


@dataclass(frozen=True)
class Observation:
  now: float
  model_time: float
  frame: int
  speed: float
  curvature: float
  lane_offset: float
  path_bias: float
  lane_width: float
  confidence: float
  geometry_valid: bool = True
  active: bool = True
  override: bool = False
  lane_change: bool = False
  blinker: bool = False


class PathRecovery:
  VERSION = 2
  DEAD_BAND_M = .10
  PATH_RELEASE_BAND_M = .03
  CONFIRM_SECONDS = 1.5
  MAX_CURVATURE = .0015
  MAX_LAT_ACCEL = .12
  CURVATURE_RATE = .0003
  LAT_ACCEL_RATE = .03

  def __init__(self):
    self.previous_time = None
    self.previous_frame = None
    self.evidence_time = None
    self.evidence_direction = 0
    self.confirmed = False
    self.trim = 0.

  def reset(self):
    self.evidence_time = None
    self.evidence_direction = 0
    self.confirmed = False
    self.trim = 0.

  def update(self, obs):
    values = (obs.now, obs.model_time, obs.speed, obs.curvature, obs.lane_offset,
              obs.path_bias, obs.lane_width, obs.confidence)
    valid = (all(math.isfinite(v) for v in values)
             and obs.geometry_valid and obs.active and not obs.override
             and not obs.lane_change and not obs.blinker
             and .5 <= obs.speed <= 35.
             and 0 <= obs.now - obs.model_time <= .2
             and 2.6 <= obs.lane_width <= 4.2 and obs.confidence >= .9
             and abs(obs.curvature) <= .005
             and abs(obs.lane_offset) <= .5 and abs(obs.path_bias) <= .5)
    dt = 0. if self.previous_time is None else obs.now - self.previous_time
    self.previous_time = obs.now
    if not valid or not 0 <= dt <= .2 or (self.previous_frame is not None and obs.frame < self.previous_frame):
      self.reset()
      self.previous_frame = obs.frame
      return 0., 'gated'
    fresh_frame = self.previous_frame is None or obs.frame > self.previous_frame
    self.previous_frame = obs.frame
    # Only intervene when the car and the predicted path remain off-center on
    # the same side. A path already recovering toward center needs no trim.
    # Once confirmed, a small path-estimate fluctuation must not discard all
    # evidence and restart the 1.5 s timer. Keep the original entry threshold
    # and correction target, but use a lower path threshold to release evidence.
    # The existing slew limit fades trim toward zero during a brief weak bias.
    path_threshold = self.PATH_RELEASE_BAND_M if self.confirmed else self.DEAD_BAND_M
    persistent_bias = (abs(obs.lane_offset) > self.DEAD_BAND_M
                       and abs(obs.path_bias) > path_threshold
                       and obs.lane_offset * obs.path_bias < 0)
    if not persistent_bias:
      self.reset()
      return 0., 'centered_or_recovering'
    direction = 1 if obs.lane_offset > 0 else -1
    if self.evidence_direction != direction:
      self.reset()
      self.evidence_direction = direction
      self.evidence_time = obs.model_time
    if fresh_frame and obs.model_time - self.evidence_time >= self.CONFIRM_SECONDS:
      self.confirmed = True
    if not self.confirmed:
      return self.trim, 'confirming'
    distance = max(12., min(45., obs.speed * 1.5))
    bias = max(0., min(abs(obs.lane_offset), abs(obs.path_bias)) - self.DEAD_BAND_M)
    bound = min(self.MAX_CURVATURE, self.MAX_LAT_ACCEL / obs.speed**2)
    target = direction * min(bound, 2 * bias / distance**2)
    step = min(self.CURVATURE_RATE, self.LAT_ACCEL_RATE / obs.speed**2) * dt
    self.trim += max(-step, min(step, target-self.trim))
    self.trim = max(-bound, min(bound, self.trim))
    state = 'recovering' if bias > 0 else ('releasing' if self.trim else 'waiting_for_path_bias')
    return self.trim, state


def recovery_enabled(car_params, controller, toggles):
  """Explicit baseline switch, then supported-controller compatibility checks."""
  return (PATH_RECOVERY_ENABLED and car_params.carName == 'toyota' and car_params.carFingerprint == 'TOYOTA_CAMRY_TSS2'
          and car_params.lateralTuning.which() == 'torque' and response_enabled(toggles)
          and not getattr(controller, 'use_nnff', False) and not getattr(controller, 'use_nnff_lite', False))


def recovery_inputs_valid(sm, car_state):
  # controlsd receives carState on a dedicated socket. It is deliberately not
  # one of sm's subscriptions, so passing it to sm.all_checks raises KeyError.
  return bool(car_state.canValid and sm.all_checks(['modelV2', 'liveParameters']))


@dataclass(frozen=True)
class LaneGeometry:
  valid: bool = False
  lane_offset: float = 0.
  path_bias: float = 0.
  lane_width: float = 0.
  confidence: float = 0.


def lane_geometry(model, speed, camera_offset=.04):
  """Measure lane/path agreement; invalid or uncertain geometry fails closed."""
  try:
    probabilities = [float(model.laneLineProbs[i]) for i in (1, 2)]
    confidence = min(probabilities)
    uncertainty = [float(model.laneLineStds[i]) for i in (1, 2)]
    if (any(not math.isfinite(v) or not .9 <= v <= 1. for v in probabilities)
        or any(not math.isfinite(v) or not 0 <= v <= .35 for v in uncertainty)
        or not math.isfinite(speed) or not math.isfinite(camera_offset)):
      return LaneGeometry()

    def xy(series):
      xs, ys = list(map(float, series.x)), list(map(float, series.y))
      if (len(xs) != len(ys) or len(xs) < 2
          or not all(math.isfinite(v) for v in (*xs, *ys))
          or any(b <= a for a, b in zip(xs, xs[1:]))):
        raise ValueError('Invalid geometry')
      return xs, ys

    def sample(points, distance):
      xs, ys = points
      if not xs[0] <= distance <= xs[-1]:
        raise ValueError('Short geometry')
      index = min(len(xs)-2, max(0, bisect_right(xs, distance)-1))
      fraction = (distance-xs[index])/(xs[index+1]-xs[index])
      return ys[index] + fraction*(ys[index+1]-ys[index])

    left, right, path = xy(model.laneLines[1]), xy(model.laneLines[2]), xy(model.position)
    near = (0., 2.5, 5., 7.5, 10.)
    lookahead = max(12., min(30., speed*.8))
    centers, widths = [], []
    for distance in (*near, lookahead):
      l, r = sample(left, distance), sample(right, distance)
      centers.append((l+r)*.5)
      widths.append(r-l)
    if any(not 2.6 <= width <= 4.2 for width in widths):
      return LaneGeometry()
    mean_y = sum(centers[:5])/5
    slope = sum((x-5.)*(y-mean_y) for x, y in zip(near, centers[:5]))/62.5
    offset = mean_y - 5.*slope + camera_offset
    bias = sample(path, lookahead)-centers[-1]
    if not all(math.isfinite(v) for v in (offset, bias)):
      return LaneGeometry()
    return LaneGeometry(True, offset, bias, widths[0], confidence)
  except (AttributeError, TypeError, ValueError, IndexError, OverflowError):
    return LaneGeometry()


class ModelPathRecovery:
  """20 Hz geometry and persistent evidence, with a 100 Hz correction ramp."""
  def __init__(self):
    self.recovery = PathRecovery()
    self.frame = None
    self.geometry = LaneGeometry()
    self.status = {}

  def update(self, model, *, speed, curvature, now, model_time, model_valid,
             enabled, active, override, blinker, limited=False):
    try:
      frame = int(model.frameId)
      lane_change = str(model.meta.laneChangeState) != 'off'
      turning = str(model.meta.turnDirection) != 'none'
    except (AttributeError, TypeError, ValueError):
      frame, lane_change, turning = -1, True, True
    fresh = (model_valid and math.isfinite(now) and math.isfinite(model_time)
             and 0 <= now-model_time <= .2 and frame >= 0)
    if not enabled or not fresh:
      self.frame = None
      self.geometry = LaneGeometry()
    elif frame != self.frame:
      self.geometry = lane_geometry(model, speed)
      self.frame = frame
    geometry = self.geometry
    trim, state = self.recovery.update(Observation(
      now=now, model_time=model_time, frame=frame, speed=speed, curvature=curvature,
      lane_offset=geometry.lane_offset, path_bias=geometry.path_bias,
      lane_width=geometry.lane_width, confidence=geometry.confidence,
      geometry_valid=geometry.valid and fresh and enabled,
      active=active and not limited, override=override,
      lane_change=lane_change or turning, blinker=blinker))
    self.status = {'path_recovery_version': PathRecovery.VERSION,
                   'path_recovery_confirmed': self.recovery.confirmed,
                   'path_recovery_enabled': bool(enabled), 'path_recovery_state': state,
                   'path_recovery_active': bool(trim), 'path_recovery_curvature_delta': trim,
                   'path_recovery_lane_offset_m': geometry.lane_offset if geometry.valid else None,
                   'path_recovery_path_bias_m': geometry.path_bias if geometry.valid else None}
    return curvature + trim
