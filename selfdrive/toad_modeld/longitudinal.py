"""Optional v0.11.2 cruise/model coordination around this fork's lead MPC.

The existing MPC retains its cruise obstacle, following policy and acceleration
limits. This is a backport of candidate selection, not a replacement solver.
"""
import math
import numpy as np

from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.toad_modeld.artifact import is_modern_model


ACCEL_LEARNING_MIN_SPEED = 10 * CV.MPH_TO_MS


def coordination_enabled(toggles, experimental, v_ego):
  # Preserve the existing blended planner through stop/go and hybrid launch.
  # The override learner cannot contribute below this speed anyway.
  return (experimental and is_modern_model(toggles) and
          getattr(toggles, 'toadpilot_modern_longitudinal', False) is True and
          math.isfinite(v_ego) and v_ego >= ACCEL_LEARNING_MIN_SPEED)


class AccelBoost:
  """Upstream override learner, adapted to explicit inputs and optional operation."""
  def __init__(self):
    self.total_boost = 0.0
    self.boost_this_override = 0.0
    self.boost_eligible = False

  def update(self, enabled, gas_pressed, v_ego, model_accel, mpc_accel, cruise_accel, dt):
    speed_scale = float(np.interp(v_ego, [ACCEL_LEARNING_MIN_SPEED, 2 * ACCEL_LEARNING_MIN_SPEED], [0.0, 1.0]))
    boost_scale = speed_scale * float(np.interp(model_accel, [-1.0, -0.5, 5.0], [0.0, 1.0, 1.0], right=0.0))
    model_limited = model_accel + self.total_boost * boost_scale < min(mpc_accel, cruise_accel) - 0.1
    if not enabled or not gas_pressed:
      self.boost_this_override = 0.0
      self.boost_eligible = enabled and model_limited
    if not enabled:
      self.total_boost = 0.0
    elif gas_pressed and self.boost_eligible:
      increase = max(0.0, min(0.025 * dt * speed_scale,
                              0.05 - self.boost_this_override, 0.2 - self.total_boost))
      self.total_boost += increase
      self.boost_this_override += increase
    return model_accel + self.total_boost * boost_scale


class LongitudinalCoordinator:
  def __init__(self):
    self.cruise_accel = 0.0
    self.boost = AccelBoost()

  def reset(self, accel=0.0):
    self.cruise_accel = float(accel) if math.isfinite(accel) else 0.0
    self.boost = AccelBoost()

  def update(self, *, mpc_accel, mpc_stop, mpc_source, action, v_ego, v_cruise,
             minimum, maximum, dt, boost_enabled, enabled, gas_pressed, brake_pressed):
    model_accel = float(action.desiredAcceleration)
    if (not action.longitudinalValid or minimum > maximum or dt <= 0 or
        not all(math.isfinite(v) for v in (mpc_accel, model_accel, v_ego, v_cruise, minimum, maximum, dt))):
      self.reset()
      raise ValueError('Invalid modern longitudinal inputs')
    # Same e2e cruise jerk profile as upstream v0.11.2. The final choice also
    # remains constrained by this fork's existing cruise obstacle in the MPC.
    target = float(np.clip(v_cruise - v_ego, min(-0.5, maximum), maximum))
    jerk = float(np.interp(v_ego, [0., 10., 25., 40.], [1.6, 1.2, 0.8, 0.6]))
    self.cruise_accel = float(np.clip(target, self.cruise_accel - jerk * dt, self.cruise_accel + jerk * dt))
    model_accel = self.boost.update(boost_enabled and enabled and not brake_pressed and not action.shouldStop,
                                    gas_pressed, v_ego, model_accel, mpc_accel, self.cruise_accel, dt)
    candidates = [(mpc_accel, mpc_source), (self.cruise_accel, 'cruise'), (model_accel, 'e2e')]
    accel, source = min(candidates, key=lambda candidate: candidate[0])
    stop = bool(mpc_stop or action.shouldStop or (v_ego < 0.3 and self.cruise_accel < 0.1))
    return float(np.clip(accel, minimum, maximum)), stop, source
