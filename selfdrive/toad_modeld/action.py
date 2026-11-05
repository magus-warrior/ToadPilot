"""RDF4/v15 action units and smoothing, adapted from the pinned StarPilot modeld."""
import math

LAT_SMOOTH_SECONDS = 0.1
LONG_SMOOTH_SECONDS = 0.3
MODEL_DT = 0.05


def convert_action(raw_action, v_ego, previous_curvature=0.0, previous_accel=0.0):
  raw_curvature, raw_accel = map(float, raw_action)
  if not all(math.isfinite(v) for v in (raw_curvature, raw_accel, v_ego, previous_curvature, previous_accel)):
    raise ValueError('Non-finite RDF4 action')
  curvature = raw_curvature / max(1.0, v_ego) ** 2
  lat_alpha = 1 - math.exp(-MODEL_DT / LAT_SMOOTH_SECONDS)
  long_alpha = 1 - math.exp(-MODEL_DT / LONG_SMOOTH_SECONDS)
  curvature = (lat_alpha * curvature + (1 - lat_alpha) * previous_curvature) if v_ego > 0.3 else previous_curvature
  acceleration = long_alpha * raw_accel + (1 - long_alpha) * previous_accel
  return curvature, acceleration, bool(v_ego < 0.3 and raw_accel < 0.1)


def combine_longitudinal(mpc_accel, mpc_stop, model_action, minimum, maximum):
  """Experimental Mode may brake earlier, but cannot override an MPC stop or accelerate harder."""
  acceleration = float(model_action.desiredAcceleration)
  if not model_action.longitudinalValid or not all(math.isfinite(v) for v in (acceleration, mpc_accel, minimum, maximum)):
    raise ValueError('Modern model action is missing or invalid')
  if minimum > maximum:
    raise ValueError('Invalid acceleration limits')
  return max(minimum, min(maximum, mpc_accel, acceleration)), bool(mpc_stop or model_action.shouldStop)


def convert_plan_action(plan, v_ego, lat_action_t, long_action_t, previous_curvature=0.0, previous_accel=0.0):
  """v11 plan-to-action conversion from StarPilot 6bd48c28 (upstream mlsim path)."""
  import numpy as np
  from openpilot.selfdrive.toad_modeld.constants import ModelConstants, Plan
  plan = np.asarray(plan)
  if (plan.shape != (33, 15) or not np.isfinite(plan).all() or
      not all(math.isfinite(v) for v in (v_ego, lat_action_t, long_action_t, previous_curvature, previous_accel)) or
      lat_action_t <= 0 or long_action_t <= 0):
    raise ValueError('Invalid v11 plan or action horizon')
  speeds, accels = plan[:, Plan.VELOCITY][:, 0], plan[:, Plan.ACCELERATION][:, 0]
  raw_accel = 2 * (np.interp(long_action_t, ModelConstants.T_IDXS, speeds) - speeds[0]) / long_action_t - accels[0]
  should_stop = speeds[0] < 0.3 and raw_accel < 0.1
  yaws, yaw_rates = plan[:, Plan.T_FROM_CURRENT_EULER][:, 2], plan[:, Plan.ORIENTATION_RATE][:, 2]
  speed = max(v_ego, 1.0)
  raw_curvature = 2 * np.interp(lat_action_t, ModelConstants.T_IDXS, yaws) / (speed * lat_action_t) - yaw_rates[0] / speed
  lat_alpha = 1 - math.exp(-MODEL_DT / LAT_SMOOTH_SECONDS)
  long_alpha = 1 - math.exp(-MODEL_DT / LONG_SMOOTH_SECONDS)
  curvature = lat_alpha * raw_curvature + (1 - lat_alpha) * previous_curvature if v_ego > 0.3 else previous_curvature
  acceleration = long_alpha * raw_accel + (1 - long_alpha) * previous_accel
  return float(curvature), float(acceleration), bool(should_stop)


def action_from_outputs(outputs, v_ego, action_t, previous_curvature=0.0, previous_accel=0.0):
  if 'action' in outputs:
    return convert_action(outputs['action'][0], v_ego, previous_curvature, previous_accel)
  return convert_plan_action(outputs['plan'][0], v_ego, *action_t, previous_curvature, previous_accel)
