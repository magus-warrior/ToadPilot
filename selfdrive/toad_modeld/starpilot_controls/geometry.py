"""StarPilot model curvature/profile geometry, pinned to 4a553da9 (MIT)."""
import numpy as np

PROFILE_MIN_SPEED = 3.0
PROFILE_MAX_CURVATURE = 0.1

def calculate_road_curvature(modelData, v_ego):
    orientation_rate = np.array(modelData.orientationRate.z)
    timebase = np.array(modelData.orientationRate.t)
    velocity = np.array(modelData.velocity.x)
    lateral_acceleration = orientation_rate * velocity
    index = np.argmax(np.abs(lateral_acceleration))
    predicted_lateral_acc = float(lateral_acceleration[index])
    time_to_curve = float(timebase[index])
    return (float(predicted_lateral_acc / max(v_ego, 1) ** 2), max(time_to_curve, 1))

def extract_curve_profile(modelData):
    try:
        orientation_rate = np.abs(np.array(modelData.orientationRate.z))
        velocity = np.array(modelData.velocity.x)
        distances = np.array(modelData.position.x)
    except (AttributeError, TypeError, ValueError):
        return (np.array([]), np.array([]))
    if not len(orientation_rate) == len(velocity) == len(distances):
        return (np.array([]), np.array([]))
    if not (np.all(np.isfinite(orientation_rate)) and np.all(np.isfinite(velocity)) and np.all(np.isfinite(distances))):
        return (np.array([]), np.array([]))
    curvatures = orientation_rate / np.clip(velocity, PROFILE_MIN_SPEED, None)
    curvatures = np.where(velocity < PROFILE_MIN_SPEED, 0.0, np.minimum(curvatures, PROFILE_MAX_CURVATURE))
    return (curvatures, distances)
