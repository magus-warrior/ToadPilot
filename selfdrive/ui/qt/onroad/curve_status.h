#pragma once

#include <cmath>

inline bool curveSpeedLimited(float cruise_speed_kph, bool map_enabled, float map_target_ms,
                              bool vision_enabled, float vision_target_ms) {
  if (!std::isfinite(cruise_speed_kph) || cruise_speed_kph <= 0 || cruise_speed_kph >= 255) return false;
  const float cruise_ms = cruise_speed_kph / 3.6f;
  const auto limiting = [cruise_ms](bool enabled, float target) {
    return enabled && std::isfinite(target) && target > 0 && cruise_ms - target > 0.5f;
  };
  return limiting(map_enabled, map_target_ms) || limiting(vision_enabled, vision_target_ms);
}
