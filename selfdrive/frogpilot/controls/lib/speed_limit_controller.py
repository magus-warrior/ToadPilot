# PFEIFER - SLC - Modified by FrogAi for FrogPilot
#!/usr/bin/env python3
import json
import math
import os
import time

from openpilot.common.conversions import Conversions as CV

from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_distance_to_point
from openpilot.selfdrive.frogpilot.frogpilot_variables import TO_RADIANS, params, params_memory

# mapd publishes these shared parameters once per second.
def fresh_map_value(key, now):
  try:
    with open(params_memory.get_param_path(key), "rb") as f:
      if 0 <= now - os.fstat(f.fileno()).st_mtime <= 3.0:
        return json.load(f)
  except (OSError, ValueError):
    pass
  return None


class SpeedLimitController:
  def __init__(self):
    self.experimental_mode = False
    self.speed_limit_changed = False

    self.desired_speed_limit = 0
    self.map_speed_limit = 0
    self.speed_limit = 0
    self.upcoming_speed_limit = 0

    self.source = "None"
    self.accepted_limit = None

    self.previous_speed_limit = params.get_float("PreviousSpeedLimit")

  def update(self, dashboard_speed_limit, enabled, navigation_speed_limit, v_cruise, v_ego, frogpilot_toggles, gps_position):
    self.update_map_speed_limit(gps_position, v_ego, frogpilot_toggles)
    previous_source = self.source
    max_speed_limit = v_cruise if enabled else 0

    self.speed_limit = self.get_speed_limit(dashboard_speed_limit, max_speed_limit, navigation_speed_limit, frogpilot_toggles)
    self.desired_speed_limit = self.speed_limit
    if self.source == "None":
      self.speed_limit_changed = False
    elif previous_source == "None" or abs(self.speed_limit - self.previous_speed_limit) > 0.1:
      self.speed_limit_changed = True
      if abs(self.speed_limit - self.previous_speed_limit) > 0.1:
        params.put_float_nonblocking("PreviousSpeedLimit", self.speed_limit)
      self.previous_speed_limit = self.speed_limit

    self.experimental_mode = frogpilot_toggles.slc_fallback_experimental_mode and self.speed_limit == 0

  def update_map_speed_limit(self, position, v_ego, frogpilot_toggles):
    self.map_speed_limit = self.upcoming_speed_limit = 0.
    if position is None:
      return
    now = time.time()
    current = fresh_map_value("MapSpeedLimit", now)
    if isinstance(current, (float, int)) and math.isfinite(current) and current > 1:
      self.map_speed_limit = current
    upcoming = fresh_map_value("NextMapSpeedLimit", now)
    if not isinstance(upcoming, dict):
      return
    limit, lat, lon = (upcoming.get(key) for key in ("speedlimit", "latitude", "longitude"))
    if not all(isinstance(v, (float, int)) and math.isfinite(v) for v in (limit, lat, lon)):
      return
    if limit <= 1 or abs(lat) > 90 or abs(lon) > 180:
      return
    self.upcoming_speed_limit = limit
    seconds = (frogpilot_toggles.map_speed_lookahead_higher if self.previous_speed_limit < limit
               else frogpilot_toggles.map_speed_lookahead_lower)
    if seconds > 0 and v_ego > 0:
      distance = calculate_distance_to_point(position["latitude"] * TO_RADIANS, position["longitude"] * TO_RADIANS,
                                             lat * TO_RADIANS, lon * TO_RADIANS)
      if distance < seconds * v_ego:
        self.map_speed_limit = limit

  def get_offset(self, speed_limit, frogpilot_toggles):
    if speed_limit <= 1 or (self.source == "None" and frogpilot_toggles.slc_fallback_set_speed):
      return 0.
    units = CV.MS_TO_KPH if frogpilot_toggles.is_metric else CV.MS_TO_MPH
    bounds = (29, 49, 59, 79, 99, 119, 140) if frogpilot_toggles.is_metric else (24, 34, 44, 54, 64, 74, 99)
    posted = round(speed_limit * units)
    for band, upper in enumerate(bounds, start=1):
      if posted <= upper:
        return getattr(frogpilot_toggles, f"speed_limit_offset{band}")
    return 0.

  def resolve_accepted_limit(self, target, toggles, engaged):
    if not engaged:
      self.accepted_limit = None
    elif self.source == "None" and self.accepted_limit is not None and toggles.slc_fallback_set_speed:
      # A temporary source gap cannot release an accepted cap back to a higher MAX.
      # Keep the offset with its posted value so it is applied exactly once.
      return self.accepted_limit
    resolved = (target, self.get_offset(target, toggles))
    if (engaged and self.source != "None" and not self.speed_limit_changed and target > 1
        and math.isclose(target, self.desired_speed_limit, abs_tol=0.01)):
      self.accepted_limit = resolved
    return resolved

  def get_speed_limit(self, dashboard_speed_limit, max_speed_limit, navigation_speed_limit, frogpilot_toggles):
    limits = {
      "Dashboard": dashboard_speed_limit,
      "Map Data": self.map_speed_limit,
      "Navigation": navigation_speed_limit
    }
    filtered_limits = {source: float(limit) for source, limit in limits.items() if math.isfinite(limit) and limit > 1}

    if filtered_limits:
      if frogpilot_toggles.speed_limit_priority_highest:
        self.source = max(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      if frogpilot_toggles.speed_limit_priority_lowest:
        self.source = min(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      for priority in [
        frogpilot_toggles.speed_limit_priority1,
        frogpilot_toggles.speed_limit_priority2,
        frogpilot_toggles.speed_limit_priority3
      ]:
        if priority is not None and priority in filtered_limits:
          self.source = priority
          return filtered_limits[priority]

    self.source = "None"

    if frogpilot_toggles.slc_fallback_previous_speed_limit:
      return self.previous_speed_limit

    if frogpilot_toggles.slc_fallback_set_speed:
      return max_speed_limit

    return 0
