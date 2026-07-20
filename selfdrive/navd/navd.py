#!/usr/bin/env python3
import hashlib
import json
import math
import os
import threading
from dataclasses import dataclass
from enum import IntEnum

import requests

import cereal.messaging as messaging
from cereal import log
from openpilot.common.api import Api
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.navd.helpers import (Coordinate, coordinate_from_param,
                                    distance_along_geometry, maxspeed_to_ms,
                                    minimum_distance,
                                    parse_banner_instructions)
from openpilot.common.swaglog import cloudlog

from openpilot.selfdrive.frogpilot.frogpilot_variables import get_frogpilot_toggles

REROUTE_DISTANCE = 25
MANEUVER_TRANSITION_THRESHOLD = 10
REROUTE_COUNTER_MIN = 3
INTERSECTION_CONDITION_MAX_DISTANCE = 10_000.0
INTERSECTION_CONDITION_PASSED_TOLERANCE = 2.0


class IntersectionConditionType(IntEnum):
  UNKNOWN = 0
  STOP_SIGN = 1
  TRAFFIC_SIGNAL = 2


INTERSECTION_CONDITION_CAPNP_TYPES = {
  IntersectionConditionType.UNKNOWN: "unknown",
  IntersectionConditionType.STOP_SIGN: "stopSign",
  IntersectionConditionType.TRAFFIC_SIGNAL: "trafficSignal",
}


@dataclass(frozen=True)
class RouteIntersectionCondition:
  step_index: int
  geometry_index: int
  condition_type: IntersectionConditionType
  coordinate: Coordinate
  condition_id: int


def intersection_condition_id(condition_type: IntersectionConditionType, coordinate: Coordinate) -> int:
  """Return an ID that remains stable when Mapbox re-slices or recomputes a route."""
  identity = f"{int(condition_type)}:{coordinate.latitude:.7f}:{coordinate.longitude:.7f}".encode()
  condition_id = int.from_bytes(hashlib.blake2b(identity, digest_size=8, person=b"toad-nav-stop").digest(), "big")
  return condition_id or 1


def extract_route_intersection_conditions(route) -> list[RouteIntersectionCondition]:
  """Parse Mapbox conditions using geometry indexes relative to the route leg.

  Mapbox nests intersections inside steps, but ``geometry_index`` is relative
  to the start of the *leg*, not the containing step. Step geometries repeat
  their shared boundary point, so remove that duplicate while reconstructing
  the leg geometry used to validate indexes and recover a missing location.
  """
  leg_geometry_coordinates = []
  for step in route:
    step_coordinates = step.get("geometry", {}).get("coordinates", [])
    if not isinstance(step_coordinates, list):
      continue
    start_index = 1 if leg_geometry_coordinates and step_coordinates and \
      step_coordinates[0] == leg_geometry_coordinates[-1] else 0
    leg_geometry_coordinates.extend(step_coordinates[start_index:])

  conditions = []
  for step_index, step in enumerate(route):
    for intersection in step.get("intersections", []):
      geometry_index = intersection.get("geometry_index")
      if type(geometry_index) is not int or not 0 <= geometry_index < len(leg_geometry_coordinates):
        continue

      if intersection.get("stop_sign"):
        condition_type = IntersectionConditionType.STOP_SIGN
      elif intersection.get("traffic_signal"):
        condition_type = IntersectionConditionType.TRAFFIC_SIGNAL
      else:
        continue

      location = intersection.get("location", leg_geometry_coordinates[geometry_index])
      if not isinstance(location, (list, tuple)) or len(location) < 2:
        continue
      try:
        longitude, latitude = float(location[0]), float(location[1])
      except (TypeError, ValueError):
        continue
      if not math.isfinite(latitude) or not math.isfinite(longitude):
        continue

      coordinate = Coordinate(latitude, longitude)
      conditions.append(RouteIntersectionCondition(
        step_index=step_index,
        geometry_index=geometry_index,
        condition_type=condition_type,
        coordinate=coordinate,
        condition_id=intersection_condition_id(condition_type, coordinate),
      ))

  return sorted(conditions, key=lambda condition: (condition.geometry_index, condition.step_index,
                                                     condition.condition_type))


def distance_to_geometry_index(geometry: list[Coordinate], geometry_index: int) -> float | None:
  if not 0 <= geometry_index < len(geometry):
    return None
  return sum(geometry[index].distance_to(geometry[index + 1]) for index in range(geometry_index))


def geometry_length(geometry: list[Coordinate]) -> float:
  return distance_to_geometry_index(geometry, len(geometry) - 1) or 0.0


def flatten_route_geometry(route_geometry: list[list[Coordinate]]) -> tuple[list[Coordinate], list[int]]:
  """Reconstruct a leg geometry and each step's starting leg index."""
  leg_geometry = []
  step_start_indexes = []
  for step_geometry in route_geometry:
    shares_boundary = bool(leg_geometry and step_geometry and leg_geometry[-1] == step_geometry[0])
    step_start_indexes.append(len(leg_geometry) - 1 if shares_boundary else len(leg_geometry))
    leg_geometry.extend(step_geometry[1:] if shares_boundary else step_geometry)
  return leg_geometry, step_start_indexes


def projected_distance_along_geometry(geometry: list[Coordinate], position: Coordinate) -> float:
  """Project a position onto a path and return road distance from the path start."""
  if len(geometry) < 2:
    return 0.0

  closest_distance = math.inf
  closest_along_geometry = 0.0
  distance_before_segment = 0.0
  for start, end in zip(geometry, geometry[1:]):
    # Project in a local metric plane. Raw longitude degrees shrink with
    # latitude and otherwise bias progress on diagonal roads.
    longitude_scale = math.cos(math.radians((start.latitude + end.latitude + position.latitude) / 3.0))
    segment_x = (end.longitude - start.longitude) * longitude_scale
    segment_y = end.latitude - start.latitude
    segment_length_squared = segment_x ** 2 + segment_y ** 2
    if segment_length_squared == 0:
      continue

    position_x = (position.longitude - start.longitude) * longitude_scale
    position_y = position.latitude - start.latitude
    projection_scale = max(0.0, min((position_x * segment_x + position_y * segment_y) /
                                    segment_length_squared, 1.0))
    projection = Coordinate(
      start.latitude + (end.latitude - start.latitude) * projection_scale,
      start.longitude + (end.longitude - start.longitude) * projection_scale,
    )
    distance_to_path = projection.distance_to(position)
    if distance_to_path < closest_distance:
      closest_distance = distance_to_path
      closest_along_geometry = distance_before_segment + start.distance_to(projection)

    distance_before_segment += start.distance_to(end)

  return closest_along_geometry


def next_intersection_condition_for_step(conditions: list[RouteIntersectionCondition], step_index: int,
                                         geometry: list[Coordinate], along_geometry: float) -> tuple[float, RouteIntersectionCondition] | None:
  """Find the next condition and its road distance within the current route step."""
  if not math.isfinite(along_geometry):
    return None

  candidates = []
  for condition in conditions:
    if condition.step_index != step_index:
      continue
    # The Mapbox index is leg-global. The intersection location is the safe
    # way to locate it when this helper only has one step's geometry.
    condition_along_geometry = projected_distance_along_geometry(geometry, condition.coordinate)

    remaining_distance = condition_along_geometry - along_geometry
    if (remaining_distance < -INTERSECTION_CONDITION_PASSED_TOLERANCE or
        remaining_distance > INTERSECTION_CONDITION_MAX_DISTANCE):
      continue
    # Keep the small signed distance after the stop line. Consumers use this
    # to positively distinguish a passed condition from a transient absence.
    candidates.append((remaining_distance, condition))

  if not candidates:
    return None
  return min(candidates, key=lambda candidate: (candidate[0], candidate[1].geometry_index))


def next_intersection_condition_on_route(conditions: list[RouteIntersectionCondition], step_index: int,
                                         route_geometry: list[list[Coordinate]],
                                         along_geometry: float) -> tuple[float, RouteIntersectionCondition] | None:
  """Find the next condition using Mapbox's leg-global geometry indexes."""
  if not math.isfinite(along_geometry) or not 0 <= step_index < len(route_geometry):
    return None

  leg_geometry, step_start_indexes = flatten_route_geometry(route_geometry)
  current_step_start_distance = distance_to_geometry_index(leg_geometry, step_start_indexes[step_index])
  if current_step_start_distance is None:
    return None
  current_route_distance = current_step_start_distance + along_geometry

  candidates = []
  for condition in conditions:
    if condition.step_index < step_index or condition.step_index >= len(route_geometry):
      continue

    condition_route_distance = distance_to_geometry_index(leg_geometry, condition.geometry_index)
    if condition_route_distance is None:
      continue

    remaining_distance = condition_route_distance - current_route_distance
    if remaining_distance < -INTERSECTION_CONDITION_PASSED_TOLERANCE:
      continue

    if remaining_distance > INTERSECTION_CONDITION_MAX_DISTANCE:
      continue
    # Keep the small signed distance after the stop line. Consumers use this
    # to positively distinguish a passed condition from a transient absence.
    candidates.append((remaining_distance, condition))

  if not candidates:
    return None
  return min(candidates, key=lambda candidate: (candidate[0], candidate[1].geometry_index,
                                                  candidate[1].step_index))


class RouteEngine:
  def __init__(self, sm, pm):
    self.sm = sm
    self.pm = pm

    self.params = Params()

    # Get last gps position from params
    self.last_position = coordinate_from_param("LastGPSPosition", self.params)
    self.last_bearing = None

    self.gps_ok = False
    self.localizer_valid = False

    self.nav_destination = None
    self.step_idx = None
    self.route = None
    self.route_geometry = None

    self.recompute_backoff = 0
    self.recompute_countdown = 0

    self.ui_pid = None

    self.reroute_counter = 0


    self.api = None
    self.mapbox_token = None
    if "MAPBOX_TOKEN" in os.environ:
      self.mapbox_token = os.environ["MAPBOX_TOKEN"]
      self.mapbox_host = "https://api.mapbox.com"
    else:
      # A public token is sufficient for Mapbox Directions. Prefer an existing
      # secret token for compatibility, but allow the Fleet Manager UI's single
      # public-token setup as well.
      self.mapbox_token = (self.params.get("MapboxSecretKey", encoding='utf8') or
                           self.params.get("MapboxPublicKey", encoding='utf8'))
      self.mapbox_host = "https://api.mapbox.com"

    # FrogPilot variables
    self.frogpilot_toggles = get_frogpilot_toggles()

    self.approaching_intersection = False
    self.approaching_turn = False

    self.nav_speed_limit = 0

    self.route_intersection_conditions = []

  def update(self):
    self.sm.update(0)

    if self.sm.updated["managerState"]:
      ui_pid = [p.pid for p in self.sm["managerState"].processes if p.name == "ui" and p.running]
      if ui_pid:
        if self.ui_pid and self.ui_pid != ui_pid[0]:
          cloudlog.warning("UI restarting, sending route")
          threading.Timer(5.0, self.send_route).start()
        self.ui_pid = ui_pid[0]

    self.update_location()
    try:
      self.recompute_route()
      self.send_instruction()
    except Exception:
      cloudlog.exception("navd.failed_to_compute")

    # Update FrogPilot parameters
    if self.sm['frogpilotPlan'].togglesUpdated:
      self.frogpilot_toggles = get_frogpilot_toggles()

  def update_location(self):
    location = self.sm['liveLocationKalman']
    self.gps_ok = location.gpsOK

    self.localizer_valid = (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid

    if self.localizer_valid:
      self.last_bearing = math.degrees(location.calibratedOrientationNED.value[2])
      self.last_position = Coordinate(location.positionGeodetic.value[0], location.positionGeodetic.value[1])

  def recompute_route(self):
    if self.last_position is None:
      return

    new_destination = coordinate_from_param("NavDestination", self.params)
    if new_destination is None:
      self.clear_route()
      self.reset_recompute_limits()
      return

    should_recompute = self.should_recompute()
    if new_destination != self.nav_destination:
      cloudlog.warning(f"Got new destination from NavDestination param {new_destination}")
      should_recompute = True

    # Don't recompute when GPS drifts in tunnels
    if not self.gps_ok and self.step_idx is not None:
      return

    if self.recompute_countdown == 0 and should_recompute:
      self.recompute_countdown = 2**self.recompute_backoff
      self.recompute_backoff = min(6, self.recompute_backoff + 1)
      self.calculate_route(new_destination)
      self.reroute_counter = 0
    else:
      self.recompute_countdown = max(0, self.recompute_countdown - 1)

  def calculate_route(self, destination):
    cloudlog.warning(f"Calculating route {self.last_position} -> {destination}")
    self.nav_destination = destination

    lang = self.params.get('LanguageSetting', encoding='utf8')
    if lang is not None:
      lang = lang.replace('main_', '')

    token = self.mapbox_token
    if "MAPBOX_TOKEN" not in os.environ:
      token = (self.params.get("MapboxSecretKey", encoding='utf8') or
               self.params.get("MapboxPublicKey", encoding='utf8'))
    if token is None:
      token = self.api.get_token()

    params = {
      'access_token': token,
      'annotations': 'maxspeed',
      'geometries': 'geojson',
      'overview': 'full',
      'steps': 'true',
      'banner_instructions': 'true',
      'alternatives': 'false',
      'language': lang,
    }

    # TODO: move waypoints into NavDestination param?
    waypoints = self.params.get('NavDestinationWaypoints', encoding='utf8')
    waypoint_coords = []
    if waypoints is not None and len(waypoints) > 0:
      waypoint_coords = json.loads(waypoints)

    coords = [
      (self.last_position.longitude, self.last_position.latitude),
      *waypoint_coords,
      (destination.longitude, destination.latitude)
    ]
    params['waypoints'] = f'0;{len(coords)-1}'
    if self.last_bearing is not None:
      params['bearings'] = f"{(self.last_bearing + 360) % 360:.0f},90" + (';'*(len(coords)-1))

    coords_str = ';'.join([f'{lon},{lat}' for lon, lat in coords])
    url = self.mapbox_host + '/directions/v5/mapbox/driving-traffic/' + coords_str
    try:
      resp = requests.get(url, params=params, timeout=10)
      if resp.status_code != 200:
        cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)
      resp.raise_for_status()

      r = resp.json()
      r1 = resp.json()

      # Function to remove specified keys recursively unnessary for display
      def remove_keys(obj, keys_to_remove):
        if isinstance(obj, list):
          return [remove_keys(item, keys_to_remove) for item in obj]
        elif isinstance(obj, dict):
          return {key: remove_keys(value, keys_to_remove) for key, value in obj.items() if key not in keys_to_remove}
        else:
          return obj

      keys_to_remove = ['geometry', 'annotation', 'incidents', 'intersections', 'components', 'sub', 'waypoints']
      self.r2 = remove_keys(r1, keys_to_remove)
      self.r3 = {}

      # Add items for display under "routes"
      if 'routes' in self.r2 and len(self.r2['routes']) > 0:
        first_route = self.r2['routes'][0]
        nav_destination_json = self.params.get('NavDestination')

        try:
          nav_destination_data = json.loads(nav_destination_json)
          place_name = nav_destination_data.get('place_name', 'Default Place Name')
          first_route['Destination'] = place_name
          first_route['Metric'] = self.params.get_bool("IsMetric")
          self.r3['CurrentStep'] = 0
          self.r3['uuid'] = self.r2['uuid']
        except json.JSONDecodeError as e:
          print(f"Error decoding JSON: {e}")

      # Save slim json as file
      with open('navdirections.json', 'w') as json_file:
        json.dump(self.r2, json_file, indent=4)
      with open('CurrentStep.json', 'w') as json_file:
        json.dump(self.r3, json_file, indent=4)

      if len(r['routes']):
        self.route = r['routes'][0]['legs'][0]['steps']
        self.route_geometry = []
        self.route_intersection_conditions = extract_route_intersection_conditions(self.route)

        maxspeed_idx = 0
        maxspeeds = r['routes'][0]['legs'][0]['annotation']['maxspeed']

        # Convert coordinates
        for step in self.route:
          coords = []

          for c in step['geometry']['coordinates']:
            coord = Coordinate.from_mapbox_tuple(c)

            # Last step does not have maxspeed
            if (maxspeed_idx < len(maxspeeds)):
              maxspeed = maxspeeds[maxspeed_idx]
              if ('unknown' not in maxspeed) and ('none' not in maxspeed):
                coord.annotations['maxspeed'] = maxspeed_to_ms(maxspeed)

            coords.append(coord)
            maxspeed_idx += 1

          self.route_geometry.append(coords)
          maxspeed_idx -= 1  # Every segment ends with the same coordinate as the start of the next

        self.step_idx = 0
      else:
        cloudlog.warning("Got empty route response")
        self.clear_route()

      # clear waypoints to avoid a re-route including past waypoints
      # TODO: only clear once we're past a waypoint
      self.params.remove('NavDestinationWaypoints')

    except requests.exceptions.RequestException:
      cloudlog.exception("failed to get route")
      self.clear_route()

    self.send_route()

  def send_instruction(self):
    msg = messaging.new_message('navInstruction', valid=True)
    fp_msg = messaging.new_message('frogpilotNavigation', valid=True)

    if self.step_idx is None:
      msg.valid = False
      self.pm.send('navInstruction', msg)

      fp_msg.frogpilotNavigation.navigationSpeedLimit = 0
      self.pm.send('frogpilotNavigation', fp_msg)
      return

    instruction_step_index = self.step_idx
    step = self.route[instruction_step_index]
    geometry = self.route_geometry[instruction_step_index]
    along_geometry = distance_along_geometry(geometry, self.last_position)
    distance_to_maneuver_along_geometry = step['distance'] - along_geometry

    # Banner instructions are for the following maneuver step, don't use empty last step
    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]

    # Current instruction
    msg.navInstruction.maneuverDistance = distance_to_maneuver_along_geometry
    instruction = parse_banner_instructions(banner_step['bannerInstructions'], distance_to_maneuver_along_geometry)
    if instruction is not None:
      for k,v in instruction.items():
        setattr(msg.navInstruction, k, v)

    # All instructions
    maneuvers = []
    for i, step_i in enumerate(self.route):
      if i < self.step_idx:
        distance_to_maneuver = -sum(self.route[j]['distance'] for j in range(i+1, self.step_idx)) - along_geometry
      elif i == self.step_idx:
        distance_to_maneuver = distance_to_maneuver_along_geometry
      else:
        distance_to_maneuver = distance_to_maneuver_along_geometry + sum(self.route[j]['distance'] for j in range(self.step_idx+1, i+1))

      instruction = parse_banner_instructions(step_i['bannerInstructions'], distance_to_maneuver)
      if instruction is None:
        continue
      maneuver = {'distance': distance_to_maneuver}
      if 'maneuverType' in instruction:
        maneuver['type'] = instruction['maneuverType']
      if 'maneuverModifier' in instruction:
        maneuver['modifier'] = instruction['maneuverModifier']
      maneuvers.append(maneuver)

    msg.navInstruction.allManeuvers = maneuvers

    # Compute total remaining time and distance
    remaining = 1.0 - along_geometry / max(step['distance'], 1)
    total_distance = step['distance'] * remaining
    total_time = step['duration'] * remaining

    if step['duration_typical'] is None:
      total_time_typical = total_time
    else:
      total_time_typical = step['duration_typical'] * remaining

    # Add up totals for future steps
    for i in range(self.step_idx + 1, len(self.route)):
      total_distance += self.route[i]['distance']
      total_time += self.route[i]['duration']
      if self.route[i]['duration_typical'] is None:
        total_time_typical += self.route[i]['duration']
      else:
        total_time_typical += self.route[i]['duration_typical']

    msg.navInstruction.distanceRemaining = total_distance
    msg.navInstruction.timeRemaining = total_time
    msg.navInstruction.timeRemainingTypical = total_time_typical

    # Speed limit
    closest_idx, closest = min(enumerate(geometry), key=lambda p: p[1].distance_to(self.last_position))
    if closest_idx > 0:
      # If we are not past the closest point, show previous
      if along_geometry < distance_along_geometry(geometry, geometry[closest_idx]):
        closest = geometry[closest_idx - 1]

    if ('maxspeed' in closest.annotations) and self.localizer_valid:
      msg.navInstruction.speedLimit = closest.annotations['maxspeed']
      self.nav_speed_limit = closest.annotations['maxspeed']
    if not self.localizer_valid or ('maxspeed' not in closest.annotations):
      self.nav_speed_limit = 0

    # Speed limit sign type
    if 'speedLimitSign' in step:
      if step['speedLimitSign'] == 'mutcd':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.mutcd
      elif step['speedLimitSign'] == 'vienna':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.vienna

    self.pm.send('navInstruction', msg)

    # Transition to next route segment
    if distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD:
      if self.step_idx + 1 < len(self.route):
        self.step_idx += 1
        self.reset_recompute_limits()

        # Update the 'CurrentStep' value in the JSON
        if 'routes' in self.r2 and len(self.r2['routes']) > 0:
          self.r3['CurrentStep'] = self.step_idx
        # Write the modified JSON data back to the file
        with open('CurrentStep.json', 'w') as json_file:
          json.dump(self.r3, json_file, indent=4)
      else:
        cloudlog.warning("Destination reached")

        # Clear route if driving away from destination
        dist = self.nav_destination.distance_to(self.last_position)
        if dist > REROUTE_DISTANCE:
          self.params.remove("NavDestination")
          self.clear_route()

    condition_along_geometry = projected_distance_along_geometry(geometry, self.last_position)
    next_condition = next_intersection_condition_on_route(self.route_intersection_conditions, instruction_step_index,
                                                         self.route_geometry, condition_along_geometry)
    # Automatic map constraints need a live GPS fix, not only a valid
    # dead-reckoned localizer position. Smooth dead-reckoning drift otherwise
    # looks like fresh navigation data to downstream age checks.
    condition_valid = next_condition is not None and self.localizer_valid and self.gps_ok
    distance_to_condition = next_condition[0] if condition_valid else 0.0

    if self.frogpilot_toggles.conditional_navigation:
      v_ego = self.sm['carState'].vEgo
      seconds_to_stop = interp(v_ego, [0, 22.5, 45], [5, 10, 10])

      self.approaching_intersection = (self.frogpilot_toggles.conditional_navigation_intersections and condition_valid and
                                       distance_to_condition < max((seconds_to_stop * v_ego), 25))

      self.approaching_turn = (self.frogpilot_toggles.conditional_navigation_turns and
                               distance_to_maneuver_along_geometry < max((seconds_to_stop * v_ego), 25))
    else:
      self.approaching_intersection = False
      self.approaching_turn = False

    fp_msg.frogpilotNavigation.approachingIntersection = self.approaching_intersection
    fp_msg.frogpilotNavigation.approachingTurn = self.approaching_turn
    fp_msg.frogpilotNavigation.navigationSpeedLimit = self.nav_speed_limit
    fp_msg.frogpilotNavigation.intersectionConditionValid = condition_valid
    if condition_valid:
      condition = next_condition[1]
      fp_msg.frogpilotNavigation.intersectionConditionType = INTERSECTION_CONDITION_CAPNP_TYPES[condition.condition_type]
      fp_msg.frogpilotNavigation.intersectionConditionDistance = distance_to_condition
      fp_msg.frogpilotNavigation.intersectionConditionId = condition.condition_id

    self.pm.send('frogpilotNavigation', fp_msg)

  def send_route(self):
    coords = []

    if self.route is not None:
      for path in self.route_geometry:
        coords += [c.as_dict() for c in path]

    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = coords
    self.pm.send('navRoute', msg)

  def clear_route(self):
    self.route = None
    self.route_geometry = None
    self.route_intersection_conditions = []
    self.step_idx = None
    self.nav_destination = None

  def reset_recompute_limits(self):
    self.recompute_backoff = 0
    self.recompute_countdown = 0

  def should_recompute(self):
    if self.step_idx is None or self.route is None:
      return True

    # Don't recompute in last segment, assume destination is reached
    if self.step_idx == len(self.route) - 1:
      return False

    # Compute closest distance to all line segments in the current path
    min_d = REROUTE_DISTANCE + 1
    path = self.route_geometry[self.step_idx]
    for i in range(len(path) - 1):
      a = path[i]
      b = path[i + 1]

      if a.distance_to(b) < 1.0:
        continue

      min_d = min(min_d, minimum_distance(a, b, self.last_position))

    if min_d > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
    return self.reroute_counter > REROUTE_COUNTER_MIN
    # TODO: Check for going wrong way in segment


def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute', 'frogpilotNavigation'])
  sm = messaging.SubMaster(['carState', 'liveLocationKalman', 'managerState', 'frogpilotPlan'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
