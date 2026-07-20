import math
from types import SimpleNamespace

from cereal import custom
from openpilot.selfdrive.navd.helpers import Coordinate
from openpilot.selfdrive.navd.navd import (INTERSECTION_CONDITION_PASSED_TOLERANCE,
                                          IntersectionConditionType,
                                          RouteEngine,
                                          distance_to_geometry_index,
                                          extract_route_intersection_conditions,
                                          flatten_route_geometry,
                                          geometry_length,
                                          next_intersection_condition_for_step,
                                          next_intersection_condition_on_route,
                                          projected_distance_along_geometry)


def route_step(coordinates, intersections=None):
  return {
    "geometry": {"coordinates": coordinates},
    "intersections": intersections or [],
  }


def test_conditions_keep_leg_global_geometry_indexes():
  route = [
    route_step(
      [[0.0, 0.0], [0.001, 0.0], [0.001, 0.001]],
      [{"geometry_index": 2, "location": [0.001, 0.001], "stop_sign": True}],
    ),
    route_step(
      [[0.001, 0.001], [0.002, 0.001], [0.003, 0.001]],
      # Index 3 is relative to the leg and deliberately outside this step's
      # local [0, 2] range.
      [{"geometry_index": 3, "location": [0.002, 0.001], "traffic_signal": True}],
    ),
  ]

  conditions = extract_route_intersection_conditions(route)

  assert [(condition.step_index, condition.geometry_index, condition.condition_type) for condition in conditions] == [
    (0, 2, IntersectionConditionType.STOP_SIGN),
    (1, 3, IntersectionConditionType.TRAFFIC_SIGNAL),
  ]

  first_geometry = [Coordinate.from_mapbox_tuple(coordinate) for coordinate in route[0]["geometry"]["coordinates"]]
  selected_distance, selected_condition = next_intersection_condition_for_step(conditions, 0, first_geometry, 0.0)
  assert selected_condition.condition_type == IntersectionConditionType.STOP_SIGN
  assert math.isclose(selected_distance, distance_to_geometry_index(first_geometry, 2), rel_tol=1e-6)


def test_condition_distance_follows_curved_route_not_gps_chord():
  geometry = [Coordinate(0.0, 0.0), Coordinate(0.0, 0.001), Coordinate(0.001, 0.001)]
  current_position = Coordinate(0.0002, 0.0005)
  condition_coordinate = geometry[-1]
  route = [route_step(
    [[0.0, 0.0], [0.001, 0.0], [0.001, 0.001]],
    [{"geometry_index": 2, "location": [0.001, 0.001], "stop_sign": True}],
  )]
  conditions = extract_route_intersection_conditions(route)

  along_geometry = projected_distance_along_geometry(geometry, current_position)
  distance_to_condition, _ = next_intersection_condition_for_step(conditions, 0, geometry, along_geometry)

  assert distance_to_condition > current_position.distance_to(condition_coordinate) * 1.5
  assert math.isclose(distance_to_condition,
                      distance_to_geometry_index(geometry, 2) - along_geometry,
                      rel_tol=1e-6)


def test_condition_at_start_of_future_step_is_visible_before_step_transition():
  route = [
    route_step(
      [[0.0, 0.0], [0.001, 0.0]],
      [{"geometry_index": 0, "location": [0.0, 0.0], "stop_sign": True}],
    ),
    route_step(
      [[0.001, 0.0], [0.002, 0.0]],
      # The shared boundary is index 1 in the full leg geometry.
      [{"geometry_index": 1, "location": [0.001, 0.0], "traffic_signal": True}],
    ),
  ]
  route_geometry = [
    [Coordinate.from_mapbox_tuple(coordinate) for coordinate in step["geometry"]["coordinates"]]
    for step in route
  ]
  conditions = extract_route_intersection_conditions(route)
  current_along_geometry = geometry_length(route_geometry[0]) / 2

  distance_to_condition, condition = next_intersection_condition_on_route(
    conditions, 0, route_geometry, current_along_geometry,
  )

  assert condition.step_index == 1
  assert condition.geometry_index == 1
  assert condition.condition_type == IntersectionConditionType.TRAFFIC_SIGNAL
  assert math.isclose(distance_to_condition,
                      geometry_length(route_geometry[0]) - current_along_geometry,
                      rel_tol=1e-6)


def test_later_step_condition_uses_leg_global_index_for_route_distance():
  route = [
    route_step([[0.0, 0.0], [0.001, 0.0], [0.001, 0.001]]),
    route_step(
      [[0.001, 0.001], [0.002, 0.001], [0.003, 0.001]],
      # Full leg: A, B, C, D, E. The stop at E is global index 4,
      # which would be rejected if interpreted as a step-local index.
      [{"geometry_index": 4, "location": [0.003, 0.001], "stop_sign": True}],
    ),
  ]
  route_geometry = [
    [Coordinate.from_mapbox_tuple(coordinate) for coordinate in step["geometry"]["coordinates"]]
    for step in route
  ]
  conditions = extract_route_intersection_conditions(route)
  current_along_geometry = geometry_length(route_geometry[0]) / 2

  distance_to_condition, condition = next_intersection_condition_on_route(
    conditions, 0, route_geometry, current_along_geometry,
  )

  expected_distance = (geometry_length(route_geometry[0]) - current_along_geometry +
                       geometry_length(route_geometry[1]))
  assert condition.step_index == 1
  assert condition.geometry_index == 4
  assert math.isclose(distance_to_condition, expected_distance, rel_tol=1e-6)


def test_projection_excludes_lateral_gps_offset_from_route_progress():
  geometry = [Coordinate(0.0, 0.0), Coordinate(0.0, 0.001)]
  on_route = Coordinate(0.0, 0.0005)
  laterally_offset = Coordinate(0.0002, 0.0005)

  assert math.isclose(projected_distance_along_geometry(geometry, on_route),
                      projected_distance_along_geometry(geometry, laterally_offset),
                      rel_tol=1e-6)


def test_projection_accounts_for_longitude_scale_at_high_latitude():
  geometry = [Coordinate(60.0, 0.0), Coordinate(60.001, 0.002)]
  # In the local metric plane this point is a perpendicular offset from the
  # exact midpoint. A raw-degree projection would incorrectly report ~38%.
  laterally_offset_from_midpoint = Coordinate(60.0007, 0.0006)

  projected = projected_distance_along_geometry(geometry, laterally_offset_from_midpoint)

  assert math.isclose(projected, geometry[0].distance_to(geometry[1]) / 2.0, abs_tol=0.15)


def test_passed_condition_is_removed_after_small_position_tolerance():
  route = [route_step(
    [[0.0, 0.0], [0.001, 0.0]],
    [{"geometry_index": 1, "location": [0.001, 0.0], "stop_sign": True}],
  )]
  geometry = [Coordinate.from_mapbox_tuple(coordinate) for coordinate in route[0]["geometry"]["coordinates"]]
  conditions = extract_route_intersection_conditions(route)
  condition_along = distance_to_geometry_index(geometry, 1)

  within_tolerance = next_intersection_condition_for_step(
    conditions, 0, geometry, condition_along + INTERSECTION_CONDITION_PASSED_TOLERANCE,
  )
  assert math.isclose(within_tolerance[0], -INTERSECTION_CONDITION_PASSED_TOLERANCE)
  assert next_intersection_condition_for_step(
    conditions, 0, geometry, condition_along + INTERSECTION_CONDITION_PASSED_TOLERANCE + 0.01,
  ) is None


def test_condition_id_survives_route_reslicing_and_bad_indexes_are_ignored():
  condition = {"geometry_index": 1, "location": [-122.1234567, 37.7654321], "stop_sign": True}
  original_route = [route_step([[-122.124, 37.765], [-122.1234567, 37.7654321]], [condition])]
  midpoint = [-122.1237, 37.7652]
  resliced_condition = condition | {"geometry_index": 2}
  resliced_route = [
    route_step([[-122.124, 37.765], midpoint]),
    route_step([midpoint, [-122.1234567, 37.7654321]], [resliced_condition]),
  ]
  invalid_route = [route_step(
    [[0.0, 0.0]],
    [
      {"geometry_index": -1, "location": [0.0, 0.0], "stop_sign": True},
      {"geometry_index": 1, "location": [0.0, 0.0], "traffic_signal": True},
      {"geometry_index": 0, "location": [math.nan, 0.0], "stop_sign": True},
    ],
  )]

  assert extract_route_intersection_conditions(original_route)[0].condition_id == \
         extract_route_intersection_conditions(resliced_route)[0].condition_id
  assert extract_route_intersection_conditions(invalid_route) == []


def test_flattened_step_geometry_matches_leg_indexing_at_boundaries():
  route_geometry = [
    [Coordinate(0.0, 0.0), Coordinate(0.0, 0.001), Coordinate(0.001, 0.001)],
    [Coordinate(0.001, 0.001), Coordinate(0.001, 0.002)],
  ]

  leg_geometry, step_start_indexes = flatten_route_geometry(route_geometry)

  assert len(leg_geometry) == 4
  assert step_start_indexes == [0, 2]
  assert leg_geometry[2] == route_geometry[1][0]


def test_navigation_schema_exposes_bounded_condition_metadata():
  message = custom.FrogPilotNavigation.new_message()
  message.intersectionConditionValid = True
  message.intersectionConditionType = "trafficSignal"
  message.intersectionConditionDistance = 42.5
  message.intersectionConditionId = 123

  assert message.intersectionConditionValid
  assert str(message.intersectionConditionType) == "trafficSignal"
  assert message.intersectionConditionDistance == 42.5
  assert message.intersectionConditionId == 123


class MessageCollector:
  def __init__(self):
    self.messages = {}

  def send(self, service, message):
    self.messages[service] = message


def test_route_engine_preserves_approach_trigger_and_publishes_metadata_when_trigger_disabled():
  route = [route_step(
    [[0.0, 0.0], [0.0001, 0.0]],
    [{"geometry_index": 1, "location": [0.0001, 0.0], "stop_sign": True}],
  ) | {
    "distance": 11.1,
    "duration": 3.0,
    "duration_typical": None,
    "bannerInstructions": [],
  }]
  geometry = [[Coordinate.from_mapbox_tuple(coordinate) for coordinate in route[0]["geometry"]["coordinates"]]]
  collector = MessageCollector()
  engine = RouteEngine.__new__(RouteEngine)
  engine.pm = collector
  engine.sm = {"carState": SimpleNamespace(vEgo=3.0)}
  engine.step_idx = 0
  engine.route = route
  engine.route_geometry = geometry
  engine.route_intersection_conditions = extract_route_intersection_conditions(route)
  engine.last_position = geometry[0][0]
  engine.localizer_valid = True
  engine.gps_ok = True
  engine.nav_speed_limit = 0.0
  engine.approaching_intersection = False
  engine.approaching_turn = False
  engine.frogpilot_toggles = SimpleNamespace(
    conditional_navigation=True,
    conditional_navigation_intersections=True,
    conditional_navigation_turns=False,
  )

  engine.send_instruction()
  navigation = collector.messages["frogpilotNavigation"].frogpilotNavigation
  assert navigation.approachingIntersection
  assert navigation.intersectionConditionValid
  assert str(navigation.intersectionConditionType) == "stopSign"
  assert math.isclose(navigation.intersectionConditionDistance, 11.1, rel_tol=0.01)
  condition_id = navigation.intersectionConditionId

  engine.frogpilot_toggles.conditional_navigation = False
  engine.send_instruction()
  navigation = collector.messages["frogpilotNavigation"].frogpilotNavigation
  assert not navigation.approachingIntersection
  assert navigation.intersectionConditionValid
  assert navigation.intersectionConditionId == condition_id


def test_route_engine_requires_live_gps_for_condition_metadata():
  route = [route_step(
    [[0.0, 0.0], [0.0001, 0.0]],
    [{"geometry_index": 1, "location": [0.0001, 0.0], "stop_sign": True}],
  ) | {
    "distance": 11.1,
    "duration": 3.0,
    "duration_typical": None,
    "bannerInstructions": [],
  }]
  geometry = [[Coordinate.from_mapbox_tuple(coordinate) for coordinate in route[0]["geometry"]["coordinates"]]]
  collector = MessageCollector()
  engine = RouteEngine.__new__(RouteEngine)
  engine.pm = collector
  engine.sm = {"carState": SimpleNamespace(vEgo=3.0)}
  engine.step_idx = 0
  engine.route = route
  engine.route_geometry = geometry
  engine.route_intersection_conditions = extract_route_intersection_conditions(route)
  engine.last_position = geometry[0][0]
  engine.localizer_valid = True
  engine.gps_ok = False
  engine.nav_speed_limit = 0.0
  engine.approaching_intersection = False
  engine.approaching_turn = False
  engine.frogpilot_toggles = SimpleNamespace(
    conditional_navigation=True,
    conditional_navigation_intersections=True,
    conditional_navigation_turns=False,
  )

  engine.send_instruction()

  navigation = collector.messages["frogpilotNavigation"].frogpilotNavigation
  assert not navigation.intersectionConditionValid
  assert not navigation.approachingIntersection
