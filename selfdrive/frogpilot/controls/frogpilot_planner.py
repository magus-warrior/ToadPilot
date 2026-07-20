#!/usr/bin/env python3
import json
import math
import time

import cereal.messaging as messaging

from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import A_CHANGE_COST, DANGER_ZONE_COST, J_EGO_COST, STOP_DISTANCE
from openpilot.selfdrive.controls.lib.longitudinal_planner import Lead

from openpilot.selfdrive.frogpilot.controls.lib.auto_traffic_mode import (AutoTrafficMode, blend_traffic_mode_value,
                                                                        valid_auto_traffic_lead)
from openpilot.selfdrive.frogpilot.controls.lib.conditional_experimental_mode import ConditionalExperimentalMode
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_acceleration import FrogPilotAcceleration
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_events import FrogPilotEvents
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_following import FrogPilotFollowing
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_vcruise import FrogPilotVCruise
from openpilot.selfdrive.frogpilot.controls.lib.navigation_trial_recorder import NavigationTrialRecorder
from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_lane_width, calculate_road_curvature
from openpilot.selfdrive.frogpilot.frogpilot_variables import CRUISING_SPEED, PLANNER_TIME, THRESHOLD, params, params_memory
from openpilot.selfdrive.frogpilot.toadpilot import TOADPILOT_CONFIG_VERSION, TOADPILOT_NAVIGATION_STATUS_KEY


def safe_rounded_float(value, digits=3):
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    return None
  value = float(value)
  return round(value, digits) if math.isfinite(value) else None


class FrogPilotPlanner:
  def __init__(self):
    self.auto_traffic_mode = AutoTrafficMode()
    self.auto_traffic_status = self.auto_traffic_mode.update(v_ego=0.0, lead_present=False, dt=0.0)
    self.cem = ConditionalExperimentalMode(self)
    self.frogpilot_acceleration = FrogPilotAcceleration(self)
    self.frogpilot_events = FrogPilotEvents(self)
    self.frogpilot_following = FrogPilotFollowing(self)
    self.frogpilot_vcruise = FrogPilotVCruise(self)
    self.navigation_trial_recorder = NavigationTrialRecorder(async_writes=True)
    self.lead_one = Lead()

    self.tracking_lead_filter = FirstOrderFilter(0, 1, DT_MDL)

    self.lateral_check = False
    self.model_stopped = False
    self.road_curvature_detected = False
    self.slower_lead = False
    self.tracking_lead = False

    self.lane_width_left = 0
    self.lane_width_right = 0
    self.model_length = 0
    self.road_curvature = 0
    self.v_cruise = 0
    self.last_update_monotonic = None
    self.navigation_status_last_publish = -math.inf
    self.navigation_status_signature = None
    self.navigation_status_removed = False
    self.navigation_status_publish_failures = 0
    self.navigation_trial_build = {
      "software_version": params.get("Version", encoding="utf-8") or "unknown",
      "git_revision": params.get("GitCommit", encoding="utf-8") or "unknown",
      "git_branch": params.get("GitBranch", encoding="utf-8") or "unknown",
    }

  def close(self):
    self.navigation_trial_recorder.close()

  def update(self, carState, controlsState, frogpilotCarState, frogpilotNavigation, liveLocationKalman, modelData,
             radarless_model, radarState, frogpilot_toggles, *, navigation_age_s=float("inf"), navigation_valid=False,
             navigation_service_alive=False, navigation_service_valid=False, navigation_updated=False, carControl=None):
    if radarless_model:
      model_leads = list(modelData.leadsV3)
      if len(model_leads) > 0:
        distance_offset = blend_traffic_mode_value(frogpilot_toggles.increased_stopped_distance, 0.0,
                                                   self.frogpilot_following.traffic_mode_blend)
        model_lead = model_leads[0]
        self.lead_one.update(model_lead.x[0] - distance_offset, model_lead.y[0], model_lead.v[0], model_lead.a[0], model_lead.prob)
      else:
        self.lead_one.reset()
    else:
      self.lead_one = radarState.leadOne

    v_cruise = min(controlsState.vCruise, V_CRUISE_MAX) * CV.KPH_TO_MS
    v_ego = max(carState.vEgo, 0)
    v_lead = self.lead_one.vLead

    now = time.monotonic()
    update_dt = DT_MDL if self.last_update_monotonic is None else max(now - self.last_update_monotonic, 0.0)
    self.last_update_monotonic = now
    finite_navigation_age = (isinstance(navigation_age_s, (int, float)) and
                             not isinstance(navigation_age_s, bool) and math.isfinite(navigation_age_s))
    fresh_navigation = bool(navigation_valid and finite_navigation_age and
                            navigation_age_s <= self.frogpilot_vcruise.map_stop_controller.MAX_OBSERVATION_AGE_S)
    auto_lead_present = valid_auto_traffic_lead(
      bool(self.lead_one.status), radarless_model=radarless_model,
      model_probability=getattr(self.lead_one, "prob", 0.0),
      probability_threshold=getattr(frogpilot_toggles, "lead_detection_probability", 0.5),
    )
    if controlsState.enabled and getattr(frogpilot_toggles, "toadpilot_automatic_traffic_mode", False):
      self.auto_traffic_status = self.auto_traffic_mode.update(
        v_ego=v_ego,
        lead_present=auto_lead_present,
        lead_distance=self.lead_one.dRel,
        lead_speed=v_lead,
        approaching_intersection=bool(fresh_navigation and frogpilotNavigation.approachingIntersection),
        dt=update_dt,
      )
    else:
      self.auto_traffic_mode.reset()
      self.auto_traffic_status = self.auto_traffic_mode.update(v_ego=v_ego, lead_present=False, dt=0.0)

    if controlsState.enabled:
      self.frogpilot_acceleration.update(frogpilotCarState, v_ego, frogpilot_toggles)
    else:
      self.frogpilot_acceleration.max_accel = 0
      self.frogpilot_acceleration.min_accel = 0

    if controlsState.enabled and frogpilot_toggles.conditional_experimental_mode:
      self.cem.update(carState, frogpilotCarState, frogpilotNavigation, v_ego, frogpilot_toggles)
    elif frogpilot_toggles.force_stops or frogpilot_toggles.green_light_alert or frogpilot_toggles.show_stopping_point:
      self.cem.curve_detected = False
      self.cem.stop_sign_and_light(frogpilotCarState, v_ego, frogpilot_toggles)
    else:
      self.cem.stop_light_detected = False

    self.frogpilot_events.update(carState, controlsState, frogpilotCarState, self.lead_one.dRel, modelData, v_cruise, frogpilot_toggles)

    self.frogpilot_following.update(carState.aEgo, controlsState, frogpilotCarState, self.lead_one.dRel, v_ego, frogpilot_toggles)

    localizer_valid = (liveLocationKalman.status == log.LiveLocationKalman.Status.valid) and liveLocationKalman.positionGeodetic.valid
    if liveLocationKalman.gpsOK and localizer_valid:
      gps_position = {
        "latitude": liveLocationKalman.positionGeodetic.value[0],
        "longitude": liveLocationKalman.positionGeodetic.value[1],
        "bearing": math.degrees(liveLocationKalman.calibratedOrientationNED.value[2])
      }
    else:
      gps_position = None

    check_lane_width = frogpilot_toggles.adjacent_paths or frogpilot_toggles.adjacent_path_metrics or frogpilot_toggles.blind_spot_path or frogpilot_toggles.lane_detection
    if check_lane_width and v_ego >= frogpilot_toggles.minimum_lane_change_speed:
      self.lane_width_left = calculate_lane_width(modelData.laneLines[0], modelData.laneLines[1], modelData.roadEdges[0])
      self.lane_width_right = calculate_lane_width(modelData.laneLines[3], modelData.laneLines[2], modelData.roadEdges[1])
    else:
      self.lane_width_left = 0
      self.lane_width_right = 0

    self.lateral_check = v_ego >= frogpilot_toggles.pause_lateral_below_speed
    self.lateral_check |= not (carState.leftBlinker or carState.rightBlinker) and frogpilot_toggles.pause_lateral_below_signal
    self.lateral_check |= carState.standstill

    self.model_length = modelData.position.x[-1]

    self.model_stopped = self.model_length < CRUISING_SPEED * PLANNER_TIME
    self.model_stopped |= self.frogpilot_vcruise.forcing_stop

    self.road_curvature = calculate_road_curvature(modelData, v_ego)

    self.road_curvature_detected = (1 / abs(self.road_curvature))**0.5 < v_ego > CRUISING_SPEED

    self.tracking_lead = self.set_lead_status()

    self.v_cruise = self.frogpilot_vcruise.update(
      carState, controlsState, frogpilotCarState, frogpilotNavigation, gps_position, v_cruise, v_ego,
      frogpilot_toggles, navigation_age_s=navigation_age_s, navigation_valid=navigation_valid, now=now,
    )
    self.update_navigation_trial(carState, controlsState, frogpilotCarState, frogpilotNavigation, carControl,
                                 frogpilot_toggles, navigation_age_s, navigation_valid, navigation_service_alive,
                                 navigation_service_valid, navigation_updated, bool(liveLocationKalman.gpsOK),
                                 localizer_valid, now)
    self.publish_navigation_status(carState, frogpilotCarState, frogpilotNavigation, frogpilot_toggles,
                                   navigation_age_s, navigation_valid, now)

  def update_navigation_trial(self, carState, controlsState, frogpilotCarState, frogpilotNavigation, carControl,
                              frogpilot_toggles, navigation_age_s, navigation_valid, navigation_service_alive,
                              navigation_service_valid, navigation_updated, navigation_gps_ok,
                              navigation_localizer_valid, now):
    if not getattr(frogpilot_toggles, "toadpilot_telemetry", False):
      return

    finite_navigation_age = (isinstance(navigation_age_s, (int, float)) and
                             not isinstance(navigation_age_s, bool) and math.isfinite(navigation_age_s))
    condition_valid = bool(navigation_valid and finite_navigation_age and
                           navigation_age_s <= self.frogpilot_vcruise.map_stop_controller.MAX_OBSERVATION_AGE_S and
                           frogpilotNavigation.intersectionConditionValid)
    source_fresh = bool(navigation_valid and finite_navigation_age and
                        navigation_age_s <= self.frogpilot_vcruise.map_stop_controller.MAX_OBSERVATION_AGE_S)
    nav_condition_type = str(frogpilotNavigation.intersectionConditionType) if condition_valid else "unknown"
    condition_type = {"stopSign": "stop_sign", "trafficSignal": "traffic_signal"}.get(nav_condition_type)
    condition = {
      "valid": bool(condition_valid and condition_type is not None),
      "id": str(frogpilotNavigation.intersectionConditionId) if condition_valid else "",
      "type": condition_type or "unknown",
      "distance_m": float(frogpilotNavigation.intersectionConditionDistance) if condition_valid else 0.0,
    }

    constraint = self.frogpilot_vcruise.map_stop_constraint
    map_stop_enabled = bool(getattr(frogpilot_toggles, "toadpilot_map_stop_assist", False))
    if condition_type == "stop_sign" and not map_stop_enabled:
      controller = {"state": "observer_only", "active": False, "forcing_stop": False}
    elif condition_type == "stop_sign" and constraint is not None:
      controller = self.frogpilot_vcruise.map_stop_controller.diagnostic_snapshot()
      controller.update({
        "holding": constraint.state.value == "holding",
        "released": constraint.state.value == "released",
        "cancelled": constraint.state.value == "cancelled",
        "cancel_reason": constraint.reason if constraint.state.value == "cancelled" else None,
        "forcing_stop": self.frogpilot_vcruise.forcing_stop,
      })
    elif constraint is not None and constraint.state.value == "cancelled":
      controller = self.frogpilot_vcruise.map_stop_controller.diagnostic_snapshot()
      controller.update({
        "cancelled": True,
        "cancel_reason": constraint.reason,
        "forcing_stop": self.frogpilot_vcruise.forcing_stop,
      })
    else:
      controller = {"state": "traffic_signal_context" if condition_type == "traffic_signal" else "idle",
                    "active": False, "forcing_stop": False}

    if carControl is not None:
      controller["output_accel_mps2"] = float(carControl.actuators.accel)

    self.navigation_trial_recorder.update(
      condition,
      controller,
      vehicle={
        "speed_mps": float(carState.vEgo),
        "accel_mps2": float(carState.aEgo),
        "standstill": bool(carState.standstill),
        "brake_pressed": bool(carState.brakePressed),
        "gas_pressed": bool(carState.gasPressed),
        "cruise_standstill": bool(carState.cruiseState.standstill),
      },
      model={
        "stop_detected": bool(self.cem.stop_light_detected),
        "model_length_m": float(self.model_length),
      },
      lead={
        "present": bool(self.lead_one.status),
        "distance_m": float(self.lead_one.dRel),
        "speed_mps": float(self.lead_one.vLead),
        "relative_speed_mps": float(self.lead_one.vLead - carState.vEgo),
      },
      planner={
        "forcing_stop": bool(self.frogpilot_vcruise.forcing_stop),
        "target_speed_mps": float(self.v_cruise),
        "target_accel_mps2": float(controlsState.aTarget),
        "stop_distance_m": (constraint.target_distance_m if constraint is not None else None),
      },
      traffic={
        "active": bool(frogpilotCarState.trafficMode),
        "reason": self.auto_traffic_status.reason,
        "confidence": float(self.auto_traffic_status.confidence),
        "manual_override": int(getattr(frogpilotCarState, "trafficModeOverride", 0)),
      },
      navigation={
        "observation_age_s": navigation_age_s,
        "source_fresh": source_fresh,
        "source_updated": bool(navigation_updated),
        "service_valid": bool(navigation_service_valid),
        "service_alive": bool(navigation_service_alive),
        "condition_valid": condition_valid,
        "gps_ok": bool(navigation_gps_ok),
        "localizer_valid": bool(navigation_localizer_valid),
      },
      provenance={
        "build": self.navigation_trial_build,
        "config": {
          "config_version": TOADPILOT_CONFIG_VERSION,
          "toadpilot_enabled": bool(getattr(frogpilot_toggles, "toadpilot_enabled", False)),
          "profile": str(getattr(frogpilot_toggles, "toadpilot_profile", "unknown")),
          "map_stop_assist_enabled": map_stop_enabled,
          "automatic_traffic_mode_enabled": bool(getattr(frogpilot_toggles, "toadpilot_automatic_traffic_mode", False)),
          "telemetry_enabled": bool(getattr(frogpilot_toggles, "toadpilot_telemetry", False)),
        },
        "settings": {
          "traffic_follow_s": self.frogpilot_following.t_follow,
          "traffic_stopping_distance_m": self.frogpilot_following.desired_follow_distance,
          "increased_stopped_distance_m": getattr(frogpilot_toggles, "increased_stopped_distance", 0.0),
          "start_accel_ramp_mps2": getattr(frogpilot_toggles, "start_accel_ramp", 0.0),
          "experimental_mode": bool(self.cem.experimental_mode),
          "longitudinal_personality": str(controlsState.personality),
        },
        "controller_constants": {
          "comfortable_decel_mps2": self.frogpilot_vcruise.map_stop_controller.comfortable_decel_mps2,
          "stop_sign_hold_s": self.frogpilot_vcruise.map_stop_controller.stop_sign_hold_s,
          "stop_line_buffer_m": self.frogpilot_vcruise.map_stop_controller.STOP_LINE_BUFFER_M,
          "hold_capture_distance_m": self.frogpilot_vcruise.map_stop_controller.HOLD_CAPTURE_DISTANCE_M,
          "standstill_speed_mps": self.frogpilot_vcruise.map_stop_controller.STANDSTILL_SPEED_MPS,
          "max_observation_age_s": self.frogpilot_vcruise.map_stop_controller.MAX_OBSERVATION_AGE_S,
          "passed_distance_m": self.frogpilot_vcruise.map_stop_controller.PASSED_DISTANCE_M,
          "arming_margin_m": self.frogpilot_vcruise.map_stop_controller.ARMING_MARGIN_M,
          "max_initial_required_decel_mps2": self.frogpilot_vcruise.map_stop_controller.MAX_INITIAL_REQUIRED_DECEL_MPS2,
          "required_confirmation_samples": self.frogpilot_vcruise.map_stop_controller.REQUIRED_CONFIRMATION_SAMPLES,
          "required_absence_samples": self.frogpilot_vcruise.map_stop_controller.REQUIRED_ABSENCE_SAMPLES,
        },
      },
      monotonic_time=now,
      wall_time=time.time(),
    )

  def publish_navigation_status(self, carState, frogpilotCarState, frogpilotNavigation, frogpilot_toggles,
                                navigation_age_s, navigation_valid, now):
    if not getattr(frogpilot_toggles, "toadpilot_telemetry", False):
      if not self.navigation_status_removed:
        params_memory.remove(TOADPILOT_NAVIGATION_STATUS_KEY)
        self.navigation_status_removed = True
      return
    self.navigation_status_removed = False

    finite_navigation_age = (float(navigation_age_s) if isinstance(navigation_age_s, (int, float)) and
                             not isinstance(navigation_age_s, bool) and math.isfinite(navigation_age_s) else None)
    raw_condition_distance = frogpilotNavigation.intersectionConditionDistance
    finite_condition_distance = safe_rounded_float(raw_condition_distance)
    condition_valid = bool(navigation_valid and finite_navigation_age is not None and
                           finite_navigation_age <= self.frogpilot_vcruise.map_stop_controller.MAX_OBSERVATION_AGE_S and
                           frogpilotNavigation.intersectionConditionValid and finite_condition_distance is not None)
    condition_type = str(frogpilotNavigation.intersectionConditionType) if condition_valid else "none"
    condition_distance = float(raw_condition_distance) if condition_valid else None
    constraint = self.frogpilot_vcruise.map_stop_constraint
    if condition_type == "trafficSignal":
      map_stop_state = "traffic_signal_context"
      map_stop_reason = "static_map_has_no_signal_color"
      map_stop_active = False
    elif constraint is not None:
      map_stop_state = constraint.state.value
      map_stop_reason = constraint.reason
      map_stop_active = constraint.active
    else:
      map_stop_state = "idle"
      map_stop_reason = "no_mapped_stop"
      map_stop_active = False

    signature = (
      self.auto_traffic_status.active, self.auto_traffic_status.reason,
      bool(frogpilotCarState.trafficMode), int(getattr(frogpilotCarState, "trafficModeOverride", 0)),
      condition_type, map_stop_state, map_stop_reason, map_stop_active,
    )
    if now - self.navigation_status_last_publish < 1.0 and signature == self.navigation_status_signature:
      return

    payload = {
      "schema_version": 1,
      "published_at_unix": round(time.time(), 3),
      "summary_source": "frogpilot_planner_live",
      "auto_traffic_enabled": bool(getattr(frogpilot_toggles, "toadpilot_automatic_traffic_mode", False)),
      "auto_traffic_active": bool(self.auto_traffic_status.active),
      "auto_traffic_reason": self.auto_traffic_status.reason,
      "auto_traffic_confidence": safe_rounded_float(self.auto_traffic_status.confidence),
      "auto_traffic_stop_launch_cycles": int(self.auto_traffic_status.stop_launch_cycles),
      "traffic_mode_effective": bool(frogpilotCarState.trafficMode),
      "traffic_mode_override": int(getattr(frogpilotCarState, "trafficModeOverride", 0)),
      "map_stop_enabled": bool(getattr(frogpilot_toggles, "toadpilot_map_stop_assist", False)),
      "map_stop_active": bool(map_stop_active),
      "map_stop_state": map_stop_state,
      "map_stop_reason": map_stop_reason,
      "condition_valid": condition_valid,
      "condition_type": {"stopSign": "stop_sign", "trafficSignal": "traffic_signal"}.get(condition_type, "none"),
      "condition_distance": safe_rounded_float(condition_distance),
      "navigation_age_s": safe_rounded_float(finite_navigation_age),
      "speed_mps": safe_rounded_float(carState.vEgo),
      "model_stop_detected": bool(self.cem.stop_light_detected),
      "forcing_stop": bool(self.frogpilot_vcruise.forcing_stop),
      "status_publish_failures": self.navigation_status_publish_failures,
    }
    recorder_health = self.navigation_trial_recorder.health_snapshot()
    payload.update({
      "trial_logging_healthy": bool(recorder_health["healthy"]),
      "trial_logging_pending": int(recorder_health["writes_in_flight"]),
      "trial_logging_successes": int(recorder_health["write_successes"]),
      "trial_logging_failures": int(recorder_health["write_failures"] + recorder_health["write_dropped"] +
                                    recorder_health["retention_failures"]),
      "trial_logging_last_error": recorder_health["last_error"],
    })
    if constraint is not None:
      payload.update({
        "map_estimated_distance_m": safe_rounded_float(constraint.along_route_distance_m),
        "map_target_speed_mps": safe_rounded_float(constraint.target_speed_mps),
        "map_required_decel_mps2": safe_rounded_float(constraint.required_decel_mps2),
        "map_hold_elapsed_s": safe_rounded_float(constraint.hold_elapsed_s),
        "map_late_stop": bool(constraint.late_stop),
      })
    try:
      encoded_payload = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
      params_memory.put_nonblocking(TOADPILOT_NAVIGATION_STATUS_KEY, encoded_payload)
    except (OSError, OverflowError, TypeError, ValueError):
      self.navigation_status_publish_failures += 1
      return
    self.navigation_status_last_publish = now
    self.navigation_status_signature = signature

  def set_lead_status(self):
    following_lead = self.lead_one.status
    following_lead &= self.lead_one.dRel < self.model_length + STOP_DISTANCE

    self.tracking_lead_filter.update(following_lead)
    return self.tracking_lead_filter.x >= THRESHOLD**2

  def publish(self, sm, pm, toggles_updated):
    frogpilot_plan_send = messaging.new_message("frogpilotPlan")
    frogpilot_plan_send.valid = sm.all_checks(service_list=["carState", "controlsState"])
    frogpilotPlan = frogpilot_plan_send.frogpilotPlan

    frogpilotPlan.accelerationJerk = A_CHANGE_COST * self.frogpilot_following.acceleration_jerk
    frogpilotPlan.accelerationJerkStock = A_CHANGE_COST * self.frogpilot_following.base_acceleration_jerk
    frogpilotPlan.dangerJerk = DANGER_ZONE_COST * self.frogpilot_following.danger_jerk
    frogpilotPlan.speedJerk = J_EGO_COST * self.frogpilot_following.speed_jerk
    frogpilotPlan.speedJerkStock = J_EGO_COST * self.frogpilot_following.base_speed_jerk
    frogpilotPlan.tFollow = self.frogpilot_following.t_follow

    frogpilotPlan.desiredFollowDistance = self.frogpilot_following.desired_follow_distance

    frogpilotPlan.experimentalMode = self.cem.experimental_mode or self.frogpilot_vcruise.slc.experimental_mode

    frogpilotPlan.forcingStop = self.frogpilot_vcruise.forcing_stop
    frogpilotPlan.forcingStopLength = self.frogpilot_vcruise.tracked_model_length

    frogpilotPlan.frogpilotEvents = self.frogpilot_events.events.to_msg()

    frogpilotPlan.laneWidthLeft = self.lane_width_left
    frogpilotPlan.laneWidthRight = self.lane_width_right

    frogpilotPlan.lateralCheck = self.lateral_check

    frogpilotPlan.maxAcceleration = self.frogpilot_acceleration.max_accel
    frogpilotPlan.minAcceleration = self.frogpilot_acceleration.min_accel

    frogpilotPlan.mtscSpeed = self.frogpilot_vcruise.mtsc_target
    frogpilotPlan.vtscControllingCurve = self.frogpilot_vcruise.mtsc_target > self.frogpilot_vcruise.vtsc_target
    frogpilotPlan.vtscSpeed = self.frogpilot_vcruise.vtsc_target

    frogpilotPlan.autoTrafficMode = self.auto_traffic_status.active
    frogpilotPlan.autoTrafficModeReason = self.auto_traffic_status.reason
    frogpilotPlan.autoTrafficModeConfidence = self.auto_traffic_status.confidence
    frogpilotPlan.trafficModeBlend = self.frogpilot_following.traffic_mode_blend

    map_stop_constraint = self.frogpilot_vcruise.map_stop_constraint
    if map_stop_constraint is not None:
      frogpilotPlan.mapStopActive = map_stop_constraint.active
      frogpilotPlan.mapStopState = map_stop_constraint.state.value
      frogpilotPlan.mapStopDistance = map_stop_constraint.along_route_distance_m or 0.0

    frogpilotPlan.redLight = self.cem.stop_light_detected

    frogpilotPlan.roadCurvature = self.road_curvature

    frogpilotPlan.slcMapSpeedLimit = self.frogpilot_vcruise.slc.map_speed_limit
    frogpilotPlan.slcOverridden = bool(self.frogpilot_vcruise.override_slc)
    frogpilotPlan.slcOverriddenSpeed = float(self.frogpilot_vcruise.overridden_speed)
    frogpilotPlan.slcSpeedLimit = self.frogpilot_vcruise.slc_target
    frogpilotPlan.slcSpeedLimitOffset = self.frogpilot_vcruise.slc_offset
    frogpilotPlan.slcSpeedLimitSource = self.frogpilot_vcruise.slc.source
    frogpilotPlan.speedLimitChanged = self.frogpilot_vcruise.slc.speed_limit_changed
    frogpilotPlan.unconfirmedSlcSpeedLimit = self.frogpilot_vcruise.slc.desired_speed_limit
    frogpilotPlan.upcomingSLCSpeedLimit = self.frogpilot_vcruise.slc.upcoming_speed_limit

    frogpilotPlan.togglesUpdated = toggles_updated

    frogpilotPlan.vCruise = float(self.v_cruise)

    pm.send("frogpilotPlan", frogpilot_plan_send)
