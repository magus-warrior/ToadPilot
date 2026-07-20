#!/usr/bin/env python3
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE, STOP_DISTANCE, desired_follow_distance, get_jerk_factor, get_T_FOLLOW

from openpilot.selfdrive.frogpilot.frogpilot_variables import CITY_SPEED_LIMIT

TRAFFIC_MODE_BP = [0., CITY_SPEED_LIMIT]
TRAFFIC_MODE_BLEND_TIME = 2.0

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.following_lead = False
    self.slower_lead = False

    self.acceleration_jerk = 0
    self.danger_jerk = 0
    self.desired_follow_distance = 0
    self.speed_jerk = 0
    self.t_follow = 0
    self.traffic_mode_blend = 0.0

  def update(self, aEgo, controlsState, frogpilotCarState, lead_distance, v_ego, frogpilot_toggles):
    if controlsState.enabled:
      if aEgo >= 0:
        normal_acceleration_jerk, normal_danger_jerk, normal_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_acceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed,
          frogpilot_toggles.standard_jerk_acceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed,
          frogpilot_toggles.relaxed_jerk_acceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed,
          frogpilot_toggles.custom_personalities, controlsState.personality
        )
      else:
        normal_acceleration_jerk, normal_danger_jerk, normal_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_deceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed_decrease,
          frogpilot_toggles.standard_jerk_deceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed_decrease,
          frogpilot_toggles.relaxed_jerk_deceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed_decrease,
          frogpilot_toggles.custom_personalities, controlsState.personality
        )
      normal_t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities, controlsState.personality
      )

      if aEgo >= 0:
        traffic_acceleration_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_acceleration))
        traffic_speed_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed))
      else:
        traffic_acceleration_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_deceleration))
        traffic_speed_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed_decrease))
      traffic_danger_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_danger))
      traffic_t_follow = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_follow))

      blend_step = DT_MDL / TRAFFIC_MODE_BLEND_TIME
      blend_target = 1.0 if frogpilotCarState.trafficMode else 0.0
      self.traffic_mode_blend = float(np.clip(blend_target, self.traffic_mode_blend - blend_step,
                                             self.traffic_mode_blend + blend_step))
      blend = self.traffic_mode_blend
      self.base_acceleration_jerk = float(np.interp(blend, [0.0, 1.0], [normal_acceleration_jerk, traffic_acceleration_jerk]))
      self.base_danger_jerk = float(np.interp(blend, [0.0, 1.0], [normal_danger_jerk, traffic_danger_jerk]))
      self.base_speed_jerk = float(np.interp(blend, [0.0, 1.0], [normal_speed_jerk, traffic_speed_jerk]))
      self.t_follow = float(np.interp(blend, [0.0, 1.0], [normal_t_follow, traffic_t_follow]))
    else:
      self.traffic_mode_blend = 0.0
      self.base_acceleration_jerk = 0
      self.base_danger_jerk = 0
      self.base_speed_jerk = 0
      self.t_follow = 0

    self.acceleration_jerk = self.base_acceleration_jerk
    self.danger_jerk = self.base_danger_jerk
    self.speed_jerk = self.base_speed_jerk

    self.following_lead = self.frogpilot_planner.tracking_lead and lead_distance < (self.t_follow + 1) * v_ego

    if controlsState.enabled and self.frogpilot_planner.tracking_lead:
      self.update_follow_values(lead_distance, v_ego, self.frogpilot_planner.lead_one.vLead, frogpilot_toggles)
      self.desired_follow_distance = int(desired_follow_distance(v_ego, self.frogpilot_planner.lead_one.vLead, self.t_follow))
    else:
      self.desired_follow_distance = 0

  def update_follow_values(self, lead_distance, v_ego, v_lead, frogpilot_toggles):
    # Offset by FrogAi for FrogPilot for a more natural approach to a faster lead
    if frogpilot_toggles.human_following and v_lead > v_ego:
      distance_factor = max(lead_distance - (v_ego * self.t_follow), 1)
      standstill_offset = max(STOP_DISTANCE - v_ego, 1)
      acceleration_offset = float(np.clip((v_lead - v_ego) * standstill_offset - COMFORT_BRAKE, 1, distance_factor))
      self.acceleration_jerk /= standstill_offset
      self.speed_jerk /= standstill_offset
      self.t_follow /= acceleration_offset

    # Offset by FrogAi for FrogPilot for a more natural approach to a slower lead
    if (frogpilot_toggles.conditional_slower_lead or frogpilot_toggles.human_following) and v_lead < v_ego:
      distance_factor = max(lead_distance - (v_lead * self.t_follow), 1)
      far_lead_offset = max(v_lead - CITY_SPEED_LIMIT, 1)
      braking_offset = float(np.clip(min(v_ego - v_lead, v_lead) * far_lead_offset - COMFORT_BRAKE, 1, distance_factor))
      if frogpilot_toggles.human_following:
        self.t_follow /= braking_offset
      self.slower_lead = braking_offset / far_lead_offset > 1
