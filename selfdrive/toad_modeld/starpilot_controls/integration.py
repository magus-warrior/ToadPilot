"""Run native StarPilot stop/curve managers on legacy FrogPilot messages.

The installed acceleration/following profiles remain the inputs to the native
longitudinal core. No model, launch curve, or numeric user preference is changed.
"""
from types import SimpleNamespace as NS
import numpy as np
from cereal import car
from openpilot.selfdrive.toad_modeld.starpilot_long.compat import View
from openpilot.selfdrive.toad_modeld.starpilot_controls.compat import ParamsAdapter
from openpilot.selfdrive.toad_modeld.starpilot_controls.conditional_experimental_mode import ConditionalExperimentalMode
from openpilot.selfdrive.toad_modeld.starpilot_controls.constants import MINIMUM_LATERAL_ACCELERATION, PLANNER_TIME
from openpilot.selfdrive.toad_modeld.starpilot_controls.geometry import calculate_road_curvature, extract_curve_profile
from openpilot.selfdrive.toad_modeld.starpilot_controls.vcruise import StarPilotVCruise


class StarPilotControls:
  def __init__(self, parent, params, params_memory, *, state_path='/data/toadpilot/starpilot-controls.json'):
    self.parent = parent
    self.params = ParamsAdapter(params, path=state_path)
    self.params_memory = ParamsAdapter(params_memory)
    self.starpilot_following = parent.frogpilot_following
    self.starpilot_weather = NS(weather_id=0, reduce_lateral_acceleration=0.0)
    self.curve_profile = (np.array([]), np.array([]))
    self.driving_in_curve = False
    self.lateral_acceleration = 0.0
    self.time_to_curve = 1.0
    self.raw_model_stopped = False
    self.starpilot_cem = ConditionalExperimentalMode(self)
    self.starpilot_vcruise = StarPilotVCruise(self)
    raw_cp = params.get('CarParamsPersistent')
    if raw_cp:
      with car.CarParams.from_bytes(raw_cp) as cp:
        self.cp = cp.as_builder()
    else:
      self.cp = NS(carFingerprint='TOYOTA_CAMRY_TSS2', minSteerSpeed=0.0)

  def __getattr__(self, name):
    return getattr(self.parent, name)

  def prepare(self, car_state, controls, frogcar, navigation, model, car_control, toggles):
    if car_control is None:
      car_control = NS(longActive=controls.enabled)
    self.sm = dict(carState=View(car_state, vCruise=controls.vCruise,
                                vCruiseCluster=controls.vCruiseCluster),
                   selfdriveState=controls, controlsState=controls, carControl=car_control,
                   starpilotCarState=View(frogcar, trafficModeEnabled=frogcar.trafficMode,
                                         dashboardStopSign=0,
                                         alwaysOnLateralEnabled=getattr(frogcar, 'alwaysOnLateral', False)),
                   modelV2=model, carParams=self.cp, frogpilotNavigation=navigation)
    self.params_memory.manual_enabled = toggles.experimental_mode_via_press
    # Unavailable optional inputs are explicitly disabled, not fabricated.
    self.toggles = View(toggles, curve_speed_controller=toggles.vision_turn_speed_controller,
                        csc_no_lead=False, nav_longitudinal_allowed=False,
                        force_stop_distance_offset=0, conditional_open_road=False)
    self.raw_model_stopped = self.parent.model_length < 5.0 * PLANNER_TIME
    self.road_curvature, self.time_to_curve = calculate_road_curvature(model, car_state.vEgo)
    self.curve_profile = extract_curve_profile(model)
    self.lateral_acceleration = car_state.vEgo ** 2 * controls.curvature
    self.driving_in_curve = abs(self.lateral_acceleration) >= MINIMUM_LATERAL_ACCELERATION
    self.parent.road_curvature = self.road_curvature

  def update_mode(self, v_ego, v_cruise, controls_enabled):
    active = controls_enabled or self.sm['starpilotCarState'].alwaysOnLateralEnabled
    if active and self.toggles.conditional_experimental_mode:
      self.starpilot_cem.update(v_ego, self.sm, self.toggles, v_cruise)
    else:
      self.starpilot_cem.deactivate()
      self.starpilot_cem.curve_detected = False
      self.starpilot_cem.stop_sign_and_light(v_ego, self.sm, PLANNER_TIME - 2)

  def update_cruise(self, v_cruise, v_ego, controls_enabled, now):
    result = self.starpilot_vcruise.update(controls_enabled, now, True, v_cruise, v_ego, self.sm, self.toggles)
    return result

  def close(self):
    self.starpilot_vcruise.csc.flush_data()
