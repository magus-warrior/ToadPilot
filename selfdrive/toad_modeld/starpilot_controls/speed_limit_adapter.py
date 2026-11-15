"""Keep the installed map/navigation speed-limit sources and preferences."""
from openpilot.common.realtime import DT_MDL


class SpeedLimitController:
  def __init__(self, vcruise):
    self.starpilot_planner = vcruise.starpilot_planner
    self.delegate = self.starpilot_planner.parent.frogpilot_vcruise.slc
    self.starpilot_toggles = None
    self.target = self.offset = self.overridden_speed = 0.0
    self.confirmation_pending = self.confirmation_button_consumed = False
    self.confirmation_time = 0.0

  @property
  def experimental_mode(self):
    return self.delegate.experimental_mode

  def update(self, dashboard_limit, now, time_validated, v_cruise, v_cruise_diff,
             v_ego, v_ego_diff, sm, *, active, display_only):
    toggles = self.starpilot_toggles
    self.confirmation_button_consumed = False
    if not (active or display_only):
      self.target = self.offset = self.overridden_speed = 0.0
      self.confirmation_pending = False
      return
    nav = sm['frogpilotNavigation']
    self.delegate.update(dashboard_limit, sm['selfdriveState'].enabled,
                         nav.navigationSpeedLimit, v_cruise + v_cruise_diff, v_ego, toggles)
    desired = self.delegate.desired_speed_limit
    if self.delegate.speed_limit_changed:
      self.confirmation_time += DT_MDL
      memory = self.starpilot_planner.params_memory
      accepted = bool((sm['starpilotCarState'].accelPressed and str(sm['selfdriveState'].state) == 'enabled') or
                      memory.get_bool('SpeedLimitAccepted'))
      confirmation = toggles.speed_limit_confirmation_lower if desired < self.target else toggles.speed_limit_confirmation_higher
      if accepted or not confirmation:
        self.target = desired
        self.delegate.speed_limit_changed = False
        self.confirmation_button_consumed = accepted
        if accepted:
          memory.remove('SpeedLimitAccepted')
      elif sm['starpilotCarState'].decelPressed or self.confirmation_time >= 30.0:
        self.delegate.speed_limit_changed = False
      self.confirmation_pending = self.delegate.speed_limit_changed
    else:
      self.confirmation_time = 0.0
      self.confirmation_pending = False
      if not self.target:
        self.target = desired
    self.offset = self.delegate.get_offset(self.target, toggles)
    override = (self.overridden_speed > self.target + self.offset or
                sm['carState'].gasPressed and v_ego > self.target + self.offset)
    if override and active and sm['selfdriveState'].enabled:
      if toggles.speed_limit_controller_override_manual:
        if sm['carState'].gasPressed:
          self.overridden_speed = v_ego + v_ego_diff
      else:
        self.overridden_speed = v_cruise + v_cruise_diff
    else:
      self.overridden_speed = 0.0
