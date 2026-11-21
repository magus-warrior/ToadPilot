"""Use the pinned StarPilot planner with the existing FrogPilot service schema."""
import numpy as np

from cereal import messaging
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_planned_cruise_speed, planner_required_services
from openpilot.selfdrive.frogpilot.frogpilot_variables import THRESHOLD
from openpilot.selfdrive.toad_modeld.artifact import is_modern_model
from openpilot.selfdrive.toad_modeld.starpilot_long.compat import StateView, View, vehicle_view
from openpilot.selfdrive.toad_modeld.starpilot_long.lead_behavior import should_disable_far_lead_throttle
from openpilot.selfdrive.toad_modeld.starpilot_long.mpc import LEAD_DANGER_FACTOR, STOP_DISTANCE, desired_follow_distance
from openpilot.selfdrive.toad_modeld.starpilot_long.planner import LongitudinalPlanner, FORCE_STOP_HANDOFF_MAX_VCRUISE
from openpilot.selfdrive.toad_modeld.starpilot_long.vehicle_tunes import get_far_lead_coast_headway_floor


class StarPilotPlanner:
  def __init__(self, cp, init_v=0., init_a=0.):
    if cp.carName != 'toyota' or cp.carFingerprint != 'TOYOTA_CAMRY_TSS2' or not cp.openpilotLongitudinalControl:
      raise ValueError('StarPilot adapter requires Camry TSS2 openpilot longitudinal control')
    self.core = LongitudinalPlanner(vehicle_view(cp), init_v=init_v, init_a=init_a)
    self.tracking_filter = FirstOrderFilter(0., 1., DT_MDL)
    self.a_desired_trajectory_full = self.core.a_desired_trajectory.copy()

  def __getattr__(self, name):
    return getattr(self.core, name)

  def update(self, radarless_model, sm, toggles):
    if radarless_model:
      raise ValueError('This adapter requires the existing radarState lead publisher')
    lead = sm['radarState'].leadOne
    speed = sm['carState'].vEgo
    model_x = sm['modelV2'].position.x
    length = float(model_x[-1]) if len(model_x) else 0.
    experimental = bool(sm['controlsState'].experimentalMode)
    # An end-to-end stop/short path must not remove a measured lead from ACC.
    lead_visible = bool(lead.status and (not experimental or lead.dRel < length + STOP_DISTANCE))
    tracking = bool(self.tracking_filter.update(lead_visible) >= THRESHOLD ** 2)
    following = tracking and lead.dRel < sm['frogpilotPlan'].tFollow * 2 * speed
    disable_throttle = False
    # StarPilot's additional coast veto is part of experimental driving.
    # Ordinary ACC leaves acceleration control to cruise and the lead MPC.
    if sm['controlsState'].experimentalMode and tracking and lead.status:
      gap = desired_follow_distance(speed, lead.vLead, sm['frogpilotPlan'].tFollow)
      disable_throttle = should_disable_far_lead_throttle(
        speed, lead.dRel, gap, max(0., speed - lead.vLead), following,
        get_far_lead_coast_headway_floor(self.core.CP))
    plan = View(sm['frogpilotPlan'], vCruise=get_planned_cruise_speed(sm, self.core.CP, toggles),
                trackingLead=tracking, dangerFactor=LEAD_DANGER_FACTOR,
                disableThrottle=disable_throttle)
    self.state = StateView(sm, plan=plan)
    # The legacy selector labels only v7 as tinygrad. Registered on-device
    # models also publish action commands, which the pinned planner requires
    # this flag to consume. Keep the override local to this adapter.
    planner_toggles = View(toggles, tinygrad_model=True) if is_modern_model(toggles) else toggles
    self.core.update(self.state, planner_toggles)
    self.a_desired_trajectory_full = self.core.a_desired_trajectory.copy()

  def publish(self, classic_model, radarless_model, sm, pm, toggles):
    if classic_model or radarless_model:
      raise ValueError('StarPilot planner requires a registered modern model')
    message = messaging.new_message('longitudinalPlan')
    required = planner_required_services(sm['controlsState'].experimentalMode, radarless_model)
    message.valid = sm.all_checks(service_list=required)
    if sm['controlsState'].experimentalMode:
      message.valid = message.valid and bool(sm['modelV2'].action.longitudinalValid)
    output = message.longitudinalPlan
    core = self.core
    output.modelMonoTime = sm.logMonoTime['modelV2']
    output.processingDelay = message.logMonoTime / 1e9 - output.modelMonoTime / 1e9
    output.solverExecutionTime = core.mpc.solve_time
    output.speeds = core.v_desired_trajectory.tolist()
    output.accels = core.a_desired_trajectory.tolist()
    output.jerks = core.j_desired_trajectory.tolist()
    selected = sm['radarState'].leadTwo if core.mpc.source == 'lead1' else sm['radarState'].leadOne
    output.hasLead = bool(selected.status)
    output.longitudinalPlanSource = core.mpc.source
    output.fcw = core.fcw
    output.aTarget = float(core.output_a_target)
    plan = self.state['starpilotPlan']
    force_handoff = plan.forcingStop and (plan.forcingStopLength < 1. or plan.vCruise <= FORCE_STOP_HANDOFF_MAX_VCRUISE)
    output.shouldStop = bool(core.output_should_stop or force_handoff)
    output.allowBrake = True
    output.allowThrottle = core.allow_throttle
    if not np.isfinite([output.aTarget, *core.v_desired_trajectory, *core.a_desired_trajectory]).all():
      message.valid = False
    pm.send('longitudinalPlan', message)
