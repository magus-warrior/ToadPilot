"""Regression coverage for profile bypass and premature stopped-lead holds."""
from types import SimpleNamespace as NS

import numpy as np
import pytest
from cereal import log

from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_acceleration import FrogPilotAcceleration
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_following import FrogPilotFollowing
from openpilot.selfdrive.frogpilot.tests.test_starpilot_braking_settings import settings
from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import Publisher, planner_setup
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.toad_modeld.starpilot_long.adapter import StarPilotPlanner


def stopped_lead_case(tmp_path, gap, *, speed=.4, experimental=False):
  fixture, sm, toggles = planner_setup(tmp_path)
  planner = StarPilotPlanner(fixture.core.CP.original, init_v=speed)
  sm['controlsState'].experimentalMode = experimental
  state = sm['carState']
  state.vEgo = state.vEgoCluster = speed
  state.standstill = speed == 0.
  lead = sm['radarState'].leadOne
  lead.status = lead.radar = True
  lead.modelProb, lead.dRel, lead.vLead, lead.vLeadK = 1., gap, 0., 0.
  lead.aLeadK, lead.aLeadTau = 0., 1.5
  plan = sm['frogpilotPlan']
  plan.vCruise, plan.tFollow, plan.trafficModeBlend = 20., .7, 1.
  plan.minAcceleration, plan.maxAcceleration = -2.4, 3.
  model = sm['modelV2']
  model.position.x = (np.asarray(ModelConstants.T_IDXS) * max(speed, .01)).tolist()
  model.velocity.x = [max(speed, .01)] * 33
  model.action.desiredAcceleration, model.action.shouldStop = .4, False
  model.meta.disengagePredictions.gasPressProbs = [1.] * 6
  return planner, sm, toggles


@pytest.mark.parametrize('experimental', [False, True])
@pytest.mark.parametrize('gap', [7.1, 8., 8.5])
@pytest.mark.parametrize('speed', [0., .4])
def test_stopped_lead_outside_gap_does_not_force_hold(tmp_path, experimental, gap, speed):
  planner, sm, toggles = stopped_lead_case(tmp_path, gap, speed=speed, experimental=experimental)
  for _ in range(40):
    planner.update(False, sm, toggles)
    assert planner.mpc.solution_status == 0
  assert not planner.output_should_stop
  if speed == 0.:
    assert planner.output_a_target > 0.
  else:
    # A rolling approach may taper speed without entering the stop state.
    assert planner.output_a_target > -.16


@pytest.mark.parametrize('experimental', [False, True])
@pytest.mark.parametrize('gap', [3., 5., 5.5])
def test_stopped_lead_inside_target_keeps_hold(tmp_path, experimental, gap):
  planner, sm, toggles = stopped_lead_case(tmp_path, gap, experimental=experimental)
  for _ in range(40):
    planner.update(False, sm, toggles)
  publisher = Publisher()
  planner.publish(False, False, sm, publisher, toggles)
  assert publisher.message.longitudinalPlan.shouldStop
  assert planner.output_a_target < 0.


@pytest.mark.parametrize('traffic', [False, True])
def test_nonurgent_lead_braking_uses_selected_response(tmp_path, traffic):
  runs = []
  for intensity in (.25, 2.):
    fixture, sm, toggles = planner_setup(tmp_path / str(intensity))
    settings(toggles, 2, intensity)
    planner = StarPilotPlanner(fixture.core.CP.original, init_v=20.)
    sm['controlsState'].vCruise = 108.
    sm['controlsState'].personality = log.LongitudinalPersonality.aggressive
    sm['frogpilotCarState'].trafficMode = traffic
    state = sm['carState']
    state.vEgo = state.vEgoCluster = 20.
    state.aEgo = -.01
    lead = sm['radarState'].leadOne
    lead.status = lead.radar = True
    lead.modelProb, lead.aLeadTau = 1., 1.5
    lead.dRel, lead.vLead, lead.vLeadK, lead.aLeadK = 40., 18., 18., -.8
    model = sm['modelV2']
    model.position.x = (np.asarray(ModelConstants.T_IDXS) * 20.).tolist()
    model.velocity.x = [20.] * 33
    model.action.desiredAcceleration, model.action.shouldStop = .4, False
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    parent = NS(tracking_lead=True, lead_one=lead, v_cruise=30.)
    following = parent.frogpilot_following = FrogPilotFollowing(parent)
    following.traffic_mode_blend = float(traffic)
    acceleration = FrogPilotAcceleration(parent)
    rows = []
    for _ in range(10):
      following.update(-.01, sm['controlsState'], sm['frogpilotCarState'], 40., 20., toggles)
      acceleration.update(sm['frogpilotCarState'], 20., toggles)
      plan = sm['frogpilotPlan']
      plan.vCruise, plan.tFollow = 30., following.t_follow
      plan.minAcceleration, plan.maxAcceleration = acceleration.min_accel, acceleration.max_accel
      plan.accelerationJerk, plan.speedJerk = 200. * following.acceleration_jerk, 5. * following.speed_jerk
      plan.dangerJerk, plan.trafficModeBlend = 100. * following.danger_jerk, float(traffic)
      planner.update(False, sm, toggles)
      assert planner.mpc.solution_status == 0
      rows.append(planner.output_a_target)
    runs.append(np.asarray(rows))
  # The old post-MPC cap flattened both profiles to -0.6267 immediately.
  assert runs[0].mean() < runs[1].mean() - .02
  assert runs[1][0] > -.4


@pytest.mark.parametrize('gap,minimum', [(10., -2.4), (40., -.6)])
def test_urgent_or_out_of_profile_lead_braking_retains_cap(tmp_path, gap, minimum):
  planner, _, _ = planner_setup(tmp_path)
  lead = NS(status=True, radar=True, dRel=gap, vLead=18., aLeadK=-.8)
  baseline = planner.core.get_close_lead_brake_cap(lead, 20., -3.5)
  actual = planner.core.get_close_lead_brake_cap(lead, 20., -3.5, comfort_accel_min=minimum)
  assert actual == pytest.approx(baseline)
  assert actual < 0.


@pytest.mark.parametrize('stop_kind', ['model', 'forced'])
def test_explicit_stop_still_holds_outside_lead_target(tmp_path, stop_kind):
  planner, sm, toggles = stopped_lead_case(tmp_path, 8., experimental=True)
  if stop_kind == 'model':
    sm['modelV2'].action.shouldStop = True
  else:
    sm['frogpilotPlan'].forcingStop = True
    sm['frogpilotPlan'].forcingStopLength = 4.
    sm['frogpilotPlan'].vCruise = 0.
  for _ in range(40):
    planner.update(False, sm, toggles)
  publisher = Publisher()
  planner.publish(False, False, sm, publisher, toggles)
  assert publisher.message.longitudinalPlan.shouldStop


@pytest.mark.parametrize('experimental', [False, True])
def test_closed_loop_stopped_lead_reaches_target_without_premature_hold(tmp_path, experimental):
  planner, sm, toggles = stopped_lead_case(tmp_path, 14., speed=3., experimental=experimental)
  speed, gap, previous = 3., 14., 0.
  first_hold_gap = None
  for _ in range(600):
    state = sm['carState']
    state.vEgo = state.vEgoCluster = speed
    state.aEgo, state.standstill = previous, speed < .01
    lead = sm['radarState'].leadOne
    lead.dRel = gap
    model = sm['modelV2']
    model.position.x = (np.asarray(ModelConstants.T_IDXS) * max(speed, .01)).tolist()
    model.velocity.x = [max(speed, .01)] * 33
    planner.update(False, sm, toggles)
    assert planner.mpc.solution_status == 0
    accel = planner.output_a_target
    if planner.output_should_stop:
      if first_hold_gap is None:
        first_hold_gap = gap
      # Represent the downstream stopping state, which finishes a requested stop.
      accel = min(accel, -.6)
    following_speed = max(0., speed + accel * .05)
    gap -= (speed + following_speed) * .025
    assert gap > 3., (gap, speed, accel)
    speed, previous = following_speed, accel
  assert first_hold_gap is not None and first_hold_gap < 6.5
  assert speed < .1
  assert 5. <= gap <= 6.2
