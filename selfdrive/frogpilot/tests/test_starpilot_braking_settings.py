"""Exercise selected profiles/intensities through FrogPilot and the native planner."""
from types import SimpleNamespace as NS

import numpy as np
import pytest
from cereal import log

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import A_CHANGE_COST, DANGER_ZONE_COST, J_EGO_COST
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_acceleration import FrogPilotAcceleration
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_following import FrogPilotFollowing
from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import planner_setup
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.toad_modeld.starpilot_long.adapter import StarPilotPlanner


def settings(toggles, profile, intensity=1.):
  toggles.custom_personalities = True
  toggles.toadpilot_starpilot_response = True
  toggles.toadpilot_enabled = True
  toggles.car_model = 'TOYOTA_CAMRY_TSS2'
  toggles.acceleration_profile = 2
  toggles.deceleration_profile = profile
  toggles.map_acceleration = toggles.map_deceleration = False
  toggles.sport_plus = toggles.human_acceleration = False
  toggles.human_following = toggles.conditional_slower_lead = False
  for personality in ('aggressive', 'standard', 'relaxed'):
    setattr(toggles, f'{personality}_follow', 1.)
    for field in ('acceleration', 'deceleration', 'danger', 'speed', 'speed_decrease'):
      setattr(toggles, f'{personality}_jerk_{field}', intensity)
  toggles.traffic_mode_follow = [.7, 1.]
  for field in ('acceleration', 'deceleration', 'danger', 'speed', 'speed_decrease'):
    setattr(toggles, f'traffic_mode_jerk_{field}', [intensity, intensity])


def run_cruise(tmp_path, profile, traffic, *, intensity=1., slowing=True, model_stop=False):
  fixture, sm, toggles = planner_setup(tmp_path)
  settings(toggles, profile, intensity)
  planner = StarPilotPlanner(fixture.core.CP.original, init_v=25.)
  parent = NS(tracking_lead=False, lead_one=sm['radarState'].leadOne, v_cruise=20. if slowing else 30.)
  following = parent.frogpilot_following = FrogPilotFollowing(parent)
  following.traffic_mode_blend = float(traffic)
  acceleration = FrogPilotAcceleration(parent)
  sm['controlsState'].personality = log.LongitudinalPersonality.aggressive
  sm['controlsState'].vCruise = parent.v_cruise * 3.6
  sm['frogpilotCarState'].trafficMode = traffic
  ego, previous_accel = 25., -.01 if slowing else .01
  samples = []
  # Settle the lead filters at cruising speed before changing the target.
  for frame in range(140):
    state = sm['carState']
    state.vEgo = state.vEgoCluster = ego
    state.aEgo = previous_accel
    model = sm['modelV2']
    model.velocity.x = [ego] * 33
    model.position.x = (np.asarray(ModelConstants.T_IDXS) * ego).tolist()
    model.action.desiredAcceleration = .4
    model.action.shouldStop = model_stop
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    following.update(previous_accel, sm['controlsState'], sm['frogpilotCarState'], 200., ego, toggles)
    acceleration.update(sm['frogpilotCarState'], ego, toggles)
    plan = sm['frogpilotPlan']
    target_speed = 25. if frame < 60 else parent.v_cruise
    plan.vCruise = target_speed
    sm['controlsState'].vCruise = target_speed * 3.6
    plan.minAcceleration, plan.maxAcceleration = acceleration.min_accel, acceleration.max_accel
    plan.accelerationJerk = A_CHANGE_COST * following.acceleration_jerk
    plan.dangerJerk = DANGER_ZONE_COST * following.danger_jerk
    plan.speedJerk = J_EGO_COST * following.speed_jerk
    plan.tFollow = following.t_follow
    plan.trafficModeBlend = following.traffic_mode_blend
    planner.update(False, sm, toggles)
    target = planner.output_a_target
    assert planner.mpc.solution_status == 0
    assert np.isfinite(target)
    if frame >= 60:
      samples.append((target, acceleration.min_accel, acceleration.max_accel))
      ego = max(0., ego + target * DT_MDL)
      previous_accel = target
  return np.asarray(samples)


@pytest.mark.parametrize('profile,minimum', [(1, -.6), (0, -1.2), (2, -2.4)])
@pytest.mark.parametrize('traffic', [False, True])
def test_cruise_slowing_respects_selected_profile(tmp_path, profile, minimum, traffic):
  samples = run_cruise(tmp_path, profile, traffic)
  np.testing.assert_allclose(samples[:, 1], minimum)
  assert samples[:, 0].min() >= minimum - 1e-5
  assert samples[:, 0].min() < -.1


@pytest.mark.parametrize('traffic', [False, True])
def test_normal_mode_does_not_treat_model_stop_as_profile_override(tmp_path, traffic):
  samples = run_cruise(tmp_path, 1, traffic, model_stop=True)
  assert samples[:, 0].min() >= -.60001


@pytest.mark.parametrize('traffic', [False, True])
@pytest.mark.parametrize('slowing', [False, True])
def test_selected_intensity_changes_native_planner_response(tmp_path, traffic, slowing):
  responsive = run_cruise(tmp_path, 2, traffic, intensity=.25, slowing=slowing)
  smooth = run_cruise(tmp_path, 2, traffic, intensity=2., slowing=slowing)
  # Compare the initial response before either run reaches the cruise target.
  direction = -1. if slowing else 1.
  assert direction * responsive[:10, 0].mean() > direction * smooth[:10, 0].mean() + .02


@pytest.mark.parametrize('traffic', [False, True])
def test_braking_and_acceleration_intensities_are_independent(tmp_path, traffic):
  _, sm, toggles = planner_setup(tmp_path)
  settings(toggles, 2)
  toggles.aggressive_jerk_acceleration = .2
  toggles.aggressive_jerk_speed = .3
  toggles.aggressive_jerk_deceleration = 1.7
  toggles.aggressive_jerk_speed_decrease = 1.9
  toggles.traffic_mode_jerk_acceleration = [.4, .4]
  toggles.traffic_mode_jerk_speed = [.6, .6]
  toggles.traffic_mode_jerk_deceleration = [2.1, 2.1]
  toggles.traffic_mode_jerk_speed_decrease = [2.3, 2.3]
  following = FrogPilotFollowing(NS(tracking_lead=False))
  following.traffic_mode_blend = float(traffic)
  sm['controlsState'].personality = log.LongitudinalPersonality.aggressive
  sm['frogpilotCarState'].trafficMode = traffic
  for measured_accel in (.1, -.1):
    following.update(measured_accel, sm['controlsState'], sm['frogpilotCarState'], 100., 10., toggles)
    expected = ((.4, .6) if measured_accel > 0 else (2.1, 2.3)) if traffic else ((.2, .3) if measured_accel > 0 else (1.7, 1.9))
    assert (following.acceleration_jerk, following.speed_jerk) == pytest.approx(expected)


@pytest.mark.parametrize('traffic', [False, True])
def test_starpilot_preserves_enabled_human_following(tmp_path, traffic):
  _, sm, toggles = planner_setup(tmp_path)
  settings(toggles, 2)
  toggles.human_following = True
  sm['controlsState'].personality = log.LongitudinalPersonality.aggressive
  sm['frogpilotCarState'].trafficMode = traffic
  lead = NS(vLead=20.)
  results = []
  for response_enabled in (False, True):
    toggles.toadpilot_starpilot_response = response_enabled
    following = FrogPilotFollowing(NS(tracking_lead=True, lead_one=lead))
    following.traffic_mode_blend = float(traffic)
    following.update(-.1, sm['controlsState'], sm['frogpilotCarState'], 70., 25., toggles)
    results.append((following.t_follow, following.acceleration_jerk, following.speed_jerk))
  assert results[1] == pytest.approx(results[0])


@pytest.mark.parametrize('traffic', [False, True])
@pytest.mark.parametrize('scenario', ['braking_lead', 'cut_in', 'forced_stop'])
def test_eco_and_human_following_preserve_necessary_braking(tmp_path, monkeypatch, traffic, scenario):
  from openpilot.selfdrive.toad_modeld.starpilot_long import planner as source

  fixture, sm, toggles = planner_setup(tmp_path)
  settings(toggles, 1, intensity=.5)
  toggles.human_following = True
  planner = StarPilotPlanner(fixture.core.CP.original, init_v=27.)
  clock = [100.]
  monkeypatch.setattr(source, 'time', NS(monotonic=lambda: clock[0]))
  parent = NS(tracking_lead=scenario != 'forced_stop', lead_one=sm['radarState'].leadOne, v_cruise=30.)
  following = parent.frogpilot_following = FrogPilotFollowing(parent)
  following.traffic_mode_blend = float(traffic)
  acceleration = FrogPilotAcceleration(parent)
  sm['controlsState'].personality = log.LongitudinalPersonality.aggressive
  sm['controlsState'].vCruise = 108.
  sm['frogpilotCarState'].trafficMode = traffic
  ego, lead_speed, gap, previous_accel = 27., 27., 160. if scenario == 'forced_stop' else 45., 0.
  targets = []
  for frame in range(360):
    clock[0] += DT_MDL
    lead_accel = -2.5 if scenario == 'braking_lead' and frame >= 40 and lead_speed > 0. else 0.
    lead_speed = max(0., lead_speed + lead_accel * DT_MDL)
    if scenario == 'cut_in' and frame == 40:
      gap, lead_speed = 20., 18.
    state = sm['carState']
    state.vEgo = state.vEgoCluster = ego
    state.aEgo, state.standstill = previous_accel, ego < .01
    model = sm['modelV2']
    model.velocity.x = [max(.01, ego)] * 33
    model.position.x = (np.asarray(ModelConstants.T_IDXS) * max(.01, ego)).tolist()
    model.action.desiredAcceleration, model.action.shouldStop = .4, False
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    lead = sm['radarState'].leadOne
    lead.status, lead.radar, lead.modelProb = parent.tracking_lead, True, 1.
    lead.dRel, lead.vLead, lead.vLeadK = gap, lead_speed, lead_speed
    lead.aLeadK, lead.aLeadTau = lead_accel, 1.5
    following.update(previous_accel, sm['controlsState'], sm['frogpilotCarState'], gap, ego, toggles)
    acceleration.update(sm['frogpilotCarState'], ego, toggles)
    plan = sm['frogpilotPlan']
    plan.vCruise = 0. if scenario == 'forced_stop' else 30.
    plan.minAcceleration, plan.maxAcceleration = acceleration.min_accel, acceleration.max_accel
    plan.accelerationJerk = A_CHANGE_COST * following.acceleration_jerk
    plan.dangerJerk = DANGER_ZONE_COST * following.danger_jerk
    plan.speedJerk = J_EGO_COST * following.speed_jerk
    plan.tFollow = following.t_follow
    plan.trafficModeBlend = following.traffic_mode_blend
    plan.forcingStop = scenario == 'forced_stop'
    if plan.forcingStop:
      plan.forcingStopLength = max(gap, 0.)
    planner.update(False, sm, toggles)
    target = planner.output_a_target
    assert planner.mpc.solution_status == 0
    assert np.isfinite(target) and -3.51 <= target <= 2.01
    targets.append(target)
    next_ego = max(0., ego + target * DT_MDL)
    gap += ((0. if plan.forcingStop else lead_speed) - (ego + next_ego) / 2.) * DT_MDL
    assert gap > 0., (scenario, frame, ego, gap, target)
    ego, previous_accel = next_ego, target
  # Eco's ordinary -0.6 limit must not prevent a necessary stronger response.
  assert min(targets) < -.65
  if scenario in ('braking_lead', 'forced_stop'):
    assert ego < 1.
