"""Regressions for the legacy radar/StarPilot vehicle-speed adapter."""
import numpy as np
import pytest

from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import planner_setup
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.toad_modeld.starpilot_long.adapter import StarPilotPlanner
from openpilot.selfdrive.toad_modeld.starpilot_long.compat import StateView, View


def test_adapter_uses_radar_speed_frame_without_changing_car_state(tmp_path):
  _, sm, _ = planner_setup(tmp_path)
  sm['carState'].vEgo = 35.
  sm['carState'].vEgoCluster = 35.525
  display_speed = sm['carState'].vEgoCluster
  state = StateView(sm, plan=View(sm['frogpilotPlan']))
  assert state['carState'].vEgoCluster == state['carState'].vEgo == 35.
  assert sm['carState'].vEgoCluster == display_speed
  assert state['carState'].vCruise == sm['controlsState'].vCruise


def run_follow(tmp_path, offset, speed, braking):
  fixture, sm, toggles = planner_setup(tmp_path)
  planner = StarPilotPlanner(fixture.core.CP.original, init_v=speed)
  sm['controlsState'].personality = 'aggressive'
  sm['controlsState'].vCruise = (speed + 3.) * 3.6
  sm['frogpilotPlan'].vCruise = speed + 3.
  sm['frogpilotPlan'].speedJerk = 2.5
  sm['frogpilotPlan'].tFollow = 1.
  ego, lead_speed, gap, acceleration = speed, speed, speed + 20., 0.
  samples = []
  for frame in range(160):
    if braking and frame >= 40:
      lead_speed = max(0., lead_speed - .125)
    state = sm['carState']
    state.vEgo, state.vEgoCluster, state.aEgo = ego, ego * offset, acceleration
    state.standstill = ego < .01
    model = sm['modelV2']
    model.velocity.x = [max(.01, ego)] * 33
    model.position.x = (np.asarray(ModelConstants.T_IDXS) * max(.01, ego)).tolist()
    model.action.desiredAcceleration = .4
    model.action.shouldStop = False
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
      lead.status, lead.radar, lead.modelProb = True, True, 1.
      lead.dRel, lead.vLead, lead.vLeadK = gap, lead_speed, lead_speed
      lead.aLeadK = -2.5 if braking and frame >= 40 and lead_speed > 0. else 0.
      lead.aLeadTau = 1.5
    planner.update(False, sm, toggles)
    acceleration = planner.core.output_a_target
    assert planner.mpc.solution_status == 0
    assert np.isfinite(acceleration) and -3.51 <= acceleration <= 2.01
    next_ego = max(0., ego + acceleration * .05)
    gap += (lead_speed - (ego + next_ego) / 2.) * .05
    assert gap > 0.
    samples.append((acceleration, gap, planner.core.effective_t_follow))
    ego = next_ego
  return np.asarray(samples)


@pytest.mark.parametrize('speed', [15., 35.])
@pytest.mark.parametrize('braking', [False, True])
def test_display_speed_cannot_change_following_or_braking(tmp_path, speed, braking):
  measured = run_follow(tmp_path, 1., speed, braking)
  corrected_display = run_follow(tmp_path, 1.015, speed, braking)
  np.testing.assert_allclose(corrected_display, measured, rtol=0., atol=1e-9)
  if braking:
    assert measured[-1, 0] < -.5
