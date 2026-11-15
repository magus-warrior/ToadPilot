"""Traffic headway reaches the solver without another approach allowance."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import planner_setup
from openpilot.selfdrive.modeld.constants import ModelConstants


@pytest.mark.parametrize('radar', [False, True])
def test_traffic_entry_clears_extra_approach_headway(tmp_path, radar):
  planner, _, _ = planner_setup(tmp_path)
  core = planner.core
  lead = NS(status=True, radar=radar, modelProb=1., dRel=16., vLead=8., aLeadK=-.3)
  for _ in range(30):
    normal = core.get_dynamic_t_follow(.7, lead, 10.)
  assert normal > .75
  # Do not carry extra normal-mode headway into the selected traffic gap.
  assert core.get_dynamic_t_follow(.7, lead, 10., 1.) == pytest.approx(.7)
  # Full traffic follows changes to the supplied profile immediately.
  assert core.get_dynamic_t_follow(.8, lead, 10., 1.) == pytest.approx(.8)


def test_traffic_transition_gradually_removes_approach_allowance(tmp_path):
  planner, _, _ = planner_setup(tmp_path)
  lead = NS(status=True, radar=True, modelProb=1., dRel=16., vLead=8., aLeadK=-.3)
  results = []
  for blend in (0., .5, 1.):
    planner.core.effective_t_follow = None
    for _ in range(30):
      value = planner.core.get_dynamic_t_follow(.7, lead, 10., blend)
    results.append(value)
  assert results[0] > results[1] > results[2]
  assert results[1] - .7 == pytest.approx((results[0] - .7) / 2.)


def approaching_lead(tmp_path, blend, speed=10., lead_speed=8., gap=16., lead_accel=-.3):
  planner, sm, toggles = planner_setup(tmp_path)
  planner.core.v_desired_filter.x = speed
  sm['carState'].vEgo = sm['carState'].vEgoCluster = speed
  plan = sm['frogpilotPlan']
  plan.tFollow, plan.trafficModeBlend = .7, blend
  plan.accelerationJerk, plan.speedJerk = 100., 2.5
  sm['frogpilotCarState'].trafficMode = blend > 0.
  model = sm['modelV2']
  model.position.x = (np.asarray(ModelConstants.T_IDXS) * speed).tolist()
  model.velocity.x = [speed] * 33
  model.action.desiredAcceleration = .4
  model.action.shouldStop = False
  model.meta.disengagePredictions.gasPressProbs = [1.] * 6
  for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
    lead.status = lead.radar = True
    lead.dRel, lead.vLead, lead.vLeadK, lead.aLeadK = gap, lead_speed, lead_speed, lead_accel
    lead.modelProb, lead.aLeadTau = 1., 1.5
  return planner, sm, toggles


@pytest.mark.parametrize('blend', [0., .5, 1.])
def test_solver_uses_selected_traffic_headway(tmp_path, blend):
  planner, sm, toggles = approaching_lead(tmp_path, blend)
  for _ in range(35):
    planner.update(False, sm, toggles)
  assert planner.mpc.solution_status == 0
  # At full traffic the actual optimization receives the profile's headway,
  # including when an approaching lead would add spacing in normal mode.
  if blend == 1.:
    np.testing.assert_allclose(planner.mpc.params[:, 4], .7, rtol=0., atol=1e-7)
  else:
    assert planner.mpc.params[0, 4] > .7 + .02
  assert np.isfinite(planner.output_a_target)


def test_traffic_avoids_extra_braking_at_same_lead_gap(tmp_path):
  targets = []
  for blend in (0., 1.):
    planner, sm, toggles = approaching_lead(tmp_path, blend, speed=12., lead_speed=11., gap=20., lead_accel=0.)
    for _ in range(60):
      planner.update(False, sm, toggles)
    assert planner.mpc.solution_status == 0
    targets.append(planner.output_a_target)
  # Both respond to the closing lead. Traffic does not brake as strongly
  # just to create the separate normal-mode approach allowance.
  assert targets[0] < targets[1] < 0.
  assert targets[1] - targets[0] > .01
