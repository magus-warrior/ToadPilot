"""Exercise the isolated native solver through the real legacy message adapter."""
import math
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.common.params import Params
from openpilot.selfdrive.car import gen_empty_fingerprint
from openpilot.selfdrive.car.toyota.interface import CarInterface
from openpilot.selfdrive.car.toyota.values import CAR
from openpilot.selfdrive.frogpilot.tests.test_new_upstream_features import planner_fixture
from openpilot.selfdrive.toad_modeld.starpilot_long.adapter import StarPilotPlanner
from openpilot.selfdrive.toad_modeld.starpilot_long import planner as source
from openpilot.selfdrive.toad_modeld.starpilot_long.mpc import N


class Publisher:
  def send(self, service, message):
    self.message = message


def planner_setup(tmp_path):
  cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS2, gen_empty_fingerprint(), [], False, False, Params(str(tmp_path)), False)
  _, sm, toggles = planner_fixture()
  sm['controlsState'].experimentalMode = False
  sm['frogpilotPlan'].tFollow = 1.
  sm['frogpilotPlan'].dangerJerk = 100.
  sm['frogpilotPlan'].accelerationJerk = 100.
  sm['frogpilotPlan'].speedJerk = 5.
  sm['carState'].vEgo = 0.
  return StarPilotPlanner(cp), sm, toggles


@pytest.mark.parametrize('scenario', ['cruise', 'opening_lead', 'braking_lead', 'cut_in', 'lead_departure', 'forced_stop'])
def test_closed_loop_legacy_adapter_and_real_solver(tmp_path, monkeypatch, scenario):
  p, sm, toggles = planner_setup(tmp_path)
  clock = [100.]
  monkeypatch.setattr(source, 'time', NS(monotonic=lambda: clock[0]))
  ego = 0. if scenario == 'lead_departure' else 27.
  lead_speed = ego
  gap = 7. if scenario == 'lead_departure' else 45.
  if scenario == 'forced_stop':
    # At 27 m/s, 45 metres is physically insufficient at the -3.5 m/s² limit.
    gap = 160.
  minimum_gap = gap
  publisher = Publisher()
  previous_accel = 0.
  for frame in range(360):
    clock[0] += .05
    t = frame * .05
    cs = sm['carState']
    cs.vEgo, cs.aEgo = ego, previous_accel
    cs.standstill = ego < .01
    model = sm['modelV2']
    model.velocity.x = [max(ego, 0.01)] * 33
    model.position.x = (np.asarray(model.velocity.t or np.linspace(0., 10., 33)) * max(ego, .01)).tolist()
    # Publish a physically coherent model trajectory for a permitted launch.
    if scenario == 'lead_departure':
      from openpilot.selfdrive.modeld.constants import ModelConstants
      model.velocity.x = (ego + np.asarray(ModelConstants.T_IDXS) * .4).tolist()
      model.position.x = (ego * np.asarray(ModelConstants.T_IDXS) + .2 * np.asarray(ModelConstants.T_IDXS) ** 2).tolist()
    model.action.desiredAcceleration = .4
    model.action.shouldStop = False
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    sm['controlsState'].vCruise = 108.
    sm['controlsState'].longControlState = 'pid'
    lead = sm['radarState'].leadOne
    lead.status = scenario in ('opening_lead', 'braking_lead', 'cut_in', 'lead_departure')
    lead.radar = True
    lead.modelProb = 1.
    if scenario == 'opening_lead':
      lead_speed = 29.
    elif scenario == 'braking_lead':
      lead_speed = max(0., lead_speed - (2.5 * .05 if t >= 2. else 0.))
    elif scenario == 'cut_in' and frame == 40:
      gap, lead_speed = 20., 24.
    elif scenario == 'lead_departure':
      lead_speed = min(6., max(0., t - 1.) * .7)
    lead.dRel, lead.vLead, lead.vLeadK = gap, lead_speed, lead_speed
    lead.aLeadK = -2.5 if scenario == 'braking_lead' and t >= 2. and lead_speed > 0. else 0.
    lead.aLeadTau = 1.5
    if scenario == 'forced_stop':
      sm['frogpilotPlan'].forcingStop = True
      sm['frogpilotPlan'].forcingStopLength = max(gap, 0.)
      sm['frogpilotPlan'].vCruise = 0.
    p.update(False, sm, toggles)
    p.publish(False, False, sm, publisher, toggles)
    plan = publisher.message.longitudinalPlan
    assert publisher.message.valid
    assert math.isfinite(plan.aTarget) and -3.51 <= plan.aTarget <= 2.01
    assert np.isfinite(p.mpc.x_sol).all()
    assert p.mpc.solution_status == 0
    assert len(p.mpc.solver.get(N, 'x')) == 3
    accel = plan.aTarget
    next_ego = max(0., ego + accel * .05)
    gap += ((lead_speed if scenario != 'forced_stop' else 0.) - (ego + next_ego) / 2.) * .05
    if lead.status or scenario == 'forced_stop':
      minimum_gap = min(minimum_gap, gap)
      assert gap > 0., (scenario, frame, ego, gap, accel)
    ego, previous_accel = next_ego, accel
  if scenario in ('braking_lead', 'forced_stop'):
    assert ego < 1.
  if scenario == 'lead_departure':
    assert ego > 1.


def test_new_solver_keeps_terminal_safety_constraints(tmp_path):
  p, _, _ = planner_setup(tmp_path)
  # A terminal four-slack vector distinguishes this build from the legacy solver.
  assert len(p.mpc.solver.get(N, 'sl')) == 4
