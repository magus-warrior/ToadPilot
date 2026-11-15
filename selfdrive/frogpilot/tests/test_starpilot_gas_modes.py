"""Keep ordinary FrogPilot gas control separate from experimental StarPilot gas control."""
import numpy as np
import pytest

from openpilot.selfdrive.frogpilot.tests.test_starpilot_model_actions import setup_case
from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import Publisher
from openpilot.selfdrive.toad_modeld.starpilot_long.lead_behavior import should_disable_far_lead_throttle
from openpilot.selfdrive.toad_modeld.starpilot_long.mpc import desired_follow_distance
from openpilot.selfdrive.toad_modeld.starpilot_long.vehicle_tunes import get_far_lead_coast_headway_floor


@pytest.mark.parametrize('speed', [2.5, 3.75, 5., 25.])
def test_acc_ignores_model_gas_predictions(tmp_path, speed):
  targets = []
  for probability in [.1, .39, .41, 1.]:
    planner, sm, toggles = setup_case(tmp_path / str(probability), speed=speed, experimental=False)
    sm['carControl'].orientationNED = [0., 0., 0.]
    sm['modelV2'].action.desiredAcceleration = .4
    sm['modelV2'].meta.disengagePredictions.gasPressProbs = [probability] * 6
    publisher = Publisher()
    samples = []
    for _ in range(10):
      planner.update(False, sm, toggles)
      planner.publish(False, False, sm, publisher, toggles)
      assert publisher.message.valid and planner.mpc.solution_status == 0
      assert publisher.message.longitudinalPlan.allowThrottle
      samples.append(publisher.message.longitudinalPlan.aTarget)
    targets.append(samples)
  np.testing.assert_allclose(targets, np.tile(targets[-1], (len(targets), 1)), atol=1e-6, rtol=0.)


def test_leaving_experimental_mode_clears_its_throttle_hysteresis(tmp_path):
  planner, sm, toggles = setup_case(tmp_path, experimental=True)
  model = sm['modelV2']
  model.meta.disengagePredictions.gasPressProbs = [.1] * 6
  for _ in range(6):
    planner.update(False, sm, toggles)
  assert not planner.allow_throttle

  # Leaving experimental must release its gas veto even if the prediction
  # remains low, rather than waiting for StarPilot's hysteresis to reopen it.
  planner.update(False, sm, toggles)
  assert not planner.allow_throttle
  sm['controlsState'].experimentalMode = False
  planner.update(False, sm, toggles)
  assert planner.allow_throttle
  assert planner.model_allow_throttle_transition_t == 0.


def test_reentering_experimental_mode_reenables_model_gas_control(tmp_path):
  planner, sm, toggles = setup_case(tmp_path, experimental=False)
  sm['carControl'].orientationNED = [0., 0., 0.]
  sm['modelV2'].meta.disengagePredictions.gasPressProbs = [.1] * 6
  for _ in range(6):
    planner.update(False, sm, toggles)
  assert planner.allow_throttle

  sm['controlsState'].experimentalMode = True
  for _ in range(6):
    planner.update(False, sm, toggles)
  assert not planner.allow_throttle
  assert planner.output_a_target <= -.3 + 1e-6


@pytest.mark.parametrize('experimental', [False, True])
def test_extra_starpilot_lead_coast_veto_requires_experimental_mode(tmp_path, experimental):
  speed = 25.
  planner, sm, toggles = setup_case(tmp_path, speed=speed, experimental=experimental)
  sm['frogpilotPlan'].tFollow = .5
  lead = sm['radarState'].leadOne
  lead.status, lead.radar, lead.modelProb = True, True, 1.
  lead.vLead = lead.vLeadK = speed - 1.
  gap = desired_follow_distance(speed, lead.vLead, .5)
  lead.dRel = gap + 8.
  planner.tracking_filter.x = 1.
  assert should_disable_far_lead_throttle(
    speed, lead.dRel, gap, 1., False, get_far_lead_coast_headway_floor(planner.core.CP))
  planner.update(False, sm, toggles)
  assert planner.state['starpilotPlan'].disableThrottle == experimental
  assert planner.allow_throttle == (not experimental)
  assert planner.mpc.solution_status == 0


@pytest.mark.parametrize('experimental', [False, True])
def test_low_gas_prediction_does_not_weaken_braking_lead_response(tmp_path, experimental):
  planner, sm, toggles = setup_case(tmp_path, speed=25., experimental=experimental)
  sm['carControl'].orientationNED = [0., 0., 0.]
  sm['modelV2'].meta.disengagePredictions.gasPressProbs = [.1] * 6
  lead = sm['radarState'].leadOne
  lead.status, lead.radar, lead.modelProb = True, True, 1.
  lead.dRel, lead.vLead, lead.vLeadK, lead.aLeadK = 25., 18., 18., -2.5
  publisher = Publisher()
  targets = []
  for _ in range(30):
    planner.update(False, sm, toggles)
    planner.publish(False, False, sm, publisher, toggles)
    assert publisher.message.valid and planner.mpc.solution_status == 0
    targets.append(publisher.message.longitudinalPlan.aTarget)
  assert np.isfinite(targets).all()
  assert min(targets) < -1.
