"""Normal following uses measured leads and the selected driving profile."""
import numpy as np
import pytest
from types import SimpleNamespace as NS

from cereal import log

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import A_CHANGE_COST, DANGER_ZONE_COST, J_EGO_COST
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_acceleration import FrogPilotAcceleration
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_following import FrogPilotFollowing
from openpilot.selfdrive.frogpilot.tests.test_starpilot_braking_settings import settings
from openpilot.selfdrive.frogpilot.tests.test_starpilot_model_actions import setup_case
from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import Publisher
from openpilot.selfdrive.modeld.constants import ModelConstants


def follow_case(tmp_path, *, speed=25., radar=True, experimental=False):
  planner, sm, toggles = setup_case(tmp_path, speed=speed, experimental=experimental)
  sm['frogpilotPlan'].vCruise = speed + 5.
  lead = sm['radarState'].leadOne
  lead.status, lead.radar, lead.modelProb = True, radar, 1.
  lead.dRel = speed + 14.
  lead.vLead = lead.vLeadK = speed
  lead.aLeadK, lead.aLeadTau = 0., 1.5
  planner.tracking_filter.x = 1.
  sm['modelV2'].action.desiredAcceleration = .4
  return planner, sm, toggles


def predictions(model, speed, variant):
  lead_t = np.asarray(ModelConstants.LEAD_T_IDXS)
  lead_accel = -2. if variant == 'lead_prediction' else 0.
  lead_v = np.maximum(0., speed + lead_accel * lead_t)
  lead_x = speed + 14. + speed * lead_t + .5 * lead_accel * lead_t ** 2
  model.leadsV3 = [dict(prob=1., x=lead_x.tolist(), y=[0.] * len(lead_t),
                        v=lead_v.tolist(), a=[lead_accel] * len(lead_t))]
  if variant == 'uncertainty':
    model.meta.desirePrediction = [1. / 8.] * 8
    model.meta.disengagePredictions.brakePressProbs = [.9] * 6
  if variant == 'action':
    model.action.desiredAcceleration = -2.
    model.action.shouldStop = True
    model.meta.disengagePredictions.gasPressProbs = [.1] * 6
  if variant == 'short_path':
    model.position.x = [0.] * 33
    model.velocity.x = [0.] * 33
    model.acceleration.x = [-2.] * 33


def samples(planner, sm, toggles, count=50):
  publisher = Publisher()
  rows = []
  for _ in range(count):
    planner.update(False, sm, toggles)
    planner.publish(False, False, sm, publisher, toggles)
    output = publisher.message.longitudinalPlan
    assert publisher.message.valid and planner.mpc.solution_status == 0
    rows.append((output.aTarget, output.shouldStop, output.allowThrottle))
  return np.asarray(rows)


@pytest.mark.parametrize('radar', [True, False])
@pytest.mark.parametrize('speed', [8., 25.])
@pytest.mark.parametrize('variant', ['lead_prediction', 'uncertainty', 'action', 'short_path'])
def test_normal_follow_is_independent_of_model_driving_predictions(tmp_path, speed, radar, variant):
  runs = []
  for prediction in ('baseline', variant):
    planner, sm, toggles = follow_case(tmp_path / prediction, speed=speed, radar=radar)
    predictions(sm['modelV2'], speed, prediction)
    runs.append(samples(planner, sm, toggles))
  np.testing.assert_allclose(runs[0], runs[1], atol=1e-6, rtol=0.)


def test_experimental_follow_still_consumes_model_predictions(tmp_path):
  runs = []
  for variant in ('baseline', 'action'):
    planner, sm, toggles = follow_case(tmp_path / variant, experimental=True)
    predictions(sm['modelV2'], 25., variant)
    runs.append(samples(planner, sm, toggles))
  assert runs[1][-1, 0] < runs[0][-1, 0] - .5
  assert runs[1][-1, 1]  # Model stop retains authority in experimental mode.


def test_normal_follow_does_not_require_unused_model_action(tmp_path):
  planner, sm, toggles = follow_case(tmp_path)
  sm['modelV2'].action.longitudinalValid = False
  sm['modelV2'].action.desiredAcceleration = float('nan')
  samples(planner, sm, toggles)


def test_normal_lead_departure_is_independent_of_model_brake_request(tmp_path):
  runs = []
  for variant in ('baseline', 'action'):
    planner, sm, toggles = follow_case(tmp_path / variant, speed=0.)
    lead = sm['radarState'].leadOne
    lead.dRel, lead.vLead, lead.vLeadK, lead.aLeadK = 9., 1.2, 1.2, .3
    predictions(sm['modelV2'], 0., variant)
    runs.append(samples(planner, sm, toggles))
  np.testing.assert_allclose(runs[0], runs[1], atol=1e-6, rtol=0.)
  assert runs[0][-1, 0] > 0.
  assert not runs[0][-1, 1]


def test_leaving_experimental_clears_model_uncertainty_and_launch(tmp_path):
  planner, sm, toggles = follow_case(tmp_path, speed=0., experimental=True)
  predictions(sm['modelV2'], 0., 'uncertainty')
  sm['modelV2'].action.shouldStop = True
  samples(planner, sm, toggles, count=10)
  assert planner.uncert_slow.x > 0.
  assert planner.model_launch_armed and planner.model_launch_stop_seen
  sm['controlsState'].experimentalMode = False
  samples(planner, sm, toggles, count=1)
  assert planner.uncert_slow.x == planner.uncert_fast.x == 0.
  assert not planner.model_launch_armed and not planner.model_launch_stop_seen


@pytest.mark.parametrize('slowing', [False, True])
def test_sport_following_uses_selected_acceleration_and_braking_intensity(tmp_path, slowing):
  runs = []
  for intensity in (.25, 2.):
    planner, sm, toggles = follow_case(tmp_path / str(intensity))
    settings(toggles, 2, intensity)
    sm['controlsState'].personality = log.LongitudinalPersonality.aggressive
    lead = sm['radarState'].leadOne
    lead.dRel = 31.
    parent = NS(tracking_lead=True, lead_one=lead, v_cruise=30.)
    following = parent.frogpilot_following = FrogPilotFollowing(parent)
    acceleration = FrogPilotAcceleration(parent)
    rows = []
    for frame in range(140):
      lead.vLead = lead.vLeadK = 25. if frame < 100 else (24. if slowing else 27.)
      measured_accel = -.01 if slowing else .01
      following.update(measured_accel, sm['controlsState'], sm['frogpilotCarState'], lead.dRel, 25., toggles)
      acceleration.update(sm['frogpilotCarState'], 25., toggles)
      plan = sm['frogpilotPlan']
      plan.tFollow = following.t_follow
      plan.minAcceleration, plan.maxAcceleration = acceleration.min_accel, acceleration.max_accel
      plan.accelerationJerk = A_CHANGE_COST * following.acceleration_jerk
      plan.dangerJerk = DANGER_ZONE_COST * following.danger_jerk
      plan.speedJerk = J_EGO_COST * following.speed_jerk
      planner.update(False, sm, toggles)
      assert planner.mpc.solution_status == 0
      rows.append(planner.output_a_target)
    runs.append(np.asarray(rows[100:120]))
  direction = -1. if slowing else 1.
  assert direction * runs[0].mean() > direction * runs[1].mean() + .02


@pytest.mark.parametrize('other_speed', [None, 0., 35.])
def test_second_lead_cannot_change_first_leads_measured_speed(tmp_path, other_speed):
  planner, sm, _ = follow_case(tmp_path)
  mpc = planner.mpc
  mpc.set_weights(v_ego=25.)
  mpc.set_cur_state(25., 0.)
  other = sm['radarState'].leadTwo
  other.status = other_speed is not None
  other.dRel, other.vLead, other.aLeadK, other.aLeadTau = 60., other_speed or 0., 0., 1.5
  for _ in range(50):
    first_path = mpc.process_lead(sm['radarState'].leadOne, lead_index=0)
    mpc.process_lead(other, lead_index=1)
  np.testing.assert_allclose(first_path[:, 1], 25., atol=1e-6, rtol=0.)
