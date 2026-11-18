"""Exercise learned pickup through the Camry planner's normal update and publish path."""
import numpy as np
import pytest

from cereal import log
from openpilot.selfdrive.frogpilot.tests.test_starpilot_model_actions import setup_case, step
from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import Publisher
from openpilot.selfdrive.modeld.constants import ModelConstants


def pickup_case(tmp_path, *, speed=20., model='toad-pop-v2', learning=True):
  planner, sm, toggles = setup_case(tmp_path, model=model, speed=speed)
  toggles.toadpilot_modern_longitudinal = True
  toggles.toadpilot_accel_boost = learning
  sm['carControl'].longActive = True
  sm['controlsState'].personality = log.LongitudinalPersonality.aggressive
  plan = sm['frogpilotPlan']
  plan.minAcceleration, plan.maxAcceleration = -2.4, 2.
  plan.tFollow = 1.5
  sm['modelV2'].action.desiredAcceleration = .4
  return planner, sm, toggles


def set_lead(lead, speed, *, distance=48., accel=0.):
  lead.status = lead.radar = True
  lead.dRel = distance
  lead.vLead = lead.vLeadK = speed
  lead.aLeadK = accel
  lead.modelProb, lead.aLeadTau = 1., 1.5


@pytest.mark.parametrize('model', ['toad-pop-v2', 'toad-rdf6'])
@pytest.mark.parametrize('opening_lead', [False, True])
def test_repeated_driver_requests_raise_published_pickup(tmp_path, model, opening_lead):
  p, sm, toggles = pickup_case(tmp_path, model=model)
  if opening_lead:
    set_lead(sm['radarState'].leadOne, 26.)
  out, _ = step(p, sm, toggles, count=50)
  assert out.aTarget == pytest.approx(.4, abs=.001)
  for press in range(1, 6):
    sm['carControl'].longActive = False
    sm['carState'].gasPressed = True
    sm['controlsState'].longControlState = 'off'
    _, held = step(p, sm, toggles, count=48)
    # Existing command slew can retain the previous frame's target briefly.
    assert held[-1] <= .4001
    assert p.pickup_learner.total_boost == pytest.approx(min(.05 * press, .2))
    sm['carControl'].longActive = True
    sm['carState'].gasPressed = False
    sm['controlsState'].longControlState = 'pid'
    out, _ = step(p, sm, toggles, count=45)
    assert out.aTarget == pytest.approx(.4 + min(.05 * press, .2), abs=.001)
    assert sm['modelV2'].action.desiredAcceleration == pytest.approx(.4)


@pytest.mark.parametrize('scene', ['low_speed_pickup', 'low_speed_stop', 'standstill_stop', 'brake_request', 'coast_request',
                                    'model_stop', 'red_light', 'approach', 'closing_primary', 'closing_secondary',
                                    'decelerating_lead', 'brake_pressed', 'force_coast', 'paused', 'no_authority',
                                    'stale_radar', 'invalid_action', 'gas_pressed'])
def test_ineligible_scene_matches_original_plan_with_full_learned_boost(tmp_path, scene):
  speed = {'low_speed_pickup': 4., 'low_speed_stop': 3., 'standstill_stop': 0.}.get(scene, 20.)
  traces = []
  for learning in (False, True):
    p, sm, toggles = pickup_case(tmp_path / str(learning), speed=speed, learning=learning)
    p.update(False, sm, toggles)
    if learning:
      p.pickup_learner.total_boost = .2
    model = sm['modelV2'].action
    if scene in ('low_speed_stop', 'standstill_stop'):
      model.desiredAcceleration = -.8
      model.shouldStop = True
    elif scene == 'brake_request':
      model.desiredAcceleration = -.2
    elif scene == 'coast_request':
      model.desiredAcceleration = 0.
    elif scene == 'model_stop':
      model.shouldStop = True
    elif scene == 'red_light':
      sm['frogpilotPlan'].redLight = True
    elif scene == 'approach':
      sm['frogpilotPlan'].approachStopLength = 80.
    elif scene in ('closing_primary', 'closing_secondary'):
      set_lead(sm['radarState'].leadOne, 26.)
      lead = sm['radarState'].leadTwo if scene == 'closing_secondary' else sm['radarState'].leadOne
      set_lead(lead, 15., distance=30., accel=-1.5)
    elif scene == 'decelerating_lead':
      set_lead(sm['radarState'].leadOne, 26., accel=-2.)
    elif scene == 'brake_pressed':
      sm['carState'].brakePressed = True
    elif scene == 'force_coast':
      sm['frogpilotCarState'].forceCoast = True
    elif scene == 'paused':
      sm['frogpilotCarState'].pauseLongitudinal = True
    elif scene == 'gas_pressed':
      sm['carState'].gasPressed = True
    elif scene == 'no_authority':
      sm['carControl'].longActive = False
    elif scene == 'stale_radar':
      sm.valid['radarState'] = False
    elif scene == 'invalid_action':
      model.longitudinalValid = False

    pm, samples = Publisher(), []
    for _ in range(30):
      p.update(False, sm, toggles)
      p.publish(False, False, sm, pm, toggles)
      output = pm.message.longitudinalPlan
      samples.append((output.aTarget, output.shouldStop, pm.message.valid))
    traces.append(samples)
  assert traces[0] == traces[1]
  accel, stop, valid = traces[1][-1]
  if scene in ('low_speed_stop', 'standstill_stop'):
    assert accel < 0. and stop and valid
  elif scene in ('stale_radar', 'invalid_action'):
    assert not valid


def test_lost_eligibility_disarms_pedal_until_next_press(tmp_path):
  p, sm, toggles = pickup_case(tmp_path)
  step(p, sm, toggles)
  sm['carState'].gasPressed = True
  step(p, sm, toggles, count=5)
  earned = p.pickup_learner.total_boost
  assert 0. < earned < .05
  sm['frogpilotPlan'].redLight = True
  step(p, sm, toggles, count=1)
  sm['frogpilotPlan'].redLight = False
  step(p, sm, toggles, count=20)
  assert p.pickup_learner.total_boost == earned
  sm['carState'].gasPressed = False
  step(p, sm, toggles, count=2)
  sm['carState'].gasPressed = True
  step(p, sm, toggles, count=5)
  assert p.pickup_learner.total_boost > earned


@pytest.mark.parametrize('reset', ['setting', 'mode', 'disengage'])
def test_reset_clears_learned_pickup(tmp_path, reset):
  p, sm, toggles = pickup_case(tmp_path)
  step(p, sm, toggles)
  p.pickup_learner.total_boost = .2
  if reset == 'setting':
    toggles.toadpilot_accel_boost = False
  elif reset == 'mode':
    sm['controlsState'].experimentalMode = False
  else:
    sm['controlsState'].enabled = False
  p.update(False, sm, toggles)
  assert p.pickup_learner is None
  toggles.toadpilot_accel_boost = sm['controlsState'].experimentalMode = sm['controlsState'].enabled = True
  out, _ = step(p, sm, toggles)
  assert out.aTarget == pytest.approx(.4, abs=.001)


@pytest.mark.parametrize(('mpc', 'cruise', 'profile', 'expected'),
                         [(.45, 30., 2., .45), (1., 20.046, 2., .46), (1., 30., .48, .48)])
def test_pickup_respects_each_existing_upper_limit(tmp_path, mpc, cruise, profile, expected):
  p, sm, toggles = pickup_case(tmp_path)
  step(p, sm, toggles)
  p.pickup_learner.total_boost = .2
  target = p.get_learned_pickup(p.state, .4, mpc, False, cruise, profile, .1)
  assert target == pytest.approx(expected)


def test_closed_loop_stop_from_eight_mph_with_learned_pickup(tmp_path):
  traces = []
  for learning in (False, True):
    speed = 8. * .44704
    p, sm, toggles = pickup_case(tmp_path / str(learning), speed=speed, learning=learning)
    p.update(False, sm, toggles)
    if learning:
      p.pickup_learner.total_boost = .2
    pm, samples = Publisher(), []
    for _ in range(150):
      car, model = sm['carState'], sm['modelV2']
      car.vEgo = car.vEgoCluster = speed
      car.standstill = speed < .01
      model.position.x = (np.asarray(ModelConstants.T_IDXS) * speed).tolist()
      model.velocity.x = [speed] * 33
      model.action.desiredAcceleration = -.8
      model.action.shouldStop = speed < .6
      p.update(False, sm, toggles)
      p.publish(False, False, sm, pm, toggles)
      plan = pm.message.longitudinalPlan
      assert pm.message.valid and p.mpc.solution_status == 0
      samples.append((plan.aTarget, plan.shouldStop))
      car.aEgo = plan.aTarget
      speed = max(0., speed + plan.aTarget * .05)
    assert speed == 0. and samples[-1][1]
    traces.append(samples)
  assert traces[0] == traces[1]


def test_gentle_pickup_caps_a_learned_request(tmp_path):
  p, sm, toggles = pickup_case(tmp_path)
  p.update(False, sm, toggles)
  p.pickup_learner.total_boost = .2
  toggles.toadpilot_pickup_strength = 'gentle'
  plan, _ = step(p, sm, toggles, count=50)
  assert plan.aTarget == pytest.approx(.5, abs=.001)
  assert p.pickup_learner.total_boost == pytest.approx(.1)
