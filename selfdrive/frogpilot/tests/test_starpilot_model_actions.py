"""Verify registered model actions and turn-speed braking through the real adapter."""
import numpy as np
import pytest

from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import planner_setup, Publisher
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.toad_modeld.artifact import MODELS
from openpilot.selfdrive.toad_modeld.starpilot_long.adapter import StarPilotPlanner


def setup_case(tmp_path, *, model='toad-south-carolina', speed=25., experimental=True):
  fixture, sm, toggles = planner_setup(tmp_path)
  toggles.model, toggles.model_version = model, MODELS[model]['version']
  # The legacy selector uses this flag only for v7. Modern registered models
  # must be recognized without changing the global model/process selector.
  toggles.tinygrad_model = False
  toggles.classic_model = False
  sm['controlsState'].experimentalMode = experimental
  sm['carState'].vEgo = sm['carState'].vEgoCluster = speed
  sm['carState'].standstill = speed < .01
  sm['frogpilotPlan'].minAcceleration = -.6
  sm['frogpilotPlan'].maxAcceleration = .8
  m = sm['modelV2']
  m.velocity.x = [speed] * 33
  m.position.x = (np.asarray(ModelConstants.T_IDXS) * speed).tolist()
  m.meta.disengagePredictions.gasPressProbs = [1.] * 6
  return StarPilotPlanner(fixture.core.CP.original, init_v=speed), sm, toggles


def step(p, sm, toggles, count=50):
  publisher = Publisher()
  samples = []
  for _ in range(count):
    p.update(False, sm, toggles)
    p.publish(False, False, sm, publisher, toggles)
    assert publisher.message.valid
    assert p.mpc.solution_status == 0
    samples.append(publisher.message.longitudinalPlan.aTarget)
  return publisher.message.longitudinalPlan, np.asarray(samples)


@pytest.mark.parametrize('model', MODELS)
def test_registered_model_braking_reaches_output_with_legacy_flag_false(tmp_path, model):
  p, sm, toggles = setup_case(tmp_path, model=model)
  sm['modelV2'].action.desiredAcceleration = -2.8
  output, _ = step(p, sm, toggles)
  assert output.aTarget == pytest.approx(-2.8, abs=.01)
  assert not toggles.tinygrad_model  # Adapter must not mutate shared settings.


@pytest.mark.parametrize('speed', [.2, 25.])
def test_model_stop_reaches_published_plan(tmp_path, speed):
  p, sm, toggles = setup_case(tmp_path, speed=speed)
  sm['modelV2'].action.desiredAcceleration = -1.2
  sm['modelV2'].action.shouldStop = True
  output, _ = step(p, sm, toggles)
  assert output.shouldStop
  assert output.aTarget <= -1.19


def test_acc_mode_keeps_model_action_out_of_cruise_control(tmp_path):
  p, sm, toggles = setup_case(tmp_path, experimental=False)
  sm['modelV2'].action.desiredAcceleration = -2.8
  sm['modelV2'].action.shouldStop = True
  output, _ = step(p, sm, toggles)
  assert output.aTarget > -.6
  assert not output.shouldStop


@pytest.mark.parametrize('experimental', [False, True])
def test_turn_speed_drop_keeps_selected_profile_without_stop_or_braking_lead(tmp_path, experimental):
  p, sm, toggles = setup_case(tmp_path, experimental=experimental)
  sm['frogpilotPlan'].vCruise = 12.
  sm['frogpilotPlan'].vtscSpeed = 12.
  sm['frogpilotPlan'].roadCurvature = .01
  sm['carState'].steeringAngleDeg = 12.
  sm['modelV2'].action.desiredAcceleration = .4
  _, targets = step(p, sm, toggles, count=100)
  assert targets.min() >= -.60001
  assert targets[-1] < -.1


def test_invalid_model_action_is_not_published_as_valid(tmp_path):
  p, sm, toggles = setup_case(tmp_path)
  sm['modelV2'].action.longitudinalValid = False
  p.update(False, sm, toggles)
  publisher = Publisher()
  p.publish(False, False, sm, publisher, toggles)
  assert not publisher.message.valid


def test_model_driven_stop_closed_loop_keeps_positive_stopping_distance(tmp_path):
  p, sm, toggles = setup_case(tmp_path, speed=27.)
  speed, gap, previous_accel = 27., 160., 0.
  publisher = Publisher()
  for _ in range(360):
    sm['carState'].vEgo = sm['carState'].vEgoCluster = speed
    sm['carState'].aEgo = previous_accel
    sm['carState'].standstill = speed < .01
    model = sm['modelV2']
    model.velocity.x = [max(speed, .01)] * 33
    model.position.x = (np.asarray(ModelConstants.T_IDXS) * max(speed, .01)).tolist()
    model.action.desiredAcceleration = -2.5
    model.action.shouldStop = speed < .6
    p.update(False, sm, toggles)
    p.publish(False, False, sm, publisher, toggles)
    output = publisher.message.longitudinalPlan
    assert publisher.message.valid and p.mpc.solution_status == 0
    assert -3.51 <= output.aTarget <= .81
    next_speed = max(0., speed + output.aTarget * .05)
    gap -= (speed + next_speed) * .025
    assert gap > 0.
    speed, previous_accel = next_speed, output.aTarget
  assert speed < .01
  assert output.shouldStop
