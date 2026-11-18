"""Verify registered model actions and turn-speed braking through the real adapter."""
import numpy as np
import pytest

from openpilot.selfdrive.controls import radard
from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import planner_setup, Publisher
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.toad_modeld.artifact import MODELS
from openpilot.selfdrive.toad_modeld.starpilot_long.adapter import StarPilotPlanner
from openpilot.selfdrive.toad_modeld.starpilot_long.mpc import T_IDXS


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


@pytest.mark.parametrize('stopped_car_ahead', [False, True])
def test_lead_prediction_tracks_the_vehicle_selected_by_radar(tmp_path, stopped_car_ahead):
  ego = .8 if stopped_car_ahead else 2.5
  planner, sm, toggles = setup_case(tmp_path, model='toad-pop-v2', speed=ego)
  sm['carControl'].longActive = True
  toggles.lead_detection_probability = .17
  toggles.allow_far_lead_tracking = False
  toggles.increased_stopped_distance = 0.

  model = sm['modelV2']
  model.action.desiredAcceleration = .4
  moving_distance = 40. if stopped_car_ahead else 18.
  camera_accel = 0. if stopped_car_ahead else .5
  horizon = np.asarray(ModelConstants.LEAD_T_IDXS)
  model_lead = model.init('leadsV3', 2)[0]
  model_lead.prob = .95
  model_lead.x = (moving_distance + radard.RADAR_TO_CAMERA + 6. * horizon + .5 * camera_accel * horizon ** 2).tolist()
  model_lead.y, model_lead.v, model_lead.a = [0.] * 6, (6. + camera_accel * horizon).tolist(), [camera_accel] * 6
  model_lead.xStd, model_lead.yStd, model_lead.vStd = [1.] * 6, [1.] * 6, [1.] * 6

  kalman = radard.KalmanParams(.05)
  moving = radard.Track(11, 6., kalman)
  moving.update(moving_distance, 0., 6. - ego, 6., True)
  tracks = {11: moving}
  if stopped_car_ahead:
    stopped = radard.Track(22, 0., kalman)
    stopped.update(8., 0., -ego, 0., True)
    tracks[22] = stopped
  fused = radard.get_lead(ego, True, tracks, model_lead, ego, model, False, toggles, 0.)
  sm['radarState'].mdMonoTime = sm.logMonoTime['modelV2'] = 10_000_000_000
  sm['radarState'].leadOne = fused
  plan, _ = step(planner, sm, toggles, count=55)
  position_at_two_seconds = np.interp(2., T_IDXS, planner.mpc.lead_xv_0[:, 0])
  if stopped_car_ahead:
    assert fused['radarTrackId'] == 22 and fused['modelProb'] == 0.
    assert position_at_two_seconds == pytest.approx(8.)
    assert plan.aTarget < 0.
  else:
    assert fused['radarTrackId'] == 11 and fused['modelProb'] > .5
    # Radar measures a steady 6 m/s. The matched camera forecast accelerates.
    assert planner.mpc.lead_xv_0[-1, 0] == pytest.approx(moving_distance + 60. + 25.)
    assert planner.mpc.lead_xv_0[-1, 1] == pytest.approx(11.)
    assert plan.aTarget > 0.


def test_lead_frame_switch_and_measured_mode(tmp_path):
  planner, sm, toggles = setup_case(tmp_path, model='toad-pop-v2', speed=.8)
  sm['carControl'].longActive = True
  radar, model = sm['radarState'], sm['modelV2']
  model.action.desiredAcceleration = .4
  lead = radar.leadOne
  lead.status = lead.radar = True
  lead.modelProb, lead.dRel, lead.vLead, lead.aLeadTau = .95, 8., 0., 1.5
  sm.logMonoTime = {'modelV2': 10_000_000_000}
  radar.mdMonoTime = sm.logMonoTime['modelV2']
  camera = model.init('leadsV3', 2)[0]
  camera.prob, camera.x, camera.v = .95, [8.] * 6, [0.] * 6
  step(planner, sm, toggles, count=55)
  sm.logMonoTime['modelV2'] += 50_000_000
  times = np.asarray(ModelConstants.LEAD_T_IDXS)
  camera.x, camera.v = (40. + 6. * times).tolist(), [6.] * 6
  plan, _ = step(planner, sm, toggles, count=1)
  assert np.allclose(planner.mpc.lead_xv_0[:, 0], 8.)
  assert plan.aTarget < -.05
  # The old snapshot is owned, even when the incoming message is reused.
  assert planner.core.matched_leads(planner.state, True)[0].x == (8.,) * 6
  radar.mdMonoTime = sm.logMonoTime['modelV2']
  lead.dRel = 40.
  lead.vLead = lead.vLeadK = 6.
  camera.x, camera.v = (40. + 6. * times + .25 * times**2).tolist(), (6. + .5 * times).tolist()
  radar.mdMonoTime = sm.logMonoTime['modelV2'] = sm.logMonoTime['modelV2'] + 50_000_000
  sm['modelV2'].position.x = (np.asarray(ModelConstants.T_IDXS) * 8.).tolist()
  planner.tracking_filter.x = 1.
  step(planner, sm, toggles, count=1)
  assert planner.mpc.lead_xv_0[-1, 1] == pytest.approx(11.)
  toggles.toadpilot_matched_lead_prediction = False
  step(planner, sm, toggles, count=1)
  assert planner.mpc.lead_xv_0[-1, 1] == pytest.approx(6.)
  toggles.toadpilot_matched_lead_prediction = True
  for offset in (-500_000_000, -1, 1):
    radar.mdMonoTime = sm.logMonoTime['modelV2'] + offset
    assert not planner.core.matched_leads(planner.state, True)


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
