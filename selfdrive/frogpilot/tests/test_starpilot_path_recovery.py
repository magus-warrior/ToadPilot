from dataclasses import replace
import math

import pytest

from openpilot.selfdrive.toad_modeld.path_recovery import Observation, PathRecovery

from types import SimpleNamespace as NS
from openpilot.selfdrive.toad_modeld.path_recovery import ModelPathRecovery, lane_geometry, recovery_enabled


def observation(t=0., **kwargs):
  return replace(Observation(now=t, model_time=t, frame=round(t*20), speed=4.,
                            curvature=0., lane_offset=.33, path_bias=-.29,
                            lane_width=3.3, confidence=.95), **kwargs)


def run(seconds=8, **kwargs):
  recovery = PathRecovery()
  outputs = [recovery.update(observation(i*.05, **kwargs))[0] for i in range(round(seconds*20))]
  return recovery, outputs


def test_persistent_left_bias_requests_right_and_right_bias_is_mirrored():
  _, left = run()
  _, right = run(lane_offset=-.33, path_bias=.29)
  assert left[-1] > 0
  assert right == pytest.approx([-v for v in left])
  assert all(v == 0 for v in left[:30])


@pytest.mark.parametrize('lane_offset,path_bias', [(-.05,.10),(.05,-.1),(.33,.2),(-.33,-.2)])
def test_good_centering_and_already_recovering_paths_receive_no_trim(lane_offset, path_bias):
  _, outputs = run(lane_offset=lane_offset, path_bias=path_bias)
  assert outputs == [0.] * len(outputs)


@pytest.mark.parametrize('changes', [dict(override=True),dict(active=False),dict(lane_change=True),
  dict(blinker=True),dict(geometry_valid=False),dict(confidence=.89),dict(lane_width=5.),
  dict(lane_offset=.6),dict(path_bias=.6),dict(speed=.4),dict(speed=36.),dict(curvature=.02),
  dict(lane_offset=math.nan),dict(path_bias=math.inf),dict(model_time=0.)])
def test_releases_immediately_when_an_input_is_unsafe(changes):
  recovery, outputs = run()
  assert outputs[-1] > 0
  trim, reason = recovery.update(observation(8., **changes))
  assert trim == 0 and reason == 'gated'


def test_duplicate_frames_do_not_confirm_persistence():
  recovery = PathRecovery()
  for i in range(200):
    # Advancing receipt time does not turn one model frame into new evidence.
    trim, _ = recovery.update(observation(i*.001, frame=1, model_time=0.))
    assert trim == 0


def test_direction_reversal_requires_fresh_confirmation():
  recovery, outputs = run()
  assert outputs[-1] > 0
  trim, reason = recovery.update(observation(8.,lane_offset=-.33,path_bias=.29))
  assert trim == 0 and reason == 'confirming'


@pytest.mark.parametrize('speed', [.5, 1., 4., 10., 20., 35.])
def test_correction_limits_and_slew_are_bounded_at_all_supported_speeds(speed):
  _, outputs = run(speed=speed)
  assert max(abs(v) for v in outputs) <= PathRecovery.MAX_CURVATURE+1e-12
  assert max(abs(v)*speed**2 for v in outputs) <= PathRecovery.MAX_LAT_ACCEL+1e-12
  assert max(abs(b-a)/.05 for a,b in zip(outputs,outputs[1:])) <= PathRecovery.CURVATURE_RATE+1e-12
  assert max(abs(b-a)*speed**2/.05 for a,b in zip(outputs,outputs[1:])) <= PathRecovery.LAT_ACCEL_RATE+1e-12


def test_new_trim_moves_the_active_torque_controller_toward_right_steering():
  from openpilot.selfdrive.toad_modeld.starpilot_response import SteeringResponse
  base, trimmed = SteeringResponse(), SteeringResponse()
  recovery = PathRecovery()
  args=dict(active=True, speed=4., desired_curvature=0., measured_curvature=0.,
            roll=0., delay=.22, factor=2.37, offset=0., friction=.13, deadzone=0.,
            steer_max=1., limited=False, override=False)
  for i in range(800):
    trim,_=recovery.update(observation(i*.01, frame=i//5))
    base_output,_=base.update(**args)
    new_output,_=trimmed.update(**{**args,'desired_curvature':trim})
  assert new_output < base_output  # Negative Toyota torque steers right.


def test_time_gap_clears_old_bias_evidence():
  recovery, outputs = run()
  assert outputs[-1] > 0
  assert recovery.update(observation(9.))[0] == 0
  assert recovery.update(observation(9.05))[0] == 0


def test_repeated_frame_with_advanced_time_cannot_confirm_bias():
  recovery = PathRecovery()
  for i in range(100):
    assert recovery.update(observation(i*.05,frame=1))[0] == 0


def test_model_and_controller_update_rates_give_the_same_ramp():
  outputs = []
  for dt in [.01, .05]:
    recovery = PathRecovery()
    for i in range(round(3./dt)+1):
      t=i*dt
      frame=int(math.floor((t+1e-8)/.05))
      trim,_=recovery.update(observation(t,frame=frame,model_time=min(t,frame*.05)))
    outputs.append(trim)
  assert outputs[0] > 0
  assert outputs[0] == pytest.approx(outputs[1],abs=2e-5)


def model(offset=.33, path_bias=-.29, slope=0.):
  # Lane origin includes the same 4 cm camera correction as the existing model
  # coordinate convention; sloped lanes verify the near-field fit.
  xs=[0.,2.5,5.,7.5,10.,12.,20.,30.,45.]
  center=offset-.04
  line=lambda shift:NS(x=xs,y=[center+slope*x+shift for x in xs])
  return NS(frameId=0,laneLines=[line(-5.),line(-1.65),line(1.65),line(5.)],
            laneLineProbs=[.5,.96,.96,.5],laneLineStds=[.1,.1,.1,.1],
            position=line(path_bias),meta=NS(laneChangeState='off',turnDirection='none'))


def test_model_geometry_fit_removes_lane_heading_and_preserves_offset_sign():
  for offset in [-.33,.33]:
    geometry=lane_geometry(model(offset=offset,path_bias=-offset,slope=.02),4.)
    assert geometry.valid
    assert geometry.lane_offset==pytest.approx(offset)
    assert geometry.path_bias==pytest.approx(-offset)


@pytest.mark.parametrize('kind',['low_probability','high_uncertainty','nan','short_path','nonmonotonic','missing'])
def test_geometry_rejects_unreliable_lane_data(kind):
  m=model()
  if kind=='low_probability':m.laneLineProbs[1]=.89
  elif kind=='high_uncertainty':m.laneLineStds[2]=.36
  elif kind=='nan':m.laneLines[1].y[0]=math.nan
  elif kind=='short_path':m.position=NS(x=[0.,1.],y=[0.,0.])
  elif kind=='nonmonotonic':m.laneLines[1].x=[0.]*9
  elif kind=='missing':m=NS()
  assert not lane_geometry(m,4.).valid


def enabled_inputs():
  cp=NS(carName='toyota',carFingerprint='TOYOTA_CAMRY_TSS2',lateralTuning=NS(which=lambda:'torque'))
  controller=NS(use_nnff=False,use_nnff_lite=False)
  toggles=NS(toadpilot_enabled=True,toadpilot_starpilot_response=True,
             car_model='TOYOTA_CAMRY_TSS2',model='toad-south-carolina',model_version='v11',
             toadpilot_profile='observe',toadpilot_lane_centering_assist=False)
  return cp,controller,toggles


def test_recovery_is_disabled_for_clean_model_baseline():
  assert not recovery_enabled(*enabled_inputs())


def test_explicit_enable_preserves_supported_loop_without_deprecated_options(monkeypatch):
  import openpilot.selfdrive.toad_modeld.path_recovery as recovery_module
  monkeypatch.setattr(recovery_module, 'PATH_RECOVERY_ENABLED', True)
  assert recovery_enabled(*enabled_inputs())


@pytest.mark.parametrize('kind',['other_car','pid','nnff','old_model','response_off'])
def test_new_recovery_is_isolated_to_the_active_supported_controller(kind, monkeypatch):
  import openpilot.selfdrive.toad_modeld.path_recovery as recovery_module
  monkeypatch.setattr(recovery_module, 'PATH_RECOVERY_ENABLED', True)
  cp,controller,toggles=enabled_inputs()
  if kind=='other_car':cp.carFingerprint='TOYOTA_CAMRY'
  elif kind=='pid':cp.lateralTuning.which=lambda:'pid'
  elif kind=='nnff':controller.use_nnff=True
  elif kind=='old_model':toggles.model='classic'
  elif kind=='response_off':toggles.toadpilot_starpilot_response=False
  assert not recovery_enabled(cp,controller,toggles)


def exercise_wrapper(**overrides):
  recovery=ModelPathRecovery()
  m=model()
  args=dict(speed=4.,curvature=0.,model_valid=True,enabled=True,active=True,
            override=False,blinker=False)
  args.update(overrides)
  for i in range(401):
    t=10.+i*.01
    m.frameId=i//5
    result=recovery.update(m,now=t,model_time=10.+(i//5)*.05,**args)
  return recovery,m,result,args


def test_wrapper_uses_real_model_geometry_and_yields_to_the_driver():
  recovery,m,result,args=exercise_wrapper()
  assert result>0 and recovery.status['path_recovery_active']
  result=recovery.update(m,now=14.01,model_time=14.,**{**args,'override':True})
  assert result==0 and not recovery.status['path_recovery_active']


@pytest.mark.parametrize('kind',['stale','invalid','turn','lane_change','blinker','limited','disabled'])
def test_wrapper_drops_trim_when_its_runtime_gates_change(kind):
  recovery,m,result,args=exercise_wrapper()
  assert result>0
  model_time=14.
  if kind=='stale':model_time=13.
  elif kind=='invalid':args['model_valid']=False
  elif kind=='turn':m.meta.turnDirection='right'
  elif kind=='lane_change':m.meta.laneChangeState='laneChangeStarting'
  elif kind=='blinker':args['blinker']=True
  elif kind=='limited':args['limited']=True
  elif kind=='disabled':args['enabled']=False
  assert recovery.update(m,now=14.01,model_time=model_time,**args)==0


def test_downstream_curvature_limiter_remains_authoritative():
  from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, MAX_LATERAL_JERK
  from openpilot.common.realtime import DT_CTRL
  recovery,m,_,args=exercise_wrapper()
  requested=recovery.update(m,now=14.01,model_time=14.,**args)
  applied,_=clip_curvature(30.,0.,requested,True,0.)
  assert applied <= MAX_LATERAL_JERK/30.**2*DT_CTRL


def test_geometry_accepts_the_actual_cereal_model_schema():
  from cereal import messaging
  sample=model()
  message=messaging.new_message('modelV2')
  m=message.modelV2
  m.frameId=sample.frameId
  m.laneLineProbs=sample.laneLineProbs
  m.laneLineStds=sample.laneLineStds
  m.init('laneLines',4)
  for i in range(4):
    m.laneLines[i].x=sample.laneLines[i].x
    m.laneLines[i].y=sample.laneLines[i].y
  m.position.x=sample.position.x
  m.position.y=sample.position.y
  assert lane_geometry(m.as_reader(),4.).lane_offset==pytest.approx(.33)


def test_each_lane_probability_must_be_finite_and_in_range():
  m=model()
  m.laneLineProbs[2]=1.01
  assert not lane_geometry(m,4.).valid


def test_controlsd_dedicated_car_state_socket_is_not_a_submaster_subscription():
  from cereal import messaging
  from openpilot.selfdrive.toad_modeld.path_recovery import recovery_inputs_valid
  sm=messaging.SubMaster(['modelV2','liveParameters'])
  for service in sm.data:
    sm.alive[service]=sm.valid[service]=sm.freq_ok[service]=True
  # Reproduce the integration error caught after restart, with real SubMaster.
  with pytest.raises(KeyError,match='carState'):
    sm.all_checks(['modelV2','carState','liveParameters'])
  assert recovery_inputs_valid(sm,NS(canValid=True))
  assert not recovery_inputs_valid(sm,NS(canValid=False))
  sm.alive['modelV2']=False
  assert not recovery_inputs_valid(sm,NS(canValid=True))


@pytest.mark.parametrize('speed', [.5, 4., 20., 35.])
def test_confirmed_recovery_survives_path_threshold_jitter_and_fades_within_limits(speed):
  recovery, outputs = run(speed=speed)
  previous = outputs[-1]
  assert previous > 0 and recovery.confirmed
  for i in range(40):
    t = 8. + i*.05
    bias = -.08 if i % 2 == 0 else -.12
    trim, _ = recovery.update(observation(t, speed=speed, path_bias=bias))
    assert recovery.confirmed
    assert trim > 0
    assert abs(trim-previous) <= min(recovery.CURVATURE_RATE,
                                   recovery.LAT_ACCEL_RATE/speed**2)*.05+1e-12
    previous = trim
  # A sustained weak bias must taper out, rather than hold stale torque or
  # produce a negative target after subtracting the original 10 cm dead band.
  for i in range(180):
    trim, state = recovery.update(observation(10.+i*.05, speed=speed, path_bias=-.08))
    assert 0 <= trim <= previous+1e-12
    assert previous-trim <= min(recovery.CURVATURE_RATE,
                               recovery.LAT_ACCEL_RATE/speed**2)*.05+1e-12
    previous = trim
  assert trim == 0 and state == 'waiting_for_path_bias'
  assert recovery.confirmed
  trim, state = recovery.update(observation(19., speed=speed, path_bias=-.16))
  assert trim > 0 and state == 'recovering'


@pytest.mark.parametrize('changes', [dict(path_bias=-.03), dict(path_bias=.08),
                                    dict(lane_offset=.10), dict(lane_offset=-.20, path_bias=.20)])
def test_hysteresis_never_keeps_old_trim_after_path_recovery_centering_or_reversal(changes):
  recovery, _ = run()
  trim, _ = recovery.update(observation(8., **changes))
  assert trim == 0 and not recovery.confirmed


def test_weak_path_bias_cannot_start_a_new_recovery():
  recovery, outputs = run(path_bias=-.08)
  assert outputs == [0.]*len(outputs)
  assert not recovery.confirmed


def test_wrapper_publishes_recovery_version_and_confirmation():
  recovery, _, _, _ = exercise_wrapper()
  assert recovery.status['path_recovery_version'] == 2
  assert recovery.status['path_recovery_confirmed'] is True
