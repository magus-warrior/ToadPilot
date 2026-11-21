"""Native stop management through legacy messages and the real longitudinal solver."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
from cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.frogpilot.tests.test_starpilot_model_actions import setup_case
from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import Publisher
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.toad_modeld.starpilot_controls.integration import StarPilotControls
from openpilot.selfdrive.toad_modeld.starpilot_controls import conditional_experimental_mode as cem_source
from openpilot.selfdrive.toad_modeld.starpilot_long import planner as planner_source


def controls_setup(tmp_path, *, speed=15., force=True):
  solver, sm, toggles = setup_case(tmp_path/'car', model='toad-rdf6',speed=speed)
  values=dict(toadpilot_enabled=True, toadpilot_starpilot_response=True, vEgoStarting=.25,
              stoppingDecelRate=.3, human_acceleration=True, start_accel_ramp=0.,
              toadpilot_takeoff_style='planner', toadpilot_improved_stop_and_go=True,
              experimental_mode_via_press=True, conditional_experimental_mode=True,
              conditional_model_stop_time=9., conditional_limit=0., conditional_limit_lead=0.,
              conditional_signal=0., conditional_signal_lane_detection=False, lane_detection_width=2.5,
              conditional_curves=False, conditional_curves_lead=False, conditional_lead=False,
              conditional_stopped_lead=False, conditional_slower_lead=False,
              force_stops=force, force_standstill=False, vision_turn_speed_controller=False,
              speed_limit_controller=False, show_speed_limits=False, car_model='TOYOTA_CAMRY_TSS2')
  for key,value in values.items():setattr(toggles,key,value)
  parent=NS(frogpilot_following=NS(following_lead=False,slower_lead=False),
            frogpilot_vcruise=NS(slc=NS(experimental_mode=False)),
            tracking_lead=False,lead_one=sm['radarState'].leadOne,model_length=75.,model_stopped=False,
            lane_width_left=3.,lane_width_right=3.,road_curvature_detected=False,road_curvature=0.)
  ctx=StarPilotControls(parent,Params(str(tmp_path/'params')),Params(str(tmp_path/'memory')),state_path=tmp_path/'curve.json')
  model=sm['modelV2'];model.orientationRate.t=list(ModelConstants.T_IDXS)
  sm['carControl'].longActive=True
  nav=messaging.new_message('frogpilotNavigation').frogpilotNavigation
  return ctx,parent,solver,sm,toggles,nav


def update(ctx,parent,sm,toggles,nav,speed,now):
  model=sm['modelV2'];parent.model_length=model.position.x[-1]
  parent.model_stopped=parent.model_length<50. or ctx.starpilot_vcruise.forcing_stop
  ctx.prepare(sm['carState'],sm['controlsState'],sm['frogpilotCarState'],nav,model,sm['carControl'],toggles)
  ctx.update_mode(speed,30.,True)
  target=ctx.update_cruise(30.,speed,True,now)
  wire=sm['frogpilotPlan'];cruise=ctx.starpilot_vcruise
  wire.vCruise=target;wire.redLight=ctx.starpilot_cem.stop_light_detected
  wire.forcingStop=cruise.forcing_stop;wire.forcingStopLength=cruise.tracked_model_length
  wire.approachStopLength=cruise.approach_stop_length;wire.stopSignConfirmed=cruise.stop_sign_confirmed
  sm['controlsState'].experimentalMode=ctx.starpilot_cem.experimental_mode
  return wire


@pytest.mark.parametrize('case',['lead_beyond_stop','curved_approach','traffic_mode'])
def test_native_detector_keeps_real_stop_context(tmp_path,monkeypatch,case):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path)
  clock=[100.];monkeypatch.setattr(cem_source,'time',NS(monotonic=lambda:clock[0]))
  if case=='lead_beyond_stop':
    parent.tracking_lead=True;parent.lead_one.status=True;parent.lead_one.radar=True
    parent.lead_one.dRel=120.;parent.lead_one.vLead=15.;parent.lead_one.modelProb=1.
  if case=='curved_approach':parent.road_curvature_detected=True
  if case=='traffic_mode':sm['frogpilotCarState'].trafficMode=True
  times=np.asarray(ModelConstants.T_IDXS);sm['modelV2'].position.x=np.minimum(15*times,75.).tolist()
  sm['modelV2'].action.desiredAcceleration=-1.
  for _ in range(80):
    clock[0]+=.05;update(ctx,parent,sm,toggles,nav,15.,clock[0])
  assert ctx.starpilot_cem.stop_light_detected
  assert ctx.starpilot_cem.experimental_mode


@pytest.mark.parametrize('minimum',[-.6,-2.4])
@pytest.mark.parametrize('model_action',[0.,-.2])
def test_full_approach_to_stop_with_weak_direct_action(tmp_path,monkeypatch,model_action,minimum):
  ctx,parent,solver,sm,toggles,nav=controls_setup(tmp_path,speed=27.)
  clock=[100.]
  monkeypatch.setattr(cem_source,'time',NS(monotonic=lambda:clock[0]))
  monkeypatch.setattr(planner_source,'time',NS(monotonic=lambda:clock[0]))
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  control=LongControl(solver.core.CP.original)
  speed,gap,last_accel=27.,160.,0.
  pub=Publisher();approached=forced=False
  times=np.asarray(ModelConstants.T_IDXS)
  sm['frogpilotPlan'].minAcceleration=minimum
  for frame in range(600):
    clock[0]+=.05
    cs=sm['carState'];cs.vEgo=cs.vEgoCluster=speed;cs.aEgo=last_accel;cs.standstill=speed<.01
    decel=max(speed**2/(2*max(gap,1.)),.1)
    tstop=speed/decel;elapsed=np.minimum(times,tstop)
    model=sm['modelV2']
    model.position.x=(speed*elapsed-.5*decel*elapsed**2).tolist()
    model.velocity.x=np.maximum(speed-decel*times,0.).tolist()
    model.acceleration.x=np.where(times<tstop,-decel,0.).tolist()
    model.action.desiredAcceleration=model_action;model.action.shouldStop=speed<.3
    wire=update(ctx,parent,sm,toggles,nav,speed,clock[0])
    approached |= wire.approachStopLength>0.;forced |= wire.forcingStop
    solver.update(False,sm,toggles);solver.publish(False,False,sm,pub,toggles)
    out=pub.message.longitudinalPlan
    assert pub.message.valid and solver.mpc.solution_status==0
    assert -3.51<=out.aTarget<=.81
    # The planner's stop flag is completed by the 100-Hz brake controller;
    # aTarget alone is not the applied acceleration during a stopping state.
    for _ in range(5):
      cs.vEgo=cs.vEgoCluster=speed;cs.aEgo=last_accel;cs.standstill=speed<.01
      applied=control.update(True,cs,out.aTarget,out.shouldStop,(-3.5,.8),toggles,
                             forcing_stop=wire.forcingStop,has_lead=out.hasLead)
      next_speed=max(0.,speed+applied*.01)
      gap-=(speed+next_speed)*.005
      assert gap>0.,(frame,gap,speed,out.aTarget,applied)
      speed,last_accel=next_speed,applied
  assert approached and forced
  assert speed<.3,(speed,gap,last_accel)
  assert out.shouldStop


def test_force_stop_preference_is_honored_and_open_road_accel_unchanged(tmp_path,monkeypatch):
  ctx,parent,solver,sm,toggles,nav=controls_setup(tmp_path,force=False)
  clock=[100.];monkeypatch.setattr(cem_source,'time',NS(monotonic=lambda:clock[0]))
  times=np.asarray(ModelConstants.T_IDXS)
  sm['modelV2'].position.x=(times*15.).tolist();sm['modelV2'].action.desiredAcceleration=.8
  for _ in range(80):
    clock[0]+=.05;wire=update(ctx,parent,sm,toggles,nav,15.,clock[0])
  assert not wire.forcingStop and wire.approachStopLength==0.
  assert wire.vCruise==30.
  assert not ctx.starpilot_cem.experimental_mode


def test_driver_override_releases_native_stop_and_prevents_recommit(tmp_path,monkeypatch):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path)
  clock=[100.];monkeypatch.setattr(cem_source,'time',NS(monotonic=lambda:clock[0]))
  sm['modelV2'].position.x=np.minimum(np.asarray(ModelConstants.T_IDXS)*15.,75.).tolist()
  for _ in range(80):clock[0]+=.05;update(ctx,parent,sm,toggles,nav,15.,clock[0])
  assert ctx.starpilot_vcruise.forcing_stop
  sm['carState'].gasPressed=True
  clock[0]+=.05;update(ctx,parent,sm,toggles,nav,15.,clock[0])
  assert ctx.starpilot_vcruise.override_force_stop
  assert not ctx.starpilot_vcruise.forcing_stop


def test_green_release_clears_force_stop_after_native_confirmation(tmp_path,monkeypatch):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path)
  clock=[100.];monkeypatch.setattr(cem_source,'time',NS(monotonic=lambda:clock[0]))
  times=np.asarray(ModelConstants.T_IDXS)
  sm['modelV2'].position.x=np.minimum(times*15.,75.).tolist()
  for _ in range(80):
    clock[0]+=.05;update(ctx,parent,sm,toggles,nav,15.,clock[0])
  assert ctx.starpilot_vcruise.forcing_stop
  sm['modelV2'].position.x=(times*15.).tolist()
  sm['modelV2'].action.desiredAcceleration=.8
  for _ in range(160):
    clock[0]+=.05;wire=update(ctx,parent,sm,toggles,nav,15.,clock[0])
  assert not ctx.starpilot_cem.stop_light_detected
  assert not wire.forcingStop and wire.approachStopLength==0.
  assert wire.vCruise==30.


def test_brief_model_dropout_does_not_abandon_active_stop(tmp_path,monkeypatch):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path)
  clock=[100.];monkeypatch.setattr(cem_source,'time',NS(monotonic=lambda:clock[0]))
  times=np.asarray(ModelConstants.T_IDXS)
  sm['modelV2'].position.x=np.minimum(times*15.,75.).tolist()
  for _ in range(80):
    clock[0]+=.05;update(ctx,parent,sm,toggles,nav,15.,clock[0])
  sm['modelV2'].position.x=(times*15.).tolist()
  for _ in range(6):
    clock[0]+=.05;wire=update(ctx,parent,sm,toggles,nav,15.,clock[0])
  assert ctx.starpilot_cem.experimental_mode and wire.forcingStop


def test_close_lead_does_not_arm_independent_force_stop(tmp_path,monkeypatch):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path,speed=4.)
  clock=[100.];monkeypatch.setattr(cem_source,'time',NS(monotonic=lambda:clock[0]))
  parent.tracking_lead=True;parent.frogpilot_following.following_lead=True
  lead=parent.lead_one;lead.status=True;lead.radar=True;lead.modelProb=1.;lead.dRel=6.;lead.vLead=0.
  sm['modelV2'].position.x=np.minimum(np.asarray(ModelConstants.T_IDXS)*4.,8.).tolist()
  for _ in range(100):
    clock[0]+=.05;wire=update(ctx,parent,sm,toggles,nav,4.,clock[0])
  assert not wire.forcingStop and wire.approachStopLength==0.


@pytest.mark.parametrize('has_lead,target,immediate',[(True,.4,True),(False,0.,False),(False,-.2,False),(False,.2,False)])
def test_stop_release_preserves_lead_launch_and_requires_confirmation(tmp_path,has_lead,target,immediate):
  from cereal import car
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  _,_,solver,sm,toggles,_=controls_setup(tmp_path,speed=0.)
  control=LongControl(solver.core.CP.original)
  control.long_control_state=car.CarControl.Actuators.LongControlState.stopping
  control.last_output_accel=-1.
  output=control.update(True,sm['carState'],target,False,(-3.5,2.),toggles,has_lead=has_lead)
  if immediate:
    assert control.long_control_state==car.CarControl.Actuators.LongControlState.pid
    assert output==pytest.approx(target)
  else:
    assert control.long_control_state==car.CarControl.Actuators.LongControlState.stopping
    assert output<0.
    for _ in range(40):
      output=control.update(True,sm['carState'],target,False,(-3.5,2.),toggles,has_lead=has_lead)
    if target>0.15:
      assert control.long_control_state==car.CarControl.Actuators.LongControlState.pid
      assert output==pytest.approx(target)
    else:
      assert control.long_control_state==car.CarControl.Actuators.LongControlState.stopping


def test_native_curve_controller_keeps_straight_road_target(tmp_path):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path,force=False)
  toggles.vision_turn_speed_controller=True
  sm['modelV2'].position.x=(np.asarray(ModelConstants.T_IDXS)*15.).tolist()
  for frame in range(80):
    wire=update(ctx,parent,sm,toggles,nav,15.,100.+frame*.05)
  assert wire.vCruise==30.
  assert ctx.starpilot_vcruise.csc_target==pytest.approx(30.)


def test_learning_storage_failure_does_not_interrupt_stop_management(tmp_path):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path)
  unavailable=tmp_path/'not-a-directory';unavailable.write_text('file')
  ctx.params.path=unavailable/'curve.json'
  sm['modelV2'].position.x=np.minimum(np.asarray(ModelConstants.T_IDXS)*15.,75.).tolist()
  for frame in range(80):
    wire=update(ctx,parent,sm,toggles,nav,15.,100.+frame*.05)
  assert wire.forcingStop


def test_committed_turn_does_not_arm_a_new_stop(tmp_path,monkeypatch):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path,speed=6.)
  clock=[100.];monkeypatch.setattr(cem_source,'time',NS(monotonic=lambda:clock[0]))
  sm['carState'].leftBlinker=True;sm['carState'].steeringAngleDeg=60.
  sm['controlsState'].curvature=.05
  sm['modelV2'].position.x=np.minimum(np.asarray(ModelConstants.T_IDXS)*6.,20.).tolist()
  for _ in range(100):
    clock[0]+=.05;wire=update(ctx,parent,sm,toggles,nav,6.,clock[0])
  assert not wire.forcingStop


@pytest.mark.parametrize('manual_override',[True,False])
def test_speed_limit_adapter_retains_confirmation_and_override_preferences(tmp_path,manual_override):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path,force=False)
  toggles.speed_limit_controller=True; following=ctx.starpilot_vcruise.slc
  toggles.speed_limit_confirmation_lower=toggles.speed_limit_confirmation_higher=True
  toggles.speed_limit_controller_override_manual=manual_override
  toggles.speed_limit_controller_override_set_speed=not manual_override
  following.target=30.
  following.delegate=NS(experimental_mode=False,source="Dashboard",desired_speed_limit=20.,speed_limit_changed=True,
                        update=lambda *args:None,resolve_accepted_limit=lambda target, *_: (target, 0.))
  sm['controlsState'].state='enabled'
  ctx.params_memory.legacy.put_bool('SpeedLimitAccepted',True)
  update(ctx,parent,sm,toggles,nav,15.,100.)
  assert following.target==20.
  assert not ctx.params_memory.get_bool('SpeedLimitAccepted')
  sm['carState'].gasPressed=True;sm['carState'].vEgo=25.
  update(ctx,parent,sm,toggles,nav,25.,100.05)
  assert following.overridden_speed==pytest.approx(25. if manual_override else 30.)


@pytest.mark.parametrize('show_limits', [True, False])
def test_speed_limit_toggle_releases_and_restores_active_cruise_cap(tmp_path, show_limits):
  ctx,parent,_,sm,toggles,nav=controls_setup(tmp_path,force=False)
  toggles.speed_limit_controller=True
  toggles.show_speed_limits=show_limits
  following=ctx.starpilot_vcruise.slc
  following.target=12.
  following.delegate=NS(experimental_mode=False,desired_speed_limit=12.,speed_limit_changed=False,
                        update=lambda *args:None,resolve_accepted_limit=lambda target, *_: (target, 0.),source='Map Data')
  sm['modelV2'].position.x=(np.asarray(ModelConstants.T_IDXS)*15.).tolist()
  assert update(ctx,parent,sm,toggles,nav,15.,100.).vCruise==pytest.approx(12.)
  toggles.speed_limit_controller=False
  assert update(ctx,parent,sm,toggles,nav,15.,100.05).vCruise==pytest.approx(30.)
  toggles.speed_limit_controller=True
  assert update(ctx,parent,sm,toggles,nav,15.,100.10).vCruise==pytest.approx(12.)


def test_confirmed_stop_approach_respects_brake_limits_and_releases(tmp_path):
  _, _, planner, sm, _, _ = controls_setup(tmp_path, speed=7., force=False)
  cs, model, plan = sm['carState'], sm['modelV2'], sm['frogpilotPlan']
  cs.gearShifter, cs.canValid = 'drive', True
  times = np.asarray(ModelConstants.T_IDXS)
  elapsed = np.minimum(times, 6.)
  model.position.x = (7.*elapsed - 7./12.*elapsed**2).tolist()
  model.velocity.x = np.maximum(0., 7. - 7./6.*times).tolist()
  model.action.desiredAcceleration = -.5
  plan.minAcceleration = -1.2
  planner.core.output_a_target = -.2
  assert planner.stop_approach_target(sm, -.2) == pytest.approx(-.275)
  assert planner.stop_approach_target(sm, -1.8) == pytest.approx(-1.2)
  planner.core.output_a_target = -2.5
  assert planner.stop_approach_target(sm, -.2) == -2.5
  planner.core.output_a_target = -.2
  for signal, field in ((cs, 'brakePressed'), (cs, 'gasPressed'), (cs, 'leftBlinker'), (cs, 'rightBlinker'),
                        (sm['radarState'].leadOne, 'status'), (sm['radarState'].leadTwo, 'status')):
    setattr(signal, field, True)
    assert planner.stop_approach_target(sm, -1.8) == -.2
    setattr(signal, field, False)
  model.velocity.x = [7.] * 33
  assert planner.stop_approach_target(sm, -1.8) == -.2
