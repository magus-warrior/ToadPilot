import json
import math
from dataclasses import asdict
from types import SimpleNamespace as NS

import pytest

from cereal import car
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.frogpilot.toadpilot import ToadPilotConfig, decode_toadpilot_config
from openpilot.selfdrive.toad_modeld.delay import DelaySelector
from openpilot.selfdrive.toad_modeld.starpilot_response import SteeringResponse, ThrottlePermission


def test_v11_settings_migrate_without_activating_or_losing_existing_takeoff():
  old = asdict(ToadPilotConfig(profile='observe', responsive_takeoff=True, live_steering_delay=True))
  old.pop('starpilot_response')
  old.pop('takeoff_style')
  old.pop('pickup_strength')
  old.pop('matched_lead_prediction')
  old['version'] = 11
  config = decode_toadpilot_config(json.dumps(old))
  assert config.version == 13 and not config.starpilot_response
  assert config.takeoff_style == 'existing'
  for name, value in old.items():
    if name != 'version':
      assert getattr(config, name) == value


def test_throttle_rejects_chatter_and_confirms_sustained_transitions():
  gate = ThrottlePermission()
  for _ in range(40):
    for probability in (.34, .46, .39, .41):
      assert gate.update(probability, 27., .05)
  for _ in range(4):
    assert gate.update(.34, 27., .05)
  assert not gate.update(.34, 27., .05)
  for _ in range(4):
    assert not gate.update(.46, 27., .05)
  assert gate.update(.46, 27., .05)
  assert gate.update(.1, 4., .05)
  assert not gate.update(math.nan, 27., .05)
  assert gate.update(.1, 27., .05, reset=True)


def test_steering_delay_is_rate_independent_and_falls_back_when_stale():
  estimate = NS(status='estimated', version=1, validBlocks=5, calPerc=100,
                lateralDelay=.35, lateralDelayEstimateStd=.02)
  model, controls = DelaySelector(.12), DelaySelector(.12)
  for _ in range(20):
    model.update(True, estimate, True, .1)
  for _ in range(100):
    controls.update(True, estimate, True, .1, dt=.01)
  assert model.current == pytest.approx(controls.current)
  before = controls.current
  controls.update(True, estimate, True, 2., dt=.01)
  assert controls.current == pytest.approx(before - .001)


def test_steering_offset_fades_at_creep_and_output_stays_bounded():
  response = SteeringResponse()
  args = dict(active=False, speed=27., desired_curvature=0., measured_curvature=0., roll=0.,
              delay=.22, factor=2.37, offset=.09, friction=.13, deadzone=0.,
              steer_max=1., limited=False, override=False)
  response.update(**args)
  out, data = response.update(**{**args, 'active': True})
  assert out > 0 and data['applied_offset'] == .09
  _, data = response.update(**{**args, 'active': True, 'speed': .3})
  assert data['applied_offset'] == 0.
  for speed in (.1, 1., 10., 27., 40.):
    for curvature in (-.1, .1):
      for _ in range(150):
        out, data = response.update(**{**args, 'active': True, 'speed': speed, 'desired_curvature': curvature})
        assert math.isfinite(out) and abs(out) <= 1.
  out, data = response.update(**{**args, 'active': True, 'factor': math.nan})
  assert out == 0. and not data['valid']


@pytest.mark.parametrize('style,expected', [('existing', 1.), ('planner', .3)])
def test_real_longcontrol_keeps_existing_launch_as_backup_and_honors_new_stops(style, expected):
  cp = NS(enableGasInterceptor=False, startingState=False, stopAccel=-2., startAccel=0.,
          carFingerprint='TOYOTA_CAMRY_TSS2', openpilotLongitudinalControl=True,
          longitudinalTuning=NS(kpBP=[0.], kpV=[0.], kiBP=[0.], kiV=[0.], kf=1.))
  toggles = NS(vEgoStarting=.25, stoppingDecelRate=.3, human_acceleration=False,
               start_accel_ramp=1., toadpilot_enabled=True, toadpilot_takeoff_style=style)
  state = NS(vEgo=0., aEgo=0., brakePressed=False, cruiseState=NS(standstill=True))
  control = LongControl(cp)
  control.update(True, state, 0., True, (-3., 2.), toggles)
  state.cruiseState.standstill = False
  assert control.update(True, state, .3, False, (-3., 2.), toggles) == pytest.approx(expected)
  assert toggles.start_accel_ramp == 1.
  assert control.update(True, state, -.4, True, (-3., 2.), toggles) <= 0.
  assert control.long_control_state == LongCtrlState.stopping


def test_real_linear_camry_controller_uses_new_loop_and_learned_bias(tmp_path):
  from openpilot.common.params import Params
  from openpilot.selfdrive.car import gen_empty_fingerprint
  from openpilot.selfdrive.car.toyota.interface import CarInterface
  from openpilot.selfdrive.car.toyota.values import CAR
  from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
  from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
  from openpilot.selfdrive.frogpilot.tests.test_toadpilot_upstream_torque_ports import LinearTorqueInterface
  cp = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS2, gen_empty_fingerprint(), [], False, False, Params(str(tmp_path)), False)
  control = LatControlTorque(cp, LinearTorqueInterface())
  control.toadpilot_assist.status_due = lambda _: False
  values = asdict(ToadPilotConfig(starpilot_response=True, profile='observe'))
  toggles = NS(**{'toadpilot_' + name: value for name, value in values.items()}, car_model='TOYOTA_CAMRY_TSS2',
               model='toad-rdf4', model_version='v15')
  state = NS(vEgo=27., aEgo=0., steeringAngleDeg=0., steeringPressed=False)
  params = NS(angleOffsetDeg=0., roll=0.)
  control.update_live_torque_params(2.37, .09, .13, toggles)
  control.update(False, state, VehicleModel(cp), params, False, 0., False, None, frogpilot_toggles=toggles)
  output, _, log = control.update(True, state, VehicleModel(cp), params, False, 0., False, None, frogpilot_toggles=toggles)
  assert log.active and output > 0 and math.isfinite(output)


@pytest.mark.parametrize('speed,offset,roll', [
  pytest.param(4., 0., 0., id='town'),
  pytest.param(20., 0., 0., id='highway'),
  pytest.param(30., 0., 0., id='fast'),
  pytest.param(20., .09, 0., id='offset'),
  pytest.param(20., 0., .01, id='roll'),
  pytest.param(30., .09, .01, id='offset_and_roll'),
])
def test_steering_response_mirrors_left_and_right_transitions(speed, offset, roll):
  left, right = SteeringResponse(), SteeringResponse()
  mismatch = []
  for i in range(1000):
    target = .25 * math.sin(2 * math.pi * .8 * i * .01) if i >= 100 else 0.
    outputs = []
    for sign, response in ((1., left), (-1., right)):
      output, data = response.update(
        active=True, speed=speed, desired_curvature=sign * target / speed**2,
        measured_curvature=sign * .8 * target / speed**2,
        roll=sign * roll, delay=.22, factor=2.37, offset=sign * offset,
        friction=.13, deadzone=0., steer_max=1., limited=False, override=False)
      assert data['valid'] and math.isfinite(output) and abs(output) <= 1.
      outputs.append(output)
    mismatch.append(abs(sum(outputs)))
  assert max(mismatch) < 1e-12
