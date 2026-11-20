from types import SimpleNamespace as State

import pytest

from openpilot.selfdrive.controls.lib.frogpilot_camry_torque import FrogPilotCamryTorque


def controller():
  tune = State(kp=1., ki=.1, latAccelFactor=2.37, latAccelOffset=0., friction=.13, steeringAngleDeadzoneDeg=0.)
  cp = State(steerLimitTimer=.4, steerActuatorDelay=.12, lateralTuning=State(torque=tune))
  return FrogPilotCamryTorque(cp, None)


@pytest.mark.parametrize("speed", [1., 12., 34.])
def test_frogpilot_feedback_and_camry_direction(speed):
  ctrl = controller()
  state = State(vEgo=speed, steeringAngleDeg=0., steeringPressed=False, steeringTorque=0.)
  vm = State(calc_curvature=lambda angle, speed, roll: 0.)
  params = State(angleOffsetDeg=0., roll=0.)
  command = lambda curvature: ctrl.update(True, state, vm, params, False, curvature, False, .22, None, None, None)
  steer, _, log = command(.0001)
  assert log.active and log.p > 0 and -1 <= steer < 0
  state.steeringPressed, state.steeringTorque = True, 20.
  assert command(.0001)[0] == 0.
  state.steeringPressed = False
  assert command(.0001)[0] == 0.
  for _ in range(24):
    steer, _, _ = command(.0001)
  assert steer < 0.


def test_friction_is_in_lateral_acceleration_units_and_bad_tune_is_ignored():
  ctrl = controller()
  ctrl.update_live_torque_params(2.5, .02, .1)
  assert ctrl.pid.pos_limit == 2.5 and ctrl.pid.neg_limit == -2.5
  ctrl.update_live_torque_params(float("nan"), 0., .2)
  assert ctrl.pid.pos_limit == 2.5 and ctrl.torque_params.friction == .1
