import json
import math
from dataclasses import asdict
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
import pytest
from cereal import car, log, messaging
from openpilot.selfdrive.frogpilot.toadpilot import ToadPilotConfig, decode_toadpilot_config, config_from_toggles
from openpilot.selfdrive.toad_modeld.delay import DelaySelector
from openpilot.selfdrive.toad_modeld.lag_estimator import LateralLagEstimator
from openpilot.selfdrive.toad_modeld.lagd import cache_key, restore_cache, atomic_json
from openpilot.selfdrive.toad_modeld.longitudinal import LongitudinalCoordinator, coordination_enabled, AccelBoost
from openpilot.selfdrive.toad_modeld.camera_timing import CameraTiming
from openpilot.selfdrive.car.toyota.hill_compensation import HillCompensation
from openpilot.system.manager.process_config import run_toad_lagd

FIELDS = ('live_steering_delay', 'modern_longitudinal', 'accel_boost', 'hill_compensation', 'camera_timing_checks')


def test_schema_migration_preserves_every_saved_choice_and_defaults_new_features_off():
  old = asdict(ToadPilotConfig(enabled=False, profile='observe', smooth_braking=False, responsive_takeoff=False))
  for key in FIELDS:
    old.pop(key)
  old['version'] = 10
  result = asdict(decode_toadpilot_config(json.dumps(old)))
  assert result['version'] == 11
  assert all(result[key] == value for key, value in old.items() if key != 'version')
  assert all(result[key] is False for key in FIELDS)


@pytest.mark.parametrize('field', FIELDS)
def test_new_config_roundtrip_and_malformed_fail_closed(field):
  config = asdict(ToadPilotConfig())
  config[field] = True
  assert getattr(decode_toadpilot_config(json.dumps(config)), field) is True
  snapshot = NS(**{'toadpilot_' + key: value for key, value in config.items() if key != 'version'})
  assert getattr(config_from_toggles(snapshot), field) is True
  for bad in (1, 'true', None):
    config[field] = bad
    result = decode_toadpilot_config(json.dumps(config))
    assert not result.enabled and not getattr(result, field)


def estimate(**changes):
  return NS(**dict(status='estimated', version=1, validBlocks=5, calPerc=100,
                   lateralDelay=0.35, lateralDelayEstimateStd=0.02) | changes)


def test_delay_accepts_only_confident_fresh_estimate_and_limits_changes():
  selector = DelaySelector(0.12)
  assert selector.update(True, estimate(), True, 0.1) == pytest.approx(0.125)
  for _ in range(100):
    selector.update(True, estimate(), True, 0.1)
  assert selector.current == pytest.approx(0.35)
  assert selector.update(True, estimate(), True, 1.1) == pytest.approx(0.345)
  assert selector.update(False, estimate(), True, 0) == 0.12
  for changes in (dict(status='unestimated'), dict(version=0), dict(validBlocks=4), dict(calPerc=99),
                  dict(lateralDelay=math.nan), dict(lateralDelay=0.66), dict(lateralDelayEstimateStd=0.11)):
    assert selector.update(True, estimate(**changes), True, 0.1) == 0.12
  for age in (-1, 2, math.nan):
    assert selector.update(True, estimate(), True, age) == 0.12
  assert selector.update(True, estimate(), False, 0) == 0.12


def test_lag_process_only_runs_for_enabled_modern_model_onroad():
  for mid, ver, modern in [('north-dakota', 'v2', False), ('toad-pop-v2', 'v11', True), ('toad-rdf4', 'v15', True)]:
    for on in (False, True):
      for enabled in (False, True):
        toggles = NS(model=mid, model_version=ver, toadpilot_live_steering_delay=enabled)
        assert run_toad_lagd(on, None, None, False, False, toggles) == (on and enabled and modern)


def test_delay_cache_requires_same_vehicle_and_model_and_fresh_route_evidence():
  with TemporaryDirectory() as directory:
    path = Path(directory) / 'cache.json'
    key = cache_key(b'car', 'toad-rdf4')
    assert key != cache_key(b'car2', 'toad-rdf4') != cache_key(b'car', 'toad-pop-v2')
    assert restore_cache(path, key) is None
    record = dict(key=key, version=1, delay=0.3, blocks=10)
    atomic_json(path, record)
    assert restore_cache(path, key) == (0.3, 4)
    assert restore_cache(path, 'other') is None
    for overrides in (dict(delay=True), dict(delay=0.8), dict(blocks=4), dict(version=2)):
      atomic_json(path, record | overrides)
      assert restore_cache(path, key) is None


def test_calibrated_localizer_adapter_and_recovery_gates():
  learner = LateralLagEstimator(NS(steerActuatorDelay=0.12), 0.05)
  pose = log.LiveLocationKalman.new_message()
  pose.angularVelocityCalibrated.value = [0., 0., 0.02]
  pose.angularVelocityCalibrated.std = [0.01, 0.01, 0.01]
  pose.angularVelocityCalibrated.valid = True
  pose.inputsOK = pose.sensorsOK = pose.posenetOK = True
  learner.handle_log(10, 'liveLocationKalman', pose)
  assert learner.pose_valid and learner.yaw_rate == pytest.approx(0.02)
  learner.lat_active = learner.calibrator.calib_valid = True
  learner.v_ego = 25
  learner.desired_curvature = 0.02 / 25
  learner.update_points()
  assert learner.points.okay[-1]
  learner.steering_pressed = True
  learner.update_points()
  learner.steering_pressed = False
  learner.t = 11
  learner.update_points()
  assert not learner.points.okay[-1]
  learner.t = 12.1
  learner.update_points()
  assert learner.points.okay[-1]
  pose.inputsOK = False
  learner.handle_log(13, 'liveLocationKalman', pose)
  learner.update_points()
  assert not learner.points.okay[-1]
  learner.desired_curvature = math.nan
  learner.update_points()
  assert not learner.points.okay[-1] and learner.points.desired[-1] == 0


def test_cross_correlation_recovers_known_delay_and_rejects_flat_data():
  rng = np.random.default_rng(42)
  signal = np.convolve(rng.normal(size=1200), np.ones(3) / 3, mode='same')
  actual = np.roll(signal, 6)
  mask = np.ones(1200, dtype=bool)
  mask[:20] = False
  lag, corr, confidence = LateralLagEstimator.actuator_delay(signal, actual, mask, 0.05, 0.15, 0.65)
  assert lag == pytest.approx(0.3, abs=0.01)
  assert corr > 0.99 and confidence > 0.7
  learner = LateralLagEstimator(NS(steerActuatorDelay=0.12), 0.05)
  for i in range(1200):
    learner.points.update(i * 0.05, 0, 0, True)
  learner.update_estimate()
  assert learner.block_avg.valid_blocks == 0
  assert str(learner.get_msg(True).liveDelay.status) == 'unestimated'


def test_camera_guard_rejects_skew_stale_repeated_and_missing_frames():
  guard = CameraTiming()
  assert guard.update(1_000_000_000, 1_020_000_000, 1_005_000_000, 1_025_000_000, 1_050_000_000)
  assert not guard.update(1_000_000_000, 1_020_000_000, 1_005_000_000, 1_025_000_000, 1_050_000_000)
  assert not CameraTiming().update(1_000_000_000, 1_020_000_000, 1_020_000_000, 1_040_000_000, 1_050_000_000)
  assert not CameraTiming().update(1_000_000_000, 1_020_000_000, 1_000_000_000, 1_020_000_000, 1_250_000_000)
  assert not CameraTiming().update(0, 0, 0, 0, 1_000_000_000)
  assert not CameraTiming().update(math.nan, 1, 1, 1, 1)
  assert guard.update(1_100_000_000, 1_120_000_000, 1_100_000_000, 1_120_000_000, 1_150_000_000)


def coordinator_step(coordinator, **changes):
  args = dict(mpc_accel=1.0, mpc_stop=False, mpc_source='lead0',
              action=NS(desiredAcceleration=0.5, longitudinalValid=True, shouldStop=False),
              v_ego=25., v_cruise=30., minimum=-3.5, maximum=2., dt=0.05,
              boost_enabled=False, enabled=True, gas_pressed=False, brake_pressed=False)
  return coordinator.update(**(args | changes))


def test_coordination_preserves_lead_braking_stop_limits_and_cruise_jerk():
  coord = LongitudinalCoordinator()
  assert coordinator_step(coord)[0] == pytest.approx(0.04)
  assert coordinator_step(coord, mpc_accel=-2, mpc_stop=True) == (-2., True, 'lead0')
  action = NS(desiredAcceleration=-3., longitudinalValid=True, shouldStop=True)
  assert coordinator_step(coord, action=action) == (-3., True, 'e2e')
  action.desiredAcceleration = -10
  assert coordinator_step(coord, action=action)[0] == -3.5
  assert coordinator_step(coord, maximum=-0.3)[0] <= -0.3
  action.longitudinalValid = False
  with pytest.raises(ValueError):
    coordinator_step(coord, action=action)
  with pytest.raises(ValueError):
    coordinator_step(coord, mpc_accel=math.nan)


def test_boost_requires_override_is_bounded_and_never_raises_lead_limit():
  learner = AccelBoost()
  assert learner.update(True, False, 25, 0, 1, 1, .05) == 0
  for _ in range(500):
    learner.update(True, True, 25, 0, 1, 1, .05)
  assert learner.total_boost == pytest.approx(.05)
  for _ in range(6):
    learner.update(True, False, 25, 0, 1, 1, .05)
    for _ in range(100):
      learner.update(True, True, 25, 0, 1, 1, .05)
  assert learner.total_boost == pytest.approx(.2)
  assert learner.update(True, False, 25, -2, 1, 1, .05) == -2
  assert learner.update(False, False, 25, 0, 1, 1, .05) == 0
  assert learner.total_boost == 0
  coord = LongitudinalCoordinator()
  coord.boost.total_boost = .2
  accel, _, source = coordinator_step(coord, boost_enabled=True, mpc_accel=-2)
  assert accel == -2 and source == 'lead0'


@pytest.mark.parametrize('speed', [0., .1, 1., 1.99])
def test_acceleration_learning_does_not_change_hybrid_launch_boost(speed):
  from openpilot.selfdrive.frogpilot.controls.lib.start_accel_ramp import StartAccelRamp

  outputs = []
  for learning in (False, True):
    coordinator = LongitudinalCoordinator()
    coordinator.boost.total_boost = .2  # Already learned the largest allowed adjustment.
    ramp = StartAccelRamp()
    action = NS(desiredAcceleration=.3, longitudinalValid=True, shouldStop=False)
    accel, stop, _ = coordinator_step(coordinator, v_ego=speed, v_cruise=10.,
      action=action, mpc_accel=.4, boost_enabled=learning)
    # The actual downstream hybrid launch floor, using the device's saved setting.
    output = ramp.update(accel, 1., speed, accel, True, True, stop, False, False)
    outputs.append((accel, stop, output))
    assert output >= accel
    assert ramp.started == (not stop)
    # A model stop must cancel the launch even with the learned maximum present.
    action.shouldStop = True
    stopped_accel, must_stop, _ = coordinator_step(coordinator, v_ego=speed, v_cruise=10.,
      action=action, mpc_accel=.4, boost_enabled=learning)
    assert must_stop
    assert ramp.update(stopped_accel, 1., speed, stopped_accel, False, True, must_stop, False, False) == stopped_accel
    assert not ramp.started
  assert outputs[0] == outputs[1]


def test_hill_correction_is_transient_bounded_and_resets_missing_attitude():
  hill = HillCompensation(.01)
  hill.update([0, .1, 0])
  assert hill.correction(True) == 0  # no startup pulse
  for _ in range(30):
    hill.update([0, .2, 0])
  assert 0 < hill.correction(True) < 1.5
  assert hill.correction(False) == 0
  for _ in range(2000):
    hill.update([0, .2, 0])
  assert abs(hill.correction(True)) < .001
  hill.update([])
  assert hill.correction(True) == 0
  hill.update([0, .3, 0])
  assert hill.correction(True) == 0
  hill.update([0, math.nan, 0])
  assert hill.correction(True) == 0


def planner_fixture():
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  from openpilot.selfdrive.modeld.constants import ModelConstants
  services = ['carControl', 'carState', 'controlsState', 'modelV2', 'frogpilotPlan', 'frogpilotCarState', 'liveParameters', 'radarState']
  class State(dict):
    logMonoTime = {'modelV2': 0}
    alive = valid = dict.fromkeys(services, True)
    def all_checks(self, service_list):
      return all(self.valid[k] for k in service_list)
  sm = State({k: getattr(messaging.new_message(k), k) for k in services})
  sm['carState'].vEgo = 25.
  sm['controlsState'].enabled = sm['controlsState'].experimentalMode = True
  sm['controlsState'].longControlState = 'pid'
  sm['controlsState'].vCruise = 108
  plan = sm['frogpilotPlan']
  plan.vCruise = 30
  plan.minAcceleration, plan.maxAcceleration = -3.5, 2.
  plan.accelerationJerk = plan.dangerJerk = plan.speedJerk = 1.
  plan.tFollow = 1.5
  model = sm['modelV2']
  model.position.x = (np.array(ModelConstants.T_IDXS) * 25).tolist()
  model.velocity.x = [25.] * 33
  model.acceleration.x = [0.] * 33
  model.orientationRate.z = [0.] * 33
  model.action.desiredAcceleration = -.8
  model.action.longitudinalValid = True
  CP = NS(openpilotLongitudinalControl=True, longitudinalActuatorDelay=.2, steerRatio=15., wheelbase=2.8)
  toggles = NS(model='toad-pop-v2', model_version='v11', taco_tune=False, increased_stopped_distance=0, vEgoStopping=.05,
               toadpilot_modern_longitudinal=True, toadpilot_accel_boost=False)
  return LongitudinalPlanner(CP, init_v=25.), sm, toggles


def test_real_planner_solver_update_publish_and_switch_off():
  planner, sm, toggles = planner_fixture()
  pm = Mock()
  for _ in range(20):
    planner.update(False, sm, toggles)
  planner.publish(False, False, sm, pm, toggles)
  msg = pm.send.call_args.args[1]
  assert msg.valid
  assert planner.mpc.mode == 'acc'
  assert msg.longitudinalPlan.aTarget <= -.79
  assert planner.a_desired == pytest.approx(msg.longitudinalPlan.aTarget)
  sm['modelV2'].action.longitudinalValid = False
  planner.update(False, sm, toggles)
  planner.publish(False, False, sm, pm, toggles)
  assert not pm.send.call_args.args[1].valid
  sm['modelV2'].action.longitudinalValid = True
  toggles.toadpilot_modern_longitudinal = False
  planner.update(False, sm, toggles)
  assert planner.mpc.mode == 'blended' and planner.coordinated_output is None
  assert planner.coordinator.boost.total_boost == 0
  toggles.toadpilot_modern_longitudinal = True
  sm['controlsState'].experimentalMode = False
  planner.update(False, sm, toggles)
  assert planner.mpc.mode == 'acc' and planner.coordinated_output is None


@pytest.mark.parametrize('speed', [0., .1, 1., 1.99, 4.4])
def test_real_planner_learning_keeps_existing_low_speed_path(speed):
  from openpilot.selfdrive.modeld.constants import ModelConstants

  outputs = []
  for learning in (False, True):
    fixture, sm, toggles = planner_fixture()
    planner = type(fixture)(fixture.CP, init_v=speed)
    sm['carState'].vEgo = speed
    sm['modelV2'].position.x = (np.array(ModelConstants.T_IDXS) * speed).tolist()
    sm['modelV2'].velocity.x = [speed] * 33
    sm['modelV2'].action.desiredAcceleration = .3
    toggles.toadpilot_modern_longitudinal = learning
    toggles.toadpilot_accel_boost = learning
    pm = Mock()
    samples = []
    for _ in range(10):
      planner.update(False, sm, toggles)
      planner.publish(False, False, sm, pm, toggles)
      plan = pm.send.call_args.args[1].longitudinalPlan
      assert planner.mpc.mode == 'blended'
      assert planner.coordinated_output is None
      samples.append((plan.aTarget, plan.shouldStop))
    outputs.append(samples)
  assert outputs[0] == outputs[1]


def test_full_delay_learner_reaches_confident_estimate_from_delayed_signal():
  learner = LateralLagEstimator(NS(steerActuatorDelay=.12), .05)
  rng = np.random.default_rng(81)
  signal = np.convolve(rng.normal(size=4500), np.ones(3) / 3, mode='same') * .3
  for i in range(6, len(signal)):
    learner.t = i * .05
    learner.points.update(learner.t, signal[i], signal[i-6], True)
    if i % 5 == 0:
      learner.update_estimate()
  message = learner.get_msg(True)
  assert message.liveDelay.validBlocks >= 5
  assert str(message.liveDelay.status) == 'estimated'
  assert message.liveDelay.lateralDelay == pytest.approx(.3, abs=.01)
  assert message.liveDelay.calPerc == 100
  with log.Event.from_bytes(message.to_bytes()) as reader:
    assert reader.which() == 'liveDelay'
    assert reader.liveDelay.version == 1


def test_real_toyota_controller_applies_hill_only_when_eligible():
  from openpilot.selfdrive.frogpilot.tests.test_toadpilot_high_torque_controller import TestToadPilotHighTorqueControllerSequence
  fixture = TestToadPilotHighTorqueControllerSequence('runTest')
  fixture.setUp()
  try:
    fixture.CP.openpilotLongitudinalControl = True
    fixture.CC.longActive = True
    fixture.CC.actuators.accel = .3
    fixture.CC.orientationNED = [0., 0., 0.]
    fixture.toggles.toadpilot_hill_compensation = True
    fixture.update()
    fixture.CC.orientationNED = [0., .1, 0.]
    for _ in range(30):
      fixture.update()
    pid = fixture.controller.long_pid
    with patch.object(pid, 'update', wraps=pid.update) as update:
      for _ in range(3):
        output = fixture.update()
      assert update.call_args.kwargs['feedforward'] > fixture.controller.prev_accel
      assert fixture.controller.params.ACCEL_MIN <= output.accel <= fixture.controller.params.ACCEL_MAX
      for blocker in ('disabled', 'stopping', 'gas', 'brake', 'missing'):
        fixture.toggles.toadpilot_hill_compensation = blocker != 'disabled'
        fixture.CC.actuators.longControlState = 'stopping' if blocker == 'stopping' else 'pid'
        fixture.car_state.gasPressed = blocker == 'gas'
        fixture.car_state.brakePressed = blocker == 'brake'
        fixture.CC.orientationNED = [] if blocker == 'missing' else [0., .1, 0.]
        for _ in range(3):
          fixture.update()
        assert update.call_args.kwargs['feedforward'] == pytest.approx(fixture.controller.prev_accel)
      fixture.CC.longActive = False
      update.reset_mock()
      for _ in range(3):
        fixture.update()
      update.assert_not_called()
  finally:
    fixture.tearDown()


def test_lagd_loop_masks_missing_samples_and_resets_history_after_gap():
  from openpilot.selfdrive.toad_modeld import lagd
  CP = car.CarParams.new_message()
  CP.steerActuatorDelay = .12
  inputs = ['carControl', 'carState', 'controlsState', 'liveCalibration', 'liveLocationKalman']
  class State(dict):
    frame = -1
    updated = dict.fromkeys(inputs, True)
    logMonoTime = dict.fromkeys(inputs, 0)
    def update(self):
      self.frame += 1
      if self.frame == 65:
        raise StopIteration
      now = 10 + self.frame * .05 + (.2 if self.frame >= 40 else 0)
      self.logMonoTime = dict.fromkeys(inputs, int(now * 1e9))
    def all_checks(self):
      return self.frame != 30
  sm = State({name: getattr(messaging.new_message(name), name) for name in inputs})
  sm['carControl'].latActive = True
  sm['carState'].vEgo = 25.
  sm['controlsState'].desiredCurvature = .0004
  sm['controlsState'].lateralControlState.init('torqueState')
  sm['liveCalibration'].calStatus = 'calibrated'
  pose = sm['liveLocationKalman']
  pose.inputsOK = pose.sensorsOK = pose.posenetOK = True
  pose.angularVelocityCalibrated.valid = True
  pose.angularVelocityCalibrated.value = [0., 0., .01]
  pose.angularVelocityCalibrated.std = [0., 0., .01]
  learner = LateralLagEstimator(CP, .05)
  pm = Mock()
  params = Mock()
  params.get.side_effect = lambda key, **kwargs: CP.to_bytes() if key == 'CarParams' else 'toad-pop-v2'
  with patch.object(lagd, 'Params', return_value=params), patch.object(lagd, 'config_realtime_process'), \
       patch.object(lagd.messaging, 'SubMaster', return_value=sm), patch.object(lagd.messaging, 'PubMaster', return_value=pm), \
       patch.object(lagd, 'restore_cache', return_value=None), patch.object(lagd, 'atomic_json'), \
       patch.object(lagd, 'LateralLagEstimator', return_value=learner):
    with pytest.raises(StopIteration):
      lagd.main()
  messages = [call.args[1] for call in pm.send.call_args_list]
  assert len(messages) == 13
  assert not messages[6].valid  # missing input at frame 30
  assert not messages[8].valid  # sample gap at frame 40
  assert messages[-1].valid
  assert learner.block_avg.valid_blocks == 0
  times, _, _, okay = learner.points.get()
  assert np.count_nonzero(times) == 25  # history reset at frame 40; all later samples retained
  assert not okay[-1]  # two-second recovery has not elapsed since the gap


def test_delay_ui_status_rejects_stale_or_invalid_data():
  from openpilot.selfdrive.toad_modeld.delay import read_status
  with TemporaryDirectory() as directory:
    path = Path(directory) / 'status.json'
    with patch('openpilot.selfdrive.toad_modeld.delay.time.time', return_value=100):
      valid = dict(valid=True, status='estimated', progress=100, delay=.3, updated=99)
      atomic_json(path, valid)
      assert read_status(path) == valid
      for override in (dict(updated=90), dict(updated=101), dict(valid=False), dict(delay='bad'), dict(progress=101)):
        atomic_json(path, valid | override)
        assert read_status(path) == {}
