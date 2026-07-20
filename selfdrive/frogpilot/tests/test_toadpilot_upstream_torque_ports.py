import math
import unittest

from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from panda import Panda

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import gen_empty_fingerprint
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.toyota.interface import CarInterface
from openpilot.selfdrive.car.toyota.values import CAR
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.latcontrol_torque import (DelayedLateralResponse,
                                                                 LatControlTorque,
                                                                 kp_independent_low_speed_error,
                                                                 torque_model_data_good,
                                                                 use_toadpilot_upstream_response)
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.selfdrive.frogpilot.toadpilot import ToadPilotConfig
from openpilot.selfdrive.locationd.torqued import (TorqueEstimator, live_location_sample_valid,
                                                   toadpilot_extended_torque_sample)
from openpilot.selfdrive.modeld.constants import ModelConstants


class LinearTorqueInterface:
  use_nnff = False
  use_nnff_lite = False

  def torque_from_lateral_accel(self):
    return self.torque_from_lateral_accel_linear

  def torque_from_lateral_accel_linear(self, *args, **kwargs):
    return CarInterfaceBase.torque_from_lateral_accel_linear(self, *args, **kwargs)


class NeuralTorqueInterface(LinearTorqueInterface):
  use_nnff = True
  lat_torque_nn_model = SimpleNamespace(friction_override=False)

  @staticmethod
  def get_ff_nn(values):
    # Deterministic stand-in: enough to exercise the established NNFF path.
    return 0.05 * float(values[1])


def model_data(*, finite=True):
  length = len(ModelConstants.T_IDXS)
  value = 0.0 if finite else math.nan
  return SimpleNamespace(
    orientation=SimpleNamespace(x=[value] * length, y=[0.0] * length),
    acceleration=SimpleNamespace(y=np.linspace(0.0, 0.4, length).tolist()),
  )


def controller_toggles(*, enabled=True, profile="adaptive", steer_kp=1.0):
  return SimpleNamespace(
    toadpilot_enabled=enabled,
    toadpilot_profile=profile,
    toadpilot_auto_tune=False,
    toadpilot_lane_centering_assist=True,
    toadpilot_smooth_lane_transitions=True,
    toadpilot_straight_line_smoothing=True,
    toadpilot_experimental_max_torque=1500,
    toadpilot_improved_stop_and_go=True,
    toadpilot_curve_assist_in_experimental=False,
    toadpilot_smart_saturation_warning=True,
    toadpilot_telemetry=False,
    use_custom_lat_accel_factor=False,
    use_custom_steer_friction=False,
    steer_lat_accel_factor=0.0,
    steer_friction=0.0,
    steer_kp=[[0.0], [steer_kp]],
  )


def location_sample(*, valid=True, finite=True):
  yaw_rate = 0.02 if finite else math.nan
  return SimpleNamespace(
    angularVelocityCalibrated=SimpleNamespace(valid=valid, value=[0.0, 0.0, yaw_rate]),
    orientationNED=SimpleNamespace(valid=valid, value=[0.01, 0.0, 0.0]),
    inputsOK=valid,
    sensorsOK=valid,
    posenetOK=valid,
  )


class TestDelayedLateralResponse(unittest.TestCase):
  def test_history_is_fixed_at_one_second(self):
    response = DelayedLateralResponse(0.12)

    for frame in range(10_000):
      _, _, jerk, ready = response.update(frame * 0.001, frame * 0.00001)
      self.assertEqual(ready, frame + 1 >= response.delay_frames)
      self.assertTrue(math.isfinite(jerk))

    expected_len = int(1.0 / DT_CTRL)
    self.assertEqual(response.buffer_len, expected_len)
    self.assertEqual(len(response.lat_accel_requests), expected_len)
    self.assertEqual(len(response.curvature_requests), expected_len)
    self.assertEqual(response.lat_accel_requests.maxlen, expected_len)
    self.assertEqual(response.curvature_requests.maxlen, expected_len)

  def test_delay_alignment_and_filtered_jerk_are_finite(self):
    response = DelayedLateralResponse(0.12)
    returned = []
    jerks = []
    readiness = []
    for _ in range(response.delay_frames):
      expected_accel, _, jerk, ready = response.update(1.0, 0.01)
      returned.append(expected_accel)
      jerks.append(jerk)
      readiness.append(ready)

    self.assertTrue(all(value == 0.0 for value in returned[:-1]))
    self.assertEqual(returned[-1], 1.0)
    self.assertTrue(all(math.isfinite(value) for value in jerks))
    self.assertTrue(any(abs(value) > 0.0 for value in jerks))
    self.assertTrue(all(not ready for ready in readiness[:-1]))
    self.assertTrue(readiness[-1])

  def test_nonfinite_input_clears_history_and_fails_neutral(self):
    response = DelayedLateralResponse(0.12)
    for _ in range(response.buffer_len):
      response.update(1.0, 0.01)

    self.assertEqual(response.update(math.nan, 0.01), (0.0, 0.0, 0.0, False))
    self.assertTrue(all(value == 0.0 for value in response.lat_accel_requests))
    self.assertTrue(all(value == 0.0 for value in response.curvature_requests))
    self.assertEqual(response.jerk_filter.x, 0.0)

    expected_accel, expected_curvature, jerk, ready = response.update(0.5, 0.005)
    self.assertFalse(ready)
    self.assertEqual(expected_accel, 0.0)
    self.assertEqual(expected_curvature, 0.0)
    self.assertTrue(math.isfinite(jerk))

    for _ in range(response.delay_frames - 2):
      self.assertFalse(response.update(0.5, 0.005)[3])
    self.assertTrue(response.update(0.5, 0.005)[3])


class TestTorqueResponseGates(unittest.TestCase):
  def test_upstream_response_port_stays_disabled_for_all_profiles(self):
    configurations = (
      (ToadPilotConfig(profile="adaptive"), True, False, False, True),
      (ToadPilotConfig(profile="full"), True, False, False, True),
      (ToadPilotConfig(profile="observe"), True, False, False, True),
      (ToadPilotConfig(enabled=False), True, False, False, True),
      (ToadPilotConfig(), False, False, False, True),
      (ToadPilotConfig(), True, True, False, True),
      (ToadPilotConfig(), True, False, True, True),
      (ToadPilotConfig(), True, False, False, False),
    )
    for args in configurations:
      with self.subTest(args=args):
        self.assertFalse(use_toadpilot_upstream_response(*args))

  def test_model_validation_is_confined_to_existing_nnff_inputs(self):
    self.assertTrue(torque_model_data_good(model_data()))
    self.assertFalse(torque_model_data_good(None))
    self.assertFalse(torque_model_data_good(SimpleNamespace()))
    self.assertFalse(torque_model_data_good(SimpleNamespace(
      orientation=SimpleNamespace(x=[0.0] * (CONTROL_N - 1), y=[0.0] * CONTROL_N),
      acceleration=SimpleNamespace(y=[0.0] * CONTROL_N),
    )))
    self.assertFalse(torque_model_data_good(model_data(finite=False)))

  def test_low_speed_term_is_independent_of_kp(self):
    base_error = 0.2
    shaped_error = 0.5
    low_speed_contribution = shaped_error - base_error
    for kp in (0.5, 0.8, 1.0, 1.25, 2.0):
      normalized_error = kp_independent_low_speed_error(base_error, shaped_error, kp)
      self.assertAlmostEqual(kp * (normalized_error - base_error), low_speed_contribution)

    self.assertEqual(kp_independent_low_speed_error(base_error, shaped_error, 0.0), shaped_error)
    self.assertEqual(kp_independent_low_speed_error(base_error, shaped_error, math.nan), shaped_error)


class CamryControllerFixture(unittest.TestCase):
  def setUp(self):
    self.params_dir = TemporaryDirectory()
    self.params = Params(self.params_dir.name)

  def tearDown(self):
    self.params_dir.cleanup()

  def new_controller(self, interface):
    CP = CarInterface.get_params(
      CAR.TOYOTA_CAMRY_TSS2, gen_empty_fingerprint(), [], False, False, self.params, False,
    )
    controller = LatControlTorque(CP, interface())
    controller.toadpilot_assist.status_due = lambda _config: False
    return controller, VehicleModel(CP)

  @staticmethod
  def control_inputs():
    CS = SimpleNamespace(
      vEgo=20.0,
      aEgo=0.0,
      steeringAngleDeg=0.5,
      steeringRateDeg=0.0,
      steeringPressed=False,
      brakePressed=False,
    )
    live_params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
    llk = SimpleNamespace(
      angularVelocityCalibrated=SimpleNamespace(value=[0.0, 0.0, 0.0]),
      calibratedOrientationNED=SimpleNamespace(value=[0.0, 0.0, 0.0]),
    )
    return CS, live_params, llk

  def update_controller(self, controller, VM, toggles, model):
    CS, live_params, llk = self.control_inputs()
    return controller.update(
      True, CS, VM, live_params, False, 0.001, False, llk,
      model_data=model, frogpilot_toggles=toggles,
    )


class TestTorqueResponseOutputNeutrality(CamryControllerFixture):
  def test_disabled_observe_and_nnff_ignore_delayed_response(self):
    cases = (
      ("disabled", LinearTorqueInterface, controller_toggles(enabled=False), None),
      ("observe", LinearTorqueInterface, controller_toggles(profile="observe"), None),
      ("nnff", NeuralTorqueInterface, controller_toggles(), model_data()),
    )
    for name, interface, toggles, model in cases:
      with self.subTest(case=name):
        baseline, baseline_vm = self.new_controller(interface)
        injected, injected_vm = self.new_controller(interface)
        injected.delayed_lateral_response.update = lambda *_args: (100.0, 10.0, 1_000.0, True)

        baseline_output = self.update_controller(baseline, baseline_vm, toggles, model)
        injected_output = self.update_controller(injected, injected_vm, toggles, model)

        self.assertAlmostEqual(baseline_output[0], injected_output[0], places=12)
        self.assertAlmostEqual(baseline_output[2].error, injected_output[2].error, places=12)
        self.assertAlmostEqual(baseline_output[2].p, injected_output[2].p, places=12)
        self.assertAlmostEqual(baseline_output[2].f, injected_output[2].f, places=12)

  def test_linear_response_does_not_switch_on_model_array_gaps(self):
    complete, complete_vm = self.new_controller(LinearTorqueInterface)
    missing, missing_vm = self.new_controller(LinearTorqueInterface)
    toggles = controller_toggles()

    for _ in range(25):
      complete_output = self.update_controller(complete, complete_vm, toggles, model_data())
      missing_output = self.update_controller(missing, missing_vm, toggles, None)
      self.assertAlmostEqual(complete_output[0], missing_output[0], places=12)
      self.assertAlmostEqual(complete_output[2].error, missing_output[2].error, places=12)


class TestTorqueLearnerLocationValidity(CamryControllerFixture):
  def test_above_stock_can_torque_is_excluded_from_normalized_learning(self):
    self.assertFalse(toadpilot_extended_torque_sample(True, 1500))
    self.assertTrue(toadpilot_extended_torque_sample(True, 1501))
    self.assertTrue(toadpilot_extended_torque_sample(True, math.nan))
    self.assertFalse(toadpilot_extended_torque_sample(False, 1700))

  def test_localizer_validity_requires_all_consumed_signals(self):
    self.assertTrue(live_location_sample_valid(location_sample()))
    self.assertFalse(live_location_sample_valid(location_sample(valid=False)))
    self.assertFalse(live_location_sample_valid(location_sample(finite=False)))

    missing_axis = location_sample()
    missing_axis.angularVelocityCalibrated.value = [0.0, 0.0]
    self.assertFalse(live_location_sample_valid(missing_axis))

  def test_invalid_localizer_sample_never_reaches_learner_or_observer(self):
    CP = CarInterface.get_params(
      CAR.TOYOTA_CAMRY_TSS2, gen_empty_fingerprint(), [], False, False, self.params, False,
    )
    with patch("openpilot.selfdrive.locationd.torqued.Params", return_value=self.params):
      estimator = TorqueEstimator(CP)
    estimator.toadpilot_observe_enabled = True
    estimator.raw_points["steer_torque"].extend([0.75] * estimator.hist_len)

    with patch.object(estimator.filtered_points, "add_point") as add_point:
      estimator.handle_log(10.0, "liveLocationKalman", location_sample(valid=False))

    add_point.assert_not_called()
    self.assertFalse(estimator.toadpilot_collecting)
    self.assertEqual(estimator.toadpilot_observer.accepted_samples, 0)

  def test_authorized_raw_can_diagnostic_records_actual_extension_and_peak(self):
    CP = CarInterface.get_params(
      CAR.TOYOTA_CAMRY_TSS2, gen_empty_fingerprint(), [], False, False, self.params, False,
    )
    CP.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_TOADPILOT_HIGH_TORQUE
    with patch("openpilot.selfdrive.locationd.torqued.Params", return_value=self.params):
      estimator = TorqueEstimator(CP)

    def car_output(raw, state="extended", reason="curve_undertracking"):
      return SimpleNamespace(actuatorsOutput=SimpleNamespace(
        steer=-1.0, steerOutputCan=raw,
        toadPilotHighTorqueState=state, toadPilotHighTorqueReason=reason,
      ))

    estimator.handle_log(10.0, "carOutput", car_output(-1600))
    estimator.handle_log(10.0, "carState", SimpleNamespace(
      vEgo=10.0,
      steeringPressed=True,
      steeringTorqueEps=-1550.0,
      steeringRateDeg=42.5,
      steerFaultTemporary=True,
      steerFaultPermanent=False,
      steerFaultStatusInitial=21,
    ))
    active = estimator.toadpilot_torque_snapshot(inputs_valid=True, experimental_max_torque=1550)
    self.assertTrue(active["controller_authorized"])
    self.assertTrue(active["extension_active"])
    self.assertEqual(active["current_output_can"], -1600.0)
    self.assertEqual(active["peak_abs_output_can"], 1600.0)
    self.assertEqual(active["extended_output_samples"], 1)
    self.assertEqual(active["output_sample_semantics"], "torqued_latest_carOutput_at_20hz_not_CAN_frame_count")
    self.assertEqual(active["selected_max_torque"], 1550)
    self.assertEqual(active["extra_delta_up_per_10ms"], 5)
    self.assertEqual(active["controller_state"], "extended")
    self.assertEqual(active["controller_reason"], "curve_undertracking")
    self.assertEqual(active["last_extended_monotonic"], 10.0)
    self.assertTrue(active["car_state_valid"])
    self.assertEqual(active["current_eps_torque"], -1550.0)
    self.assertEqual(active["peak_abs_eps_torque"], 1550.0)
    self.assertEqual(active["peak_abs_steering_rate_deg"], 42.5)
    self.assertEqual(active["driver_override_samples"], 1)
    self.assertTrue(active["temporary_fault_seen"])
    self.assertFalse(active["permanent_fault_seen"])
    self.assertEqual(active["extended_window_car_state_samples"], 1)
    self.assertEqual(active["extended_window_peak_abs_eps_torque"], 1550.0)
    self.assertEqual(active["extended_window_peak_abs_steering_rate_deg"], 42.5)
    self.assertEqual(active["extended_window_driver_override_samples"], 1)
    self.assertTrue(active["extended_window_temporary_fault_seen"])
    self.assertEqual(active["extended_window_temporary_fault_statuses"], [21])
    self.assertFalse(active["extended_window_permanent_fault_seen"])

    # The EPS can report a fault after the controller has begun its 0.4-second
    # unwind. Keep that response attributed through the 0.5-second tail.
    estimator.handle_log(10.1, "carOutput", car_output(-1400))
    estimator.handle_log(10.1, "carState", SimpleNamespace(
      vEgo=10.0,
      steeringPressed=True,
      steeringTorqueEps=-1400.0,
      steeringRateDeg=90.0,
      steerFaultTemporary=False,
      steerFaultPermanent=True,
    ))
    inactive = estimator.toadpilot_torque_snapshot(inputs_valid=True)
    self.assertFalse(inactive["extension_active"])
    self.assertEqual(inactive["current_output_can"], -1400.0)
    self.assertEqual(inactive["peak_abs_output_can"], 1600.0)
    self.assertEqual(inactive["extended_output_samples"], 1)
    self.assertTrue(inactive["permanent_fault_seen"])
    self.assertTrue(inactive["extended_window_permanent_fault_seen"])
    self.assertEqual(inactive["extended_window_car_state_samples"], 2)

    # An unrelated state sample after the tail remains a whole-route metric,
    # but cannot inflate the extension-window peak or sample count.
    estimator.handle_log(11.0, "carOutput", car_output(-1400))
    estimator.handle_log(11.0, "carState", SimpleNamespace(
      vEgo=10.0,
      steeringPressed=False,
      steeringTorqueEps=-1450.0,
      steeringRateDeg=120.0,
      steerFaultTemporary=False,
      steerFaultPermanent=False,
    ))
    after_tail = estimator.toadpilot_torque_snapshot(inputs_valid=True)
    self.assertEqual(after_tail["extended_window_car_state_samples"], 2)
    self.assertEqual(after_tail["extended_window_peak_abs_steering_rate_deg"], 90.0)
    self.assertEqual(after_tail["selected_max_torque"], 1550)

    stale = estimator.toadpilot_torque_snapshot(inputs_valid=False)
    self.assertFalse(stale["extension_active"])
    self.assertFalse(stale["inputs_valid"])
    self.assertFalse(stale["car_state_valid"])
    self.assertIsNone(stale["current_eps_torque"])

  def test_stock_controller_diagnostic_never_labels_extension(self):
    CP = CarInterface.get_params(
      CAR.TOYOTA_CAMRY_TSS2, gen_empty_fingerprint(), [], False, False, self.params, False,
    )
    with patch("openpilot.selfdrive.locationd.torqued.Params", return_value=self.params):
      estimator = TorqueEstimator(CP)
    estimator.update_toadpilot_torque_diagnostic(10.0, 1700)
    status = estimator.toadpilot_torque_snapshot()
    self.assertFalse(status["controller_authorized"])
    self.assertFalse(status["extension_active"])
    self.assertFalse(status["output_valid"])


if __name__ == "__main__":
  unittest.main()
