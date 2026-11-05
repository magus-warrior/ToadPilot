import math
from types import SimpleNamespace
import unittest

from cereal import log
from openpilot.selfdrive.toad_modeld.action import combine_longitudinal, convert_action, convert_plan_action
from openpilot.selfdrive.toad_modeld.artifact import MODEL_ID, MODEL_VERSION
from openpilot.system.manager.process_config import run_classic_modeld, run_new_modeld, run_tinygrad_modeld, run_toad_modeld
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
import numpy as np


class TestModernModel(unittest.TestCase):
  def test_v11_plan_action_constant_acceleration_and_turn(self):
    from openpilot.selfdrive.toad_modeld.constants import ModelConstants, Plan
    t = np.asarray(ModelConstants.T_IDXS)
    plan = np.zeros((33, 15))
    plan[:, Plan.VELOCITY][:, 0] = 10 - 0.5 * t
    plan[:, Plan.ACCELERATION][:, 0] = -0.5
    plan[:, Plan.T_FROM_CURRENT_EULER][:, 2] = 0.02 * t
    plan[:, Plan.ORIENTATION_RATE][:, 2] = 0.02
    curvature, accel, stop = convert_plan_action(plan, 10, 0.3, 0.5)
    self.assertAlmostEqual(curvature, (1 - math.exp(-0.5)) * 0.002)
    self.assertAlmostEqual(accel, (1 - math.exp(-0.05 / 0.3)) * -0.5)
    self.assertFalse(stop)
    self.assertTrue(convert_plan_action(np.zeros((33, 15)), 0, 0.3, 0.5)[2])
    for invalid in (np.zeros((17, 15)), np.full((33, 15), np.nan)):
      with self.assertRaises(ValueError):
        convert_plan_action(invalid, 10, 0.3, 0.5)
    with self.assertRaises(ValueError):
      convert_plan_action(plan, 10, 0, 0.5)

  def test_v15_curvature_is_acceleration_scaled(self):
    alpha = 1 - math.exp(-0.05 / 0.1)
    for speed in (1, 10, 30):
      curvature, acceleration, stop = convert_action([2.0, 1.0], speed)
      self.assertAlmostEqual(curvature, alpha * 2.0 / speed ** 2)
      self.assertAlmostEqual(acceleration, 1 - math.exp(-0.05 / 0.3))
      self.assertFalse(stop)

  def test_stop_uses_raw_action_and_standstill_holds_curvature(self):
    curvature, acceleration, stop = convert_action([99, -0.2], 0.1, 0.005, 1.0)
    self.assertEqual(curvature, 0.005)
    self.assertGreater(acceleration, 0)
    self.assertTrue(stop)
    self.assertFalse(convert_action([0, 0.1], 0)[2])
    self.assertFalse(convert_action([0, -1], 0.3)[2])

  def test_invalid_actions_are_rejected(self):
    for value in (math.nan, math.inf, -math.inf):
      with self.assertRaises(ValueError):
        convert_action([value, 1], 15)
      with self.assertRaises(ValueError):
        convert_action([1, value], 15)

  def test_model_cannot_override_mpc_braking_or_stop(self):
    action = log.ModelDataV2.Action(desiredAcceleration=2, shouldStop=False, longitudinalValid=True)
    self.assertEqual(combine_longitudinal(-1, True, action, -3.5, 2), (-1, True))
    action.desiredAcceleration = -2
    self.assertEqual(combine_longitudinal(1, False, action, -3.5, 2), (-2, False))
    action.shouldStop = True
    self.assertEqual(combine_longitudinal(1, False, action, -3.5, 2), (-2, True))
    action.desiredAcceleration = -20
    self.assertEqual(combine_longitudinal(1, False, action, -3.5, 2), (-3.5, True))

  def test_legacy_action_is_not_mistaken_for_modern_acceleration(self):
    action = log.ModelDataV2.Action(desiredCurvature=0.01)
    with self.assertRaises(ValueError):
      combine_longitudinal(1, False, action, -3.5, 2)

  def test_schema_roundtrip_and_default_compatibility(self):
    message = log.Event.new_message()
    message.init('modelV2')
    message.modelV2.action.desiredCurvature = 0.01
    with log.Event.from_bytes(message.to_bytes()) as decoded:
      self.assertFalse(decoded.modelV2.action.longitudinalValid)
    message.clear_write_flag()
    message.modelV2.action.desiredAcceleration = -0.5
    message.modelV2.action.shouldStop = True
    message.modelV2.action.longitudinalValid = True
    with log.Event.from_bytes(message.to_bytes()) as decoded:
      self.assertEqual(decoded.modelV2.action.desiredAcceleration, -0.5)
      self.assertTrue(decoded.modelV2.action.shouldStop)
      self.assertTrue(decoded.modelV2.action.longitudinalValid)

  def test_only_one_model_process_is_selected(self):
    functions = (run_classic_modeld, run_new_modeld, run_tinygrad_modeld, run_toad_modeld)
    for name, version, classic, tiny, expected in (
      ('north-dakota', 'v2', True, False, 0),
      ('national-public-radio', 'v6', False, False, 1),
      ('not-too-shabby', 'v7', False, True, 2),
      (MODEL_ID, MODEL_VERSION, False, False, 3),
      ("toad-pop-v2", "v11", False, False, 3),
      ("toad-south-carolina", "v11", False, False, 3),
    ):
      toggles = SimpleNamespace(model=name, model_version=version)
      for started in (False, True):
        selected = [bool(f(started, None, None, classic, tiny, toggles)) for f in functions]
        self.assertEqual(selected, [bool(started and i == expected) for i in range(4)])

  def test_planner_uses_action_only_for_modern_experimental_mode(self):
    class State(dict):
      logMonoTime = {'modelV2': 1234}
      def all_checks(self, service_list):
        return True

    class Publisher:
      def send(self, service, message):
        self.message = message

    planner = LongitudinalPlanner.__new__(LongitudinalPlanner)
    planner.CP = SimpleNamespace(longitudinalActuatorDelay=0.2)
    planner.mpc = SimpleNamespace(solve_time=0.001, source='cruise')
    planner.v_desired_trajectory = np.full(CONTROL_N, 20.0)
    planner.a_desired_trajectory = np.zeros(CONTROL_N)
    planner.j_desired_trajectory = np.zeros(CONTROL_N)
    planner.lead_one = SimpleNamespace(status=False)
    planner.fcw = False
    planner.allow_throttle = True
    planner.model_accel_limits = (-3.5, 2.0)
    model = log.ModelDataV2.new_message()
    model.action = log.ModelDataV2.Action(desiredAcceleration=-1.2, shouldStop=True, longitudinalValid=True)
    pm = Publisher()
    for modern in (False, True):
      for experimental in (False, True):
        toggles = SimpleNamespace(model=MODEL_ID if modern else 'national-public-radio',
                                  model_version=MODEL_VERSION if modern else 'v6', vEgoStopping=0.05)
        sm = State(controlsState=SimpleNamespace(experimentalMode=experimental), modelV2=model)
        planner.publish(False, False, sm, pm, toggles)
        self.assertTrue(pm.message.valid)
        self.assertAlmostEqual(pm.message.longitudinalPlan.aTarget, -1.2 if modern and experimental else 0.0)
        self.assertEqual(pm.message.longitudinalPlan.shouldStop, modern and experimental)
    model.action.longitudinalValid = False
    planner.publish(False, False, sm, pm, toggles)
    self.assertFalse(pm.message.valid)


if __name__ == '__main__':
  unittest.main()
