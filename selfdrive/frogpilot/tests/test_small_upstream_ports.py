import math
import unittest

from types import SimpleNamespace

from openpilot.selfdrive.car.toyota.carstate import CarState, get_acc_fault_states
from openpilot.selfdrive.car.toyota.values import CAR, ToyotaFlags
from openpilot.selfdrive.controls.lib.longitudinal_planner import (ALLOW_THROTTLE_THRESHOLD,
                                                                   MIN_ALLOW_THROTTLE_SPEED,
                                                                   planner_required_services,
                                                                   should_allow_throttle)
from openpilot.system.hardware.hardwared import (is_thermally_blocked_onroad,
                                                 thermal_fan_speed_percent_desired)


class TestAllowThrottleThreshold(unittest.TestCase):
  def test_probability_boundary_above_creep_speed(self):
    v_ego = math.nextafter(MIN_ALLOW_THROTTLE_SPEED, math.inf)

    self.assertEqual(ALLOW_THROTTLE_THRESHOLD, 0.4)
    self.assertFalse(should_allow_throttle(ALLOW_THROTTLE_THRESHOLD, v_ego))
    self.assertTrue(should_allow_throttle(math.nextafter(ALLOW_THROTTLE_THRESHOLD, math.inf), v_ego))

  def test_launch_region_is_unchanged(self):
    # The probability threshold is bypassed at and below 2.5 m/s, so this port
    # cannot make a launch from standstill more aggressive.
    self.assertEqual(MIN_ALLOW_THROTTLE_SPEED, 2.5)
    self.assertTrue(should_allow_throttle(0.0, 0.0))
    self.assertTrue(should_allow_throttle(0.0, MIN_ALLOW_THROTTLE_SPEED))


class TestToyotaTemporaryAccFault(unittest.TestCase):
  @staticmethod
  def parser_cp():
    return SimpleNamespace(
      carFingerprint=CAR.TOYOTA_CAMRY_TSS2,
      flags=int(ToyotaFlags.TSS2 | ToyotaFlags.NO_STOP_TIMER | ToyotaFlags.NO_DSU),
      enableGasInterceptor=False,
      enableBsm=False,
      enableDsu=False,
    )

  def test_camry_parser_exposes_temporary_acc_fault(self):
    parser = CarState.get_can_parser(self.parser_cp())
    self.assertIn("TEMP_ACC_FAULTED", parser.vl["PCM_CRUISE_SM"])

  def test_temporary_fault_maps_only_to_noncritical_fault(self):
    cp = SimpleNamespace(vl={
      "PCM_CRUISE_2": {"ACC_FAULTED": 0},
      "PCM_CRUISE_SM": {"TEMP_ACC_FAULTED": 1},
    })
    self.assertEqual(get_acc_fault_states(cp), (False, True))

    cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] = 1
    cp.vl["PCM_CRUISE_SM"]["TEMP_ACC_FAULTED"] = 0
    self.assertEqual(get_acc_fault_states(cp), (True, False))


class TestThermalFanOverride(unittest.TestCase):
  def test_block_requires_bad_temperature_and_ignition(self):
    self.assertFalse(is_thermally_blocked_onroad(True, True, True))
    self.assertFalse(is_thermally_blocked_onroad(False, False, False))
    self.assertTrue(is_thermally_blocked_onroad(False, True, True))
    self.assertTrue(is_thermally_blocked_onroad(True, False, True))

  def test_block_forces_full_fan_and_normal_path_is_preserved(self):
    self.assertEqual(thermal_fan_speed_percent_desired(37, True), 100)
    self.assertEqual(thermal_fan_speed_percent_desired(37, False), 37)


class TestPlannerInputValidity(unittest.TestCase):
  BASE_SERVICES = {'carControl', 'carState', 'controlsState', 'modelV2', 'frogpilotCarState', 'frogpilotPlan'}

  def test_required_services_follow_active_mode(self):
    for experimental_mode in (False, True):
      for radarless_model in (False, True):
        with self.subTest(experimental_mode=experimental_mode, radarless_model=radarless_model):
          services = set(planner_required_services(experimental_mode, radarless_model))
          expected = set(self.BASE_SERVICES)
          if not experimental_mode:
            expected.add('liveParameters')
          if not radarless_model:
            expected.add('radarState')
          self.assertEqual(services, expected)


if __name__ == "__main__":
  unittest.main()
