import tempfile
import unittest

from openpilot.common.params import ParamKeyType, Params


class TestInternalParamsPersistence(unittest.TestCase):
  def test_user_settings_survive_manager_start_cleanup(self):
    with tempfile.TemporaryDirectory() as params_path:
      params = Params(params_path)
      params.put("ToadPilotConfig", '{"profile":"full"}')
      params.put("StartAccelRamp", "0.50")
      params.put("ToadPilotTorqueCheckpoint", '{"peak":1650}')
      params.put("ToadPilotTorqueLastRoute", '{"peak":1700}')
      params.put("ToadPilotStopGoLastRoute", '{"last":"controls_inactive"}')

      params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)

      self.assertEqual(params.get("ToadPilotConfig"), b'{"profile":"full"}')
      self.assertEqual(params.get("StartAccelRamp"), b"0.50")
      self.assertEqual(params.get("ToadPilotTorqueCheckpoint"), b'{"peak":1650}')
      self.assertEqual(params.get("ToadPilotTorqueLastRoute"), b'{"peak":1700}')
      self.assertEqual(params.get("ToadPilotStopGoLastRoute"), b'{"last":"controls_inactive"}')

  def test_explicit_full_clear_still_removes_user_settings(self):
    with tempfile.TemporaryDirectory() as params_path:
      params = Params(params_path)
      params.put("ToadPilotConfig", '{"profile":"full"}')

      params.clear_all()

      self.assertIsNone(params.get("ToadPilotConfig"))

  def test_runtime_status_is_not_preserved(self):
    with tempfile.TemporaryDirectory() as params_path:
      params = Params(params_path)
      for key in ("ToadPilotStatus", "ToadPilotSteeringStatus", "ToadPilotStopGoStatus", "ToadPilotNavigationStatus",
                  "ToadPilotTuneStatus", "ToadPilotTorqueStatus"):
        params.put(key, "runtime")

      params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)

      for key in ("ToadPilotStatus", "ToadPilotSteeringStatus", "ToadPilotStopGoStatus", "ToadPilotNavigationStatus",
                  "ToadPilotTuneStatus", "ToadPilotTorqueStatus"):
        self.assertIsNone(params.get(key))


if __name__ == "__main__":
  unittest.main()
