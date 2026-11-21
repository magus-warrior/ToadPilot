import pytest

from cereal import car
from openpilot.common.params import Params
from openpilot.selfdrive.frogpilot import frogpilot_variables as settings


@pytest.mark.parametrize("make, tuning, force_on, force_off, expect_custom, fingerprint", [
  ("toyota", "torque", False, False, False, ""),
  ("hyundai", "torque", False, False, False, ""),
  ("toyota", "torque", False, True, True, ""),
  ("gm", "torque", False, False, True, ""),
  ("gm", "torque", True, False, False, ""),
  ("toyota", "pid", False, False, False, ""),
  ("toyota", "torque", False, False, True, "TOYOTA_CAMRY_TSS2"),
])
def test_saved_torque_overrides_follow_auto_tune(make, tuning, force_on, force_off, expect_custom, fingerprint, tmp_path, monkeypatch):
  for name in ("params", "params_default", "params_memory"):
    path = tmp_path / name
    path.mkdir()
    monkeypatch.setattr(settings, name, Params(str(path)))

  variables = settings.FrogPilotVariables()
  params = settings.params
  cp = car.CarParams.new_message(carName=make, carFingerprint=fingerprint, openpilotLongitudinalControl=True)
  cp.lateralTuning.init(tuning)
  params.put("CarParamsPersistent", cp.to_bytes())
  params.put_bool("AdvancedLateralTune", True)
  params.put_bool("ForceAutoTune", force_on)
  params.put_bool("ForceAutoTuneOff", force_off)
  params.put("SteerLatAccelStock", "2.354832")
  params.put("SteerLatAccel", "2.37")
  params.put("SteerFrictionStock", "0.118436")
  params.put("SteerFriction", "0.13")

  variables.update("stock", started=False)
  toggles = settings.get_frogpilot_toggles()
  assert toggles.use_custom_lat_accel_factor is expect_custom
  assert toggles.use_custom_steer_friction is expect_custom
