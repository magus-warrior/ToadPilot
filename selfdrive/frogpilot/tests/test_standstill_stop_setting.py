import numpy as np
import pytest
from openpilot.selfdrive.frogpilot.tests.test_starpilot_controls import controls_setup, update
from openpilot.selfdrive.modeld.constants import ModelConstants

@pytest.mark.parametrize("seconds,sign,expected", [(0,False,False),(9,False,True),(0,True,True)])
def test_standstill_obeys_predicted_stop_setting(tmp_path, seconds, sign, expected):
  ctx, parent, _, sm, toggles, nav = controls_setup(tmp_path, speed=0., force=False)
  toggles.conditional_model_stop_time = seconds
  sm["carState"].standstill = True
  ctx.starpilot_vcruise.stop_sign_confirmed = sign
  sm["modelV2"].position.x = np.minimum(np.asarray(ModelConstants.T_IDXS), 1.).tolist()
  update(ctx, parent, sm, toggles, nav, 0., 100.)
  assert ctx.starpilot_cem.experimental_mode is expected


@pytest.mark.parametrize("forecast,expected", [("rolling", False), ("stopping", True)])
def test_moving_short_path_distinguishes_recovery_from_stop(tmp_path, forecast, expected):
  ctx, parent, _, sm, toggles, nav = controls_setup(tmp_path, speed=8., force=False)
  t = np.asarray(ModelConstants.T_IDXS)
  v = np.interp(t, [0., 3., 7., 10.], [8., 3., 5.5, 6.]) if forecast == "rolling" else np.maximum(8. - .85*t, 0.)
  x = np.r_[0., np.cumsum(.5 * (v[1:] + v[:-1]) * np.diff(t))]
  assert x[-1] < 8. * toggles.conditional_model_stop_time - ctx.starpilot_cem.STOP_LIGHT_ON_MARGIN
  model = sm["modelV2"]
  model.position.x = x.tolist()
  model.velocity.x = v.tolist()
  model.action.desiredAcceleration = -.8
  for frame in range(80):
    update(ctx, parent, sm, toggles, nav, 8., 100. + frame * .05)
  assert ctx.starpilot_cem.stop_light_detected is expected
  assert ctx.starpilot_cem.experimental_mode is expected
