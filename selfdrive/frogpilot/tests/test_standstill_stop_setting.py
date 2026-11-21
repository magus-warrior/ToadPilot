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
