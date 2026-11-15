"""Explicit parameter and UI adapters for pinned StarPilot controls."""
import json
import os
from pathlib import Path
from openpilot.common.params import UnknownKeyName

# Preserve the installed UI's CEStatus wire meanings; manual overrides are 1/2
# in both forks. Native automatic statuses are translated at this boundary.
CEStatus = dict(OFF=0, USER_DISABLED=1, USER_OVERRIDDEN=2, CURVATURE=8,
                LEAD=9, SIGNAL=5, SPEED=4, SPEED_LIMIT=13, STOP_LIGHT=11)


def is_manual_ce_status(status):
  return status in (1, 2)


def restore_persisted_ce_state(params, params_memory):
  # This fork has no persisted-mode setting. Respect its existing press toggle.
  return params_memory.get_int('CEStatus') if params_memory.manual_enabled else 0


class ParamsAdapter:
  """Bridge typed native values without changing the compiled legacy key registry.

  Native CSC learning lives in its own file. Existing user preferences and
  CEStatus continue to use the installed Params implementation.
  """
  NATIVE_KEYS = {'CurvatureData', 'CalibrationProgress', 'CalibratedLateralAcceleration'}

  def __init__(self, legacy, *, path=None):
    self.legacy = legacy
    self.path = Path(path) if path is not None else None
    self.values = {}
    self.manual_enabled = True
    if self.path is not None:
      try:
        saved = json.loads(self.path.read_text())
        if isinstance(saved, dict):
          self.values = {k: v for k, v in saved.items() if k in self.NATIVE_KEYS}
      except (OSError, ValueError):
        pass

  def get(self, key, block=False, encoding=None):
    if key in self.NATIVE_KEYS:
      return self.values.get(key)
    try:
      return self.legacy.get(key, block=block, encoding=encoding)
    except UnknownKeyName:
      return None

  def get_bool(self, key):
    try:
      return self.legacy.get_bool(key)
    except UnknownKeyName:
      return False

  def get_int(self, key, default=0):
    value = self.get(key)
    return int(float(value)) if value is not None else default

  def get_float(self, key):
    value = self.get(key)
    return float(value) if value is not None else 0.0

  def remove(self, key):
    self.legacy.remove(key)

  def put_int(self, key, value):
    self.legacy.put_int(key, value)

  def put_nonblocking(self, key, value):
    if key not in self.NATIVE_KEYS:
      self.legacy.put_nonblocking(key, json.dumps(value) if isinstance(value, (dict, list)) else str(value))
      return
    self.values[key] = value
    if self.path is not None:
      try:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.values, allow_nan=False))
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.path)
      except OSError:
        # Optional curve learning must never interrupt stop management if its
        # storage is unavailable. The current learned values remain in memory.
        pass
