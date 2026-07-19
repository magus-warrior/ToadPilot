from openpilot.common.params_pyx import Params as _Params, ParamKeyType, UnknownKeyName


class Params(_Params):
  # This is a stripped release checkout, so the native Params registry source is
  # unavailable. Keep the temporary local feature keys narrowly allowlisted until
  # they can be registered in common/params.cc in the source fork.
  _internal_keys = {
    "StartAccelRamp",
    "ToadPilotConfig",
    "ToadPilotResetTune",
    "ToadPilotSteeringResponseRequest",
    "ToadPilotStatus",
    "ToadPilotTorqueCheckpoint",
    "ToadPilotTorqueLastRoute",
    "ToadPilotTorqueStatus",
    "ToadPilotTuneStatus",
    "ToyotaPerformanceMode",  # Legacy prototype key; migration-status access only.
  }
  # The native registry in this stripped checkout does not know these keys, so
  # its transition cleanup deletes them as if they were unregistered. Preserve
  # only user settings plus compact torque checkpoint/route summaries; live
  # status and one-shot command keys must remain ephemeral.
  _persistent_internal_keys = {
    "StartAccelRamp",
    "ToadPilotConfig",
    "ToadPilotSteeringResponseRequest",
    "ToadPilotTorqueCheckpoint",
    "ToadPilotTorqueLastRoute",
    "ToyotaPerformanceMode",
  }

  @classmethod
  def _is_internal_key(cls, key):
    if isinstance(key, str):
      return key in cls._internal_keys
    if isinstance(key, bytes):
      try:
        return key.decode("utf-8") in cls._internal_keys
      except UnicodeDecodeError:
        return False

  def check_key(self, key):
    if self._is_internal_key(key):
      # The native Params methods assign this return value to a C++ string,
      # which requires bytes. Match the base implementation's contract.
      return key.encode() if isinstance(key, str) else key
    return super().check_key(key)

  def clear_all(self, tx_type=ParamKeyType.ALL):
    if tx_type == ParamKeyType.ALL:
      return super().clear_all(tx_type)

    saved_values = {key: self.get(key) for key in self._persistent_internal_keys}
    result = super().clear_all(tx_type)
    for key, value in saved_values.items():
      if value is not None and self.get(key) is None:
        self.put(key, value)
    return result
assert Params
assert ParamKeyType
assert UnknownKeyName

if __name__ == "__main__":
  import sys

  params = Params()
  key = sys.argv[1]
  assert params.check_key(key), f"unknown param: {key}"

  if len(sys.argv) == 3:
    val = sys.argv[2]
    print(f"SET: {key} = {val}")
    params.put(key, val)
  elif len(sys.argv) == 2:
    print(f"GET: {key} = {params.get(key)}")
