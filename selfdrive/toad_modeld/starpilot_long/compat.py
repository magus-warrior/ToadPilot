"""Explicit legacy-schema adapters for the supported Camry TSS2 platform."""
from types import SimpleNamespace

from openpilot.selfdrive.toad_modeld.artifact import MODELS


def is_tinygrad_model_version(version):
  return version in {entry['version'] for entry in MODELS.values()}


class View:
  def __init__(self, original, **overrides):
    self.original = original
    self.overrides = overrides

  def __getattr__(self, name):
    if name in self.overrides:
      return self.overrides[name]
    return getattr(self.original, name)


def vehicle_view(cp):
  return View(cp, brand=cp.carName)


class StateView:
  def __init__(self, sm, *, plan):
    controls = sm['controlsState']
    self.original = sm
    self.values = dict(
      selfdriveState=controls,
      # Legacy radarState lead speeds use measured vEgo. Keep the planner in
      # that same speed frame; vEgoCluster includes a display-only correction.
      carState=View(sm['carState'], vCruise=controls.vCruise, vEgoCluster=sm['carState'].vEgo),
      starpilotPlan=plan,
      starpilotCarState=View(sm['frogpilotCarState'], trafficModeEnabled=sm['frogpilotCarState'].trafficMode),
    )
    self.logMonoTime = sm.logMonoTime

  def __getitem__(self, name):
    return self.values[name] if name in self.values else self.original[name]

  def all_checks(self, service_list):
    mapping = {'selfdriveState': 'controlsState', 'starpilotPlan': 'frogpilotPlan', 'starpilotCarState': 'frogpilotCarState'}
    return self.original.all_checks(service_list=[mapping.get(name, name) for name in service_list])


def get_force_stop_distance_bias(car_fingerprint):
  # The legacy message supplies the raw model distance. Do not add the native
  # Camry's six-metre forward calibration before it has been validated on this
  # pipeline: weak-action stop simulations cross the requested line with it.
  if str(car_fingerprint) == 'TOYOTA_CAMRY_TSS2':
    return 0.0
  from openpilot.selfdrive.toad_modeld.starpilot_long.vehicle_tunes import get_force_stop_distance_bias as native_bias
  return native_bias(car_fingerprint)
