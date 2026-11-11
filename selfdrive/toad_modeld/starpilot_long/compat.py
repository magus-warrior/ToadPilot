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
      carState=View(sm['carState'], vCruise=controls.vCruise),
      starpilotPlan=plan,
      starpilotCarState=View(sm['frogpilotCarState'], trafficModeEnabled=sm['frogpilotCarState'].trafficMode),
    )
    self.logMonoTime = sm.logMonoTime

  def __getitem__(self, name):
    return self.values[name] if name in self.values else self.original[name]

  def all_checks(self, service_list):
    mapping = {'selfdriveState': 'controlsState', 'starpilotPlan': 'frogpilotPlan', 'starpilotCarState': 'frogpilotCarState'}
    return self.original.all_checks(service_list=[mapping.get(name, name) for name in service_list])
