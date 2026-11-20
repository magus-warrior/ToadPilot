from types import SimpleNamespace as NS

import pytest
from openpilot.selfdrive.controls import radard


class Inputs(dict):
  def __init__(self, failed):
    super().__init__(modelV2=NS(velocity=NS(x=[]), leadsV3=[None, None]),
                     carState=NS(vEgo=2., standstill=False),
                     frogpilotPlan=NS(trafficModeBlend=.8, togglesUpdated=False))
    self.failed = failed
    self.recv_frame = {'carState': 0}
    self.logMonoTime = {'modelV2': 1, 'carState': 1}
    self.seen = {'modelV2': True}

  def all_checks(self, services):
    return self.failed not in services


@pytest.mark.parametrize('failed,errors,valid', [
  (None, [], True), ('frogpilotPlan', [], True), ('modelV2', [], False),
  ('carState', [], False), ('frogpilotCarState', [], False), (None, ['canError'], False),
])
def test_radar_uses_healthy_tracks_when_optional_plan_drops(monkeypatch, failed, errors, valid):
  radar = radard.RadarD(NS(classic_model=False, adjacent_lead_tracking=False), .05)
  inputs = Inputs(failed)
  blends = []
  def get_lead(*args, **kwargs):
    blends.append(args[-1])
    return {'status': False}
  monkeypatch.setattr(radard, 'get_lead', get_lead)
  radar.update(inputs, NS(points=[], errors=errors))
  assert radar.radar_state_valid == valid
  assert list(radar.radar_state.radarErrors) == errors
  assert blends == ([0., 0.] if failed == 'frogpilotPlan' else [.8, .8])
  radar.update(inputs, None)
  assert not radar.radar_state_valid
