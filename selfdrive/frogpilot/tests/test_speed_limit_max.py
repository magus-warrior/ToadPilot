import os
import time
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest
from cereal import car, messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.drive_helpers import VCruiseHelper, V_CRUISE_MAX, V_CRUISE_UNSET, posted_cruise_target
from openpilot.selfdrive.frogpilot.controls.lib import speed_limit_controller as slc
from openpilot.selfdrive.toad_modeld.starpilot_controls.speed_limit_adapter import SpeedLimitController as StarPilotSpeedLimit
from openpilot.selfdrive.toad_modeld.starpilot_controls.vcruise import get_active_slc_control_target


def plan(mph, offset=5, source='Dashboard'):
  p = messaging.new_message('frogpilotPlan').frogpilotPlan
  p.slcSpeedLimit = p.unconfirmedSlcSpeedLimit = mph * CV.MPH_TO_MS
  p.slcSpeedLimitOffset = offset * CV.MPH_TO_MS
  p.slcSpeedLimitSource = source
  return p


@pytest.mark.parametrize('pcm', [False, True])
@pytest.mark.parametrize('initial', [30, 40, 50])
def test_initial_max_and_new_zones(pcm, initial):
  cp = car.CarParams.new_message(pcmCruise=pcm, openpilotLongitudinalControl=True)
  c = VCruiseHelper(cp)
  cs = car.CarState.new_message(canValid=True)
  cs.cruiseState.available = cs.cruiseState.enabled = True
  cs.cruiseState.speed = (initial - 1) * CV.MPH_TO_MS
  cs.cruiseState.speedCluster = initial * CV.MPH_TO_MS
  c.v_cruise_kph = initial * CV.MPH_TO_KPH
  toggles = NS(speed_limit_controller=True, custom_cruise_increase=1, custom_cruise_increase_long=5, set_speed_offset=0)
  def tick(p):
    c.update_v_cruise(cs, True, False, False, toggles, posted_cruise_target(p, False))
    return round(c.v_cruise_cluster_kph * CV.KPH_TO_MPH, 1)
  assert tick(plan(35)) == max(40, initial)
  assert tick(plan(35, source="Map Data")) == max(40, initial)
  assert tick(plan(0, source="None")) == max(40, initial)
  assert tick(plan(25)) == 30
  assert tick(plan(55, 10)) == 65
  assert tick(plan(45, -2)) == 43
  if pcm:
    assert c.v_cruise_kph * CV.KPH_TO_MPH == pytest.approx(42, abs=.05)
    cs.buttonEvents = [car.CarState.ButtonEvent(type="decelCruise", pressed=True)]
    assert tick(plan(45, -2)) == 43
    cs.buttonEvents = [car.CarState.ButtonEvent(type="decelCruise", pressed=False)]
    cs.cruiseState.speedCluster -= CV.MPH_TO_MS
    cs.cruiseState.speed -= CV.MPH_TO_MS
    assert tick(plan(45, -2)) == 42
    cs.buttonEvents = []
    assert tick(None) == 42
    toggles.speed_limit_controller = False
    assert tick(plan(60)) == initial - 1
  assert posted_cruise_target(plan(80, 99), False)[1] == V_CRUISE_MAX
  cs.cruiseState.available = False
  tick(plan(65))
  assert c.v_cruise_kph == V_CRUISE_UNSET


@pytest.mark.parametrize("metric, unit", [(False, CV.MPH_TO_MS), (True, CV.KPH_TO_MS)])
def test_pcm_catching_up_or_resuming_does_not_add_the_limit_twice(metric, unit):
  helper = VCruiseHelper(car.CarParams.new_message(pcmCruise=True, openpilotLongitudinalControl=True))
  cs = car.CarState.new_message(canValid=True)
  cs.cruiseState.available = cs.cruiseState.enabled = True
  toggles = NS(speed_limit_controller=True, custom_cruise_increase=1, custom_cruise_increase_long=5, set_speed_offset=0)
  sign = plan(40 * unit * CV.MS_TO_MPH, 10 * unit * CV.MS_TO_MPH)

  def update(stock, cluster, posted):
    cs.cruiseState.speed, cs.cruiseState.speedCluster = stock * unit, cluster * unit
    helper.update_v_cruise(cs, True, metric, False, toggles, posted_cruise_target(posted, metric))
    return tuple(round(kph * CV.KPH_TO_MS / unit) for kph in (helper.v_cruise_cluster_kph, helper.v_cruise_kph))

  assert update(19, 20, sign) == (50, 49)
  for stock, cluster, posted in ((49, 20, sign), (49, 50, sign), (79, 80, None), (19, 20, None)):
    assert update(stock, cluster, posted) == (50, 49)
  cs.canValid = False
  assert update(79, 80, None) == (50, 49)
  cs.canValid = True
  assert update(79, 80, sign) == (50, 49)
  cs.buttonEvents = [car.CarState.ButtonEvent(type="accelCruise", pressed=True)]
  assert update(79, 80, sign) == (50, 49)
  cs.buttonEvents = [car.CarState.ButtonEvent(type="accelCruise", pressed=False)]
  assert update(80, 81, sign) == (51, 50)
  assert update(80, 81, sign) == (51, 50)
  cs.buttonEvents = []
  assert update(80, 81, plan(30 * unit * CV.MS_TO_MPH, 5 * unit * CV.MS_TO_MPH)) == (35, 34)
  toggles.speed_limit_controller = False
  assert update(80, 81, None) == (81, 80)


def test_mismatched_pcm_and_cluster_at_activation():
  helper = VCruiseHelper(car.CarParams.new_message(pcmCruise=True, openpilotLongitudinalControl=True))
  cs = car.CarState.new_message(canValid=True)
  cs.cruiseState.available = cs.cruiseState.enabled = True
  cs.cruiseState.speed, cs.cruiseState.speedCluster = 50 * CV.MPH_TO_MS, 80 * CV.MPH_TO_MS
  helper.update_v_cruise(cs, True, False, False, NS(speed_limit_controller=True), posted_cruise_target(plan(40, 10), False))
  assert helper.v_cruise_kph * CV.KPH_TO_MPH == pytest.approx(50, abs=.05)
  assert helper.v_cruise_cluster_kph * CV.KPH_TO_MPH == pytest.approx(50, abs=.05)


@pytest.fixture
def camry():
  cp = car.CarParams.new_message(carName="toyota", pcmCruise=True, openpilotLongitudinalControl=True)
  helper = VCruiseHelper(cp)
  cs = car.CarState.new_message(canValid=True, vEgo=42 * CV.MPH_TO_MS)
  cs.cruiseState.available = cs.cruiseState.enabled = True
  cs.cruiseState.speed = cs.cruiseState.speedCluster = 80 * CV.MPH_TO_MS
  toggles = NS(speed_limit_controller=True, custom_cruise_increase=1, custom_cruise_increase_long=5, set_speed_offset=0)

  def tick(sign=None, *, engaged=True, metric=False):
    helper.update_v_cruise(cs, engaged, metric, sign is not None and sign.speedLimitChanged,
                           toggles, posted_cruise_target(sign, metric))
    units = 1. if metric else CV.KPH_TO_MPH
    return tuple(round(speed * units, 1) for speed in (helper.v_cruise_kph, helper.v_cruise_cluster_kph))

  return cs, toggles, tick


@pytest.mark.parametrize("metric", [False, True])
@pytest.mark.parametrize("already_engaged", [False, True])
def test_camry_sign_owns_first_max_and_resume(camry, metric, already_engaged):
  cs, toggles, tick = camry
  to_mph = CV.KPH_TO_MPH if metric else 1.
  sign = plan(40 * to_mph, 10 * to_mph)
  toggles.speed_limit_controller = False
  assert tick()[0] == 80
  toggles.speed_limit_controller = True
  assert tick(sign, engaged=already_engaged, metric=metric) == (50, 50)
  assert tick(sign, metric=metric) == (50, 50)
  cs.cruiseState.speed = 0.
  assert tick(metric=metric) == (50, 50)
  cs.cruiseState.enabled = False
  assert tick(engaged=False, metric=metric) == (50, 50)
  cs.cruiseState.speed = 80 * CV.MPH_TO_MS
  cs.cruiseState.enabled = True
  assert tick(metric=metric) == (50, 50)
  cs.canValid = False
  assert tick(plan(75), metric=metric) == (50, 50)
  cs.canValid = True
  cs.buttonEvents = [car.CarState.ButtonEvent(type="accelCruise", pressed=True)]
  assert tick(sign, metric=metric) == (50, 50)
  cs.buttonEvents = [car.CarState.ButtonEvent(type="accelCruise", pressed=False)]
  assert tick(sign, metric=metric) == (51, 51)
  cs.buttonEvents = []
  assert tick(plan(30 * to_mph, 5 * to_mph), metric=metric) == (35, 35)
  assert tick(plan(55 * to_mph, 5 * to_mph), metric=metric) == (60, 60)
  toggles.speed_limit_controller = False
  assert tick() == (80, 80)


@pytest.mark.parametrize("road_mph", [0, 42])
def test_camry_waits_for_first_confirmed_sign_at_road_speed(camry, road_mph):
  cs, toggles, tick = camry
  toggles.speed_limit_controller = False
  assert tick() == (80, 80)
  toggles.speed_limit_controller = True
  cs.canValid = False
  assert tick() == (0, 0)
  cs.canValid = True
  cs.vEgo = road_mph * CV.MPH_TO_MS
  assert tick(engaged=False) == (road_mph, road_mph)
  # A fallback to set speed and a pending sign are both ineligible.
  assert tick(plan(80, source="None")) == (road_mph, road_mph)
  cs.vEgo = 61 * CV.MPH_TO_MS
  pending = plan(40, 10)
  pending.speedLimitChanged = True
  cs.buttonEvents = [car.CarState.ButtonEvent(type="accelCruise", pressed=True)]
  assert tick(pending) == (road_mph, road_mph)
  pending.speedLimitChanged = False
  assert tick(pending) == (50, 50)
  cs.buttonEvents = [car.CarState.ButtonEvent(type="accelCruise", pressed=False)]
  assert tick(pending) == (50, 50)
  cs.cruiseState.speedCluster = 99 * CV.MPH_TO_MS
  assert tick() == (50, 50)


@pytest.mark.parametrize("native", [False, True], ids=["legacy", "starpilot"])
def test_camry_sign_max_caps_an_older_plan_at_the_solver(tmp_path, native):
  from openpilot.selfdrive.frogpilot.tests.test_new_upstream_features import planner_fixture
  from openpilot.selfdrive.frogpilot.tests.test_starpilot_planner import planner_setup

  if native:
    planner, sm, toggles = planner_setup(tmp_path)
  else:
    planner, sm, toggles = planner_fixture()
    planner.CP.carName, planner.CP.pcmCruise = "toyota", True
  toggles.speed_limit_controller = True
  sm["carState"].vEgo = 42 * CV.MPH_TO_MS
  with patch.object(planner.mpc, "update", wraps=planner.mpc.update) as update:
    for max_kph, plan_mph, expected_mph in [(50 * CV.MPH_TO_KPH, 80, 50), (50 * CV.MPH_TO_KPH, 35, 35),
                                           (V_CRUISE_UNSET, 80, 42), (0, 80, 0), (50 * CV.MPH_TO_KPH, float("nan"), 42)]:
      sm["controlsState"].vCruise = max_kph
      sm["frogpilotPlan"].vCruise = plan_mph * CV.MPH_TO_MS
      planner.update(False, sm, toggles)
      solver_speed = update.call_args.args[1 if native else 2]
      assert solver_speed * CV.MS_TO_MPH == pytest.approx(expected_mph)


def test_accepted_cap_survives_lost_source_with_high_cruise(tmp_path, monkeypatch):
  monkeypatch.setattr(slc, "params", Params(str(tmp_path / "params")))
  ctrl = slc.SpeedLimitController()
  cfg = NS(is_metric=False, slc_fallback_set_speed=True, speed_limit_offset3=10 * CV.MPH_TO_MS)
  high_cruise = 80 * CV.MPH_TO_MS
  limit = 40 * CV.MPH_TO_MS
  assert ctrl.resolve_accepted_limit(high_cruise, cfg, True) == (high_cruise, 0.)
  ctrl.source, ctrl.desired_speed_limit = "Dashboard", limit
  accepted = ctrl.resolve_accepted_limit(limit, cfg, True)
  assert sum(accepted) * CV.MS_TO_MPH == pytest.approx(50.)
  ctrl.source, ctrl.desired_speed_limit = "None", high_cruise
  for _ in range(4):
    assert ctrl.resolve_accepted_limit(high_cruise, cfg, True) == accepted
  ctrl.source, ctrl.desired_speed_limit, ctrl.speed_limit_changed = "Map Data", 70 * CV.MPH_TO_MS, True
  ctrl.resolve_accepted_limit(limit, cfg, True)
  ctrl.source = "None"
  assert ctrl.resolve_accepted_limit(high_cruise, cfg, True) == accepted
  assert ctrl.resolve_accepted_limit(high_cruise, cfg, False) == (high_cruise, 0.)
  assert ctrl.resolve_accepted_limit(high_cruise, cfg, True) == (high_cruise, 0.)


def test_starpilot_dropout_holds_commanded_cap_until_next_sign(tmp_path, monkeypatch):
  monkeypatch.setattr(slc, "params", Params(str(tmp_path / "params")))
  delegate = slc.SpeedLimitController()
  planner = NS(parent=NS(frogpilot_vcruise=NS(slc=delegate)), params_memory=Params(str(tmp_path / "mem")))
  adapter = StarPilotSpeedLimit(NS(starpilot_planner=planner))
  adapter.starpilot_toggles = cfg = NS(
    is_metric=False, slc_fallback_set_speed=True, slc_fallback_previous_speed_limit=False, slc_fallback_experimental_mode=False,
    speed_limit_confirmation_higher=False, speed_limit_confirmation_lower=False, speed_limit_priority_highest=False,
    speed_limit_priority_lowest=False, speed_limit_priority1="Dashboard", speed_limit_priority2="Map Data",
    speed_limit_priority3="Navigation", speed_limit_offset3=10 * CV.MPH_TO_MS, speed_limit_offset2=5 * CV.MPH_TO_MS,
  )
  sm = {"selfdriveState": NS(enabled=True, state="enabled"), "starpilotCarState": NS(accelPressed=False, decelPressed=False),
        "carState": NS(gasPressed=False)}
  cruise = 80 * CV.MPH_TO_MS
  for mph, expected in [(0, 80), (40, 50), (0, 50), (0, 50), (30, 35), (0, 35)]:
    delegate.update(mph * CV.MPH_TO_MS, True, 0., cruise, 15., cfg, None)
    adapter.update(cruise, 0., 15., 0., sm, active=True, display_only=False)
    target = get_active_slc_control_target(True, adapter.target, adapter.offset, adapter.overridden_speed, 0.)
    assert target * CV.MS_TO_MPH == pytest.approx(expected)
    assert delegate.source == ("Dashboard" if mph else "None")


def test_equivalent_signs_across_sources():
  dashboard = plan(25)
  navigation = plan(40 * CV.KPH_TO_MPH, source="Navigation")
  assert posted_cruise_target(dashboard, False) == posted_cruise_target(navigation, False)
  assert posted_cruise_target(dashboard, True) == posted_cruise_target(navigation, True)


def test_pending_denied_fallback_and_bad_data():
  p = plan(35)
  p.speedLimitChanged = True
  assert posted_cruise_target(p, False) is None
  p.speedLimitChanged = False
  p.unconfirmedSlcSpeedLimit = 20
  assert posted_cruise_target(p, False) is None
  assert posted_cruise_target(plan(50, source="None"), False) is None
  for value in (0, float("nan"), float("inf")):
    assert posted_cruise_target(plan(value), False) is None
  assert posted_cruise_target(plan(35, float("nan")), False) is None


def test_map_expiry_fallback_and_offset_bands(tmp_path, monkeypatch):
  memory = Params(str(tmp_path / "mem"))
  monkeypatch.setattr(slc, "params_memory", memory)
  monkeypatch.setattr(slc, "params", Params(str(tmp_path / "params")))
  ctrl = slc.SpeedLimitController()
  cfg = NS(is_metric=False, speed_limit_priority1="Map Data", speed_limit_priority2="Dashboard",
           speed_limit_priority3="Navigation", speed_limit_priority_highest=False, speed_limit_priority_lowest=False,
           slc_fallback_set_speed=True, slc_fallback_previous_speed_limit=False, slc_fallback_experimental_mode=False,
           map_speed_lookahead_higher=0, map_speed_lookahead_lower=0)
  for i in range(1, 8):
    setattr(cfg, f"speed_limit_offset{i}", i)
  memory.put("MapSpeedLimit", "15")
  fix = {"latitude": 37., "longitude": -122.}
  def update(dash, nav):
    ctrl.update(dash, True, nav, 19., 12., cfg, fix)
    return ctrl.source, ctrl.desired_speed_limit
  assert update(10., 12.) == ("Map Data", 15.)
  old = time.time() - 10
  os.utime(memory.get_param_path("MapSpeedLimit"), (old, old))
  assert update(10., 12.) == ("Dashboard", 10.)
  assert update(0., 12.) == ("Navigation", 12.)
  assert update(0., 0.) == ("None", 19.)
  assert ctrl.get_offset(19., cfg) == 0
  assert ctrl.previous_speed_limit == 12.
  assert update(0., 12.) == ("Navigation", 12.)
  assert ctrl.speed_limit_changed
  assert [ctrl.get_offset(n * CV.MPH_TO_MS, cfg) for n in (24, 25, 34, 35, 54, 55, 75)] == [1, 2, 2, 3, 4, 5, 7]
  cfg.is_metric = True
  assert [ctrl.get_offset(n * CV.KPH_TO_MS, cfg) for n in (29, 30, 49, 50, 99, 100, 140)] == [1, 2, 2, 3, 5, 6, 7]


@pytest.mark.parametrize("fallback_mph", [19, 80])
def test_camry_keeps_sign_and_offset_through_disengaged_source_gap(tmp_path, monkeypatch, fallback_mph):
  monkeypatch.setattr(slc, "params", Params(str(tmp_path / "params")))
  controller = slc.SpeedLimitController()
  cfg = NS(car_model="TOYOTA_CAMRY_TSS2", speed_limit_controller=True, is_metric=False,
           slc_fallback_set_speed=True, speed_limit_offset3=5 * CV.MPH_TO_MS)
  sign = 40 * CV.MPH_TO_MS
  fallback = fallback_mph * CV.MPH_TO_MS
  controller.source = "Dashboard"
  controller.desired_speed_limit = sign
  assert sum(controller.resolve_accepted_limit(sign, cfg, True)) * CV.MS_TO_MPH == pytest.approx(45)

  controller.source = "None"
  for engaged in (False, True):
    held = controller.resolve_accepted_limit(fallback, cfg, engaged)
    assert sum(held) * CV.MS_TO_MPH == pytest.approx(45)
  cfg.speed_limit_controller = False
  assert controller.resolve_accepted_limit(fallback, cfg, False) == (fallback, 0.)
