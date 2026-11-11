#!/usr/bin/env python3
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, planner_required_services
import cereal.messaging as messaging

from openpilot.selfdrive.frogpilot.frogpilot_variables import get_frogpilot_toggles
from openpilot.selfdrive.toad_modeld.starpilot_response import response_enabled


def new_longitudinal_planner(CP, toggles):
  if response_enabled(toggles) and CP.openpilotLongitudinalControl:
    from openpilot.selfdrive.toad_modeld.starpilot_long.adapter import StarPilotPlanner
    return StarPilotPlanner(CP)
  return LongitudinalPlanner(CP)

def publish_ui_plan(sm, pm, longitudinal_planner, radarless_model):
  ui_send = messaging.new_message('uiPlan')
  required_services = planner_required_services(sm['controlsState'].experimentalMode, radarless_model)
  ui_send.valid = sm.all_checks(service_list=required_services)
  uiPlan = ui_send.uiPlan
  uiPlan.frameId = sm['modelV2'].frameId
  uiPlan.position.x = list(sm['modelV2'].position.x)
  uiPlan.position.y = list(sm['modelV2'].position.y)
  uiPlan.position.z = list(sm['modelV2'].position.z)
  uiPlan.accel = longitudinal_planner.a_desired_trajectory_full.tolist()
  pm.send('uiPlan', ui_send)

def plannerd_thread():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  with car.CarParams.from_bytes(params.get("CarParams", block=True)) as msg:
    CP = msg
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  longitudinal_planner = LongitudinalPlanner(CP)
  pm = messaging.PubMaster(['longitudinalPlan', 'uiPlan'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'liveParameters', 'radarState', 'modelV2',
                            'frogpilotCarState', 'frogpilotPlan'],
                           poll='modelV2', ignore_avg_freq=['radarState'])

  # FrogPilot variables
  frogpilot_toggles = get_frogpilot_toggles()
  longitudinal_planner = new_longitudinal_planner(CP, frogpilot_toggles)

  classic_model = frogpilot_toggles.classic_model
  radarless_model = frogpilot_toggles.radarless_model

  while True:
    sm.update()
    if sm.updated['modelV2']:
      longitudinal_planner.update(radarless_model, sm, frogpilot_toggles)
      longitudinal_planner.publish(classic_model, radarless_model, sm, pm, frogpilot_toggles)
      publish_ui_plan(sm, pm, longitudinal_planner, radarless_model)

    # Update FrogPilot parameters
    if sm['frogpilotPlan'].togglesUpdated:
      refreshed_toggles = get_frogpilot_toggles()
      if response_enabled(refreshed_toggles) != response_enabled(frogpilot_toggles):
        # Settings writes are parked-only; initialize the selected planner from
        # measured motion if a refreshed snapshot selects another implementation.
        longitudinal_planner = new_longitudinal_planner(CP, refreshed_toggles)
      frogpilot_toggles = refreshed_toggles

def main():
  plannerd_thread()


if __name__ == "__main__":
  main()
