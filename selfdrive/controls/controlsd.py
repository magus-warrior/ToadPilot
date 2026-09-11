#!/usr/bin/env python3
import json
import os
import math
import time
import threading
from typing import SupportsFloat

import cereal.messaging as messaging

from cereal import car, custom, log
from msgq.visionipc import VisionIpcClient, VisionStreamType


from openpilot.common.conversions import Conversions as CV
from openpilot.common.git import get_short_branch
from openpilot.common.numpy_fast import clip
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper, DT_CTRL, DT_MDL
from openpilot.common.swaglog import cloudlog

from openpilot.selfdrive.car.car_helpers import get_car_interface, get_startup_event
from openpilot.selfdrive.car.gm.values import CC_ONLY_CAR, GMFlags
from openpilot.selfdrive.controls.lib.alertmanager import AlertManager, set_offroad_alert
from openpilot.selfdrive.controls.lib.drive_helpers import VCruiseHelper, clip_curvature
from openpilot.selfdrive.controls.lib.events import ET, EVENTS, Events
from openpilot.selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, should_stop_for_long_control
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel

from openpilot.system.hardware import HARDWARE

from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_acceleration import get_max_allowed_accel
from openpilot.selfdrive.frogpilot.frogpilot_variables import ERROR_LOGS_PATH, get_frogpilot_toggles, params_memory
from openpilot.selfdrive.frogpilot.toadpilot import (
  TOADPILOT_FINGERPRINT,
  TOADPILOT_STEERING_STATUS_KEY,
  TOADPILOT_STOP_GO_STATUS_KEY,
  ToadPilotLaneCentering,
  ToadPilotLaneContext,
  ToadPilotStraightLineSmoother,
  ToadPilotTorqueCommandTrace,
  config_from_toggles,
  extract_toadpilot_lane_context,
  toadpilot_steer_saturated,
)

SOFT_DISABLE_TIME = 3  # seconds
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1
CAMERA_OFFSET = 0.04
TOADPILOT_STATUS_TRANSITION_MIN_INTERVAL = 0.10

REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
TESTING_CLOSET = "TESTING_CLOSET" in os.environ
IGNORE_PROCESSES = {"loggerd", "encoderd", "statsd"}

ThermalStatus = log.DeviceState.ThermalStatus
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
Desire = log.Desire
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
FrogPilotButtonType = custom.FrogPilotCarState.ButtonEvent.Type
SafetyModel = car.CarParams.SafetyModel

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)
CSID_MAP = {"1": EventName.roadCameraError, "2": EventName.wideRoadCameraError, "0": EventName.driverCameraError}
ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.preEnabled, *ACTIVE_STATES)


def get_lateral_target_curvature(lat_active, measured_curvature, requested_curvature):
  """Keep the rate-limited command aligned with the steering wheel while lateral control is inactive."""
  return requested_curvature if lat_active else measured_curvature


def longitudinal_controls_inactive_reason(enabled, longitudinal_override, paused, openpilot_longitudinal,
                                          gas_pressed=False):
  if not openpilot_longitudinal:
    return "openpilot_longitudinal_unavailable"
  if not enabled:
    return "openpilot_disabled"
  if longitudinal_override:
    return "driver_accelerator_override" if gas_pressed else "longitudinal_override"
  if paused:
    return "frogpilot_pause"
  return "active"


def valid_calibrated_angular_velocity(llk, service_valid):
  """Return calibrated angular velocity only from a healthy localization sample."""
  if not service_valid:
    return []
  try:
    values = [float(value) for value in llk.angularVelocityCalibrated.value]
    valid = (llk.angularVelocityCalibrated.valid and llk.inputsOK and llk.sensorsOK and llk.posenetOK)
  except (AttributeError, TypeError, ValueError):
    return []
  if not valid or len(values) != 3 or not all(math.isfinite(value) for value in values):
    return []
  return values


def curvature_cap_alert_required(curvature_limited, controller_saturated, requested_curvature,
                                 actual_lateral_accel, v_ego):
  """Whether a sustained curvature cap leaves a meaningful raw path-tracking shortfall."""
  if not curvature_limited or not controller_saturated:
    return False

  clipped_speed = max(v_ego, MIN_LATERAL_CONTROL_SPEED)
  requested_lateral_accel = requested_curvature * clipped_speed ** 2
  undershooting = abs(requested_lateral_accel) / max(abs(actual_lateral_accel), 1e-3) > 1.2
  turning = abs(requested_lateral_accel) > 1.0
  return undershooting and turning


class Controls:
  def __init__(self, CI=None):
    self.params = Params()

    if CI is None:
      cloudlog.info("controlsd is waiting for CarParams")
      with car.CarParams.from_bytes(self.params.get("CarParams", block=True)) as msg:
        # TODO: this shouldn't need to be a builder
        self.CP = msg.as_builder()
      cloudlog.info("controlsd got CarParams")

      # Uses car interface helper functions, altering state won't be considered by card for actuation
      self.CI = get_car_interface(self.CP)
    else:
      self.CI, self.CP = CI, CI.CP

    # Ensure the current branch is cached, otherwise the first iteration of controlsd lags
    self.branch = get_short_branch()

    # Setup sockets
    self.pm = messaging.PubMaster(['controlsState', 'carControl', 'onroadEvents'])

    self.sensor_packets = ["accelerometer", "gyroscope"]
    self.camera_packets = ["roadCameraState", "driverCameraState", "wideRoadCameraState"]

    self.log_sock = messaging.sub_sock('androidLog')

    # TODO: de-couple controlsd with card/conflate on carState without introducing controls mismatches
    self.car_state_sock = messaging.sub_sock('carState', timeout=20)

    ignore = self.sensor_packets + ['testJoystick']
    if True:
      ignore += ['driverCameraState', 'managerState', 'driverMonitoringState']
    if REPLAY:
      # no vipc in replay will make them ignored anyways
      ignore += ['roadCameraState', 'wideRoadCameraState']
    if get_frogpilot_toggles().radarless_model:
      ignore += ['radarState']
    self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                                   'carOutput', 'driverMonitoringState', 'longitudinalPlan', 'liveLocationKalman',
                                   'managerState', 'liveParameters', 'radarState', 'liveTorqueParameters',
                                   'testJoystick', 'frogpilotCarState', 'frogpilotPlan'] + self.camera_packets + self.sensor_packets,
                                  ignore_alive=ignore, ignore_avg_freq=ignore+['radarState', 'testJoystick'], ignore_valid=['testJoystick', ],
                                  frequency=int(1/DT_CTRL))

    self.joystick_mode = self.params.get_bool("JoystickDebugMode")

    # read params
    self.is_metric = self.params.get_bool("IsMetric")
    self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")

    # detect sound card presence and ensure successful init
    sounds_available = HARDWARE.get_sound_card_online()

    car_recognized = self.CP.carName != 'mock'

    # cleanup old params
    if not self.CP.experimentalLongitudinalAvailable:
      self.params.remove("ExperimentalLongitudinalEnabled")
    if not self.CP.openpilotLongitudinalControl:
      self.params.remove("ExperimentalMode")

    self.CS_prev = car.CarState.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)

    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)

    self.toadpilot_lane_centering = ToadPilotLaneCentering()
    self.toadpilot_straight_line_smoother = ToadPilotStraightLineSmoother()
    self.toadpilot_lane_centering_capable = (
      self.CP.carName == "toyota" and self.CP.carFingerprint == TOADPILOT_FINGERPRINT and
      self.CP.lateralTuning.which() == "torque"
    )

    self.initialized = False
    self.state = State.disabled
    self.enabled = False
    self.active = False
    self.soft_disable_timer = 0
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
    self.last_blinker_frame = 0
    self.last_steering_pressed_frame = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]
    self.logged_comm_issue = None
    self.not_running_prev = None
    self.steer_limited = False
    self.toadpilot_saturation_count = 0.0
    self.toadpilot_stop_go_event_sequence_seen = 0
    self.toadpilot_stop_go_publish_pending = False
    self.toadpilot_stop_go_last_publish_monotonic = -math.inf
    self.toadpilot_steering_state = "unavailable"
    self.toadpilot_steering_reason = "waiting_for_controller"
    self.toadpilot_steering_publish_pending = False
    self.toadpilot_steering_last_publish_monotonic = -math.inf
    self.toadpilot_steering_state_counts = {}
    self.toadpilot_steering_reason_counts = {}
    self.toadpilot_steering_extended_observations = 0
    self.toadpilot_steering_peak_output_can = 0.0
    self.toadpilot_steering_last_transition_unix = None
    self.toadpilot_torque_command_trace = ToadPilotTorqueCommandTrace()
    self.toadpilot_diagnostic_route_started_at_unix = time.time()
    self.toadpilot_diagnostic_route_id = (
      self.params.get("CurrentRoute", encoding="utf-8") or
      f"controlsd-session:{int(self.toadpilot_diagnostic_route_started_at_unix)}"
    )
    self.toadpilot_panda_state_valid = False
    self.toadpilot_panda_safety_params = []
    self.toadpilot_panda_tx_blocked_counts = []
    self.toadpilot_panda_tx_blocked_previous = None
    self.toadpilot_panda_tx_blocked_route_delta = 0
    self.toadpilot_panda_tx_blocked_last_increment = 0
    self.toadpilot_panda_tx_blocked_last_increment_unix = None
    self.toadpilot_panda_tx_blocked_last_state = None
    self.toadpilot_panda_tx_blocked_last_output_can = None
    self.curvature = 0.0
    self.curvature_limited = False
    self.desired_curvature = 0.0
    self.experimental_mode_param = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
    self.experimental_mode = self.experimental_mode_param
    self.personality = self.read_personality_param()
    self.v_cruise_helper = VCruiseHelper(self.CP)
    self.recalibrating_seen = False

    self.can_log_mono_time = 0

    if car_recognized and not self.CP.passive and self.CP.secOcRequired and not self.CP.secOcKeyAvailable:
      self.startup_event = EventName.startupNoSecOcKey
    else:
      self.startup_event = get_startup_event(car_recognized, not self.CP.passive, len(self.CP.carFw) > 0)

    if not sounds_available:
      pass#self.events.add(EventName.soundsUnavailable, static=True)
    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)
      if len(self.CP.carFw) > 0:
        set_offroad_alert("Offroad_CarUnrecognized", True)
      else:
        set_offroad_alert("Offroad_NoFirmware", True)
    elif self.CP.passive:
      self.events.add(EventName.dashcamMode, static=True)

    # controlsd is driven by carState, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    # FrogPilot variables
    self.frogpilot_toggles = get_frogpilot_toggles()

    self.distance_pressed_previously = False

    self.planner_curves = self.frogpilot_toggles.planner_curvature_model
    self.radarless_model = self.frogpilot_toggles.radarless_model
    self.use_old_long = self.frogpilot_toggles.old_long_api

    self.display_timer = 0

    self.has_menu = self.CP.carName == "gm" and not (self.CP.flags & GMFlags.NO_CAMERA.value or self.CP.carFingerprint in CC_ONLY_CAR)

    self.error_log = ERROR_LOGS_PATH / "error.txt"

  def set_initial_state(self):
    if REPLAY:
      controls_state = self.params.get("ReplayControlsState")
      if controls_state is not None:
        with log.ControlsState.from_bytes(controls_state) as controls_state:
          self.v_cruise_helper.v_cruise_kph = controls_state.vCruise

      if any(ps.controlsAllowed for ps in self.sm['pandaStates']):
        self.state = State.enabled

  def update_events(self, CS):
    """Compute onroadEvents from carState"""

    self.events.clear()

    # Add joystick event, static on cars, dynamic on nonCars
    if self.joystick_mode:
      self.events.add(EventName.joystickDebug)
      self.startup_event = None

    # Add startup event
    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None

    # Don't add any more events if not initialized
    if not self.initialized:
      self.events.add(EventName.controlsInitializing)
      return

    # no more events while in dashcam mode
    if self.CP.passive:
      return

    # Block resume if cruise never previously enabled
    resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
    if not self.CP.pcmCruise and not self.v_cruise_helper.v_cruise_initialized and resume_pressed:
      self.events.add(EventName.resumeBlocked)

    if not self.CP.notCar:
      self.events.add_from_msg(self.sm['driverMonitoringState'].events)

    # Add car events, ignore if CAN isn't valid
    if CS.canValid:
      self.events.add_from_msg(CS.events)

    # Create events for temperature, disk space, and memory
    if self.sm['deviceState'].thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
    if self.sm['deviceState'].freeSpacePercent < 7 and not SIMULATION:
      # under 7% of space free no enable allowed
      self.events.add(EventName.outOfSpace)
    if self.sm['deviceState'].memoryUsagePercent > 90 and not SIMULATION:
      self.events.add(EventName.lowMemory)

    # TODO: enable this once loggerd CPU usage is more reasonable
    #cpus = list(self.sm['deviceState'].cpuUsagePercent)
    #if max(cpus, default=0) > 95 and not SIMULATION:
    #  self.events.add(EventName.highCpuUsage)

    # Alert if fan isn't spinning for 5 seconds
    if self.sm['peripheralState'].pandaType != log.PandaState.PandaType.unknown:
      if self.sm['peripheralState'].fanSpeedRpm < 500 and self.sm['deviceState'].fanSpeedPercentDesired > 50:
        # allow enough time for the fan controller in the panda to recover from stalls
        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:
          self.events.add(EventName.fanMalfunction)
      else:
        self.last_functional_fan_frame = self.sm.frame

    # Handle calibration status
    cal_status = self.sm['liveCalibration'].calStatus
    if cal_status != log.LiveCalibrationData.Status.calibrated:
      if cal_status == log.LiveCalibrationData.Status.uncalibrated:
        self.events.add(EventName.calibrationIncomplete)
      elif cal_status == log.LiveCalibrationData.Status.recalibrating:
        if not self.recalibrating_seen:
          set_offroad_alert("Offroad_Recalibration", True)
        self.recalibrating_seen = True
        self.events.add(EventName.calibrationRecalibrating)
      else:
        self.events.add(EventName.calibrationInvalid)

    # Handle lane change
    if self.sm['modelV2'].meta.laneChangeState == LaneChangeState.preLaneChange:
      direction = self.sm['modelV2'].meta.laneChangeDirection
      if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
         (CS.rightBlindspot and direction == LaneChangeDirection.right):
        if self.frogpilot_toggles.loud_blindspot_alert:
          self.events.add(EventName.laneChangeBlockedLoud)
        else:
          self.events.add(EventName.laneChangeBlocked)
      else:
        if direction == LaneChangeDirection.left:
          if self.sm['frogpilotPlan'].laneWidthLeft >= self.frogpilot_toggles.lane_detection_width:
            self.events.add(EventName.preLaneChangeLeft)
          else:
            self.events.add(EventName.noLaneAvailable)
        else:
          if self.sm['frogpilotPlan'].laneWidthRight >= self.frogpilot_toggles.lane_detection_width:
            self.events.add(EventName.preLaneChangeRight)
          else:
            self.events.add(EventName.noLaneAvailable)
    elif self.sm['modelV2'].meta.laneChangeState in (LaneChangeState.laneChangeStarting,
                                                    LaneChangeState.laneChangeFinishing):
      self.events.add(EventName.laneChange)

    for i, pandaState in enumerate(self.sm['pandaStates']):
      # All pandas must match the list of safetyConfigs, and if outside this list, must be silent or noOutput
      if i < len(self.CP.safetyConfigs):
        safety_mismatch = pandaState.safetyModel != self.CP.safetyConfigs[i].safetyModel or \
                          pandaState.safetyParam != self.CP.safetyConfigs[i].safetyParam or \
                          pandaState.alternativeExperience != self.CP.alternativeExperience
      else:
        safety_mismatch = pandaState.safetyModel not in IGNORED_SAFETY_MODES

      # safety mismatch allows some time for pandad to set the safety mode and publish it back from panda
      if (safety_mismatch and self.sm.frame*DT_CTRL > 10.) or pandaState.safetyRxChecksInvalid or self.mismatch_counter >= 200:
        self.events.add(EventName.controlsMismatch)

      if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
        self.events.add(EventName.relayMalfunction)

    # Handle HW and system malfunctions
    # Order is very intentional here. Be careful when modifying this.
    # All events here should at least have NO_ENTRY and SOFT_DISABLE.
    num_events = len(self.events)

    not_running = {p.name for p in self.sm['managerState'].processes if not p.running and p.shouldBeRunning}
    if self.sm.recv_frame['managerState'] and (not_running - IGNORE_PROCESSES):
      pass#self.events.add(EventName.processNotRunning)
      if not_running != self.not_running_prev:
        cloudlog.event("process_not_running", not_running=not_running, error=True)
      self.not_running_prev = not_running
    else:
      if not SIMULATION and not self.rk.lagging:
        if not self.sm.all_alive(self.camera_packets):
          pass#self.events.add(EventName.cameraMalfunction)
        elif not self.sm.all_freq_ok(self.camera_packets):
          self.events.add(EventName.cameraFrameRate)
    if not REPLAY and self.rk.lagging:
      self.events.add(EventName.controlsdLagging)
    if not self.radarless_model:
      if len(self.sm['radarState'].radarErrors) or ((not self.rk.lagging or REPLAY) and not self.sm.all_checks(['radarState'])):
        self.events.add(EventName.radarFault)
    if not self.sm.valid['pandaStates']:
      self.events.add(EventName.usbError)
    if CS.canTimeout:
      self.events.add(EventName.canBusMissing)
    elif not CS.canValid and not params_memory.get_bool("ForceOnroad"):
      self.events.add(EventName.canError)

    # generic catch-all. ideally, a more specific event should be added above instead
    has_disable_events = self.events.contains(ET.NO_ENTRY) and (self.events.contains(ET.SOFT_DISABLE) or self.events.contains(ET.IMMEDIATE_DISABLE))
    no_system_errors = (not has_disable_events) or (len(self.events) == num_events)
    if not self.sm.all_checks() and no_system_errors:
      if not self.sm.all_alive():
        pass#self.events.add(EventName.commIssue)
      elif not self.sm.all_freq_ok():
        pass#self.events.add(EventName.commIssueAvgFreq)
      else:
        pass#self.events.add(EventName.commIssue)

      logs = {
        'invalid': [s for s, valid in self.sm.valid.items() if not valid],
        'not_alive': [s for s, alive in self.sm.alive.items() if not alive],
        'not_freq_ok': [s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
      }
      if logs != self.logged_comm_issue:
        cloudlog.event("commIssue", error=True, **logs)
        self.logged_comm_issue = logs
    else:
      self.logged_comm_issue = None

    if not (self.CP.notCar and self.joystick_mode):
      if not self.sm['liveLocationKalman'].posenetOK:
        self.events.add(EventName.posenetInvalid)
      if not self.sm['liveLocationKalman'].deviceStable:
        self.events.add(EventName.deviceFalling)
      if not self.sm['liveLocationKalman'].inputsOK:
        self.events.add(EventName.locationdTemporaryError)
      if not self.sm['liveParameters'].valid and not TESTING_CLOSET and (not SIMULATION or REPLAY):
        self.events.add(EventName.paramsdTemporaryError)

    # conservative HW alert. if the data or frequency are off, locationd will throw an error
    if any((self.sm.frame - self.sm.recv_frame[s])*DT_CTRL > 10. for s in self.sensor_packets):
      pass#self.events.add(EventName.sensorDataInvalid)

    if not REPLAY:
      # Check for mismatch between openpilot and car's PCM
      cruise_mismatch = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
      self.cruise_mismatch_counter = self.cruise_mismatch_counter + 1 if cruise_mismatch else 0
      if self.cruise_mismatch_counter > int(6. / DT_CTRL):
        self.events.add(EventName.cruiseMismatch)

    # Check for FCW
    stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
    model_fcw = self.sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and not stock_long_is_braking
    planner_fcw = self.sm['longitudinalPlan'].fcw and self.enabled
    if (planner_fcw or model_fcw) and not (self.CP.notCar and self.joystick_mode):
      self.events.add(EventName.fcw)

    for m in messaging.drain_sock(self.log_sock, wait_for_one=False):
      try:
        msg = m.androidLog.message
        if any(err in msg for err in ("ERROR_CRC", "ERROR_ECC", "ERROR_STREAM_UNDERFLOW", "APPLY FAILED")):
          csid = msg.split("CSID:")[-1].split(" ")[0]
          evt = CSID_MAP.get(csid, None)
          if evt is not None:
            self.events.add(evt)
      except UnicodeDecodeError:
        pass

    # TODO: fix simulator
    if not SIMULATION or REPLAY:
      # Not show in first 1 km to allow for driving out of garage. This event shows after 5 minutes
      if not self.sm['liveLocationKalman'].gpsOK and self.sm['liveLocationKalman'].inputsOK and (self.distance_traveled > 1500):
        self.events.add(EventName.noGps)
      if self.sm['liveLocationKalman'].gpsOK:
        self.distance_traveled = 0
      self.distance_traveled += CS.vEgo * DT_CTRL

      if self.sm['modelV2'].frameDropPerc > 20:
        self.events.add(EventName.modeldLagging)

    # Add FrogPilot events
    self.events.add_from_msg(self.sm['frogpilotPlan'].frogpilotEvents)

    if self.frogpilot_toggles.block_user:
      self.events.add(EventName.blockUser, static=True)

    if self.error_log.is_file():
      if self.frogpilot_toggles.random_events:
        self.events.add(EventName.openpilotCrashedRandomEvent)
      else:
        self.events.add(EventName.openpilotCrashed)

  def data_sample(self):
    """Receive data from sockets"""

    car_state = messaging.recv_one(self.car_state_sock)
    CS = car_state.carState if car_state else self.CS_prev

    self.sm.update(0)

    if not self.initialized:
      all_valid = CS.canValid and self.sm.all_checks()
      timed_out = self.sm.frame * DT_CTRL > 6.
      if all_valid or timed_out or (SIMULATION and not REPLAY):
        available_streams = VisionIpcClient.available_streams("camerad", block=False)
        if VisionStreamType.VISION_STREAM_ROAD not in available_streams:
          self.sm.ignore_alive.append('roadCameraState')
        if VisionStreamType.VISION_STREAM_WIDE_ROAD not in available_streams:
          self.sm.ignore_alive.append('wideRoadCameraState')

        self.initialized = True
        self.set_initial_state()

        cloudlog.event(
          "controlsd.initialized",
          dt=self.sm.frame*DT_CTRL,
          timeout=timed_out,
          canValid=CS.canValid,
          invalid=[s for s, valid in self.sm.valid.items() if not valid],
          not_alive=[s for s, alive in self.sm.alive.items() if not alive],
          not_freq_ok=[s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
          error=True,
        )

    # When the panda and controlsd do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0

    # All pandas not in silent mode must have controlsAllowed when openpilot is enabled
    if self.enabled and any(not ps.controlsAllowed for ps in self.sm['pandaStates']
           if ps.safetyModel not in IGNORED_SAFETY_MODES):
      self.mismatch_counter += 1

    return CS

  def state_transition(self, CS):
    """Compute conditional state transitions and execute actions on state transitions"""

    self.v_cruise_helper.update_v_cruise(CS, self.enabled, self.is_metric, self.sm['frogpilotPlan'].speedLimitChanged, self.frogpilot_toggles)

    # decrement the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

    self.current_alert_types = [ET.PERMANENT]

    # ENABLED, SOFT DISABLING, PRE ENABLING, OVERRIDING
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      if self.events.contains(ET.USER_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)

      elif self.events.contains(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

      else:
        # ENABLED
        if self.state == State.enabled:
          if self.events.contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL):
            self.state = State.overriding
            self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]

        # SOFT DISABLING
        elif self.state == State.softDisabling:
          if not self.events.contains(ET.SOFT_DISABLE):
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled

          elif self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.soft_disable_timer <= 0:
            self.state = State.disabled

        # PRE ENABLING
        elif self.state == State.preEnabled:
          if not self.events.contains(ET.PRE_ENABLE):
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)

        # OVERRIDING
        elif self.state == State.overriding:
          if self.events.contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)
          elif not (self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL)):
            self.state = State.enabled
          else:
            self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]

    # DISABLED
    elif self.state == State.disabled:
      if self.events.contains(ET.ENABLE):
        if self.events.contains(ET.NO_ENTRY):
          self.current_alert_types.append(ET.NO_ENTRY)

        else:
          if self.events.contains(ET.PRE_ENABLE):
            self.state = State.preEnabled
          elif self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL):
            self.state = State.overriding
          else:
            self.state = State.enabled
          self.current_alert_types.append(ET.ENABLE)
          self.v_cruise_helper.initialize_v_cruise(CS, self.experimental_mode, self.sm['frogpilotPlan'].slcSpeedLimit + self.sm['frogpilotPlan'].slcSpeedLimitOffset, self.frogpilot_toggles)

    # Check if openpilot is engaged and actuators are enabled
    self.enabled = self.state in ENABLED_STATES
    self.active = self.state in ACTIVE_STATES
    if self.active or self.sm['frogpilotCarState'].alwaysOnLateralEnabled:
      self.current_alert_types.append(ET.WARNING)

  def _publish_toadpilot_stop_go_status(self, CS, planner_accel, output_accel, inactive_reason):
    if getattr(self.frogpilot_toggles, "toadpilot_telemetry", False) is not True:
      return
    improved_stop_go_active = getattr(self.frogpilot_toggles, "toadpilot_improved_stop_and_go", False) is True
    supported = (getattr(self.CP, "autoResumeSng", False) is True and self.CP.carName == "toyota" and
                 self.CP.carFingerprint == TOADPILOT_FINGERPRINT and self.CP.openpilotLongitudinalControl)
    sequence_changed = self.LoC.stop_go_event_sequence != self.toadpilot_stop_go_event_sequence_seen
    self.toadpilot_stop_go_publish_pending |= sequence_changed
    now_monotonic = time.monotonic()
    heartbeat = self.sm.frame % int(1.0 / DT_CTRL) == 0
    transition_due = (self.toadpilot_stop_go_publish_pending and
                      now_monotonic - self.toadpilot_stop_go_last_publish_monotonic >=
                      TOADPILOT_STATUS_TRANSITION_MIN_INTERVAL)
    if not heartbeat and not transition_due:
      return
    transition_publish = self.toadpilot_stop_go_publish_pending
    self.toadpilot_stop_go_event_sequence_seen = self.LoC.stop_go_event_sequence
    self.toadpilot_stop_go_publish_pending = False
    self.toadpilot_stop_go_last_publish_monotonic = now_monotonic
    payload = self.LoC.stop_go_snapshot(
      getattr(self.frogpilot_toggles, "start_accel_ramp", 0.0),
      route_id=self.toadpilot_diagnostic_route_id, supported=supported,
      improved_stop_go_active=improved_stop_go_active,
    )
    car_output_valid = bool(self.sm.valid["carOutput"])
    car_output = self.sm["carOutput"].actuatorsOutput
    payload.update({
      "fingerprint": self.CP.carFingerprint,
      "speed": round(float(CS.vEgo), 3),
      "measured_accel": round(float(CS.aEgo), 3),
      "planner_accel": round(float(planner_accel), 3) if math.isfinite(float(planner_accel)) else None,
      "output_accel": round(float(output_accel), 3) if math.isfinite(float(output_accel)) else None,
      "longcontrol_output_accel": round(float(output_accel), 3) if math.isfinite(float(output_accel)) else None,
      "controller_target_accel": (round(float(getattr(car_output, "toadPilotLongAccelTarget", 0.0)), 3)
                                  if car_output_valid else None),
      "controller_output_accel": round(float(car_output.accel), 3) if car_output_valid else None,
      "brake_release_active": (bool(getattr(car_output, "toadPilotStopGoReleaseActive", False))
                               if car_output_valid else False),
      "responsive_takeoff_enabled": getattr(self.frogpilot_toggles, "toadpilot_responsive_takeoff", False) is True,
      "smooth_stop_finish_enabled": getattr(self.frogpilot_toggles, "toadpilot_smooth_stop_finish", False) is True,
      "stop_finish_relief": round(float(self.LoC.stop_finish_relief), 3),
      "car_output_valid": car_output_valid,
      "controls_inactive_reason": inactive_reason,
      "diagnostic_trigger": "transition" if transition_publish else "heartbeat",
    })
    params_memory.put_nonblocking(
      TOADPILOT_STOP_GO_STATUS_KEY, json.dumps(payload, sort_keys=True, separators=(",", ":")),
    )

  def _update_toadpilot_panda_diagnostic(self, output):
    if not self.sm.updated["pandaStates"]:
      return
    self.toadpilot_panda_state_valid = bool(self.sm.valid["pandaStates"])
    panda_states = list(self.sm["pandaStates"])
    self.toadpilot_panda_safety_params = [int(state.safetyParam) for state in panda_states]
    counts = [int(state.safetyTxBlocked) for state in panda_states]
    self.toadpilot_panda_tx_blocked_counts = counts
    if not self.toadpilot_panda_state_valid:
      return
    previous = self.toadpilot_panda_tx_blocked_previous
    if previous is not None and len(previous) == len(counts):
      # A decreasing counter normally means a Panda reset; do not mislabel it
      # as a 32-bit wrap or a new safety block.
      increment = sum(current - prior for current, prior in zip(counts, previous) if current >= prior)
      if increment > 0:
        self.toadpilot_panda_tx_blocked_route_delta += increment
        self.toadpilot_panda_tx_blocked_last_increment = increment
        self.toadpilot_panda_tx_blocked_last_increment_unix = time.time()
        self.toadpilot_panda_tx_blocked_last_state = self.toadpilot_steering_state
        self.toadpilot_panda_tx_blocked_last_output_can = round(float(output.steerOutputCan), 1)
    self.toadpilot_panda_tx_blocked_previous = counts

  def _publish_toadpilot_steering_status(self, CS):
    output = self.sm["carOutput"].actuatorsOutput
    # The experimental envelope always retains its safety trace, even when
    # optional general TOADPILOT telemetry is off.
    if (getattr(self.frogpilot_toggles, "toadpilot_telemetry", False) is not True and
        not bool(getattr(output, "toadPilotHighTorqueAuthorized", False))):
      return
    self._update_toadpilot_panda_diagnostic(output)
    car_output_valid = bool(self.sm.valid["carOutput"])
    state = (str(getattr(output, "toadPilotHighTorqueState", "") or "stock")
             if car_output_valid else "unavailable")
    reason = (str(getattr(output, "toadPilotHighTorqueReason", "") or "not_authorized")
              if car_output_valid else "car_output_invalid")
    transition = (state, reason) != (self.toadpilot_steering_state, self.toadpilot_steering_reason)

    if self.sm.updated["carOutput"] and car_output_valid:
      self.toadpilot_steering_state_counts[state] = self.toadpilot_steering_state_counts.get(state, 0) + 1
      self.toadpilot_steering_reason_counts[reason] = self.toadpilot_steering_reason_counts.get(reason, 0) + 1
      output_can = abs(float(output.steerOutputCan))
      self.toadpilot_steering_peak_output_can = max(self.toadpilot_steering_peak_output_can, output_can)
      self.toadpilot_steering_extended_observations += int(output_can > 1500.0)
      output_t = self.sm.logMonoTime["carOutput"] * 1e-9
      self.toadpilot_torque_command_trace.update(
        output_t,
        float(output.steerOutputCan),
        target_can=int(round(float(getattr(output, "toadPilotHighTorqueTargetCan", 0.0)))),
        state=state,
        reason=reason,
        requested_steer=round(float(output.steer), 4),
        headroom_scale=round(float(getattr(output, "toadPilotHighTorqueHeadroomScale", 0.0)), 3),
        steering_rate_rise=round(float(getattr(output, "toadPilotSteeringRateRise", 0.0)), 1),
        steer_request=bool(getattr(output, "toadPilotSteerRequest", False)),
        eps_torque=round(float(CS.steeringTorqueEps), 1),
        steering_rate_deg=round(float(CS.steeringRateDeg), 1),
        steering_pressed=bool(CS.steeringPressed),
        temporary_fault=bool(CS.steerFaultTemporary),
        permanent_fault=bool(CS.steerFaultPermanent),
        eps_state=int(getattr(CS, "steerFaultStatus", 255)),
        eps_initial_fault_state=int(getattr(CS, "steerFaultStatusInitial", 255)),
      )
    if transition:
      self.toadpilot_steering_state = state
      self.toadpilot_steering_reason = reason
      self.toadpilot_steering_last_transition_unix = time.time()
      self.toadpilot_steering_publish_pending = True
    now_monotonic = time.monotonic()
    heartbeat = self.sm.frame % int(1.0 / DT_CTRL) == 0
    important_transition = transition and state in ("eps_temporary_fault", "eps_cooldown")
    transition_due = (self.toadpilot_steering_publish_pending and
                      (important_transition or
                       now_monotonic - self.toadpilot_steering_last_publish_monotonic >=
                       TOADPILOT_STATUS_TRANSITION_MIN_INTERVAL))
    if not heartbeat and not transition_due:
      return
    transition_publish = self.toadpilot_steering_publish_pending
    self.toadpilot_steering_publish_pending = False
    self.toadpilot_steering_last_publish_monotonic = now_monotonic

    payload = {
      "schema_version": 1,
      "published_at_unix": round(time.time(), 3),
      "route_started_at_unix": round(self.toadpilot_diagnostic_route_started_at_unix, 3),
      "route_id": self.toadpilot_diagnostic_route_id,
      "fingerprint": self.CP.carFingerprint,
      "command_semantics": "controller_output_before_panda_or_eps",
      "observation_semantics": "controlsd_carOutput_observations_not_CAN_acceptance",
      "car_output_valid": car_output_valid,
      "authorized": bool(getattr(output, "toadPilotHighTorqueAuthorized", False)),
      "state": state,
      "reason": reason,
      "last_transition_at_unix": self.toadpilot_steering_last_transition_unix,
      "requested_steer": round(float(output.steer), 4),
      "target_output_can": round(float(getattr(output, "toadPilotHighTorqueTargetCan", 0.0)), 1),
      "controller_output_can": round(float(output.steerOutputCan), 1),
      "peak_abs_controller_output_can": round(self.toadpilot_steering_peak_output_can, 1),
      "extended_output_observations": int(self.toadpilot_steering_extended_observations),
      "arm_progress": round(float(getattr(output, "toadPilotHighTorqueArmProgress", 0.0)), 3),
      "headroom_scale": round(float(getattr(output, "toadPilotHighTorqueHeadroomScale", 0.0)), 3),
      "steering_rate_deg": round(float(CS.steeringRateDeg), 1),
      "steering_rate_rise_100ms": round(float(getattr(output, "toadPilotSteeringRateRise", 0.0)), 1),
      "fault_cooldown_seconds": round(float(getattr(output, "toadPilotFaultCooldown", 0.0)), 2),
      "steer_request": bool(getattr(output, "toadPilotSteerRequest", False)),
      "eps_lka_state": int(getattr(CS, "steerFaultStatus", 255)),
      "eps_lta_state": int(getattr(CS, "steerFaultStatusSecondary", 255)),
      "eps_initial_fault_state": int(getattr(CS, "steerFaultStatusInitial", 255)),
      "steer_fault_temporary": bool(CS.steerFaultTemporary),
      "steer_fault_permanent": bool(CS.steerFaultPermanent),
      "state_observations": dict(self.toadpilot_steering_state_counts),
      "reason_observations": dict(self.toadpilot_steering_reason_counts),
      "panda_state_valid": self.toadpilot_panda_state_valid,
      "panda_safety_params": list(self.toadpilot_panda_safety_params),
      "panda_safety_tx_blocked_counts": list(self.toadpilot_panda_tx_blocked_counts),
      "panda_safety_tx_blocked_route_delta": int(self.toadpilot_panda_tx_blocked_route_delta),
      "panda_safety_tx_blocked_last_increment": int(self.toadpilot_panda_tx_blocked_last_increment),
      "panda_safety_tx_blocked_last_increment_at_unix": self.toadpilot_panda_tx_blocked_last_increment_unix,
      "panda_safety_tx_blocked_last_state": self.toadpilot_panda_tx_blocked_last_state,
      "panda_safety_tx_blocked_last_output_can": self.toadpilot_panda_tx_blocked_last_output_can,
      "panda_block_semantics": "aggregate_all_transmit_addresses_not_steering_specific",
      "diagnostic_trigger": "transition" if transition_publish else "heartbeat",
      "selected_max_torque": int(getattr(self.frogpilot_toggles, "toadpilot_experimental_max_torque", 1500)),
      "extra_delta_up_per_10ms": 5,
      **self.toadpilot_torque_command_trace.snapshot(),
    }
    params_memory.put_nonblocking(
      TOADPILOT_STEERING_STATUS_KEY, json.dumps(payload, sort_keys=True, separators=(",", ":")),
    )

  def state_control(self, CS):
    """Given the state, this function returns a CarControl packet"""

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and (torque_params.useParams or self.frogpilot_toggles.force_auto_tune):
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered, self.frogpilot_toggles)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.enabled

    # Check which actuators can be enabled
    standstill = CS.vEgo <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill
    CC.latActive = (self.active or self.sm['frogpilotCarState'].alwaysOnLateralEnabled) and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.joystick_mode) and self.sm['frogpilotPlan'].lateralCheck and not self.sm['frogpilotCarState'].pauseLateral
    longitudinal_override = self.events.contains(ET.OVERRIDE_LONGITUDINAL)
    longitudinal_paused = bool(self.sm['frogpilotCarState'].pauseLongitudinal)
    inactive_reason = longitudinal_controls_inactive_reason(
      self.enabled, longitudinal_override, longitudinal_paused,
      self.CP.openpilotLongitudinalControl, gas_pressed=CS.gasPressed,
    )
    CC.longActive = inactive_reason == "active"

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = self.sm.frame

    # State specific actions

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      if self.use_old_long:
        self.LoC.reset_old_long(v_pid=CS.vEgo)
      else:
        self.LoC.reset()

    self.curvature_limited = False
    if not self.joystick_mode:
      # accel PID loop
      pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_helper.v_cruise_kph * CV.KPH_TO_MS)
      if self.frogpilot_toggles.sport_plus:
        pid_accel_limits = (pid_accel_limits[0], get_max_allowed_accel(CS.vEgo))

      if self.use_old_long:
        t_since_plan = (self.sm.frame - self.sm.recv_frame['longitudinalPlan']) * DT_CTRL
        actuators.accel = self.LoC.update_old_long(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan, self.frogpilot_toggles)
      else:
        forcing_stop = bool(self.sm['frogpilotPlan'].forcingStop)
        should_stop = should_stop_for_long_control(long_plan.shouldStop, forcing_stop)
        actuators.accel = self.LoC.update(
          CC.longActive, CS, long_plan.aTarget, should_stop, pid_accel_limits, self.frogpilot_toggles,
          forcing_stop=forcing_stop, inactive_reason=inactive_reason,
          traffic_mode=bool(self.sm['frogpilotCarState'].trafficMode),
        )
        self._publish_toadpilot_stop_go_status(CS, long_plan.aTarget, actuators.accel, inactive_reason)

      self._publish_toadpilot_steering_status(CS)

      if len(long_plan.speeds):
        actuators.speed = long_plan.speeds[-1]

      # Steering PID loop and lateral MPC
      toadpilot_config = config_from_toggles(self.frogpilot_toggles)
      centering_requested = (
        self.toadpilot_lane_centering_capable and toadpilot_config.enabled and
        toadpilot_config.profile != "observe" and toadpilot_config.lane_centering_assist
      )
      lane_context = (
        extract_toadpilot_lane_context(model_v2, CS.vEgo, CAMERA_OFFSET)
        if centering_requested else ToadPilotLaneContext(reason="not_requested")
      )
      centering_steer_saturated = toadpilot_steer_saturated(
        self.steer_limited, self.sm["carOutput"].actuatorsOutput.steer,
      )
      straight_requested_curvature = self.toadpilot_straight_line_smoother.update(
        toadpilot_config,
        model_v2.action.desiredCurvature,
        CS.vEgo,
        CC.latActive and self.toadpilot_lane_centering_capable,
        CS.steeringPressed,
        centering_steer_saturated,
        model_v2.meta.laneChangeState != LaneChangeState.off,
        CS.leftBlinker or CS.rightBlinker,
        brake_pressed=CS.brakePressed,
        longitudinal_stopping=bool(long_plan.shouldStop or self.sm['frogpilotPlan'].forcingStop),
      )
      requested_curvature = self.toadpilot_lane_centering.update(
        toadpilot_config,
        straight_requested_curvature,
        CS.vEgo,
        lane_context,
        CC.latActive and self.toadpilot_lane_centering_capable,
        CS.steeringPressed,
        CS.steeringRateDeg,
        centering_steer_saturated,
        self.experimental_mode,
        model_v2.meta.laneChangeState != LaneChangeState.off,
        CS.leftBlinker or CS.rightBlinker,
        a_ego=CS.aEgo,
        brake_pressed=CS.brakePressed,
        longitudinal_stopping=bool(long_plan.shouldStop or self.sm['frogpilotPlan'].forcingStop),
      )
      # Reset toward the measured curvature while lateral control is inactive so
      # engagement cannot bypass the curvature-rate limit. Keep TOADPILOT's
      # requested curvature unchanged whenever lateral control is active.
      target_curvature = get_lateral_target_curvature(CC.latActive, self.curvature, requested_curvature)
      self.desired_curvature, self.curvature_limited = clip_curvature(
        CS.vEgo, self.desired_curvature, target_curvature, self.planner_curves, lp.roll,
      )
      # Keep model-curvature limiting distinct from downstream car/safety
      # clipping (`steer_limited`), which TOADPILOT uses for torque saturation.
      actuators.curvature = self.desired_curvature
      toadpilot_centering_status = {
        **self.toadpilot_lane_centering.snapshot(),
        **self.toadpilot_straight_line_smoother.snapshot(),
      }
      actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                                             self.steer_limited, self.desired_curvature,
                                                                             self.curvature_limited,
                                                                             self.sm['liveLocationKalman'],
                                                                             model_data=self.sm['modelV2'], frogpilot_toggles=self.frogpilot_toggles,
                                                                             experimental_mode=self.experimental_mode,
                                                                             toadpilot_longitudinal_stopping=bool(long_plan.shouldStop or self.sm['frogpilotPlan'].forcingStop),
                                                                             toadpilot_centering_status=toadpilot_centering_status)
    else:
      lac_log = log.ControlsState.LateralDebugState.new_message()
      if self.sm.recv_frame['testJoystick'] > 0:
        # reset joystick if it hasn't been received in a while
        should_reset_joystick = (self.sm.frame - self.sm.recv_frame['testJoystick'])*DT_CTRL > 0.2
        if not should_reset_joystick:
          joystick_axes = self.sm['testJoystick'].axes
        else:
          joystick_axes = [0.0, 0.0]

        if CC.longActive:
          actuators.accel = 4.0*clip(joystick_axes[0], -1, 1)

        if CC.latActive:
          steer = clip(joystick_axes[1], -1, 1)
          # max angle is 45 for angle-based cars, max curvature is 0.02
          actuators.steer, actuators.steeringAngleDeg, actuators.curvature = steer, steer * 90., steer * -0.02

        lac_log.active = self.active
        lac_log.steeringAngleDeg = CS.steeringAngleDeg
        lac_log.output = actuators.steer
        lac_log.saturated = abs(actuators.steer) >= 0.9

    if CS.steeringPressed:
      self.last_steering_pressed_frame = self.sm.frame
    recent_steer_pressed = (self.sm.frame - self.last_steering_pressed_frame)*DT_CTRL < 2.0

    # Send a "steering required alert" if saturation count has reached the limit.
    # TOADPILOT debounces only the torque-specific alert; a sustained saturation
    # condition still raises the normal safety-facing warning.
    if not lac_log.active or recent_steer_pressed or self.CP.notCar:
      self.toadpilot_saturation_count = 0.0
    if lac_log.active and not recent_steer_pressed and not self.CP.notCar:
      if self.CP.lateralTuning.which() == 'torque' and not self.joystick_mode:
        undershooting = abs(lac_log.desiredLateralAccel) / max(abs(lac_log.actualLateralAccel), 1e-3) > 1.2
        turning = abs(lac_log.desiredLateralAccel) > 1.0
        good_speed = CS.vEgo > 5
        max_torque = abs(self.sm['carOutput'].actuatorsOutput.steer) > 0.99
        torque_saturation_condition = undershooting and turning and good_speed and max_torque
        curvature_cap_warning = curvature_cap_alert_required(
          self.curvature_limited, bool(lac_log.saturated), requested_curvature,
          lac_log.actualLateralAccel, CS.vEgo,
        )
        smart_warning = (getattr(self.frogpilot_toggles, "toadpilot_enabled", False) and
                         getattr(self.frogpilot_toggles, "toadpilot_smart_saturation_warning", False))
        if smart_warning:
          # Curvature caps have already passed LatControl's saturation timer;
          # apply TOADPILOT's separate debounce only to the max-torque signal.
          saturation_limit = max(float(self.CP.steerLimitTimer), DT_CTRL)
          count_step = DT_CTRL if torque_saturation_condition else -DT_CTRL
          self.toadpilot_saturation_count = clip(self.toadpilot_saturation_count + count_step, 0.0, saturation_limit)
          torque_saturation_condition = self.toadpilot_saturation_count >= saturation_limit
        else:
          self.toadpilot_saturation_count = 0.0
        saturation_condition = curvature_cap_warning or torque_saturation_condition
        if saturation_condition:
          self.events.add(EventName.goatSteerSaturated if self.frogpilot_toggles.goat_scream_alert else EventName.steerSaturated)
      elif lac_log.saturated:
        # TODO probably should not use dpath_points but curvature
        dpath_points = model_v2.position.y
        if len(dpath_points):
          # Check if we deviated from the path
          # TODO use desired vs actual curvature
          if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
            steering_value = actuators.steeringAngleDeg
          else:
            steering_value = actuators.steer

          left_deviation = steering_value > 0 and dpath_points[0] < -0.20
          right_deviation = steering_value < 0 and dpath_points[0] > 0.20

          if left_deviation or right_deviation:
            self.events.add(EventName.steerSaturated)

    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    # decrement personality on distance button press
    if self.CP.openpilotLongitudinalControl:
      distance_pressed = params_memory.get_bool("OnroadDistanceButtonPressed")

      if self.frogpilot_toggles.personality_profile_via_distance:
        distance_pressed |= any(not be.pressed and be.type == ButtonType.gapAdjustCruise for be in CS.buttonEvents)
        distance_pressed &= not (self.sm['frogpilotCarState'].distanceLongPressed or self.sm['frogpilotCarState'].distanceVeryLongPressed)
      if self.frogpilot_toggles.personality_profile_via_distance_long:
        distance_pressed |= self.sm['frogpilotCarState'].distanceLongPressed
      if self.frogpilot_toggles.personality_profile_via_distance_very_long:
        distance_pressed |= self.sm['frogpilotCarState'].distanceVeryLongPressed
      if self.frogpilot_toggles.personality_profile_via_lkas:
        distance_pressed |= any(be.pressed and be.type == FrogPilotButtonType.lkas for be in CS.buttonEvents)

      if not distance_pressed and self.distance_pressed_previously:
        if self.display_timer > 0 or not self.has_menu:
          self.personality = (self.personality - 1) % 3
          self.params.put_nonblocking('LongitudinalPersonality', str(self.personality))
        self.display_timer = 350

      self.distance_pressed_previously = distance_pressed

      self.display_timer -= 1

    return CC, lac_log

  def update_frogpilot_variables(self, CS):
    # Update FrogPilot parameters
    if self.sm['frogpilotPlan'].togglesUpdated:
      self.frogpilot_toggles = get_frogpilot_toggles()

    conditional_experimental = (self.frogpilot_toggles.conditional_experimental_mode or
                                self.frogpilot_toggles.slc_fallback_experimental_mode)
    self.experimental_mode = (bool(self.sm['frogpilotPlan'].experimentalMode) if conditional_experimental
                              else self.experimental_mode_param)

  def publish_logs(self, CS, start_time, CC, lac_log):
    """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    orientation_value = list(self.sm['liveLocationKalman'].calibratedOrientationNED.value)
    if len(orientation_value) > 2:
      CC.orientationNED = orientation_value
    angular_rate_value = valid_calibrated_angular_velocity(
      self.sm['liveLocationKalman'], self.sm.all_checks(['liveLocationKalman']),
    )
    if angular_rate_value:
      CC.angularVelocity = angular_rate_value

    CC.cruiseControl.override = self.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
    if self.joystick_mode and self.sm.recv_frame['testJoystick'] > 0 and self.sm['testJoystick'].buttons[0]:
      CC.cruiseControl.cancel = True

    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds):
      CC.cruiseControl.resume = self.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1

    hudControl = CC.hudControl
    hudControl.setSpeed = float(self.v_cruise_helper.v_cruise_cluster_kph * CV.KPH_TO_MS)
    hudControl.speedVisible = self.enabled
    hudControl.lanesVisible = self.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.personality + 1

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True

    recent_blinker = (self.sm.frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
    ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker \
                  and not CC.latActive and self.sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.calibrated

    model_v2 = self.sm['modelV2']
    desire_prediction = model_v2.meta.desirePrediction
    if len(desire_prediction) and ldw_allowed:
      right_lane_visible = model_v2.laneLineProbs[2] > 0.5
      left_lane_visible = model_v2.laneLineProbs[1] > 0.5
      l_lane_change_prob = desire_prediction[Desire.laneChangeLeft]
      r_lane_change_prob = desire_prediction[Desire.laneChangeRight]

      lane_lines = model_v2.laneLines
      l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET))
      r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET))

      hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)

    if hudControl.rightLaneDepart or hudControl.leftLaneDepart:
      self.events.add(EventName.ldw)

    clear_event_types = set()
    if ET.WARNING not in self.current_alert_types:
      clear_event_types.add(ET.WARNING)
    if self.enabled:
      clear_event_types.add(ET.NO_ENTRY)

    alerts = self.events.create_alerts(self.current_alert_types, [self.CP, CS, self.sm, self.is_metric, self.soft_disable_timer, self.frogpilot_toggles])
    self.AM.add_many(self.sm.frame, alerts)
    current_alert = self.AM.process_alerts(self.sm.frame, clear_event_types)
    if current_alert:
      hudControl.visualAlert = current_alert.visual_alert

    if not self.CP.passive and self.initialized:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                             STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited = abs(CC.actuators.steer - CO.actuatorsOutput.steer) > 1e-2

    force_decel = (self.sm['driverMonitoringState'].awarenessStatus < 0.) or \
                  (self.state == State.softDisabling) or \
                  self.sm['frogpilotCarState'].forceCoast

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    controlsState = dat.controlsState
    if current_alert:
      controlsState.alertText1 = current_alert.alert_text_1
      controlsState.alertText2 = current_alert.alert_text_2
      controlsState.alertSize = current_alert.alert_size
      controlsState.alertStatus = current_alert.alert_status
      controlsState.alertBlinkingRate = current_alert.alert_rate
      controlsState.alertType = current_alert.alert_type
      controlsState.alertSound = current_alert.audible_alert

    controlsState.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    controlsState.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    controlsState.enabled = self.enabled
    controlsState.active = self.active
    controlsState.curvature = self.curvature
    controlsState.desiredCurvature = self.desired_curvature
    controlsState.state = self.state
    controlsState.engageable = not self.events.contains(ET.NO_ENTRY)
    controlsState.longControlState = self.LoC.long_control_state
    controlsState.vPid = float(self.LoC.v_pid)
    controlsState.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    controlsState.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)
    controlsState.upAccelCmd = float(self.LoC.pid.p)
    controlsState.uiAccelCmd = float(self.LoC.pid.i)
    controlsState.ufAccelCmd = float(self.LoC.pid.f)
    controlsState.cumLagMs = -self.rk.remaining * 1000.
    controlsState.startMonoTime = int(start_time * 1e9)
    controlsState.forceDecel = bool(force_decel)
    controlsState.experimentalMode = self.experimental_mode
    controlsState.personality = self.personality

    lat_tuning = self.CP.lateralTuning.which()
    if self.joystick_mode:
      controlsState.lateralControlState.debugState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      controlsState.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      controlsState.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      controlsState.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # onroadEvents - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('onroadEvents', len(self.events))
      ce_send.valid = True
      ce_send.onroadEvents = self.events.to_msg()
      self.pm.send('onroadEvents', ce_send)
    self.events_prev = self.events.names.copy()

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def step(self):
    start_time = time.monotonic()

    # Sample data from sockets and get a carState
    CS = self.data_sample()
    cloudlog.timestamp("Data sampled")

    self.update_events(CS)
    cloudlog.timestamp("Events updated")

    self.update_frogpilot_variables(CS)

    if not self.CP.passive and self.initialized:
      # Update control state
      self.state_transition(CS)

    # Compute actuators (runs PID loops and lateral MPC)
    CC, lac_log = self.state_control(CS)

    # Publish data
    self.publish_logs(CS, start_time, CC, lac_log)

    self.CS_prev = CS

  def read_personality_param(self):
    try:
      return int(self.params.get('LongitudinalPersonality'))
    except (ValueError, TypeError):
      return log.LongitudinalPersonality.standard

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode_param = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
      self.personality = self.read_personality_param()
      if self.CP.notCar:
        self.joystick_mode = self.params.get_bool("JoystickDebugMode")
      time.sleep(0.1)

  def controlsd_thread(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.monitor_time()
    except SystemExit:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.controlsd_thread()


if __name__ == "__main__":
  main()
