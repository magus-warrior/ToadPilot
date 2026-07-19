import math
import time

from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.frogpilot.controls.lib.start_accel_ramp import (START_ACCEL_RAMP_END_SPEED,
                                                                        START_ACCEL_RAMP_MAX_DURATION,
                                                                        StartAccelRamp)
from openpilot.selfdrive.frogpilot.toadpilot import TOADPILOT_FINGERPRINT

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState
STOP_GO_PLANNER_STOP_GRACE = 0.10
STOP_GO_PLANNER_STOP_GRACE_FRAMES = max(1, round(STOP_GO_PLANNER_STOP_GRACE / DT_CTRL))


def _finite_number(value):
  return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def should_stop_for_long_control(planner_should_stop, forcing_stop):
  """Merge explicit stop requests without treating model distance as a stop flag."""
  return bool(planner_should_stop or forcing_stop)


def cruise_standstill_for_long_control(CP, cruise_standstill, improved_stop_and_go, *, active,
                                       should_stop, brake_pressed, planner_accel):
  """Return the Toyota/PCM standstill gate that should hold longitudinal control.

  Auto-resume stop-and-go platforms can report cruise standstill for a few
  frames after the planner has released a stop. On the supported Camry Hybrid,
  the Toyota controller already releases standstill directly; treating that
  stale status as a new hold keeps LongControl in stopping or cancels the
  optional acceleration ramp. The explicit TOADPILOT setting filters only that
  status during a bounded handoff. Planner stops and brake input remain
  separate hard gates.
  """
  pcm_standstill = bool(cruise_standstill and not CP.enableGasInterceptor)
  auto_resume = bool(getattr(CP, "autoResumeSng", False))
  supported_camry = (getattr(CP, "carName", "") == "toyota" and
                     getattr(CP, "carFingerprint", "") == TOADPILOT_FINGERPRINT)
  planner_launch = (active and not should_stop and not brake_pressed and
                    _finite_number(planner_accel) and planner_accel > 0.0)
  bypass_stale_status = improved_stop_and_go and auto_resume and supported_camry and planner_launch
  return pcm_standstill and not bypass_stale_status


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill, frogpilot_toggles):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > frogpilot_toggles.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

def long_control_state_trans_old_long(CP, active, long_control_state, v_ego, v_target,
                                      v_target_1sec, brake_pressed, cruise_standstill):
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0
    self.start_accel_ramp = StartAccelRamp()
    self.stop_go_handoff_active = False
    self.stop_go_handoff_elapsed = 0.0
    self.stop_go_handoff_last_event = "idle"
    self.stop_go_handoff_last_detail = None
    self.stop_go_planner_stop_pending = False
    self.stop_go_planner_stop_elapsed = 0.0
    self.stop_go_planner_stop_frames = 0
    self.stop_go_planner_stop_recovered = False
    self.stop_go_last_handoff_duration = 0.0
    self.stop_go_last_planner_stop_duration = 0.0
    self.stop_go_event_sequence = 0
    self.stop_go_event_counts = {}
    self.stop_go_last_event_unix = None
    self.stop_go_route_started_at_unix = time.time()

  def reset(self):
    self.pid.reset()

  def _record_stop_go_event(self, event, CS, planner_accel, detail=None):
    self.stop_go_handoff_last_event = event
    self.stop_go_handoff_last_detail = detail
    self.stop_go_last_event_unix = time.time()
    self.stop_go_event_sequence += 1
    counter_key = event if detail is None else f"{event}:{detail}"
    self.stop_go_event_counts[counter_key] = self.stop_go_event_counts.get(counter_key, 0) + 1
    cloudlog.event(
      "toadpilot_stop_and_go",
      handoff_event=event,
      detail=detail,
      elapsed=round(float(self.stop_go_handoff_elapsed), 3),
      speed=round(float(CS.vEgo), 3) if _finite_number(CS.vEgo) else None,
      planner_accel=round(float(planner_accel), 3) if _finite_number(planner_accel) else None,
    )

  def _finish_stop_go_handoff(self, event, CS, planner_accel, detail=None):
    if self.stop_go_handoff_active or self.stop_go_planner_stop_pending:
      self.stop_go_last_handoff_duration = self.stop_go_handoff_elapsed
      self.stop_go_last_planner_stop_duration = self.stop_go_planner_stop_elapsed
      self._record_stop_go_event(event, CS, planner_accel, detail)
    self.stop_go_handoff_active = False
    self.stop_go_handoff_elapsed = 0.0
    self.stop_go_planner_stop_pending = False
    self.stop_go_planner_stop_elapsed = 0.0
    self.stop_go_planner_stop_frames = 0

  def _update_stop_go_handoff(self, previous_long_control_state, active, CS, planner_accel,
                              should_stop, frogpilot_toggles, *, forcing_stop=False,
                              inactive_reason="controls_inactive"):
    self.stop_go_planner_stop_recovered = False
    setting_enabled = getattr(frogpilot_toggles, "toadpilot_improved_stop_and_go", False) is True
    supported = (setting_enabled and getattr(self.CP, "autoResumeSng", False) is True and
                 getattr(self.CP, "carName", "") == "toyota" and
                 getattr(self.CP, "carFingerprint", "") == TOADPILOT_FINGERPRINT)
    valid_accel = _finite_number(planner_accel)
    valid_speed = _finite_number(CS.vEgo)

    finish_reason = None
    contradictory_planner_stop = (self.stop_go_handoff_active and should_stop and not forcing_stop and
                                  active and not CS.brakePressed and valid_accel and planner_accel > 0.0 and
                                  valid_speed and CS.vEgo < START_ACCEL_RAMP_END_SPEED)

    if not supported:
      finish_reason = "setting_or_vehicle_inactive"
    elif not active:
      finish_reason = "controls_inactive"
    elif forcing_stop:
      finish_reason = "forced_stop"
    elif should_stop and not contradictory_planner_stop:
      finish_reason = "planner_stop"
    elif CS.brakePressed:
      finish_reason = "brake"
    elif not valid_accel or planner_accel <= 0.0:
      finish_reason = "planner_cancelled"
    elif not valid_speed:
      finish_reason = "invalid_speed"
    elif CS.vEgo >= START_ACCEL_RAMP_END_SPEED:
      finish_reason = "speed_complete"
    elif self.stop_go_handoff_elapsed >= START_ACCEL_RAMP_MAX_DURATION:
      finish_reason = "timeout"

    if finish_reason is not None:
      detail = inactive_reason if finish_reason == "controls_inactive" else None
      self._finish_stop_go_handoff(finish_reason, CS, planner_accel, detail)
    elif contradictory_planner_stop:
      self.stop_go_handoff_elapsed += DT_CTRL
      self.stop_go_planner_stop_frames += 1
      self.stop_go_planner_stop_elapsed = self.stop_go_planner_stop_frames * DT_CTRL
      if not self.stop_go_planner_stop_pending:
        self.stop_go_planner_stop_pending = True
        self._record_stop_go_event("planner_stop_pending", CS, planner_accel)
      if self.stop_go_planner_stop_frames >= STOP_GO_PLANNER_STOP_GRACE_FRAMES:
        self._finish_stop_go_handoff("planner_stop_confirmed", CS, planner_accel)
    elif self.stop_go_planner_stop_pending:
      self.stop_go_last_planner_stop_duration = self.stop_go_planner_stop_elapsed
      self.stop_go_planner_stop_pending = False
      self.stop_go_planner_stop_elapsed = 0.0
      self.stop_go_planner_stop_frames = 0
      self.stop_go_planner_stop_recovered = True
      self.stop_go_handoff_elapsed += DT_CTRL
      self._record_stop_go_event("planner_stop_cleared", CS, planner_accel)
    elif self.stop_go_handoff_active:
      self.stop_go_handoff_elapsed += DT_CTRL
    elif previous_long_control_state == LongCtrlState.stopping:
      self.stop_go_handoff_active = True
      self.stop_go_handoff_elapsed = DT_CTRL
      self._record_stop_go_event("started", CS, planner_accel)

    return self.stop_go_handoff_active

  def stop_go_snapshot(self, configured_accel, route_id=None, supported=False, improved_stop_go_active=False):
    ramp_configured = _finite_number(configured_accel) and configured_accel > 0.0
    if self.stop_go_planner_stop_pending:
      state = "planner_hold"
    elif self.start_accel_ramp.started:
      state = "ramp_active"
    elif self.stop_go_handoff_active:
      state = "handoff"
    elif not supported:
      state = "unsupported"
    elif not improved_stop_go_active and not ramp_configured:
      state = "inactive"
    else:
      state = "ready"
    return {
      "schema_version": 1,
      "published_at_unix": round(time.time(), 3),
      "route_started_at_unix": round(self.stop_go_route_started_at_unix, 3),
      "route_id": route_id or f"controlsd-session:{int(self.stop_go_route_started_at_unix)}",
      "route_complete": False,
      "completed_at_unix": None,
      "summary_source": "controlsd_live",
      "state": state,
      "supported": bool(supported),
      "improved_stop_go_active": bool(improved_stop_go_active),
      "ramp_configured": bool(ramp_configured),
      "configured_accel": round(float(configured_accel), 2) if _finite_number(configured_accel) else 0.0,
      "handoff_active": bool(self.stop_go_handoff_active),
      "handoff_elapsed": round(float(self.stop_go_handoff_elapsed), 3),
      "last_handoff_duration": round(float(self.stop_go_last_handoff_duration), 3),
      "planner_stop_pending": bool(self.stop_go_planner_stop_pending),
      "planner_stop_elapsed": round(float(self.stop_go_planner_stop_elapsed), 3),
      "last_planner_stop_duration": round(float(self.stop_go_last_planner_stop_duration), 3),
      "planner_stop_grace_seconds": STOP_GO_PLANNER_STOP_GRACE,
      "ramp_armed": bool(self.start_accel_ramp.armed),
      "ramp_started": bool(self.start_accel_ramp.started),
      "ramp_accel": round(float(self.start_accel_ramp.accel), 3),
      "last_event": self.stop_go_handoff_last_event,
      "last_detail": self.stop_go_handoff_last_detail,
      "last_event_at_unix": self.stop_go_last_event_unix,
      "event_sequence": int(self.stop_go_event_sequence),
      "event_counts": dict(self.stop_go_event_counts),
    }

  def update(self, active, CS, a_target, should_stop, accel_limits, frogpilot_toggles, *,
             forcing_stop=False, inactive_reason="controls_inactive"):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    previous_long_control_state = self.long_control_state
    improved_stop_and_go = self._update_stop_go_handoff(
      previous_long_control_state, active, CS, a_target, should_stop, frogpilot_toggles,
      forcing_stop=forcing_stop, inactive_reason=inactive_reason,
    )
    cruise_standstill = cruise_standstill_for_long_control(
      self.CP, CS.cruiseState.standstill,
      improved_stop_and_go,
      active=active, should_stop=should_stop, brake_pressed=CS.brakePressed, planner_accel=a_target,
    )
    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       cruise_standstill, frogpilot_toggles)
    released_from_stop = (previous_long_control_state == LongCtrlState.stopping and
                          self.long_control_state in (LongCtrlState.starting, LongCtrlState.pid) and
                          not self.stop_go_planner_stop_recovered)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= frogpilot_toggles.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = (a_target if frogpilot_toggles.human_acceleration else self.CP.startAccel)
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    output_accel = self.start_accel_ramp.update(
      output_accel,
      getattr(frogpilot_toggles, "start_accel_ramp", 0.0),
      CS.vEgo,
      a_target,
      released_from_stop,
      active,
      should_stop,
      CS.brakePressed,
      cruise_standstill,
      preserve_on_planner_stop=self.stop_go_planner_stop_pending,
    )
    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

  def reset_old_long(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update_old_long(self, active, CS, long_plan, accel_limits, t_since_plan, frogpilot_toggles):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target_now = interp(t_since_plan, CONTROL_N_T_IDX, long_plan.accels)

      v_target = interp(self.CP.longitudinalActuatorDelay + t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target = 2 * (v_target - v_target_now) / self.CP.longitudinalActuatorDelay - a_target_now

      v_target_1sec = interp(self.CP.longitudinalActuatorDelay + t_since_plan + 1.0, CONTROL_N_T_IDX, speeds)
    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel
    self.long_control_state = long_control_state_trans_old_long(self.CP, active, self.long_control_state, CS.vEgo,
                                                                v_target, v_target_1sec, CS.brakePressed,
                                                                CS.cruiseState.standstill)

    if self.long_control_state == LongCtrlState.off:
      self.reset_old_long(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset_old_long(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset_old_long(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel
