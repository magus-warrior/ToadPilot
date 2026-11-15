"""Pinned StarPilot low-speed stop safeguards; current Camry launch curve is retained."""
import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL

LongCtrlState = car.CarControl.Actuators.LongControlState
STOPPING_RELEASE_HYSTERESIS = 0.35
STOPPING_RELEASE_MIN_ACCEL = 0.15
STOPPING_RELEASE_STRONG_ACCEL = 0.45
MOVING_STOP_FOLLOW_MIN_GAP = 0.25
interp = np.interp

def stop_release_ready(self, CS, a_target, should_stop, has_lead, starpilot_toggles, leads=None):
    if self.long_control_state != LongCtrlState.stopping:
        self.stop_release_counter = 0
        return True
    if should_stop or CS.brakePressed:
        self.stop_release_counter = 0
        return False
    if CS.vEgo > starpilot_toggles.vEgoStarting:
        self.stop_release_counter = int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))
        return True
    if has_lead and a_target > STOPPING_RELEASE_MIN_ACCEL:
        self.stop_release_counter = int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))
        return True
    if a_target >= STOPPING_RELEASE_STRONG_ACCEL and (not CS.cruiseState.standstill):
        self.stop_release_counter = int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))
        return True
    if a_target > STOPPING_RELEASE_MIN_ACCEL:
        max_frames = int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))
        self.stop_release_counter = min(self.stop_release_counter + 1, max_frames)
    else:
        self.stop_release_counter = 0
    return self.stop_release_counter >= int(round(STOPPING_RELEASE_HYSTERESIS / DT_CTRL))

def moving_stop_target_follow(output_accel, a_target, should_stop, CS, starpilot_toggles):
    follow_min_speed = max(1.5, starpilot_toggles.vEgoStopping + 1.0)
    if not should_stop or CS.brakePressed or CS.vEgo <= follow_min_speed:
        return output_accel
    if a_target >= output_accel - MOVING_STOP_FOLLOW_MIN_GAP:
        return output_accel
    follow_step = interp(CS.vEgo, [follow_min_speed, 3.0, 6.0, 10.0], [0.02, 0.03, 0.05, 0.07])
    return max(float(a_target), output_accel - float(follow_step))
