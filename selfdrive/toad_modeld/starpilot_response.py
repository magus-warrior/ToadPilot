"""StarPilot 4a553da9 response primitives adapted to the Camry's legacy units.

The generic TSS2 path is used; the older Camry's vehicle-specific tune is not.
The lead MPC, stop requests and Toyota/Panda limits remain authoritative.
"""
import math
from collections import deque

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.toad_modeld.artifact import is_modern_model


class ResponsePID(PIDController):
  """Pinned StarPilot integral anti-windup; the legacy PID differs at saturation."""
  def update(self, error, error_rate=0.0, speed=0.0, feedforward=0.0, freeze_integrator=False):
    self.speed = speed
    self.p = self.k_p * float(error)
    self.d = self.k_d * error_rate
    self.f = feedforward
    if not freeze_integrator:
      integral = self.i + self.k_i * self.i_rate * error
      requested = self.p + integral + self.d + self.f
      upper = self.i if requested > self.pos_limit else self.pos_limit
      lower = self.i if requested < self.neg_limit else self.neg_limit
      self.i = float(np.clip(integral, lower, upper))
    self.control = float(np.clip(self.p + self.i + self.d + self.f, self.neg_limit, self.pos_limit))
    return self.control


class ThrottlePermission:
  """StarPilot's hysteresis and confirmation, including its creep bypass."""
  def __init__(self):
    self.allowed = True
    self.transition_time = 0.0

  def update(self, probability, speed, dt, reset=False):
    if reset or speed <= 5.0:
      self.allowed = True
      self.transition_time = 0.0
    elif not all(math.isfinite(v) for v in (probability, speed, dt)) or dt <= 0:
      self.allowed = False
      self.transition_time = 0.0
    else:
      transition = probability <= 0.35 if self.allowed else probability > 0.45
      self.transition_time = self.transition_time + dt if transition else 0.0
      if self.transition_time + 1e-6 >= 0.25:
        self.allowed = not self.allowed
        self.transition_time = 0.0
    return self.allowed


def response_enabled(toggles):
  return (getattr(toggles, 'toadpilot_enabled', False) is True and
          getattr(toggles, 'toadpilot_starpilot_response', False) is True and
          getattr(toggles, 'car_model', None) == 'TOYOTA_CAMRY_TSS2' and is_modern_model(toggles))


class SteeringResponse:
  """Generic StarPilot torque loop in acceleration units, converted once at output."""
  def __init__(self, min_steer_speed=0.0):
    self.dt = DT_CTRL
    self.minimum_speed = max(min_steer_speed, 0.3)
    self.history = deque([0.0] * 100, maxlen=100)
    self.pid = ResponsePID(
      [[1., 1.5, 2., 3., 5., 7.5, 10., 15., 30.],
       [250., 120., 65., 30., 11.5, 5.5, 3.5, 2., 0.6]],
      0.35, k_f=1., rate=1 / self.dt)
    self.jerk_filter = FirstOrderFilter(0., 1 / (2 * math.pi * 1.2), self.dt)
    self.previous_setpoint = 0.0
    self.previous_override = False
    self.driver_direction = 0
    self.driver_hold_frames = 0

  def update(self, *, active, speed, desired_curvature, measured_curvature, roll,
             delay, factor, offset, friction, deadzone, steer_max, limited, override, driver_torque=0.0):
    values = (speed, desired_curvature, measured_curvature, roll, delay, factor,
              offset, friction, deadzone, steer_max)
    if not all(math.isfinite(v) for v in values) or factor <= 0 or friction < 0:
      self.pid.reset()
      self.history = deque([0.] * 100, maxlen=100)
      self.jerk_filter.x = 0.0
      self.previous_setpoint = 0.0
      self.driver_hold_frames = 0
      self.previous_override = False
      return 0.0, dict(valid=False, setpoint=0., measurement=0., jerk=0., feedforward=0., applied_offset=0.)
    measurement = measured_curvature * speed ** 2
    future = desired_curvature * speed ** 2
    if not active:
      self.pid.reset()
      self.history.append(desired_curvature)
      self.jerk_filter.x = 0.0
      self.previous_setpoint = future
      self.previous_override = override
      self.driver_hold_frames = 0
      return 0.0, dict(valid=True, setpoint=future, measurement=measurement, jerk=0., feedforward=0., applied_offset=0.)

    if self.previous_override and not override:
      self.pid.i *= 0.8
    frames = int(np.clip(delay / self.dt, 1, len(self.history)))
    if override:
      # Positive Toyota driver torque and positive returned steer both mean left.
      self.driver_direction = np.sign(driver_torque) if math.isfinite(driver_torque) else 0
      self.driver_hold_frames = frames
    expected = self.history[-frames] * speed ** 2
    self.history.append(desired_curvature)
    jerk = float(np.clip(self.jerk_filter.update(float(np.clip(
      (future - expected) / max(delay, self.dt), -2.5, 2.5))), -2.5, 2.5))
    delayed_setpoint = expected + jerk * delay
    # FrogPilot holds the integral below 5 m/s. Blend back to StarPilot at its
    # next gain breakpoint so the response is continuous as speed changes.
    frog_weight = max(0., min(1., (7.5 - speed) / 2.5))
    setpoint = delayed_setpoint + frog_weight * (future - delayed_setpoint)
    # Freeze on rapid unwind toward zero, symmetrically in either direction.
    unwinding = (abs(setpoint) - abs(self.previous_setpoint)) / self.dt < -1.0 and abs(setpoint) < 0.3
    self.previous_setpoint = setpoint
    kp = float(np.interp(speed, *self.pid._k_p))
    low_speed = (float(np.interp(speed, [0., 10., 20., 30.], [12., 10.5, 8., 5.])) / max(speed, 1.)) ** 2
    error = (delayed_setpoint - measurement) * (1 + low_speed / max(kp, 1e-3))
    fade = float(np.interp(speed, [0.5, 2.5], [0., 1.]))
    applied_offset = offset * fade
    ff = future - roll * 9.81 * fade - applied_offset
    jerk_deadzone = float(np.interp(speed, [0., 5., 12., 25.], [.08, .12, .18, .18]))
    jerk_deadzone *= float(np.interp(abs(setpoint), [0., .18, .35], [1., 1., 0.]))
    friction_jerk = math.copysign(max(abs(jerk) - jerk_deadzone, 0.), jerk)
    friction_error = error + .22 * friction_jerk
    if frog_weight:
      # FrogPilot 1e23dec6 uses the current target, kp=1, and interpolates
      # its low-speed factor from 15 at 0 m/s to 13 at 10 m/s.
      tracking_error = future - measurement
      frog_low_speed = (float(np.interp(speed, [0., 10.], [15., 13.])) / max(speed, 1.)) ** 2
      frog_p = tracking_error * (1. + frog_low_speed)
      error += frog_weight * (frog_p / kp - error)
      friction_error += frog_weight * (tracking_error - friction_error)
    if -deadzone < friction_error < deadzone:
      friction_error = 0.0
    # StarPilot stores the friction magnitude in torque units too; convert it
    # to acceleration before adding it to the acceleration-domain feedforward.
    threshold = 0.30  # StarPilot generic friction floor (older-Camry tuning excluded).
    ff += float(np.interp(friction_error, [-threshold, threshold], [-friction * factor, friction * factor]))
    self.pid.pos_limit = steer_max * factor
    self.pid.neg_limit = -steer_max * factor
    if speed < self.minimum_speed:
      self.pid.reset()
    output = self.pid.update(error, speed=speed, feedforward=ff,
      freeze_integrator=limited or self.driver_hold_frames > 0 or speed < max(5., self.minimum_speed) or unwinding)
    self.previous_override = override
    steer = float(np.clip(-output / factor, -steer_max, steer_max))
    if self.driver_hold_frames > 0:
      if steer * self.driver_direction <= 0:
        steer = 0.0
        self.pid.i = 0.0
      # Keep yielding through threshold chatter for one steering response delay.
      if not override:
        self.driver_hold_frames -= 1
    return steer, dict(
      valid=True, setpoint=setpoint, measurement=measurement, jerk=jerk,
      feedforward=ff, applied_offset=applied_offset)
