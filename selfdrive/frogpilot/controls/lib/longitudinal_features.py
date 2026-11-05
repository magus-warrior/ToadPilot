"""Small, bounded Camry longitudinal helpers; no IO or additional processes."""
import math

from openpilot.common.realtime import DT_CTRL


def finite(value):
  return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def smooth_stop_request(stock_request, previous_request, planner_request, speed, measured_accel, *, allowed):
  """Ease only a gentle final approach, never below the planner's braking demand.

  The stock stopping ramp is maintained separately by the caller. Relief is
  limited to 0.3 m/s² relative to that ramp and removed at standstill, on a
  stronger plan, or on any invalid/override input. This is not a braking cap.
  """
  if (not allowed or not all(finite(v) for v in
                            (stock_request, previous_request, planner_request, speed, measured_accel)) or
      not 0.05 < speed < 0.5 or not -1.2 < measured_accel < -0.15 or
      not -0.8 < planner_request <= 0.0):
    return stock_request
  # Only the low-speed part of the approach receives relief. Keep at least
  # 0.5 m/s² requested braking and never weaken a stronger planner request.
  relief = 0.3 * (0.5 - speed) / 0.45
  target = min(stock_request + relief, planner_request, -0.5)
  return max(stock_request, min(target, previous_request + 0.5 * DT_CTRL))


class StopReleaseLatch:
  """Remember a stopped state while waiting for the first positive request.

  The old edge-only release missed launches that entered PID at zero accel.
  A stop/brake/inactive/invalid state cannot authorize positive actuation.
  """
  def __init__(self):
    self.pending = False
    self.elapsed = 0.0

  def update(self, *, enabled, active, stopping, brake_pressed, speed, accel):
    if not enabled or not active or brake_pressed or not finite(speed) or not finite(accel) or not 0 <= speed < 2.0:
      self.pending = False
      self.elapsed = 0.0
      return False
    if stopping:
      self.pending = True
      self.elapsed = 0.0
      return False
    if not self.pending:
      return False
    self.elapsed += DT_CTRL
    if self.elapsed > 2.0:
      self.pending = False
      return False
    if accel > 0.0:
      self.pending = False
      return True
    return False


def smooth_brake_release(request, previous, planner_request, speed, *, allowed):
  """Ease mild brake release only; stronger braking always passes immediately.

  Never weaken the controller request. Extra braking is limited to 0.15 m/s²,
  only while the plan still requests deceleration above 2 m/s. Bypass stopping,
  overrides and launch in the caller, and abandon history on every bypass.
  """
  if (not allowed or not all(finite(v) for v in (request, previous, planner_request, speed)) or
      speed < 2.0 or not -2.0 < previous < request < 0.0 or not -2.0 < planner_request < 0.0):
    return request
  return max(request - 0.15, min(request, previous + 0.8 * DT_CTRL))
