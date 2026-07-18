import math

from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog


START_ACCEL_RAMP_PARAM = "StartAccelRamp"
START_ACCEL_RAMP_DEFAULT = 0.0
START_ACCEL_RAMP_MIN = 0.0
START_ACCEL_RAMP_MAX = 1.0
START_ACCEL_RAMP_STEP = 0.05

# The assist is confined to the initial launch and expires if the planner does
# not ask the car to move promptly after releasing a stop.
START_ACCEL_RAMP_END_SPEED = 2.0  # m/s
START_ACCEL_RAMP_UP_RATE = 2.0  # m/s^3
START_ACCEL_RAMP_MAX_DURATION = 2.0  # seconds
START_ACCEL_BREAKAWAY_FLOOR = 0.55  # m/s^2
START_ACCEL_BREAKAWAY_END_SPEED = 0.3  # m/s
START_ACCEL_BREAKAWAY_DURATION = 0.4  # seconds


def validate_start_accel_ramp(value):
  """Validate an API-provided ramp value and return its canonical value."""
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    raise ValueError("Start acceleration ramp must be a number.")

  value = float(value)
  if not math.isfinite(value):
    raise ValueError("Start acceleration ramp must be finite.")
  if not START_ACCEL_RAMP_MIN <= value <= START_ACCEL_RAMP_MAX:
    raise ValueError(
      f"Start acceleration ramp must be between {START_ACCEL_RAMP_MIN:.2f} and {START_ACCEL_RAMP_MAX:.2f} m/s²."
    )

  steps = round((value - START_ACCEL_RAMP_MIN) / START_ACCEL_RAMP_STEP)
  normalized = START_ACCEL_RAMP_MIN + steps * START_ACCEL_RAMP_STEP
  if not math.isclose(value, normalized, rel_tol=0.0, abs_tol=1e-6):
    raise ValueError(f"Start acceleration ramp must use {START_ACCEL_RAMP_STEP:.2f} m/s² increments.")
  return round(normalized, 2)


def parse_start_accel_ramp(raw_value):
  """Fail safely to the disabled default when a stored value is malformed."""
  if raw_value is None or isinstance(raw_value, bool):
    return START_ACCEL_RAMP_DEFAULT
  if isinstance(raw_value, bytes):
    try:
      raw_value = raw_value.decode("utf-8")
    except UnicodeDecodeError:
      return START_ACCEL_RAMP_DEFAULT

  try:
    value = float(raw_value)
    return validate_start_accel_ramp(value)
  except (TypeError, ValueError):
    return START_ACCEL_RAMP_DEFAULT


class StartAccelRamp:
  """Bounded launch floor armed only by a genuine stopped-to-go transition."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.armed = False
    self.started = False
    self.elapsed = 0.0
    self.accel = 0.0

  @staticmethod
  def _log(event, **values):
    cloudlog.event("start_accel_ramp", ramp_event=event, **values)

  def _reset_with_log(self, reason, output_accel, v_ego, planner_accel):
    if self.armed or self.started:
      self._log(reason, speed=round(float(v_ego), 3), planner_accel=round(float(planner_accel), 3),
                ramp_accel=round(float(self.accel), 3), output_accel=round(float(output_accel), 3))
    self.reset()

  def update(self, output_accel, configured_accel, v_ego, planner_accel, released_from_stop,
             control_active, should_stop, brake_pressed, cruise_standstill):
    configured_accel = parse_start_accel_ramp(configured_accel)
    valid_motion = math.isfinite(v_ego) and math.isfinite(planner_accel)
    launch_permitted = (control_active and configured_accel > 0.0 and valid_motion and
                        not should_stop and not brake_pressed and not cruise_standstill and
                        v_ego < START_ACCEL_RAMP_END_SPEED)
    if not launch_permitted:
      if released_from_stop:
        blocked_by = []
        if not control_active:
          blocked_by.append("inactive")
        if configured_accel <= 0.0:
          blocked_by.append("disabled")
        if not valid_motion:
          blocked_by.append("invalid_motion")
        if should_stop:
          blocked_by.append("planner_stop")
        if brake_pressed:
          blocked_by.append("brake")
        if cruise_standstill:
          blocked_by.append("cruise_standstill")
        if valid_motion and v_ego >= START_ACCEL_RAMP_END_SPEED:
          blocked_by.append("speed")
        self._log("release_blocked", blocked_by=blocked_by, speed=round(float(v_ego), 3),
                  planner_accel=round(float(planner_accel), 3), output_accel=round(float(output_accel), 3))
      reason = "brake_intervention" if brake_pressed else "cancelled"
      self._reset_with_log(reason, output_accel, v_ego, planner_accel)
      return output_accel

    if released_from_stop:
      self.armed = True
      self.started = False
      self.elapsed = 0.0
      self.accel = 0.0
      self._log("armed", configured_accel=configured_accel, speed=round(float(v_ego), 3),
                planner_accel=round(float(planner_accel), 3), output_accel=round(float(output_accel), 3))

    if not self.armed:
      return output_accel

    self.elapsed += DT_CTRL
    if self.elapsed > START_ACCEL_RAMP_MAX_DURATION:
      self._reset_with_log("expired", output_accel, v_ego, planner_accel)
      return output_accel

    # A stop release arms the ramp, but positive planner acceleration is still
    # required before it can produce a positive floor. Once launch begins, a
    # non-positive target cancels it instead of allowing a delayed re-trigger.
    if planner_accel <= 0.0:
      if self.started:
        self._reset_with_log("planner_cancelled", output_accel, v_ego, planner_accel)
      return output_accel
    if not self.started:
      self._log("started", configured_accel=configured_accel, speed=round(float(v_ego), 3),
                planner_accel=round(float(planner_accel), 3), output_accel=round(float(output_accel), 3))
    self.started = True

    speed_scale = clip(1.0 - v_ego / START_ACCEL_RAMP_END_SPEED, 0.0, 1.0)
    target_accel = configured_accel * speed_scale
    breakaway_active = (self.elapsed <= START_ACCEL_BREAKAWAY_DURATION and
                        v_ego < START_ACCEL_BREAKAWAY_END_SPEED)
    if breakaway_active:
      # Briefly overcome brake-hold/drivetrain deadband, then hand back to the
      # configured speed-fading ramp before meaningful vehicle speed develops.
      target_accel = max(target_accel, START_ACCEL_BREAKAWAY_FLOOR)
    self.accel = min(target_accel, self.accel + START_ACCEL_RAMP_UP_RATE * DT_CTRL)
    return max(output_accel, self.accel)
