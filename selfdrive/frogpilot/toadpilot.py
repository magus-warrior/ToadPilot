import json
import math
import os
import time

from dataclasses import asdict, dataclass
from types import SimpleNamespace

from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params


TOADPILOT_CONFIG_KEY = "ToadPilotConfig"
TOADPILOT_STATUS_KEY = "ToadPilotStatus"
TOADPILOT_STEERING_STATUS_KEY = "ToadPilotSteeringStatus"
TOADPILOT_STOP_GO_STATUS_KEY = "ToadPilotStopGoStatus"
TOADPILOT_STOP_GO_LAST_ROUTE_KEY = "ToadPilotStopGoLastRoute"
TOADPILOT_TUNE_STATUS_KEY = "ToadPilotTuneStatus"
TOADPILOT_TORQUE_STATUS_KEY = "ToadPilotTorqueStatus"
TOADPILOT_TORQUE_CHECKPOINT_KEY = "ToadPilotTorqueCheckpoint"
TOADPILOT_TORQUE_LAST_ROUTE_KEY = "ToadPilotTorqueLastRoute"
TOADPILOT_RESET_TUNE_KEY = "ToadPilotResetTune"
TOADPILOT_STEERING_RESPONSE_REQUEST_KEY = "ToadPilotSteeringResponseRequest"
TOADPILOT_STEERING_RESPONSE_PROFILE = "camry_tss2_eps8965b33630_torque_1700_r15_d25_e350_rt450_v1"
TOADPILOT_STEERING_RESPONSE_BUILD = "DEV-tp1700v1-DEBUG"
TOADPILOT_STEERING_RESPONSE_LATCH_ENV = "TOADPILOT_STEERING_RESPONSE_LATCHED_PROFILE"
TOADPILOT_LEGACY_STEERING_RESPONSE_PROFILES = frozenset({"camry_tss2_response_research_v1"})
TOADPILOT_EPS_FIRMWARE = b"8965B33630\x00\x00\x00\x00\x00\x00"
TOADPILOT_STEERING_RESPONSE_PANDA_SIGNATURES = frozenset({
  # DEV-tp1700v1-DEBUG, exact isolated F4/Uno and H7 builds.
  bytes.fromhex(
    "7c9842a14e0e17bd4e96372455a8be585d8b83a873b32f7db0fe1b3b4722053c"
    "21ff847492fa2945015bd233d599583d0d5e093d34745a6f7b2418f516fa55f8"
    "fcd45a6514f7a004613f57f0de38ec9359016ac5e95349e6680aca3c4525bf5a"
    "1e9a5130c2fcd19edb8ea72f9b3603d13d342320ed54756f8613bdf964a6c78f"
  ),
  bytes.fromhex(
    "762c81fe29ab340de9db4325d77687cd06e2ca926f1f3146f2eee5a833737615"
    "c1bf193c13aa0a34b6350793ba24259f33bf50cf7bc91c2191f324ffb992c82f"
    "4e7c9d61eab12ff09fed7c97a618a3fff5fb4d012d038572bf15ac4112954b54"
    "b7df50f5d2d7939da49b51bbca13f9fbe372f88c4b47bf105f79c34776c07a14"
  ),
})
TOADPILOT_FINGERPRINT = "TOYOTA_CAMRY_TSS2"
TOADPILOT_LAT_ACCEL_OFFSET_MAX = 0.5
TOADPILOT_LAT_ACCEL_FACTOR_MIN_RATIO = 0.7
TOADPILOT_LAT_ACCEL_FACTOR_MAX_RATIO = 1.3
TOADPILOT_FRICTION_MIN_RATIO = 0.5
TOADPILOT_FRICTION_MAX_RATIO = 1.5
TOADPILOT_TUNE_SCHEMA_VERSION = 1
TOADPILOT_TORQUE_CACHE_VERSION = 2
TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION = 2
TOADPILOT_STOP_GO_DIAGNOSTIC_SCHEMA_VERSION = 1
TOADPILOT_LANE_PROB_MIN = 0.65
TOADPILOT_LANE_STD_MAX = 0.35
TOADPILOT_LANE_WIDTH_MIN = 2.6
TOADPILOT_LANE_WIDTH_MAX = 4.6
TOADPILOT_LANE_OFFSET_MAX_M = 1.2
TOADPILOT_LANE_CENTER_ENTER_M = 0.10
TOADPILOT_LANE_CENTER_RELEASE_M = 0.05
TOADPILOT_LANE_PATH_BIAS_MAX_M = 0.20
TOADPILOT_LANE_CENTER_CONFIRM_SAMPLES = 8  # new 20 Hz model frames, about 0.4 seconds
TOADPILOT_LANE_SOFT_FADE_MAX_UPDATES = 25  # 100 Hz control updates, at most 0.25 seconds
TOADPILOT_LANE_CENTER_MIN_SPEED = 5.0
TOADPILOT_LANE_CENTER_FULL_SPEED = 12.0
TOADPILOT_LANE_CENTER_CAP_MIN_SPEED = 15.0
TOADPILOT_ASSIST_MAX_DECEL = -0.75
TOADPILOT_CURVE_ASSIST_MIN_SPEED = 12.0
TOADPILOT_LANE_CENTER_MAX_BASE_ACCEL = 0.65
TOADPILOT_HIGH_AUTHORITY_BINS = (
  ("0.50-0.65", 0.50, 0.65),
  ("0.65-0.80", 0.65, 0.80),
  ("0.80-1.00", 0.80, math.nextafter(1.0, math.inf)),
)
TOADPILOT_CONFIG_VERSION = 4

TOADPILOT_PROFILES = {
  # Observe keeps the stock controller output unchanged while diagnostics run.
  "observe": SimpleNamespace(entry_assist=0.0, tracking_assist=0.0, rise_alpha=0.0, fall_alpha=0.20,
                             centering_gain=0.0, centering_max_accel=0.0,
                             centering_rise_alpha=0.0, centering_fall_alpha=0.20),
  # Adaptive adds a modest residual only while entering or undertracking a curve.
  "adaptive": SimpleNamespace(entry_assist=0.035, tracking_assist=0.080, rise_alpha=0.035, fall_alpha=0.12,
                              centering_gain=0.14, centering_max_accel=0.10,
                              centering_rise_alpha=0.025, centering_fall_alpha=0.10),
  # Full uses the same gates with more assertive demand shaping. Output remains
  # capped by the stock normalized controller. Any separately authorized Camry
  # envelope is applied downstream only after sustained saturation.
  "full": SimpleNamespace(entry_assist=0.060, tracking_assist=0.150, rise_alpha=0.050, fall_alpha=0.16,
                          centering_gain=0.22, centering_max_accel=0.16,
                          centering_rise_alpha=0.035, centering_fall_alpha=0.12),
}


@dataclass(frozen=True)
class ToadPilotConfig:
  version: int = TOADPILOT_CONFIG_VERSION
  enabled: bool = True
  profile: str = "adaptive"
  auto_tune: bool = True
  lane_centering_assist: bool = True
  smooth_lane_transitions: bool = True
  improved_stop_and_go: bool = True
  curve_assist_in_experimental: bool = False
  smart_saturation_warning: bool = True
  telemetry: bool = True


DEFAULT_TOADPILOT_CONFIG = ToadPilotConfig()
FAILSAFE_TOADPILOT_CONFIG = ToadPilotConfig(enabled=False, profile="observe", lane_centering_assist=False,
                                            smooth_lane_transitions=False, improved_stop_and_go=False)
DEFAULT_TOADPILOT_CONFIG_JSON = json.dumps(asdict(DEFAULT_TOADPILOT_CONFIG), sort_keys=True, separators=(",", ":"))
_CONFIG_FIELDS = set(asdict(DEFAULT_TOADPILOT_CONFIG))
_V1_CONFIG_FIELDS = {
  "version", "enabled", "profile", "auto_tune", "curve_assist_in_experimental",
  "smart_saturation_warning", "telemetry",
}
_V2_CONFIG_FIELDS = _V1_CONFIG_FIELDS | {"lane_centering_assist"}
_V3_CONFIG_FIELDS = _V2_CONFIG_FIELDS | {"smooth_lane_transitions"}


def normalize_toadpilot_config(data, base=None, partial=False):
  if not isinstance(data, dict):
    raise ValueError("TOADPILOT configuration must be a JSON object.")

  unknown = set(data) - _CONFIG_FIELDS
  if unknown:
    raise ValueError(f"Unknown TOADPILOT setting(s): {', '.join(sorted(unknown))}")

  normalized = asdict(base or DEFAULT_TOADPILOT_CONFIG)
  if not partial:
    normalized = asdict(DEFAULT_TOADPILOT_CONFIG)
  normalized.update(data)

  if type(normalized["version"]) is not int or normalized["version"] != TOADPILOT_CONFIG_VERSION:
    raise ValueError(f"Unsupported TOADPILOT configuration version: {normalized['version']}")
  if type(normalized["profile"]) is not str or normalized["profile"] not in TOADPILOT_PROFILES:
    raise ValueError(f"Unknown TOADPILOT profile: {normalized['profile']}")

  for field in ("enabled", "auto_tune", "lane_centering_assist", "smooth_lane_transitions", "improved_stop_and_go",
                "curve_assist_in_experimental", "smart_saturation_warning", "telemetry"):
    if type(normalized[field]) is not bool:
      raise ValueError(f"TOADPILOT setting '{field}' must be a boolean.")

  return ToadPilotConfig(**normalized)


def decode_toadpilot_config(raw):
  if raw is None:
    return DEFAULT_TOADPILOT_CONFIG
  try:
    if isinstance(raw, bytes):
      raw = raw.decode("utf-8")
    data = json.loads(raw)
    if isinstance(data, dict) and type(data.get("version")) is int:
      if data["version"] == 1:
        if set(data) != _V1_CONFIG_FIELDS:
          raise ValueError("Malformed TOADPILOT version 1 configuration.")
        data = {
          **data,
          "version": TOADPILOT_CONFIG_VERSION,
          "enabled": True,
          "profile": "adaptive",
          "lane_centering_assist": True,
          "smooth_lane_transitions": True,
          "improved_stop_and_go": True,
        }
      elif data["version"] == 2:
        if set(data) != _V2_CONFIG_FIELDS:
          raise ValueError("Malformed TOADPILOT version 2 configuration.")
        data = {
          **data,
          "version": TOADPILOT_CONFIG_VERSION,
          "smooth_lane_transitions": True,
          "improved_stop_and_go": True,
        }
      elif data["version"] == 3:
        if set(data) != _V3_CONFIG_FIELDS:
          raise ValueError("Malformed TOADPILOT version 3 configuration.")
        data = {
          **data,
          "version": TOADPILOT_CONFIG_VERSION,
          "improved_stop_and_go": True,
        }
    if not isinstance(data, dict) or set(data) != _CONFIG_FIELDS:
      raise ValueError("Malformed TOADPILOT configuration.")
    return normalize_toadpilot_config(data)
  except (TypeError, ValueError, json.JSONDecodeError):
    return FAILSAFE_TOADPILOT_CONFIG


def read_toadpilot_config(params=None):
  params = params or Params()
  return decode_toadpilot_config(params.get(TOADPILOT_CONFIG_KEY))


def write_toadpilot_config(config, params=None):
  params = params or Params()
  if isinstance(config, dict):
    config = normalize_toadpilot_config(config)
  params.put(TOADPILOT_CONFIG_KEY, json.dumps(asdict(config), sort_keys=True, separators=(",", ":")))
  return config


def read_toadpilot_steering_response_request(params=None):
  """Read the persistent, reboot-required research request.

  This is deliberately separate from ``ToadPilotConfig`` and the live
  FrogPilot toggle snapshot. Saving the request must never make a controller
  or Panda envelope effective in a running process.
  """
  return read_toadpilot_steering_response_profile(params) == TOADPILOT_STEERING_RESPONSE_PROFILE


def read_toadpilot_steering_response_profile(params=None):
  """Return the stored immutable profile identifier, if it is valid UTF-8."""
  params = params or Params()
  raw = params.get(TOADPILOT_STEERING_RESPONSE_REQUEST_KEY)
  if isinstance(raw, bytes):
    try:
      raw = raw.decode("utf-8")
    except UnicodeDecodeError:
      return None
  return raw if isinstance(raw, str) and raw else None


def latch_toadpilot_steering_response_request(params=None, environ=None):
  """Latch an exact persistent request for this manager lifetime.

  Fleet writes never update this environment value. Managed processes inherit
  it only after manager starts again, keeping controller and Panda selection
  independent from live FrogPilot toggle refreshes.
  """
  environ = os.environ if environ is None else environ
  environ.pop(TOADPILOT_STEERING_RESPONSE_LATCH_ENV, None)
  if read_toadpilot_steering_response_request(params):
    environ[TOADPILOT_STEERING_RESPONSE_LATCH_ENV] = TOADPILOT_STEERING_RESPONSE_PROFILE
    return TOADPILOT_STEERING_RESPONSE_PROFILE
  return None


def read_toadpilot_steering_response_latch(environ=None):
  environ = os.environ if environ is None else environ
  # Bind the acknowledgement to one immutable research profile. A future
  # controller/Panda bundle must use a new profile identifier, so an old saved
  # request can never become effective merely because the software was updated.
  return environ.get(TOADPILOT_STEERING_RESPONSE_LATCH_ENV) == TOADPILOT_STEERING_RESPONSE_PROFILE


def toadpilot_steering_response_panda_matched(params=None):
  """Require the exact Panda image paired with this immutable profile.

  The Camry experiment intentionally supports one internal Panda. This strict
  equality also fails closed for missing, stale, malformed, or multi-Panda
  signature snapshots.
  """
  params = params or Params()
  signatures = params.get("PandaSignatures")
  return type(signatures) is bytes and signatures in TOADPILOT_STEERING_RESPONSE_PANDA_SIGNATURES


def write_toadpilot_steering_response_request(requested, params=None):
  if type(requested) is not bool:
    raise ValueError("Experimental Camry steering response request must be a boolean.")
  params = params or Params()
  if requested:
    params.put(TOADPILOT_STEERING_RESPONSE_REQUEST_KEY, TOADPILOT_STEERING_RESPONSE_PROFILE)
  else:
    params.remove(TOADPILOT_STEERING_RESPONSE_REQUEST_KEY)
  return requested


def config_from_toggles(toggles):
  try:
    return normalize_toadpilot_config({
      "version": TOADPILOT_CONFIG_VERSION,
      "enabled": getattr(toggles, "toadpilot_enabled"),
      "profile": getattr(toggles, "toadpilot_profile"),
      "auto_tune": getattr(toggles, "toadpilot_auto_tune"),
      "lane_centering_assist": getattr(toggles, "toadpilot_lane_centering_assist"),
      "smooth_lane_transitions": getattr(toggles, "toadpilot_smooth_lane_transitions"),
      "improved_stop_and_go": getattr(toggles, "toadpilot_improved_stop_and_go"),
      "curve_assist_in_experimental": getattr(toggles, "toadpilot_curve_assist_in_experimental"),
      "smart_saturation_warning": getattr(toggles, "toadpilot_smart_saturation_warning"),
      "telemetry": getattr(toggles, "toadpilot_telemetry"),
    })
  except (AttributeError, TypeError, ValueError):
    return FAILSAFE_TOADPILOT_CONFIG


def read_json_status(key, params=None):
  params = params or Params("/dev/shm/params")
  raw = params.get(key)
  try:
    status = json.loads(raw or b"{}")
    return status if isinstance(status, dict) else {}
  except (TypeError, ValueError, json.JSONDecodeError):
    return {}


def valid_toadpilot_torque_summary(summary):
  """Validate persistent torque-trial provenance before presenting it as evidence."""
  if not isinstance(summary, dict):
    return False
  published_at = summary.get("published_at_unix")
  route_started_at = summary.get("route_started_at_unix")
  extended_samples = summary.get("extended_output_samples")
  route_complete = summary.get("route_complete")
  if (isinstance(published_at, bool) or not isinstance(published_at, (int, float)) or
      isinstance(route_started_at, bool) or not isinstance(route_started_at, (int, float))):
    return False

  if (summary.get("schema_version") != TOADPILOT_TORQUE_DIAGNOSTIC_SCHEMA_VERSION or
      summary.get("fingerprint") != TOADPILOT_FINGERPRINT or
      summary.get("steering_response_profile") != TOADPILOT_STEERING_RESPONSE_PROFILE or
      summary.get("steering_response_build") != TOADPILOT_STEERING_RESPONSE_BUILD or
      summary.get("command_semantics") != "controller_command_before_panda_or_eps" or
      summary.get("controller_authorized") is not True or
      type(extended_samples) is not int or extended_samples < 0 or
      type(route_complete) is not bool or
      not isinstance(summary.get("route_id"), str) or not summary["route_id"].strip() or
      summary["route_id"] != summary["route_id"].strip() or
      not math.isfinite(float(published_at)) or not math.isfinite(float(route_started_at)) or
      float(route_started_at) > float(published_at)):
    return False

  completed_at = summary.get("completed_at_unix")
  if route_complete:
    if isinstance(completed_at, bool) or not isinstance(completed_at, (int, float)):
      return False
    if (not math.isfinite(float(completed_at)) or float(completed_at) < float(published_at) or
        summary.get("summary_source") != "manager_offroad_transition"):
      return False
  elif completed_at is not None or summary.get("summary_source") != "torqued_live":
    return False
  return True


def persist_toadpilot_torque_last_route(params=None, status_params=None, completed_at_unix=None):
  """Synchronously preserve the final validated onroad torque diagnostic."""
  params = params or Params()
  status_params = status_params or Params("/dev/shm/params")
  summary = read_json_status(TOADPILOT_TORQUE_STATUS_KEY, status_params)
  if not valid_toadpilot_torque_summary(summary):
    return False

  route_has_diagnostic = (summary["extended_output_samples"] > 0 or
                          summary.get("temporary_fault_seen") is True or
                          summary.get("permanent_fault_seen") is True)
  if not route_has_diagnostic:
    return False

  completed_at = time.time() if completed_at_unix is None else completed_at_unix
  if (isinstance(completed_at, bool) or not isinstance(completed_at, (int, float)) or
      not math.isfinite(completed_at) or float(completed_at) < float(summary["published_at_unix"])):
    return False

  final_summary = dict(summary)
  final_summary["route_complete"] = True
  final_summary["completed_at_unix"] = round(float(completed_at), 3)
  final_summary["summary_source"] = "manager_offroad_transition"
  if not valid_toadpilot_torque_summary(final_summary):
    return False
  params.put(TOADPILOT_TORQUE_LAST_ROUTE_KEY,
             json.dumps(final_summary, sort_keys=True, separators=(",", ":")))
  return True


def valid_toadpilot_stop_go_summary(summary):
  if not isinstance(summary, dict):
    return False
  published_at = summary.get("published_at_unix")
  route_started_at = summary.get("route_started_at_unix")
  route_complete = summary.get("route_complete")
  event_sequence = summary.get("event_sequence")
  if (summary.get("schema_version") != TOADPILOT_STOP_GO_DIAGNOSTIC_SCHEMA_VERSION or
      summary.get("fingerprint") != TOADPILOT_FINGERPRINT or
      not isinstance(summary.get("route_id"), str) or not summary["route_id"].strip() or
      type(route_complete) is not bool or type(event_sequence) is not int or event_sequence < 0 or
      isinstance(published_at, bool) or not isinstance(published_at, (int, float)) or
      isinstance(route_started_at, bool) or not isinstance(route_started_at, (int, float)) or
      not math.isfinite(float(published_at)) or not math.isfinite(float(route_started_at)) or
      float(route_started_at) > float(published_at)):
    return False
  completed_at = summary.get("completed_at_unix")
  if route_complete:
    return (not isinstance(completed_at, bool) and isinstance(completed_at, (int, float)) and
            math.isfinite(float(completed_at)) and float(completed_at) >= float(published_at) and
            summary.get("summary_source") == "manager_offroad_transition")
  return completed_at is None and summary.get("summary_source") == "controlsd_live"


def persist_toadpilot_stop_go_last_route(params=None, status_params=None, completed_at_unix=None):
  """Preserve one compact stop/go summary when the device goes offroad."""
  params = params or Params()
  status_params = status_params or Params("/dev/shm/params")
  summary = read_json_status(TOADPILOT_STOP_GO_STATUS_KEY, status_params)
  if not valid_toadpilot_stop_go_summary(summary) or summary["event_sequence"] == 0:
    return False
  completed_at = time.time() if completed_at_unix is None else completed_at_unix
  if (isinstance(completed_at, bool) or not isinstance(completed_at, (int, float)) or
      not math.isfinite(float(completed_at)) or float(completed_at) < float(summary["published_at_unix"])):
    return False
  final_summary = dict(summary)
  final_summary.update({
    "route_complete": True,
    "completed_at_unix": round(float(completed_at), 3),
    "summary_source": "manager_offroad_transition",
  })
  if not valid_toadpilot_stop_go_summary(final_summary):
    return False
  params.put(TOADPILOT_STOP_GO_LAST_ROUTE_KEY,
             json.dumps(final_summary, sort_keys=True, separators=(",", ":")))
  return True


def _new_high_authority_stats():
  return {
    "samples": 0,
    "sum_abs_steer": 0.0,
    "sum_abs_lateral_accel": 0.0,
    "sum_steer_sq": 0.0,
    "sum_steer_lat_accel": 0.0,
    "sum_ratio": 0.0,
    "sum_ratio_sq": 0.0,
  }


class ToadPilotHighAuthorityObserver:
  """Observe high steering authority without feeding samples into live tuning."""

  def __init__(self, fingerprint=TOADPILOT_FINGERPRINT):
    self.fingerprint = fingerprint
    self.candidate_samples = 0
    self.accepted_samples = 0
    self.rejected_samples = 0
    self.direction_stats = {direction: _new_high_authority_stats() for direction in ("left", "right")}
    self.bin_stats = {
      direction: {name: _new_high_authority_stats() for name, _, _ in TOADPILOT_HIGH_AUTHORITY_BINS}
      for direction in ("left", "right")
    }

  @staticmethod
  def _add(stats, steer, lateral_accel):
    ratio = lateral_accel / steer
    stats["samples"] += 1
    stats["sum_abs_steer"] += steer
    stats["sum_abs_lateral_accel"] += lateral_accel
    stats["sum_steer_sq"] += steer * steer
    stats["sum_steer_lat_accel"] += steer * lateral_accel
    stats["sum_ratio"] += ratio
    stats["sum_ratio_sq"] += ratio * ratio

  @staticmethod
  def _summary(stats):
    samples = stats["samples"]
    if samples == 0:
      return {
        "samples": 0,
        "mean_abs_steer": None,
        "mean_abs_lateral_accel": None,
        "lat_accel_per_steer": None,
        "mean_sample_ratio": None,
        "sample_ratio_stddev": None,
      }

    mean_ratio = stats["sum_ratio"] / samples
    ratio_variance = 0.0
    if samples > 1:
      ratio_variance = max((stats["sum_ratio_sq"] - stats["sum_ratio"] ** 2 / samples) / (samples - 1), 0.0)
    return {
      "samples": samples,
      "mean_abs_steer": round(stats["sum_abs_steer"] / samples, 6),
      "mean_abs_lateral_accel": round(stats["sum_abs_lateral_accel"] / samples, 6),
      "lat_accel_per_steer": round(stats["sum_steer_lat_accel"] / stats["sum_steer_sq"], 6),
      "mean_sample_ratio": round(mean_ratio, 6),
      "sample_ratio_stddev": round(math.sqrt(ratio_variance), 6),
    }

  def add_sample(self, steer, lateral_accel):
    try:
      steer = float(steer)
      lateral_accel = float(lateral_accel)
    except (TypeError, ValueError):
      return False

    if not math.isfinite(steer) or not math.isfinite(lateral_accel):
      self.rejected_samples += 1
      return False

    abs_steer = abs(steer)
    if abs_steer <= TOADPILOT_HIGH_AUTHORITY_BINS[0][1] or abs_steer >= TOADPILOT_HIGH_AUTHORITY_BINS[-1][2]:
      return False

    self.candidate_samples += 1
    abs_lateral_accel = abs(lateral_accel)
    if abs_lateral_accel < 0.25 or abs_lateral_accel > 4.0 or steer * lateral_accel <= 0.0:
      self.rejected_samples += 1
      return False

    bin_name = next(name for name, lower, upper in TOADPILOT_HIGH_AUTHORITY_BINS if lower <= abs_steer < upper)
    direction = "left" if steer > 0.0 else "right"
    self._add(self.direction_stats[direction], abs_steer, abs_lateral_accel)
    self._add(self.bin_stats[direction][bin_name], abs_steer, abs_lateral_accel)
    self.accepted_samples += 1
    return True

  def snapshot(self, collecting=True):
    return {
      "schema_version": TOADPILOT_TUNE_SCHEMA_VERSION,
      "observe_only": True,
      "applied_to_tune": False,
      "collecting": bool(collecting),
      "fingerprint": self.fingerprint,
      "published_at_unix": round(time.time(), 3),
      "candidate_samples": self.candidate_samples,
      "accepted_samples": self.accepted_samples,
      "rejected_samples": self.rejected_samples,
      "acceptance": {
        "absolute_steer": "0.50 < steer <= 1.00",
        "absolute_lateral_accel": "0.25 <= lateral_accel <= 4.00",
        "matching_direction_required": True,
      },
      "directions": {
        direction: self._summary(self.direction_stats[direction])
        for direction in ("left", "right")
      },
      "bins": {
        direction: {
          name: self._summary(self.bin_stats[direction][name])
          for name, _, _ in TOADPILOT_HIGH_AUTHORITY_BINS
        }
        for direction in ("left", "right")
      },
    }


@dataclass(frozen=True)
class ToadPilotLaneContext:
  """Validated model context used by the lane-centering controller."""

  valid: bool = False
  reason: str = "model_unavailable"
  frame_id: int = -1
  lane_offset_m: float = 0.0
  path_lane_bias_m: float = 0.0
  path_y_m: float = 0.0
  lane_width_m: float = 0.0
  confidence: float = 0.0
  lookahead_m: float = 0.0


def _finite_xy(series):
  try:
    xs = [float(value) for value in series.x]
    ys = [float(value) for value in series.y]
  except (AttributeError, TypeError, ValueError):
    return None

  if len(xs) != len(ys) or len(xs) < 2 or not all(math.isfinite(value) for value in (*xs, *ys)):
    return None
  if any(right <= left for left, right in zip(xs, xs[1:])):
    return None
  return xs, ys


def _sample_xy(xy, distance):
  xs, ys = xy
  if distance < xs[0] or distance > xs[-1]:
    raise ValueError("Model series does not cover the requested distance.")
  return float(interp(distance, xs, ys))


def extract_toadpilot_lane_context(model_data, v_ego, camera_offset=0.04):
  """Extract a fail-closed lane midpoint and model-path agreement signal."""

  try:
    frame_id = int(model_data.frameId)
    lane_lines = model_data.laneLines
    lane_probs = [float(value) for value in model_data.laneLineProbs]
    lane_stds = [float(value) for value in model_data.laneLineStds]
    speed = float(v_ego)
    camera_offset = float(camera_offset)
  except (AttributeError, TypeError, ValueError):
    return ToadPilotLaneContext(reason="model_invalid")

  if frame_id < 0 or not math.isfinite(speed) or not math.isfinite(camera_offset):
    return ToadPilotLaneContext(reason="model_invalid", frame_id=frame_id)
  if len(lane_lines) < 3 or len(lane_probs) < 3 or len(lane_stds) < 3:
    return ToadPilotLaneContext(reason="lane_data_missing", frame_id=frame_id)

  confidence = min(lane_probs[1], lane_probs[2])
  max_std = max(lane_stds[1], lane_stds[2])
  if (not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in (lane_probs[1], lane_probs[2])) or
      confidence < TOADPILOT_LANE_PROB_MIN):
    return ToadPilotLaneContext(reason="lane_confidence_low", frame_id=frame_id, confidence=max(confidence, 0.0))
  if (not all(math.isfinite(value) and value >= 0.0 for value in (lane_stds[1], lane_stds[2])) or
      max_std > TOADPILOT_LANE_STD_MAX):
    return ToadPilotLaneContext(reason="lane_uncertainty_high", frame_id=frame_id, confidence=confidence)

  left = _finite_xy(lane_lines[1])
  right = _finite_xy(lane_lines[2])
  try:
    position = _finite_xy(model_data.position)
  except AttributeError:
    position = None
  if left is None or right is None or position is None:
    return ToadPilotLaneContext(reason="lane_geometry_invalid", frame_id=frame_id, confidence=confidence)

  near_distances = (0.0, 2.5, 5.0, 7.5, 10.0)
  lookahead = float(clip(max(speed, 0.0) * 0.8, 12.0, 30.0))
  try:
    midpoints = []
    widths = []
    for distance in near_distances:
      left_y = _sample_xy(left, distance)
      right_y = _sample_xy(right, distance)
      midpoints.append(0.5 * (left_y + right_y))
      widths.append(right_y - left_y)

    left_far = _sample_xy(left, lookahead)
    right_far = _sample_xy(right, lookahead)
    path_y = _sample_xy(position, lookahead)
  except (TypeError, ValueError):
    return ToadPilotLaneContext(reason="lane_range_short", frame_id=frame_id, confidence=confidence,
                                lookahead_m=lookahead)

  if any(width < TOADPILOT_LANE_WIDTH_MIN or width > TOADPILOT_LANE_WIDTH_MAX for width in (*widths, right_far - left_far)):
    return ToadPilotLaneContext(reason="lane_width_invalid", frame_id=frame_id, confidence=confidence,
                                lookahead_m=lookahead)

  # A near-field linear fit is less sensitive to a noisy first lane point. Its
  # intercept estimates the lane midpoint at the vehicle while rejecting slope.
  mean_x = sum(near_distances) / len(near_distances)
  mean_y = sum(midpoints) / len(midpoints)
  denominator = sum((distance - mean_x) ** 2 for distance in near_distances)
  slope = sum((distance - mean_x) * (midpoint - mean_y)
              for distance, midpoint in zip(near_distances, midpoints)) / denominator
  lane_offset = mean_y - slope * mean_x + camera_offset
  # The model path and lane lines share the same raw model frame. Camera
  # correction is needed for vehicle cross-track offset, but not this comparison.
  far_lane_center = 0.5 * (left_far + right_far)
  path_lane_bias = path_y - far_lane_center
  values = (lane_offset, path_lane_bias, path_y, widths[0], lookahead)
  if not all(math.isfinite(value) for value in values):
    return ToadPilotLaneContext(reason="lane_geometry_invalid", frame_id=frame_id, confidence=confidence)
  if abs(lane_offset) > TOADPILOT_LANE_OFFSET_MAX_M:
    return ToadPilotLaneContext(reason="lane_offset_implausible", frame_id=frame_id, confidence=confidence,
                                lookahead_m=lookahead)

  return ToadPilotLaneContext(
    valid=True,
    reason="valid",
    frame_id=frame_id,
    lane_offset_m=lane_offset,
    path_lane_bias_m=path_lane_bias,
    path_y_m=path_y,
    lane_width_m=widths[0],
    confidence=confidence,
    lookahead_m=lookahead,
  )


def toadpilot_steer_saturated(steer_limited, applied_steer):
  """Distinguish sustained high-authority limiting from normal Toyota rate limiting."""
  if not steer_limited:
    return False
  try:
    steer = float(applied_steer)
  except (TypeError, ValueError):
    return True
  return not math.isfinite(steer) or abs(steer) >= 0.80


class ToadPilotLaneCentering:
  """Small, confidence-gated curvature trim toward the detected lane center."""

  SOFT_CONTEXT_REASONS = frozenset({
    "lane_confidence_low",
    "lane_uncertainty_high",
    "lane_width_invalid",
  })

  def __init__(self):
    self.correction_curvature = 0.0
    self.filtered_lane_offset = 0.0
    self.filter_initialized = False
    self.last_frame_id = -1
    self.same_frame_updates = 0
    self.candidate_sign = 0
    self.confirm_samples = 0
    self.soft_gate_updates = 0
    self.soft_gate_reason = None
    self.soft_gate_clear_filter = False
    self.active = False
    self.gate_reason = "not_started"
    self.last_context = ToadPilotLaneContext()

  def reset(self, reason="reset", clear_filter=False):
    self.correction_curvature = 0.0
    self.candidate_sign = 0
    self.confirm_samples = 0
    self.soft_gate_updates = 0
    self.soft_gate_reason = None
    self.soft_gate_clear_filter = False
    self.active = False
    self.gate_reason = reason
    if clear_filter:
      self.filtered_lane_offset = 0.0
      self.filter_initialized = False

  def _hard_gate(self, reason, clear_filter=False):
    # Preserve a pending geometry flush if a different hard gate interrupts an
    # invalid-evidence fade before it completes.
    self.reset(reason, clear_filter=clear_filter or self.soft_gate_clear_filter)
    return 0.0

  def _soft_gate(self, config, reason, speed, clear_filter_on_end=False):
    """Monotonically remove an existing trim during brief soft-evidence gaps."""
    if (not config.smooth_lane_transitions or self.correction_curvature == 0.0 or
        self.soft_gate_updates >= TOADPILOT_LANE_SOFT_FADE_MAX_UPDATES):
      return self._hard_gate(reason, clear_filter=clear_filter_on_end or self.soft_gate_clear_filter)

    if self.soft_gate_updates == 0:
      self.soft_gate_reason = reason
    self.soft_gate_clear_filter |= clear_filter_on_end
    self.soft_gate_updates += 1
    self.active = False
    self.gate_reason = f"fading_{self.soft_gate_reason}"
    profile = TOADPILOT_PROFILES[config.profile]
    self.correction_curvature += profile.centering_fall_alpha * (0.0 - self.correction_curvature)
    curvature_cap = profile.centering_max_accel / max(speed ** 2, TOADPILOT_LANE_CENTER_CAP_MIN_SPEED ** 2)
    self.correction_curvature = float(clip(self.correction_curvature, -curvature_cap, curvature_cap))
    if (self.soft_gate_updates >= TOADPILOT_LANE_SOFT_FADE_MAX_UPDATES or
        abs(self.correction_curvature) < 1e-9):
      return self._hard_gate(self.soft_gate_reason or reason, clear_filter=self.soft_gate_clear_filter)
    return self.correction_curvature

  def update(self, config, base_desired_curvature, v_ego, context, lateral_active,
             steering_pressed, steering_rate_deg, steer_limited, experimental_mode,
             lane_change_active, blinker_active, a_ego=0.0, brake_pressed=False,
             longitudinal_stopping=False):
    self.last_context = context if isinstance(context, ToadPilotLaneContext) else ToadPilotLaneContext(reason="model_invalid")
    try:
      base_curvature = float(base_desired_curvature)
      speed = float(v_ego)
      longitudinal_accel = float(a_ego)
      steering_rate = float(steering_rate_deg)
    except (TypeError, ValueError):
      self._hard_gate("input_invalid", clear_filter=True)
      return base_desired_curvature

    if not config.enabled:
      return base_curvature + self._hard_gate("disabled", clear_filter=True)
    if config.profile == "observe":
      return base_curvature + self._hard_gate("observe_profile", clear_filter=True)
    if not config.lane_centering_assist:
      return base_curvature + self._hard_gate("toggle_off", clear_filter=True)
    if not lateral_active:
      return base_curvature + self._hard_gate("lateral_inactive", clear_filter=True)
    if (not math.isfinite(base_curvature) or not math.isfinite(speed) or
        not math.isfinite(longitudinal_accel) or not math.isfinite(steering_rate)):
      return base_curvature + self._hard_gate("input_invalid", clear_filter=True)
    if speed < TOADPILOT_LANE_CENTER_MIN_SPEED:
      return base_curvature + self._hard_gate("speed_low", clear_filter=True)
    if brake_pressed:
      return base_curvature + self._hard_gate("brake_pressed", clear_filter=True)
    if longitudinal_stopping:
      return base_curvature + self._hard_gate("longitudinal_stopping", clear_filter=True)
    if longitudinal_accel < TOADPILOT_ASSIST_MAX_DECEL:
      return base_curvature + self._hard_gate("decelerating", clear_filter=True)
    if steering_pressed:
      return base_curvature + self._hard_gate("driver_steering", clear_filter=True)
    if steer_limited:
      return base_curvature + self._hard_gate("steer_limited", clear_filter=True)
    if abs(steering_rate) >= 75.0:
      return base_curvature + self._hard_gate("steering_rate_high", clear_filter=True)
    if lane_change_active:
      return base_curvature + self._hard_gate("lane_change", clear_filter=True)
    if blinker_active:
      return base_curvature + self._hard_gate("blinker", clear_filter=True)
    if not self.last_context.valid:
      if self.last_context.reason in self.SOFT_CONTEXT_REASONS:
        return base_curvature + self._soft_gate(config, self.last_context.reason, speed, clear_filter_on_end=True)
      return base_curvature + self._hard_gate(self.last_context.reason, clear_filter=True)
    if (abs(self.last_context.lane_offset_m) >= TOADPILOT_LANE_CENTER_RELEASE_M and
        self.correction_curvature * self.last_context.lane_offset_m < 0.0):
      # Direction changes remain an immediate gate even when soft transition
      # smoothing is enabled. Never fade stale assistance through a reversal.
      return base_curvature + self._hard_gate("direction_reversal", clear_filter=True)
    if abs(base_curvature * speed ** 2) > TOADPILOT_LANE_CENTER_MAX_BASE_ACCEL:
      return base_curvature + self._hard_gate("curve_too_strong")
    if abs(self.last_context.path_lane_bias_m) > TOADPILOT_LANE_PATH_BIAS_MAX_M:
      return base_curvature + self._hard_gate("planner_bias")
    base_lateral_accel = base_curvature * speed ** 2
    if (abs(base_lateral_accel) > 0.05 and
        base_lateral_accel * self.last_context.lane_offset_m <= 0.0):
      return base_curvature + self._hard_gate("curve_direction")
    if (abs(self.last_context.lane_offset_m) >= TOADPILOT_LANE_CENTER_ENTER_M and
        self.last_context.path_y_m * self.last_context.lane_offset_m <= 0.0):
      return base_curvature + self._soft_gate(config, "planner_direction", speed)
    if self.soft_gate_updates:
      # Once a soft fade starts, valid/invalid alternation cannot re-arm or
      # increase the correction. Finish the same cumulative bounded fade, then
      # require fresh confirmation before assistance can return.
      return base_curvature + self._soft_gate(config, self.soft_gate_reason or "soft_evidence", speed)

    new_frame = self.last_context.frame_id != self.last_frame_id
    if new_frame:
      self.last_frame_id = self.last_context.frame_id
      self.same_frame_updates = 0
      if not self.filter_initialized:
        self.filtered_lane_offset = self.last_context.lane_offset_m
        self.filter_initialized = True
      else:
        self.filtered_lane_offset += 0.20 * (self.last_context.lane_offset_m - self.filtered_lane_offset)
    else:
      self.same_frame_updates += 1
      if self.same_frame_updates > 25:
        return base_curvature + self._hard_gate("model_stale", clear_filter=True)

    deadband = TOADPILOT_LANE_CENTER_RELEASE_M if self.active else TOADPILOT_LANE_CENTER_ENTER_M
    offset_sign = 1 if self.filtered_lane_offset > deadband else -1 if self.filtered_lane_offset < -deadband else 0
    if offset_sign == 0:
      self.candidate_sign = 0
      self.confirm_samples = 0
      self.active = False
      self.gate_reason = "centered"
      target = 0.0
    else:
      if new_frame:
        if offset_sign != self.candidate_sign:
          self.candidate_sign = offset_sign
          self.confirm_samples = 1
        else:
          self.confirm_samples += 1

      if self.confirm_samples < TOADPILOT_LANE_CENTER_CONFIRM_SAMPLES:
        self.active = False
        self.gate_reason = "confirming"
        target = 0.0
      else:
        profile = TOADPILOT_PROFILES[config.profile]
        effective_offset = self.filtered_lane_offset - offset_sign * TOADPILOT_LANE_CENTER_RELEASE_M
        target = profile.centering_gain * 2.0 * effective_offset / self.last_context.lookahead_m ** 2
        # Do not allow the acceleration cap to become progressively larger
        # curvature trim as the car slows. The learner and initial road testing
        # are high-speed biased, so low speed falls back toward the stock plan.
        curvature_cap = profile.centering_max_accel / max(speed ** 2, TOADPILOT_LANE_CENTER_CAP_MIN_SPEED ** 2)
        target = float(clip(target, -curvature_cap, curvature_cap))
        # Avoid the observed hard 27 mph handoff. Launch corrections ramp from
        # zero and remain smaller than the previously validated high-speed trim.
        target *= float(interp(speed, [TOADPILOT_LANE_CENTER_MIN_SPEED, TOADPILOT_LANE_CENTER_FULL_SPEED],
                               [0.0, 1.0]))
        self.active = True
        self.gate_reason = "active"

    profile = TOADPILOT_PROFILES[config.profile]
    if target and self.correction_curvature * target < 0.0:
      self.correction_curvature = 0.0
    alpha = profile.centering_rise_alpha if abs(target) > abs(self.correction_curvature) else profile.centering_fall_alpha
    self.correction_curvature += alpha * (target - self.correction_curvature)
    curvature_cap = profile.centering_max_accel / max(speed ** 2, TOADPILOT_LANE_CENTER_CAP_MIN_SPEED ** 2)
    self.correction_curvature = float(clip(self.correction_curvature, -curvature_cap, curvature_cap))
    return base_curvature + self.correction_curvature

  def snapshot(self):
    context = self.last_context
    return {
      "lane_centering_active": bool(self.active),
      "lane_centering_gate_reason": self.gate_reason,
      "lane_offset_m": round(float(self.filtered_lane_offset), 6) if self.filter_initialized else None,
      "path_lane_bias_m": round(float(context.path_lane_bias_m), 6) if context.valid else None,
      "lane_width_m": round(float(context.lane_width_m), 6) if context.valid else None,
      "lane_centering_confidence": round(float(context.confidence), 6) if context.valid else None,
      "lane_centering_confirm_samples": int(self.confirm_samples),
      "lane_centering_soft_gate_updates": int(self.soft_gate_updates),
      "lane_centering_curvature_delta": round(float(self.correction_curvature), 8),
    }


class ToadPilotLateralAssist:
  """Camry-specific demand shaping inside the stock normalized torque scale."""

  def __init__(self):
    self.residual = 0.0
    self.last_curvature = 0.0
    self.last_status_publish = 0.0
    self.status_params = Params("/dev/shm/params")

  def reset(self, desired_curvature=0.0):
    self.residual = 0.0
    self.last_curvature = desired_curvature

  def update(self, config, desired_curvature, desired_lateral_accel, actual_lateral_accel,
             steering_pressed, steering_rate_deg, steer_limited, experimental_mode,
             v_ego=20.0, a_ego=0.0, brake_pressed=False, longitudinal_stopping=False):
    profile = TOADPILOT_PROFILES[config.profile]
    try:
      speed = float(v_ego)
      longitudinal_accel = float(a_ego)
    except (TypeError, ValueError):
      self.reset(desired_curvature)
      return 0.0
    curvature_rate = (desired_curvature - self.last_curvature) / 0.01
    self.last_curvature = desired_curvature

    direction = 1.0 if desired_lateral_accel > 0.0 else -1.0 if desired_lateral_accel < 0.0 else 0.0
    curve_strength = float(clip((abs(desired_lateral_accel) - 0.25) / 1.75, 0.0, 1.0))
    experimental_allowed = not experimental_mode or config.curve_assist_in_experimental
    hard_gate = (not config.enabled or config.profile == "observe" or steering_pressed or steer_limited or
                 abs(steering_rate_deg) >= 75.0 or not experimental_allowed or
                 not math.isfinite(speed) or not math.isfinite(longitudinal_accel) or
                 speed < TOADPILOT_CURVE_ASSIST_MIN_SPEED or
                 longitudinal_accel < TOADPILOT_ASSIST_MAX_DECEL or brake_pressed or longitudinal_stopping or
                 curvature_rate * desired_curvature < 0.0 or direction == 0.0 or curve_strength <= 0.0)
    if hard_gate:
      self.reset(desired_curvature)
      return 0.0

    entering_curve = float(clip(abs(curvature_rate) / 0.01, 0.0, 1.0))
    undertracking = float(clip(direction * (desired_lateral_accel - actual_lateral_accel) / 0.75, 0.0, 1.0))
    target_magnitude = curve_strength * (profile.entry_assist * entering_curve + profile.tracking_assist * undertracking)
    target = direction * target_magnitude

    # Never carry assistance into the opposite steering direction.
    if target and self.residual * target < 0.0:
      self.residual = 0.0
    alpha = profile.rise_alpha if abs(target) > abs(self.residual) else profile.fall_alpha
    self.residual += alpha * (target - self.residual)
    max_residual = profile.entry_assist + profile.tracking_assist
    self.residual = float(clip(self.residual, -max_residual, max_residual))
    return self.residual

  def status_due(self, config):
    if not config.enabled or not config.telemetry:
      return False
    return time.monotonic() - self.last_status_publish >= 1.0

  def publish_status(self, config, **status):
    if not self.status_due(config):
      return
    now = time.monotonic()
    self.last_status_publish = now
    payload = {
      "timestamp_monotonic": round(now, 3),
      "published_at_unix": round(time.time(), 3),
      "config": asdict(config),
      "residual_assist": round(self.residual, 6),
      **status,
    }
    self.status_params.put_nonblocking(TOADPILOT_STATUS_KEY, json.dumps(payload, sort_keys=True, separators=(",", ":")))
