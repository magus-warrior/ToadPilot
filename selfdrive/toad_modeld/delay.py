"""Bounded optional use of a fresh, confident learned steering delay."""
import math
import json
import time
from pathlib import Path


class DelaySelector:
  def __init__(self, fallback):
    self.fallback = float(fallback)
    self.current = self.fallback

  def update(self, enabled, estimate, valid, age):
    if not enabled:
      self.current = self.fallback
      return self.current
    target = self.fallback
    if (valid and math.isfinite(age) and 0 <= age <= 1.0 and estimate is not None and
        str(estimate.status) == 'estimated' and estimate.version == 1 and
        estimate.validBlocks >= 5 and estimate.calPerc == 100 and
        math.isfinite(estimate.lateralDelay) and 0.15 <= estimate.lateralDelay <= 0.65 and
        math.isfinite(estimate.lateralDelayEstimateStd) and 0 <= estimate.lateralDelayEstimateStd <= 0.1):
      target = float(estimate.lateralDelay)
    # At 20 Hz, at most 0.1 seconds of horizon change per second.
    self.current += max(-0.005, min(0.005, target - self.current))
    return self.current


def read_status(path=Path('/dev/shm/toadpilot_lateral_delay.json')):
  try:
    data = json.loads(path.read_text())
    if (not isinstance(data, dict) or data.get('valid') is not True or
        data.get('status') not in ('unestimated', 'estimated', 'invalid') or
        type(data.get('progress')) is not int or not 0 <= data['progress'] <= 100 or
        type(data.get('delay')) not in (int, float) or not math.isfinite(data['delay']) or
        not 0 <= data['delay'] <= 1 or
        type(data.get('updated')) not in (int, float) or not 0 <= time.time() - data['updated'] <= 3):
      return {}
    return data
  except (OSError, ValueError, TypeError):
    return {}
