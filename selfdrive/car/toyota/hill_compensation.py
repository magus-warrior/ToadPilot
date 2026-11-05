"""Toyota transient pitch feedforward from opendbc d1bd15ec (MIT)."""
import math
from openpilot.common.filter_simple import FirstOrderFilter


class HillCompensation:
  def __init__(self, dt):
    self.fast = FirstOrderFilter(0.0, 0.25, dt, initialized=False)
    self.slow = FirstOrderFilter(0.0, 1.5, dt, initialized=False)
    self.valid = False

  def update(self, orientation):
    self.valid = len(orientation) == 3 and all(math.isfinite(x) for x in orientation) and abs(orientation[1]) < 0.5
    if self.valid:
      self.fast.update(orientation[1])
      self.slow.update(orientation[1])
    else:
      # Missing attitude must not leave an old slope correction active.
      self.fast.initialized = self.slow.initialized = False

  def correction(self, enabled):
    if not enabled or not self.valid:
      return 0.0
    return max(-1.5, min(1.5, math.sin(self.fast.x - self.slow.x) * 9.81))
