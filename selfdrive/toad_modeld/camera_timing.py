"""Optional rejection of stale, repeated or unsynchronized camera inputs."""
import math


class CameraTiming:
  def __init__(self):
    self.last_main = None
    self.last_extra = None

  def update(self, main_sof, main_eof, extra_sof, extra_eof, now_ns):
    values = (main_sof, main_eof, extra_sof, extra_eof, now_ns)
    if not all(math.isfinite(v) for v in values):
      return False
    valid = (0 < main_sof <= main_eof <= now_ns and 0 < extra_sof <= extra_eof <= now_ns and
             now_ns - min(main_sof, extra_sof) <= 200_000_000 and
             abs(main_sof - extra_sof) <= 10_000_000 and
             (self.last_main is None or main_sof > self.last_main) and
             (self.last_extra is None or extra_sof > self.last_extra))
    self.last_main, self.last_extra = main_sof, extra_sof
    return valid
