"""Pinned upstream units; do not substitute the legacy UI constants."""
import math
from openpilot.selfdrive.modeld.constants import ModelConstants
CITY_SPEED_LIMIT = 25
CRUISING_SPEED = 5
DEFAULT_LATERAL_ACCELERATION = 2.0
MINIMUM_LATERAL_ACCELERATION = 1.3
PLANNER_TIME = ModelConstants.T_IDXS[-1]
THRESHOLD = 1 - 1 / math.e
