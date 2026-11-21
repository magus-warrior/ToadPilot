"""Road-camera paint proposals for diagnostics. Positive image y points down."""
import numpy as np
from openpilot.common.transformations.camera import get_view_frame_from_road_frame


def stripe_row(luma):
  """Propose the closer of two transverse paint edges visible across the lane corridor.

  A single transverse line, an occluded edge, or a zebra crossing without a visible
  transverse border may be missed. This output alone has no traffic-control meaning.
  """
  h, w = luma.shape
  if h < 40 or w < 40:
    return None
  row0, row1 = int(h * .54), int(h * .97)
  rows = np.arange(row0, row1)
  if len(rows) < 20:
    return None
  # Work along lines with a small slope to accommodate non-level markings. Inspect both
  # sides of the lane so a short mark on a vehicle or a curb is less likely to qualify.
  horizon = h * .5
  span = np.clip(1.18 * (rows - horizon), w * .075, w * .27)
  columns = np.linspace(-1, 1, 49)
  dx = span[:, None] * columns
  x = np.rint(w/2 + dx).astype(int)
  width = np.maximum(1, np.rint((rows - horizon) * .032).astype(int))[:, None]
  strong = np.zeros(len(rows))
  weak = np.zeros(len(rows))
  zebra = np.zeros(len(rows))
  for slope in (-.08, -.04, 0., .04, .08):
    y = rows[:, None] + np.rint(slope * dx).astype(int)
    visible = ((y + width < h) & (y - 3*width >= 0)).all(axis=1)
    y = np.clip(y, 3*width, h - 1 - width)
    below = luma[y+width,x].astype(float)
    paint = np.maximum(luma[y-1,x], luma[y-width,x]).astype(float)
    response = np.minimum(paint-below, paint-luma[y-3*width,x])
    hit = (response > 8) & visible[:, None]
    sides = np.minimum(hit[:,:18].mean(1),hit[:,-18:].mean(1))
    quality = (hit.mean(1) + sides)*.5
    strong = np.maximum(strong,np.where((hit.mean(1)>.54)&(sides>.48),quality,0.))
    weak = np.maximum(weak,np.where((hit.mean(1)>.19)&(sides>.16),quality,0.))
    # Longitudinal zebra bars end along the same transverse boundary. Require
    # several alternating paint/asphalt regions remaining bright farther uphill.
    delta = np.minimum(paint, luma[y-3*width,x].astype(float)) - below
    bars = (delta > 11.) & visible[:, None]
    runs = np.sum(~bars[:,:-1] & bars[:,1:],axis=1)
    spread = np.minimum(bars[:,:18].mean(1), bars[:,-18:].mean(1))
    fraction = bars.mean(1)
    zebra = np.maximum(zebra,np.where((runs>=3)&(runs<=9)&(spread>.18)&
                       (fraction>.28)&(fraction<.76),fraction,0.))
  # First consider the nearest visible credible row. A transverse bar must have
  # independent farther paint support; a zebra boundary supplies its own pattern.
  for idx in np.flatnonzero((strong>0)|(zebra>0))[::-1]:
    near = int(rows[idx])
    far_mask = ((near-rows)/(near-horizon)>.12)&((near-rows)/(near-horizon)<.53)
    if near >= .66*h and (zebra[idx] or (strong[idx] and np.any(weak[far_mask]))):
      return near
  return None


def camera_distance(row, shape, camera, rpy, height):
  """Approximate road-plane distance ahead of the camera at the image center."""
  if row is None or len(rpy)!=3 or not np.isfinite(rpy).all() or not .5 < height < 2.5:
    return None
  h,w = shape
  pixel = np.array([camera.width*.5, row*camera.height/h, 1.])
  exterior = get_view_frame_from_road_frame(*rpy,height)
  rotation, translation = exterior[:,:3], exterior[:,3]
  origin = -rotation.T @ translation
  ray = rotation.T @ (camera.intrinsics_inv @ pixel)
  if ray[2] >= -.01:
    return None
  distance = origin[0] - origin[2]*ray[0]/ray[2]
  return round(float(distance),2) if 1. < distance < 65. else None
