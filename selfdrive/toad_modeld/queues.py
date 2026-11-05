# Adapted from StarPilotHQ/StarPilot at 4a553da9ac22a14e34513c011aed04c1e93b4619 (MIT).
import math
import numpy as np
from tinygrad import Tensor

def _detect_desire_key(input_shapes):
  return next((key for key in input_shapes if key.startswith("desire")), None)

def _detect_vision_keys(input_shapes):
  image_keys = sorted(key for key in input_shapes if "img" in key)
  road_key = next((key for key in image_keys if "big" not in key), None)
  wide_key = next((key for key in image_keys if "big" in key), None)
  if road_key is None or wide_key is None:
    raise ValueError(f"Cannot determine road/wide image keys from {list(input_shapes)}")
  return road_key, wide_key

def make_warp_input_queues(vision_input_shapes, frame_skip, device):
  road_key, _ = _detect_vision_keys(vision_input_shapes)
  image_shape = vision_input_shapes[road_key]
  frame_count = image_shape[1] // 6
  image_buffer_shape = (frame_skip * (frame_count - 1) + 1, 6, image_shape[2], image_shape[3])

  npy = {
    "tfm": np.zeros((3, 3), dtype=np.float32),
    "big_tfm": np.zeros((3, 3), dtype=np.float32),
  }
  queues = {
    "img_q": Tensor(np.zeros(image_buffer_shape, dtype=np.uint8), device=device).contiguous().realize(),
    "big_img_q": Tensor(np.zeros(image_buffer_shape, dtype=np.uint8), device=device).contiguous().realize(),
    **{key: Tensor(value, device="NPY").realize() for key, value in npy.items()},
  }
  return queues, npy

def _packed_policy_shapes(input_shapes, include_prev_feature=False):
  desire_key = _detect_desire_key(input_shapes)
  if desire_key is None:
    raise ValueError(f"No desire input found in {list(input_shapes)}")

  shapes = {"desire": (input_shapes[desire_key][2],)}
  for key, shape in input_shapes.items():
    if key in ("features_buffer", desire_key) or "img" in key:
      continue
    shapes[key] = tuple(shape)
  if include_prev_feature:
    features_shape = input_shapes["features_buffer"]
    shapes["prev_feat"] = (features_shape[0], math.prod(features_shape[2:]))
  return shapes, [math.prod(shape) for shape in shapes.values()]

def make_supercombo_input_queues(input_shapes, frame_skip, device):
  queues, npy = make_warp_input_queues(input_shapes, frame_skip, device)
  features_shape = input_shapes["features_buffer"]
  feature_dim = math.prod(features_shape[2:])
  desire_key = _detect_desire_key(input_shapes)
  desire_shape = input_shapes[desire_key]
  packed_shapes, packed_sizes = _packed_policy_shapes(input_shapes, include_prev_feature=True)

  packed_inputs = np.zeros(sum(packed_sizes), dtype=np.float32)
  npy.update({
    key: value.reshape(shape)
    for (key, shape), value in zip(
      packed_shapes.items(), np.split(packed_inputs, np.cumsum(packed_sizes[:-1])), strict=True,
    )
  })
  queues.update({
    "feat_q": Tensor(
      np.zeros((frame_skip * features_shape[1], features_shape[0], feature_dim), dtype=np.float32),
      device=device,
    ).contiguous().realize(),
    "desire_q": Tensor(
      np.zeros((frame_skip * desire_shape[1], desire_shape[0], desire_shape[2]), dtype=np.float32),
      device=device,
    ).contiguous().realize(),
    "packed_npy_inputs": Tensor(packed_inputs, device="NPY").realize(),
  })
  return queues, npy
