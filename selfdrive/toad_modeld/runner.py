"""QCOM-only RDF4 inference with the pinned StarPilot artifact and preprocessing.

Call bootstrap.configure_runtime() in a fresh process before importing this module.
"""
import io
import pickle
import struct

import numpy as np
from tinygrad import Tensor

from openpilot.selfdrive.toad_modeld.artifact import ARTIFACT_PATH, verify_artifact
from openpilot.selfdrive.toad_modeld.nv12_info import get_nv12_info
from openpilot.selfdrive.toad_modeld.parse_model_outputs import Parser
from openpilot.selfdrive.toad_modeld.queues import make_supercombo_input_queues


def load_oob(source):
  # StarPilot protocol-5 out-of-band format. Only verified pinned files are loaded.
  header = source.read(8)
  if len(header) != 8:
    raise ValueError('Truncated model header')
  opcode_size, = struct.unpack('<q', header)
  if not 0 < opcode_size < 128 * 1024 * 1024:
    raise ValueError('Invalid model header')
  opcodes = source.read(opcode_size)
  if len(opcodes) != opcode_size:
    raise ValueError('Truncated model opcodes')

  def buffers():
    while header := source.read(8):
      if len(header) != 8:
        raise ValueError('Truncated model buffer header')
      size, = struct.unpack('<q', header)
      if not 0 <= size <= 128 * 1024 * 1024:
        raise ValueError('Invalid model buffer size')
      buffer = bytearray(size)
      if source.readinto(buffer) != size:
        raise ValueError('Truncated model buffer')
      yield pickle.PickleBuffer(buffer)

  return pickle.load(io.BytesIO(opcodes), buffers=buffers())


class ModelState:
  def __init__(self, width, height):
    verify_artifact()
    with ARTIFACT_PATH.open('rb') as model:
      self.artifact = load_oob(model)
    artifact = self.artifact
    if (artifact['format_version'] != 1 or artifact['model_type'] != 'supercombo' or
        artifact['behavior_version'] != 'v15' or artifact['image_history_pipeline'] != 'policy'):
      raise ValueError('Unsupported model artifact; expected the pinned v15 supercombo')
    self.width, self.height = width, height
    if (width, height) not in artifact:
      raise ValueError(f'No compiled camera warp for {width}x{height}')
    self.stride, self.y_height, _, self.frame_size = get_nv12_info(width, height)
    self.metadata = artifact['metadata']['model']
    self.parser = Parser()
    self.warp = artifact[(width, height)]
    self.run_policy = artifact['run_policy']
    self._reset()

  def _reset(self):
    self.queues, self.arrays = make_supercombo_input_queues(self.metadata['input_shapes'], self.artifact['frame_skip'], 'QCOM')
    self.previous_desire = np.zeros(8, dtype=np.float32)
    self.frames = {}

  def _frame(self, key, buf):
    if (buf.width, buf.height, buf.stride, buf.uv_offset) != (self.width, self.height, self.stride, self.stride * self.y_height):
      raise ValueError('Camera layout does not match the compiled NV12 warp')
    data = np.frombuffer(buf.data, dtype=np.uint8)
    if data.size < self.frame_size:
      raise ValueError(f'Camera buffer is too short: {data.size} < {self.frame_size}')
    address = data.ctypes.data
    cache_key = key, address
    if cache_key not in self.frames:
      # VisionIPC owns these buffers for the lifetime of the connected client.
      self.frames[cache_key] = Tensor.from_blob(address, (self.frame_size,), dtype='uint8', device='QCOM')
    return self.frames[cache_key]

  def _evaluate(self, frame, wide_frame, transform, wide_transform, desire, traffic_convention, action_t):
    for value, shape in ((transform, (3, 3)), (wide_transform, (3, 3)), (desire, (8,)),
                         (traffic_convention, (2,)), (action_t, (2,))):
      if np.shape(value) != shape or not np.isfinite(value).all():
        raise ValueError('Invalid model input shape or non-finite input')
    desire = np.array(desire, dtype=np.float32, copy=True)
    desire[0] = 0
    self.arrays['desire'][:] = np.where(desire - self.previous_desire > 0.99, desire, 0)
    self.previous_desire[:] = desire
    self.arrays['traffic_convention'][:] = traffic_convention
    self.arrays['action_t'][:] = action_t
    self.arrays['tfm'][:] = transform
    self.arrays['big_tfm'][:] = wide_transform
    warped = self.warp(**{k: self.queues[k] for k in self.artifact['warp_input_keys']}, frame=frame, big_frame=wide_frame)
    raw = self.run_policy(**{k: self.queues[k] for k in self.artifact['policy_input_keys']}, warped=warped)[0].numpy().flatten()
    if raw.shape != (2580,) or not np.isfinite(raw).all():
      raise ValueError('Invalid RDF4 inference output')
    self.arrays['prev_feat'][:] = raw[self.metadata['output_slices']['hidden_state']]
    outputs = self.parser.parse_outputs({k: raw[np.newaxis, s].copy() for k, s in self.metadata['output_slices'].items()})
    if any(not np.isfinite(v).all() for v in outputs.values()):
      raise ValueError('Invalid parsed RDF4 output')
    return outputs

  def run(self, buf, wide_buf, transform, wide_transform, desire, traffic_convention, action_t):
    return self._evaluate(self._frame('road', buf), self._frame('wide', wide_buf), transform, wide_transform,
                          desire, traffic_convention, action_t)

  def synthetic(self):
    if not hasattr(self, '_synthetic_frames'):
      self._synthetic_frames = [Tensor(np.full(self.frame_size, 128, dtype=np.uint8), device='QCOM').realize() for _ in range(2)]
    return self._evaluate(*self._synthetic_frames, np.eye(3, dtype=np.float32), np.eye(3, dtype=np.float32),
                          np.zeros(8, dtype=np.float32), np.array([1, 0], dtype=np.float32), np.array([0.3, 0.3], dtype=np.float32))

  def warmup(self):
    self.synthetic()
    self._reset()
