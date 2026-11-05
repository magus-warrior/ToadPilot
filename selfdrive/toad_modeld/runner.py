"""QCOM-only RDF4 inference with the pinned StarPilot artifact and preprocessing.

Call bootstrap.configure_runtime() in a fresh process before importing this module.
"""
import io
import pickle
import struct

import numpy as np
from tinygrad import Tensor

from openpilot.selfdrive.toad_modeld.artifact import MODEL_ID, MODELS, artifact_path, verify_artifact
from openpilot.selfdrive.toad_modeld.nv12_info import get_nv12_info
from openpilot.selfdrive.toad_modeld.parse_model_outputs import Parser
from openpilot.selfdrive.toad_modeld.queues import make_supercombo_input_queues, make_split_input_queues


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
  def __init__(self, width, height, model_id=MODEL_ID):
    verify_artifact(model_id=model_id)
    with artifact_path(model_id).open('rb') as model:
      self.artifact = pickle.load(model) if model.peek(1)[:1] == b'\x80' else load_oob(model)
    artifact = self.artifact
    # June artifacts sample image history in the compiled warp; newer ones do
    # it in the policy. Only the explicitly pinned June model may omit the tag.
    self.image_history_pipeline = MODELS[model_id].get('image_history_pipeline', 'policy')
    artifact_pipeline = artifact.get('image_history_pipeline',
                                     'warp' if self.image_history_pipeline == 'warp' else None)
    if (artifact['format_version'] != 1 or artifact['model_type'] not in ('supercombo', 'vision_policy') or
        artifact['behavior_version'] != MODELS[model_id]['version'] or artifact_pipeline != self.image_history_pipeline):
      raise ValueError('Unsupported pinned model artifact')
    self.warp_input_keys = (('img_q', 'big_img_q', 'tfm', 'big_tfm') if self.image_history_pipeline == 'warp'
                           else artifact['warp_input_keys'])
    self.width, self.height = width, height
    if (width, height) not in artifact:
      raise ValueError(f'No compiled camera warp for {width}x{height}')
    self.stride, self.y_height, _, self.frame_size = get_nv12_info(width, height)
    self.metadata = artifact['metadata']
    self.split = artifact['model_type'] == 'vision_policy'
    if self.split and artifact['policy_order'] != ['policy']:
      raise ValueError('Unsupported split policy order')
    self.parser = Parser()
    self.warp = artifact[(width, height)]
    self.run_policy = artifact['run_policy']
    self.frames = {}
    self._reset()

  def _reset(self):
    if self.split:
      self.queues, self.arrays = make_split_input_queues(self.metadata['vision']['input_shapes'],
                                                       self.metadata['policy']['input_shapes'], self.artifact['frame_skip'], 'QCOM')
    else:
      self.queues, self.arrays = make_supercombo_input_queues(self.metadata['model']['input_shapes'], self.artifact['frame_skip'], 'QCOM')
    self.previous_desire = np.zeros(8, dtype=np.float32)

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
    if 'action_t' in self.arrays:
      self.arrays['action_t'][:] = action_t
    self.arrays['tfm'][:] = transform
    self.arrays['big_tfm'][:] = wide_transform
    warped = self.warp(**{k: self.queues[k] for k in self.warp_input_keys}, frame=frame, big_frame=wide_frame)
    policy_inputs = {k: self.queues[k] for k in self.artifact['policy_input_keys']}
    if self.image_history_pipeline == 'warp':
      # d97100bd's compiled warp returns the two already-sampled image stacks.
      img, big_img = warped
      raw_outputs = self.run_policy(**policy_inputs, img=img, big_img=big_img)
    else:
      raw_outputs = self.run_policy(**policy_inputs, warped=warped)
    names = ['vision', 'policy'] if self.split else ['model']
    if len(raw_outputs) != len(names):
      raise ValueError('Unexpected inference output count')
    sliced = {}
    for name, tensor in zip(names, raw_outputs, strict=True):
      raw = tensor.numpy().flatten()
      metadata = self.metadata[name]
      if raw.size != np.prod(metadata['output_shapes']['outputs']) or not np.isfinite(raw).all():
        raise ValueError('Invalid model inference output')
      sliced[name] = {k: raw[np.newaxis, sl].copy() for k, sl in metadata['output_slices'].items()}
    if self.split:
      outputs = self.parser.parse_vision_outputs(sliced['vision'])
      outputs.update(self.parser.parse_policy_outputs(sliced['policy']))
    else:
      self.arrays['prev_feat'][:] = sliced['model']['hidden_state']
      outputs = self.parser.parse_outputs(sliced['model'])
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

  def warmup(self, road=None, wide=None):
    self.synthetic()
    if road is not None and wide is not None:
      # Prime the external-camera cache-maintenance/JIT path before publishing.
      # Retain buffer mappings, but discard warmup model history.
      self.run(road, wide, np.eye(3, dtype=np.float32), np.eye(3, dtype=np.float32),
               np.zeros(8, dtype=np.float32), np.array([1, 0], dtype=np.float32), np.array([0.3, 0.3], dtype=np.float32))
    self._reset()
