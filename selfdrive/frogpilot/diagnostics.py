"""Small, local driving traces with fixed disk and memory budgets."""
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

ROOT = Path('/data/toadpilot/diagnostics')
MAX_SEGMENT_BYTES = 2 * 1024 * 1024
MAX_TOTAL_BYTES = 32 * 1024 * 1024
MAX_SEGMENTS = 24
HEALTH_KEY = 'ToadPilotHealth'


def finite(value):
  value = float(value)
  return round(value, 5) if math.isfinite(value) else None


def driving_sample(sm, now):
  cs, cc, output = sm['carState'], sm['carControl'], sm['carOutput'].actuatorsOutput
  controls, plan, model = sm['controlsState'], sm['longitudinalPlan'], sm['modelV2']
  return dict(t=round(time.time(), 3), mono=round(now, 3), speed=finite(cs.vEgo),
    accel=finite(cs.aEgo), steering_pressed=bool(cs.steeringPressed), brake_pressed=bool(cs.brakePressed),
    steering_angle_deg=finite(cs.steeringAngleDeg), requested_curvature=finite(controls.desiredCurvature),
    measured_curvature=finite(controls.curvature), long_control_state=str(controls.longControlState),
    enabled=bool(controls.enabled), lat_active=bool(cc.latActive), long_active=bool(cc.longActive),
    requested_steer=finite(cc.actuators.steer), output_steer=finite(output.steer),
    requested_accel=finite(cc.actuators.accel), output_accel=finite(output.accel),
    planner_accel=finite(plan.accels[0]) if len(plan.accels) else None,
    should_stop=bool(plan.shouldStop),
    forced_stop=bool(sm['frogpilotPlan'].forcingStop) if sm.alive['frogpilotPlan'] and sm.valid['frogpilotPlan'] else None,
    traffic_mode=bool(sm['frogpilotCarState'].trafficMode) if sm.alive['frogpilotCarState'] and sm.valid['frogpilotCarState'] else None,
    model_ms=finite(model.modelExecutionTime * 1000), frame_drop_percent=finite(model.frameDropPerc))


class Recorder:
  def __init__(self, root=ROOT, segment_limit=MAX_SEGMENT_BYTES, total_limit=MAX_TOTAL_BYTES, max_segments=MAX_SEGMENTS):
    self.root = Path(root)
    self.root.mkdir(parents=True, exist_ok=True)
    self.segment_limit, self.total_limit, self.max_segments = segment_limit, total_limit, max_segments
    self.file = None
    self.session = None
    self.index = 0
    self.bytes = 0
    self.summary = {}
    self.previous_override = False
    self.error = None
    self.prune()

  def prune(self, reserve=0):
    traces = sorted(self.root.glob('*.jsonl'), key=lambda p: p.name)
    sizes = {p: p.stat().st_size + (p.with_suffix('.json').stat().st_size if p.with_suffix('.json').exists() else 0)
             for p in traces}
    total = sum(sizes.values())
    while traces and (len(traces) > self.max_segments or total + reserve > self.total_limit):
      path = traces.pop(0)
      if self.file and path == self.path:
        continue
      total -= sizes[path]
      path.unlink(missing_ok=True)
      path.with_suffix('.json').unlink(missing_ok=True)

  def start(self, model):
    self.session = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '-' + uuid.uuid4().hex[:8]
    self.index = 0
    self.model = model
    self._open()

  def _open(self):
    self.index += 1
    name = f'{self.session}-{self.index:03d}.jsonl'
    self.path = self.root / name
    fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    self.file = os.fdopen(fd, 'wb', buffering=16384)
    self.bytes = 0
    self.previous_override = False
    self.summary = dict(id=Path(name).stem, session=self.session, model=self.model, started_at=time.time(),
                        samples=0, override_events=0, requested_saturation_samples=0,
                        peak_model_ms=0., peak_frame_drop_percent=0., max_command_gap=0., complete=False)
    self._write({'type': 'header', 'schema_version': 1, 'sample_hz': 5, **self.summary})
    self.prune(reserve=self.segment_limit + 16384)

  def _write(self, value):
    data = (json.dumps(value, separators=(',', ':'), allow_nan=False) + '\n').encode()
    self.file.write(data)
    self.bytes += len(data)

  def sample(self, value):
    if self.bytes >= self.segment_limit:
      self.close('segment_limit')
      self._open()
    self._write({'type': 'sample', **value})
    s = self.summary
    s['samples'] += 1
    override = bool(value.get('steering_pressed'))
    s['override_events'] += int(override and not self.previous_override)
    self.previous_override = override
    s['requested_saturation_samples'] += int(abs(value.get('requested_steer') or 0.) >= .98)
    s['peak_model_ms'] = max(s['peak_model_ms'], value.get('model_ms') or 0.)
    s['peak_frame_drop_percent'] = max(s['peak_frame_drop_percent'], value.get('frame_drop_percent') or 0.)
    s['max_command_gap'] = max(s['max_command_gap'], abs((value.get('requested_steer') or 0.) - (value.get('output_steer') or 0.)))
    if s['samples'] % 10 == 0:
      self.file.flush()

  def close(self, reason='offroad'):
    if self.file:
      self.summary.update(completed_at=time.time(), complete=True, end_reason=reason)
      self._write({'type': 'summary', **self.summary})
      path = self.path.with_suffix('.json')
      self.file.close()
      self.file = None
      temp = path.with_suffix('.tmp')
      temp.write_text(json.dumps(self.summary, allow_nan=False))
      temp.chmod(0o600)
      temp.replace(path)
      self.prune()


def list_trips(root=ROOT):
  result = []
  for path in sorted(Path(root).glob('*.json'), reverse=True)[:MAX_SEGMENTS]:
    try:
      if path.stat().st_size <= 16384:
        result.append(json.loads(path.read_text()))
    except (OSError, ValueError):
      continue
  return result


def read_trip(trace_id, root=ROOT):
  import re
  if not re.fullmatch(r'\d{8}T\d{6}Z-[a-f0-9]{8}-\d{3,6}', trace_id):
    raise ValueError('Unknown trip.')
  path = Path(root) / (trace_id + '.jsonl')
  if not path.is_file() or path.is_symlink() or path.stat().st_size > MAX_SEGMENT_BYTES + 32768:
    raise ValueError('Unknown trip.')
  # One bounded segment, downsampled for the browser; downloads retain all samples.
  samples = []
  with path.open() as stream:
    for line in stream:
      try:
        record = json.loads(line)
        if record.get('type') == 'sample':
          samples.append(record)
      except ValueError:
        continue
  step = max(1, math.ceil(len(samples) / 1000))
  return dict(id=trace_id, samples=samples[::step])


def main():
  from cereal import messaging
  from openpilot.common.params import Params
  from openpilot.selfdrive.frogpilot.frogpilot_variables import params_memory
  from openpilot.selfdrive.frogpilot.toadpilot import read_toadpilot_config
  sm = messaging.SubMaster(['deviceState', 'carState', 'carControl', 'carOutput', 'controlsState',
                            'modelV2', 'longitudinalPlan', 'frogpilotPlan', 'frogpilotCarState'], frequency=5)
  params = Params()
  recorder = Recorder()
  stopped = False

  def stop(*_):
    nonlocal stopped
    stopped = True

  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  tick = 0
  recording_enabled = True
  try:
    while not stopped:
      start = time.monotonic()
      sm.update(0)
      onroad = bool(sm.alive['deviceState'] and sm.valid['deviceState'] and sm['deviceState'].started)
      required = ['carState', 'carControl', 'carOutput', 'controlsState', 'modelV2', 'longitudinalPlan']
      healthy = sm.all_checks(required)
      if tick % 5 == 0 or not onroad:
        recording_enabled = read_toadpilot_config(params).telemetry
      error = None
      try:
        if onroad and recording_enabled and not recorder.file:
          recorder.start(params.get('Model', encoding='utf-8') or 'unknown')
        elif not onroad or not recording_enabled:
          recorder.close('offroad' if not onroad else 'disabled')
        if onroad and recording_enabled and healthy:
          recorder.sample(driving_sample(sm, start))
      except OSError as exc:
        error = str(exc)
        try:
          recorder.close('storage_error')
        except OSError:
          if recorder.file:
            recorder.file.close()
            recorder.file = None
      if tick % 5 == 0 or not onroad:
        ds = sm['deviceState']
        health = dict(published_at_unix=time.time(), onroad=onroad, recording=bool(recorder.file and healthy),
          recording_enabled=recording_enabled,
          session=recorder.session, samples=recorder.summary.get('samples', 0), error=error,
          model=params.get('Model', encoding='utf-8') or 'unknown',
          cpu_percent=list(ds.cpuUsagePercent), memory_percent=ds.memoryUsagePercent,
          max_cpu_temp_c=max(ds.cpuTempC, default=0), free_space_percent=ds.freeSpacePercent,
          services={s: dict(alive=bool(sm.alive[s]), valid=bool(sm.valid[s])) for s in required},
          model_ms=finite(sm['modelV2'].modelExecutionTime * 1000) if sm.alive['modelV2'] else None,
          frame_drop_percent=finite(sm['modelV2'].frameDropPerc) if sm.alive['modelV2'] else None,
          disk_budget_bytes=MAX_TOTAL_BYTES)
        params_memory.put(HEALTH_KEY, json.dumps(health, allow_nan=False))
      tick += 1
      time.sleep(max((.2 if onroad else 1.) - (time.monotonic() - start), 0.01))
  finally:
    recorder.close('shutdown')


if __name__ == '__main__':
  main()
