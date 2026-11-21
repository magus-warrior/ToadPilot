"""Local road-camera evidence for Camry stop placement and posted-speed handoffs."""
import collections
import fcntl
import io
import json
import math
import os
import signal
import time
import zipfile
from pathlib import Path

import numpy as np
from PIL import Image
from cereal import messaging
from msgq.visionipc import VisionIpcClient, VisionStreamType
from openpilot.common.params import Params
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.frogpilot.crosswalk_geometry import camera_distance, stripe_row

ROOT = Path('/data/toadpilot/crosswalk-observations')
SERVICES = ['carState', 'carControl', 'controlsState', 'roadCameraState', 'liveCalibration',
            'modelV2', 'frogpilotPlan', 'longitudinalPlan', 'radarState', 'frogpilotCarState']
PERIOD = .25
MAX_BYTES = 48 * 1024**2


def atomic_json(path, value):
  temp = path.with_suffix('.tmp')
  temp.write_text(json.dumps(value, allow_nan=False))
  temp.replace(path)


def road_image(frame, client):
  # VisionIPC supplies NV12 with padded rows. Copy downsized pixels before the camera reuses them.
  h, w, stride = client.height, client.width, client.stride
  data = frame.data
  if client.rgb or not (0 < w <= stride and h > 0 and h%2 == 0 and w%2 == 0):
    return None
  end = client.uv_offset + stride*(h//2)
  if client.uv_offset < h*stride or len(data) < end:
    return None
  y = data[:h*stride].reshape(h,stride)[::4,:w:4]
  uv = data[client.uv_offset:end].reshape(h//2,stride)
  return np.stack((y,uv[::2,:w:4],uv[::2,1:w:4]),axis=-1)


def observation(sm, frame, client, memory, cfg, now):
  camera = sm['roadCameraState']
  def fresh(name, age=.30):
    return sm.seen[name] and sm.valid[name] and sm.alive[name] and 0 <= now - sm.logMonoTime[name]*1e-9 <= age
  # This fork's camerad leaves VisionIPC's optional valid bit unset. Match the authoritative
  # valid roadCameraState message by BOTH frame ID and EOF timestamp before using the pixels.
  if (not fresh('roadCameraState') or client.frame_id != camera.frameId or client.timestamp_eof != camera.timestampEof
      or not 0 <= now - client.timestamp_eof*1e-9 <= .20):
    return None
  if not fresh('carState') or not sm['carState'].canValid:
    return None
  cs = sm['carState']
  sample = dict(t=time.time(), mono=now, frame=client.frame_id, eof=client.timestamp_eof,
                vipc_valid_bit=client.valid, gear=str(cs.gearShifter), speed_mps=cs.vEgo, brake=cs.brakePressed,
                gas=cs.gasPressed, standstill=cs.standstill, left_blinker=cs.leftBlinker, right_blinker=cs.rightBlinker,
                pcm_kph=cs.cruiseState.speed*3.6, pcm_cluster_kph=cs.cruiseState.speedCluster*3.6,
                buttons=[(str(b.type),b.pressed) for b in cs.buttonEvents], metric=cfg.get('is_metric'),
                slc_enabled=cfg.get('speed_limit_controller'), model=cfg.get('model'),
                conditional=cfg.get('conditional_experimental_mode'), stop_trigger_s=cfg.get('conditional_model_stop_time'))
  if fresh('carControl'):
    sample['long_active'] = sm['carControl'].longActive
  if fresh('controlsState'):
    ctrl = sm['controlsState']
    sample.update(experimental=ctrl.experimentalMode, enabled=ctrl.enabled,
                  comma_kph=ctrl.vCruise, displayed_kph=ctrl.vCruiseCluster, mode_reason_param=memory.get_int('CEStatus'))
  if fresh('frogpilotPlan'):
    plan = sm['frogpilotPlan']
    sample.update(stop_scene=plan.redLight, mode_requested=plan.experimentalMode, source=plan.slcSpeedLimitSource,
                  pending_sign=plan.speedLimitChanged, posted_mps=plan.slcSpeedLimit, offset_mps=plan.slcSpeedLimitOffset,
                  planned_cruise_mps=plan.vCruise)
  if fresh('longitudinalPlan'):
    sample.update(hold=sm['longitudinalPlan'].shouldStop, accel_target=sm['longitudinalPlan'].aTarget)
  if fresh('modelV2'):
    path = sm['modelV2'].position.x
    sample['model_path_end_m'] = path[-1] if len(path) else None
    sample.update(model_hold=sm['modelV2'].action.shouldStop, model_accel=sm['modelV2'].action.desiredAcceleration)
  if fresh('radarState'):
    lead = sm['radarState'].leadOne
    sample['lead_distance_m'] = lead.dRel if lead.status else None
  if fresh('frogpilotCarState'):
    sample['dashboard_sign_mps'] = sm['frogpilotCarState'].dashboardSpeedLimit
  pixels = road_image(frame,client)
  if pixels is None:
    return None
  row = stripe_row(pixels[:,:,0]) if sample['gear']=='drive' else None
  sample.update(row=row, camera_distance_m=None, image_height=pixels.shape[0], sensor=str(camera.sensor))
  if fresh('liveCalibration',1.5):
    cal = sm['liveCalibration']
    sample['calibrated'] = str(cal.calStatus)=='calibrated'
    sensor = DEVICE_CAMERAS.get(('tici',str(camera.sensor)))
    if sample['calibrated'] and sensor and sensor.fcam.size==(client.width,client.height) and len(cal.height)==1:
      sample.update(camera_height_m=cal.height[0],calibration_rpy=list(cal.rpyCalib))
      sample['camera_distance_m']=camera_distance(row,pixels.shape[:2],sensor.fcam,cal.rpyCalib,cal.height[0])
  # Cap'n Proto float fields may be nonfinite when the producer has no estimate.
  for key, value in sample.items():
    if isinstance(value, float) and not math.isfinite(value):
      sample[key] = None
    elif isinstance(value, list) and any(isinstance(x, float) and not math.isfinite(x) for x in value):
      sample[key] = None
  return sample, pixels


class Clips:
  def __init__(self,root):
    self.root=root
    self.pre=collections.deque(maxlen=20)
    self.frames=[]
    self.until=0.
    self.last_close=-100.
    self.previous={}
    self.reason=None

  def finish(self):
    if self.frames:
      first=self.frames[0][0]
      name=f"approach-{first['t']:.3f}-{first['frame']}.zip"
      dest=self.root/name
      partial=dest.with_suffix('.partial')
      with zipfile.ZipFile(partial,'w') as z:
        z.writestr('samples.json',json.dumps(dict(trigger=self.reason,samples=[r for r,_ in self.frames]),allow_nan=False))
        for r,jpeg in self.frames:
          z.writestr(f"{r['frame']}.jpg",jpeg)
      partial.replace(dest)
      self.last_close=self.frames[-1][0]['mono']
      files=sorted(self.root.glob('approach-*.zip'),key=lambda p:p.stat().st_mtime)
      size=sum(p.stat().st_size for p in files)
      for p in files:
        if size<=MAX_BYTES:break
        size-=p.stat().st_size
        p.unlink()
    self.frames=[]

  def clear(self):
    self.pre.clear()
    self.previous={}

  def add(self,r,pixels):
    if r['gear'] != 'drive':
      self.finish()
      self.clear()
      return
    previous=self.previous
    self.previous=r
    stream=io.BytesIO()
    Image.fromarray(pixels,'YCbCr').save(stream,'JPEG',quality=83)
    item=(r,stream.getvalue())
    now=r['mono']
    if previous and now-previous['mono']>1.0:
      self.finish()
      self.pre.clear()
      previous={}
    reason=None
    for field in ('stop_scene','experimental','brake'):
      if r.get(field) and previous.get(field) is not True:
        reason=field
        break
    if not reason and previous and r.get('slc_enabled') != previous.get('slc_enabled'):
      reason = 'limiter_toggled'
    if not reason and r.get('slc_enabled') and r.get('displayed_kph') is not None and previous.get('displayed_kph') is not None and abs(r['displayed_kph'] - previous['displayed_kph']) >= 2.:
      reason = 'max_change'
    if not reason and r.get('slc_enabled') and previous and (r.get('posted_mps'),r.get('offset_mps'))!=(previous.get('posted_mps'),previous.get('offset_mps')):
      reason='sign_change'
    if not reason and r.get('long_active') and previous.get('speed_mps',0.)>=5.>r['speed_mps']:
      reason='low_speed_approach'
    if not reason and r.get('row') is not None and previous.get('row') is None:
      reason='paint_candidate'
    if self.frames:
      self.frames.append(item)
      if now>=self.until:self.finish()
    elif reason and now-self.last_close>=2.:
      self.reason=reason
      self.frames=[*self.pre,item]
      self.until=now+10.
    self.pre.append(item)


def main():
  os.nice(10)
  ROOT.mkdir(parents=True,exist_ok=True,mode=0o700)
  lock=(ROOT/'observer.lock').open('a')
  fcntl.flock(lock, fcntl.LOCK_EX)
  for old in ROOT.glob('approach-*.partial'):old.unlink(missing_ok=True)
  sm=messaging.SubMaster(SERVICES)
  memory=Params('/dev/shm/params')
  clips=Clips(ROOT)
  client=None
  next_status=next_settings=0.
  accepted=0
  cfg = {}
  latest = None
  running = True
  def stop(signum, frame):
    nonlocal running
    running = False
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  while running:
    start=time.monotonic()
    if start>=next_settings:
      try:cfg=json.loads(memory.get('FrogPilotToggles') or '{}')
      except ValueError:cfg={}
      next_settings=start+2.
    sample=None
    if cfg.get('toadpilot_telemetry') and cfg.get('car_model')=='TOYOTA_CAMRY_TSS2':
      if client is None:
        client=VisionIpcClient('camerad',VisionStreamType.VISION_STREAM_ROAD,True)
      if client.is_connected() or client.connect(False):
        frame=client.recv(120)
        sm.update(0)
        if frame is not None:
          value=observation(sm,frame,client,memory,cfg,time.monotonic())
          if value:
            sample,pixels=value
            if latest is None or sample['eof'] != latest['eof']:
              clips.add(sample, pixels)
              latest = sample
              accepted += 1
        else:
          client=None
    else:
      client=None
      latest=None
      clips.finish()
      clips.clear()
    if clips.frames and start - clips.frames[-1][0]['mono'] > 1.:
      clips.finish()
      clips.clear()
    if start>=next_status:
      atomic_json(ROOT/'status.json',dict(pid=os.getpid(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                 sampled_frames=accepted,last_check=time.time(),sample=latest,clip_open=bool(clips.frames)))
      next_status=start+5.
    time.sleep(max(.005,PERIOD-(time.monotonic()-start)))

  clips.finish()


if __name__=='__main__':
  main()
