#!/usr/bin/env python3
"""Dedicated RDF4 process. --shadow consumes camera frames without publishing controls."""
import argparse
import json
import os
import time
import numpy as np

from openpilot.selfdrive.toad_modeld.bootstrap import configure_runtime
from openpilot.selfdrive.toad_modeld.artifact import is_modern_model, MODEL_ID, MODELS
from openpilot.selfdrive.toad_modeld.action import action_from_outputs, LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS

from cereal import car, log
import cereal.messaging as messaging
from cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType
from setproctitle import setproctitle
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.system import sentry
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.frogpilot.frogpilot_variables import get_frogpilot_toggles
from openpilot.selfdrive.toad_modeld.constants import ModelConstants
from openpilot.selfdrive.toad_modeld.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState

PROCESS_NAME = "selfdrive.toad_modeld.modeld"


class ShadowPublisher:
  def send(self, service, message):
    if service == 'modelV2' and message.modelV2.frameId % 20 == 0:
      print(f"shadow frame={message.modelV2.frameId} valid={message.valid} inference_ms={message.modelV2.modelExecutionTime * 1000:.1f}", flush=True)


class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof


def main(shadow=False, max_frames=0, model_id=None):
  frogpilot_toggles = get_frogpilot_toggles()
  if shadow:
    model_id = model_id or MODEL_ID
  else:
    if not is_modern_model(frogpilot_toggles):
      raise RuntimeError('Modern model process started for a different selected model')
    model_id = frogpilot_toggles.model
  configure_runtime(model_id)
  from openpilot.selfdrive.toad_modeld.runner import ModelState
  cloudlog.warning("ToadPilot modeld init: %s", model_id)

  sentry.set_tag("daemon", PROCESS_NAME)
  cloudlog.bind(daemon=PROCESS_NAME)
  setproctitle(PROCESS_NAME)
  core = max(os.sched_getaffinity(0)) if shadow else 7
  config_realtime_process(core, 54)

  cloudlog.warning("setting up CL context")
  cl_context = None
  cloudlog.warning("CL context ready; loading model")

  # FrogPilot variables
  frogpilot_toggles = get_frogpilot_toggles()

  if not is_modern_model(frogpilot_toggles) and not shadow:
    raise RuntimeError("Modern model process started for a different selected model")

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_ROAD in available_streams
      main_wide_camera = VisionStreamType.VISION_STREAM_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True, cl_context)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False, cl_context)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  model = ModelState(vipc_client_main.width, vipc_client_main.height, model_id)
  warm_road = vipc_client_main.recv()
  while warm_road is None:
    warm_road = vipc_client_main.recv()
  warm_wide = vipc_client_extra.recv() if use_extra_client else warm_road
  while warm_wide is None:
    warm_wide = vipc_client_extra.recv()
  model.warmup(warm_road, warm_wide)
  previous_action = log.ModelDataV2.Action(longitudinalValid=True)
  completed_frames = 0
  shadow_timings = []
  shadow_drops = 0

  # messaging
  pm = ShadowPublisher() if shadow else PubMaster(["modelV2", "drivingModelData", "cameraOdometry"])
  sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl", "frogpilotPlan"])

  publish_state = PublishState()
  params = Params()

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_FREQ)
  frame_id = 0
  last_vipc_frame_id = None
  run_count = 0

  model_transform_main = np.eye(3, dtype=np.float32)
  model_transform_extra = np.eye(3, dtype=np.float32)
  live_calib_seen = False
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()


  cp_bytes = params.get("CarParamsPersistent") if shadow else params.get("CarParams", block=True)
  if not cp_bytes:
    raise RuntimeError('No vehicle parameters available for the shadow check')
  with car.CarParams.from_bytes(cp_bytes) as msg:
    CP = msg.as_builder()
  cloudlog.info("tinygrad_modeld got CarParams: %s", CP.carName)

  # This fork has no liveDelay service. Use its vehicle actuator delay; the
  # model smoothing and frame/action delays are added explicitly below.
  steer_delay = CP.steerActuatorDelay

  DH = DesireHelper()

  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.debug("vipc_client_main no frame")
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.debug("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        cloudlog.error(f"frames out of sync! main: {meta_main.frame_id} ({meta_main.timestamp_sof / 1e9:.5f}),\
                         extra: {meta_extra.frame_id} ({meta_extra.timestamp_sof / 1e9:.5f})")

    else:
      # Use single camera
      buf_extra = buf_main
      meta_extra = meta_main

    sm.update(0)
    desire = DH.desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["roadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    if sm.updated["liveCalibration"] and sm.seen['roadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]
      model_transform_main = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics, False).astype(np.float32)
      model_transform_extra = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics, True).astype(np.float32)
      live_calib_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < ModelConstants.DESIRE_LEN:
      vec_desire[desire] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1) if last_vipc_frame_id is not None else 0
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)
    dropped_frame = vipc_dropped_frames > 0

    # The v15 policy predicts actions at these delay horizons. Match the
    # existing steering delay and include the model's output smoothing delay.
    action_t = np.array([steer_delay + LAT_SMOOTH_SECONDS + 0.075,
                         CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS + 0.075], dtype=np.float32)
    mt1 = time.perf_counter()
    model_output = model.run(buf_main, buf_extra, model_transform_main, model_transform_extra,
                             vec_desire, traffic_convention, action_t)
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1
    if shadow:
      shadow_timings.append(model_execution_time * 1000)
      shadow_drops += vipc_dropped_frames

    if model_output is not None and not dropped_frame:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')
      curvature, acceleration, should_stop = action_from_outputs(
        model_output, v_ego, action_t, previous_action.desiredCurvature, previous_action.desiredAcceleration)
      previous_action = log.ModelDataV2.Action(desiredCurvature=curvature, desiredAcceleration=acceleration,
                                              shouldStop=should_stop, longitudinalValid=True)
      fill_model_msg(drivingdata_send, modelv2_send, model_output, previous_action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, live_calib_seen)

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, sm['frogpilotPlan'], frogpilot_toggles)
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      modelv2_send.modelV2.meta.turnDirection = DH.turn_direction
      drivingdata_send.drivingModelData.meta.laneChangeState = DH.lane_change_state
      drivingdata_send.drivingModelData.meta.laneChangeDirection = DH.lane_change_direction

      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, live_calib_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
    last_vipc_frame_id = meta_main.frame_id
    completed_frames += 1
    if max_frames and completed_frames >= max_frames:
      break

    # Update FrogPilot parameters
    if sm['frogpilotPlan'].togglesUpdated:
      frogpilot_toggles = get_frogpilot_toggles()


  if shadow:
    print(json.dumps(dict(frames=completed_frames, dropped_camera_frames=shadow_drops,
                          latency_ms={name: float(np.percentile(shadow_timings, p)) for name, p in [('p50', 50), ('p95', 95), ('max', 100)]},
                          published_controls=False)), flush=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--shadow', action='store_true')
  parser.add_argument('--max-frames', type=int, default=0)
  parser.add_argument("--model", choices=MODELS, help="Model override for shadow validation only")
  args = parser.parse_args()
  if args.model and not args.shadow:
    parser.error("--model is available only with --shadow")
  main(shadow=args.shadow, max_frames=args.max_frames, model_id=args.model)
