# otisserv - Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other of contributors.
# Fleet Manager - [actuallylemoncurd](https://github.com/actuallylemoncurd), [AlexandreSato](https://github.com/alexandreSato), [ntegan1](https://github.com/ntegan1), [royjr](https://github.com/royjr), and [sunnyhaibin] (https://github.com/sunnypilot)
# Almost everything else - ChatGPT
# dirty PR pusher - mike8643
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import base64
import json
import math
import os
import requests
import subprocess
import time
import traceback

import openpilot.system.sentry as sentry

from cereal import car, log
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import List
from urllib.parse import quote

from openpilot.common.params import ParamKeyType
from openpilot.selfdrive.car.toyota.carcontroller import LOCK_CMD, UNLOCK_CMD
from openpilot.selfdrive.car.toyota.values import CarControllerParams
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.uploader import listdir_by_creation
from openpilot.system.loggerd.xattr_cache import getxattr
from panda import Panda
from tools.lib.route import SegmentName

from openpilot.selfdrive.frogpilot.frogpilot_variables import (ERROR_LOGS_PATH, EXCLUDED_KEYS, frogpilot_default_params,
                                                              params, params_memory, update_frogpilot_toggles)
from openpilot.selfdrive.frogpilot.controls.lib.start_accel_ramp import (START_ACCEL_RAMP_DEFAULT,
                                                                        START_ACCEL_RAMP_MAX,
                                                                        START_ACCEL_RAMP_MIN,
                                                                        START_ACCEL_RAMP_PARAM,
                                                                        START_ACCEL_RAMP_STEP,
                                                                        parse_start_accel_ramp,
                                                                        validate_start_accel_ramp)
from openpilot.selfdrive.frogpilot.toadpilot import (TOADPILOT_FINGERPRINT, TOADPILOT_LEGACY_STEERING_RESPONSE_PROFILES,
                                                     TOADPILOT_NAVIGATION_STATUS_KEY,
                                                     TOADPILOT_RESET_TUNE_KEY,
                                                     TOADPILOT_STATUS_KEY, TOADPILOT_STEERING_RESPONSE_PROFILE,
                                                     TOADPILOT_STEERING_STATUS_KEY,
                                                     TOADPILOT_STOP_GO_LAST_ROUTE_KEY, TOADPILOT_STOP_GO_STATUS_KEY,
                                                     TOADPILOT_TORQUE_CACHE_VERSION, TOADPILOT_TORQUE_CHECKPOINT_KEY,
                                                     TOADPILOT_TORQUE_LAST_ROUTE_KEY,
                                                     TOADPILOT_TORQUE_STATUS_KEY,
                                                     TOADPILOT_TUNE_STATUS_KEY,
                                                     normalize_toadpilot_config, read_json_status,
                                                     read_toadpilot_config, read_toadpilot_steering_response_latch,
                                                     read_toadpilot_steering_response_profile,
                                                     read_toadpilot_steering_response_request,
                                                     toadpilot_steering_response_panda_matched,
                                                     valid_toadpilot_stop_go_summary,
                                                     valid_toadpilot_torque_summary,
                                                     write_toadpilot_config, write_toadpilot_steering_response_request)

XOR_KEY = "s8#pL3*Xj!aZ@dWq"

pi = 3.1415926535897932384626
x_pi = 3.14159265358979324 * 3000.0 / 180.0
a = 6378245.0
ee = 0.00669342162296594323

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5


# path to openpilot screen recordings and error logs
if PC:
  SCREENRECORD_PATH = os.path.join(str(Path.home()), ".comma", "media", "screen_recordings", "")
  ERROR_LOGS_PATH = os.path.join(str(Path.home()), ".comma", "error_logs", "")
  TMUX_LOGS_PATH = os.path.join(str(Path.home()), ".comma", "tmux_logs")
else:
  SCREENRECORD_PATH = "/data/media/screen_recordings/"
  ERROR_LOGS_PATH = ERROR_LOGS_PATH
  TMUX_LOGS_PATH = "/data/tmux_logs/"


def list_files(path): # still used for footage
  return sorted(listdir_by_creation(path), reverse=True)


def list_file(path): # new function for screenrecords/error-logs
  if os.path.exists(path):
    files = os.listdir(path)
    sorted_files = sorted(files, reverse=True)
  else:
    return []  # Return an empty list if there are no files or directory
  return sorted_files


def is_valid_segment(segment):
  try:
    segment_to_segment_name(Paths.log_root(), segment)
    return True
  except AssertionError:
    return False


def segment_to_segment_name(data_dir, segment):
  fake_dongle = "ffffffffffffffff"
  return SegmentName(str(os.path.join(data_dir, fake_dongle + "|" + segment)))


def all_segment_names():
  segments = []
  for segment in listdir_by_creation(Paths.log_root()):
    try:
      segments.append(segment_to_segment_name(Paths.log_root(), segment))
    except AssertionError:
      pass
  return segments


def all_routes():
  segment_names = all_segment_names()
  route_names = [segment_name.route_name for segment_name in segment_names]
  route_times = [route_name.time_str for route_name in route_names]
  unique_routes = list(dict.fromkeys(route_times))
  return sorted(unique_routes, reverse=True)

def preserved_routes():
  dirs = listdir_by_creation(Paths.log_root())
  preserved_segments = get_preserved_segments(dirs)
  return sorted(preserved_segments, reverse=True)

def has_preserve_xattr(d: str) -> bool:
  return getxattr(os.path.join(Paths.log_root(), d), PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE

def get_preserved_segments(dirs_by_creation: List[str]) -> List[str]:
  preserved = []
  for n, d in enumerate(filter(has_preserve_xattr, reversed(dirs_by_creation))):
    if n == PRESERVE_COUNT:
      break
    date_str, _, seg_str = d.rpartition("--")

    # ignore non-segment directories
    if not date_str:
      continue
    try:
      seg_num = int(seg_str)
    except ValueError:
      continue
    # preserve segment and its prior
    preserved.append(d)

  return preserved

def video_to_gif(input_path, output_path, fps=1, duration=6): # not used right now but can if want longer animated gif
  if os.path.exists(output_path):
    return
  command = [
    'ffmpeg', '-y', '-i', input_path,
    '-filter_complex',
    f'fps={fps},scale=240:-1:flags=lanczos,setpts=0.1*PTS,split[s0][s1];[s0]palettegen=max_colors=32[p];[s1][p]paletteuse=dither=bayer',
    '-t', str(duration), output_path
  ]
  subprocess.run(command)
  print(f"GIF file created: {output_path}")

def video_to_img(input_path, output_path, fps=1, duration=6):
  if os.path.exists(output_path):
    return
  subprocess.run(['ffmpeg', '-y', '-i', input_path, '-ss', '5', '-vframes', '1', output_path])
  print(f"GIF file created: {output_path}")

def segments_in_route(route):
  segment_names = [segment_name for segment_name in all_segment_names() if segment_name.time_str == route]
  segments = [segment_name.time_str + "--" + str(segment_name.segment_num) for segment_name in segment_names]
  return segments


def ffmpeg_mp4_concat_wrap_process_builder(file_list, cameratype, chunk_size=1024*512):
  command_line = ["ffmpeg"]
  if not cameratype == "qcamera":
    command_line += ["-f", "hevc"]
  command_line += ["-r", "20"]
  command_line += ["-i", "concat:" + file_list]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  if not cameratype == "qcamera":
    command_line += ["-vtag", "hvc1"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE,
    bufsize=chunk_size
  )


def ffmpeg_mp4_wrap_process_builder(filename):
  """Returns a process that will wrap the given filename
     inside a mp4 container, for easier playback by browsers
     and other devices. Primary use case is streaming segment videos
     to the vidserver tool.
     filename is expected to be a pathname to one of the following
       /path/to/a/qcamera.ts
       /path/to/a/dcamera.hevc
       /path/to/a/ecamera.hevc
       /path/to/a/fcamera.hevc
  """
  basename = filename.rsplit("/")[-1]
  extension = basename.rsplit(".")[-1]
  command_line = ["ffmpeg"]
  if extension == "hevc":
    command_line += ["-f", "hevc"]
  command_line += ["-r", "20"]
  command_line += ["-i", filename]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  if extension == "hevc":
    command_line += ["-vtag", "hvc1"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE
  )


def ffplay_mp4_wrap_process_builder(file_name):
  command_line = ["ffmpeg"]
  command_line += ["-i", file_name]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE
  )

def get_nav_active():
  if params.get("NavDestination", encoding='utf8') is not None:
    return True
  else:
    return False

def get_amap_key():
  token = params.get("AMapKey1", encoding='utf8')
  token2 = params.get("AMapKey2", encoding='utf8')
  return (token.strip() if token else None, token2.strip() if token2 else None)

def get_gmap_key():
  token = params.get("GMapKey", encoding='utf8')
  return token.strip() if token else None

def get_public_token():
  token = params.get("MapboxPublicKey", encoding='utf8')
  return token.strip() if token and token.startswith("pk") else None

def set_public_token(token):
  if not isinstance(token, str):
    raise ValueError("Mapbox token must be text.")
  token = token.strip()
  if not token.startswith("pk."):
    raise ValueError("Enter a Mapbox public token beginning with 'pk.'.")
  params.put("MapboxPublicKey", token)

def get_secret_token():
  token = params.get("MapboxSecretKey", encoding='utf8')
  return token.strip() if token and token.startswith("sk") else None

def set_secret_token(token):
  if not isinstance(token, str):
    raise ValueError("Mapbox secret token must be text.")
  token = token.strip()
  if not token.startswith("sk."):
    raise ValueError("Enter a Mapbox secret token beginning with 'sk.'.")
  params.put("MapboxSecretKey", token)

def get_search_input():
  return params.get_int("SearchInput")

def get_last_lon_lat():
  last_pos = params.get("LastGPSPosition")
  if last_pos:
    l = json.loads(last_pos)
  else:
    return 0.0, 0.0
  return l["longitude"], l["latitude"]

def get_locations():
  data = params.get("ApiCache_NavDestinations", encoding='utf-8')
  return data

def preload_favs():
  try:
    nav_destinations = json.loads(params.get("ApiCache_NavDestinations", encoding='utf8') or "{}")
  except TypeError:
    return (None, None, None, None, None)

  locations = {"home": None, "work": None, "fav1": None, "fav2": None, "fav3": None}

  for item in nav_destinations:
    label = item.get("label")
    if label in locations and locations[label] is None:
      locations[label] = item.get("place_name")

  return tuple(locations.values())

def parse_addr(postvars, lon, lat, valid_addr, token):
  addr = postvars.get("fav_val", [""])
  real_addr = None
  if addr != "favorites":
    try:
      dests = json.loads(params.get("ApiCache_NavDestinations", encoding='utf8') or "{}")
    except TypeError:
      dests = json.loads("[]")
    for item in dests:
      if "label" in item and item["label"] == addr:
        lat, lon, real_addr = item["latitude"], item["longitude"], item["place_name"]
        break
  return (real_addr, lon, lat, real_addr is not None, token)

def search_addr(postvars, lon, lat, valid_addr, token):
  if "addr_val" in postvars:
    addr = postvars.get("addr_val")
    if addr != "":
      # Properly encode the address to handle spaces
      addr_encoded = quote(addr)
      query = f"https://api.mapbox.com/geocoding/v5/mapbox.places/{addr_encoded}.json?access_token={token}&limit=1"
      # focus on place around last gps position
      lngi, lati = get_last_lon_lat()
      query += "&proximity=%s,%s" % (lngi, lati)
      r = requests.get(query)
      if r.status_code != 200:
        return (addr, lon, lat, valid_addr, token)
      j = json.loads(r.text)
      if not j["features"]:
        return (addr, lon, lat, valid_addr, token)
      lon, lat = j["features"][0]["geometry"]["coordinates"]
      valid_addr = True
  return (addr, lon, lat, valid_addr, token)

def set_destination(postvars, valid_addr):
  if postvars.get("latitude") is not None and postvars.get("longitude") is not None:
    postvars["lat"] = postvars.get("latitude")
    postvars["lon"] = postvars.get("longitude")
    postvars["save_type"] = "recent"
    nav_confirmed(postvars)
    valid_addr = True
  else:
    addr = postvars.get("place_name")
    token = get_public_token()
    data, lon, lat, valid_addr, token = search_addr(addr, lon, lat, valid_addr, token)
    postvars["lat"] = lat
    postvars["lon"] = lon
    postvars["save_type"] = "recent"
    nav_confirmed(postvars)
    valid_addr= True
  return postvars, valid_addr

def nav_confirmed(postvars):
  if postvars is not None:
    lat = float(postvars.get("lat"))
    lng = float(postvars.get("lon"))
    save_type = postvars.get("save_type")
    name = postvars.get("name") if postvars.get("name") is not None else ""
    if params.get_int("SearchInput") == 1:
      lng, lat = gcj02towgs84(lng, lat)
    params.put("NavDestination", "{\"latitude\": %f, \"longitude\": %f, \"place_name\": \"%s\"}" % (lat, lng, name))
    if name == "":
      name =  str(lat) + "," + str(lng)
    new_dest = {"latitude": float(lat), "longitude": float(lng), "place_name": name}
    if save_type == "recent":
      new_dest["save_type"] = "recent"
    else:
      new_dest["save_type"] = "favorite"
      new_dest["label"] = save_type
    val = params.get("ApiCache_NavDestinations", encoding='utf8')
    if val is not None:
      val = val.rstrip('\x00')
    dests = [] if val is None else json.loads(val)
    # type idx
    type_label_ids = {"home": None, "work": None, "fav1": None, "fav2": None, "fav3": None, "recent": []}
    idx = 0
    for d in dests:
      if d["save_type"] == "favorite":
        type_label_ids[d["label"]] = idx
      else:
        type_label_ids["recent"].append(idx)
      idx += 1
    if save_type == "recent":
      id = None
      if len(type_label_ids["recent"]) > 10:
        dests.pop(type_label_ids["recent"][-1])
    else:
      id = type_label_ids[save_type]
    if id is None:
      dests.insert(0, new_dest)
    else:
      dests[id] = new_dest
    params.put("ApiCache_NavDestinations", json.dumps(dests).rstrip("\n\r"))

def public_token_input(postvars):
  if postvars is None or "pk_token_val" not in postvars or postvars.get("pk_token_val")[0] == "":
    return postvars
  else:
    token = postvars.get("pk_token_val").strip()
    if "pk." not in token:
      return postvars
    else:
        params.put("MapboxPublicKey", token)
  return token

def app_token_input(postvars):
  if postvars is None or "sk_token_val" not in postvars or postvars.get("sk_token_val")[0] == "":
    return postvars
  else:
    token = postvars.get("sk_token_val").strip()
    if "sk." not in token:
      return postvars
    else:
        params.put("MapboxSecretKey", token)
  return token

def gmap_key_input(postvars):
  if postvars is None or "gmap_key_val" not in postvars or postvars.get("gmap_key_val")[0] == "":
    return postvars
  else:
    token = postvars.get("gmap_key_val").strip()
    params.put("GMapKey", token)
  return token

def amap_key_input(postvars):
  if postvars is None or "amap_key_val" not in postvars or postvars.get("amap_key_val")[0] == "":
    return postvars
  else:
    token = postvars.get("amap_key_val").strip()
    token2 = postvars.get("amap_key_val_2").strip()
    params.put("AMapKey1", token)
    params.put("AMapKey2", token2)
  return token

def gcj02towgs84(lng, lat):
  dlat = transform_lat(lng - 105.0, lat - 35.0)
  dlng = transform_lng(lng - 105.0, lat - 35.0)
  radlat = lat / 180.0 * pi
  magic = math.sin(radlat)
  magic = 1 - ee * magic * magic
  sqrtmagic = math.sqrt(magic)
  dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
  dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
  mglat = lat + dlat
  mglng = lng + dlng
  return [lng * 2 - mglng, lat * 2 - mglat]

def transform_lat(lng, lat):
  ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
  ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
  ret += (20.0 * math.sin(lat * pi) + 40.0 * math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
  ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 * math.sin(lat * pi / 30.0)) * 2.0 / 3.0
  return ret

def transform_lng(lng, lat):
  ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
  ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
  ret += (20.0 * math.sin(lng * pi) + 40.0 * math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
  ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 * math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
  return ret

def xor_encrypt_decrypt(data: str, key: str) -> str:
  return ''.join(chr(ord(c) ^ ord(key[i % len(key)])) for i, c in enumerate(data))

def encode_parameters(params_dict):
  serialized_data = json.dumps(params_dict)
  obfuscated_data = xor_encrypt_decrypt(serialized_data, XOR_KEY)
  encoded_data = base64.b64encode(obfuscated_data.encode('utf-8')).decode('utf-8')
  return encoded_data

def decode_parameters(encoded_string):
  obfuscated_data = base64.b64decode(encoded_string.encode('utf-8')).decode('utf-8')
  decrypted_data = xor_encrypt_decrypt(obfuscated_data, XOR_KEY)
  return json.loads(decrypted_data)

def get_all_toggle_values():
  toggle_values = {}
  for key, _, _ in frogpilot_default_params:
    if key in EXCLUDED_KEYS:
      continue
    raw_value = params.get(key)
    if isinstance(raw_value, bytes):
      value = raw_value.decode('utf-8')
    else:
      value = raw_value or "0"
    toggle_values[key] = value

  return encode_parameters(toggle_values)

def _get_toadpilot_capability():
  capability = {
    "supported": False,
    "vehicle_known": False,
    "fingerprint": "Not detected",
    "car_name": "Not detected",
    "lateral_control": "unknown",
    "openpilot_longitudinal": False,
    "steering_authority_flag": False,
    "panda_firmware_matched": toadpilot_steering_response_panda_matched(params),
    "stock_tune": {
      "lat_accel_factor": None,
      "lat_accel_offset": None,
      "friction": None,
    },
    "hard_limits": None,
  }

  car_params = params.get("CarParams") or params.get("CarParamsPersistent")
  if not car_params:
    capability["reason"] = ("No connected or saved vehicle is identified yet. Settings can still be saved; "
                            "TOADPILOT activates only after a supported car is detected.")
    return capability

  try:
    with car.CarParams.from_bytes(car_params) as CP:
      capability["vehicle_known"] = True
      capability["fingerprint"] = CP.carFingerprint
      capability["car_name"] = CP.carName
      capability["lateral_control"] = CP.lateralTuning.which()
      capability["openpilot_longitudinal"] = bool(CP.openpilotLongitudinalControl)
      capability["steering_authority_flag"] = any(
        int(config.safetyParam) & Panda.FLAG_TOYOTA_TOADPILOT_HIGH_TORQUE
        for config in CP.safetyConfigs
      )
      capability["supported"] = (CP.carName == "toyota" and CP.carFingerprint == TOADPILOT_FINGERPRINT and
                                 CP.lateralTuning.which() == "torque")
      if capability["supported"]:
        controller_params = CarControllerParams(CP)
        capability["hard_limits"] = {
          "steer_max": int(controller_params.STEER_MAX),
          "steer_delta_up": int(controller_params.STEER_DELTA_UP),
          "steer_delta_down": int(controller_params.STEER_DELTA_DOWN),
          "steer_error_max": int(controller_params.STEER_ERROR_MAX),
        }
      if CP.lateralTuning.which() == "torque":
        capability["stock_tune"] = {
          "lat_accel_factor": round(float(CP.lateralTuning.torque.latAccelFactor), 6),
          "lat_accel_offset": round(float(CP.lateralTuning.torque.latAccelOffset), 6),
          "friction": round(float(CP.lateralTuning.torque.friction), 6),
        }
      if not capability["supported"]:
        capability["reason"] = "This build currently enables TOADPILOT only for TOYOTA_CAMRY_TSS2 torque control."
  except Exception as error:
    capability["reason"] = f"Saved CarParams could not be decoded: {error}"

  return capability


def _get_toadpilot_learner():
  learner = {
    "available": False,
    "live_valid": False,
    "use_params": False,
    "version": None,
    "lat_accel_factor_raw": None,
    "lat_accel_factor_filtered": None,
    "lat_accel_offset_raw": None,
    "lat_accel_offset_filtered": None,
    "friction_raw": None,
    "friction_filtered": None,
    "total_bucket_points": 0,
    "cached_points": 0,
    "decay": None,
    "resets": None,
  }

  torque_cache = params.get("LiveTorqueParameters")
  if not torque_cache:
    return learner

  try:
    with log.Event.from_bytes(torque_cache) as event:
      if event.which() != "liveTorqueParameters":
        raise ValueError(f"unexpected event type: {event.which()}")
      live = event.liveTorqueParameters
      learner.update({
        "available": True,
        "live_valid": int(live.version) == TOADPILOT_TORQUE_CACHE_VERSION and bool(live.liveValid),
        "use_params": bool(live.useParams),
        "version": int(live.version),
        "lat_accel_factor_raw": round(float(live.latAccelFactorRaw), 6),
        "lat_accel_factor_filtered": round(float(live.latAccelFactorFiltered), 6),
        "lat_accel_offset_raw": round(float(live.latAccelOffsetRaw), 6),
        "lat_accel_offset_filtered": round(float(live.latAccelOffsetFiltered), 6),
        "friction_raw": round(float(live.frictionCoefficientRaw), 6),
        "friction_filtered": round(float(live.frictionCoefficientFiltered), 6),
        "total_bucket_points": int(live.totalBucketPoints),
        "cached_points": len(live.points),
        "decay": round(float(live.decay), 3),
        "resets": int(live.maxResets),
      })
  except Exception as error:
    learner["error"] = f"Cached LiveTorqueParameters could not be decoded: {error}"

  return learner

def _get_toadpilot_status(key, max_age_seconds=5.0):
  status = read_json_status(key)
  if not isinstance(status, dict):
    return {}
  if not status:
    return status

  published_at = status.get("published_at_unix")
  if (isinstance(published_at, bool) or not isinstance(published_at, (int, float)) or
      not math.isfinite(float(published_at))):
    status["age_seconds"] = None
    status["fresh"] = False
    return status

  raw_age_seconds = time.time() - float(published_at)
  status["age_seconds"] = round(max(raw_age_seconds, 0.0), 2)
  status["fresh"] = -1.0 <= raw_age_seconds <= max_age_seconds
  return status


def _get_toadpilot_last_torque_route():
  completed = read_json_status(TOADPILOT_TORQUE_LAST_ROUTE_KEY, params)
  checkpoint = read_json_status(TOADPILOT_TORQUE_CHECKPOINT_KEY, params)
  completed = completed if valid_toadpilot_torque_summary(completed) and completed["route_complete"] else {}
  checkpoint = checkpoint if valid_toadpilot_torque_summary(checkpoint) and not checkpoint["route_complete"] else {}

  # A late checkpoint from the same process can never downgrade its completed
  # route. Otherwise show whichever route has the newest trustworthy evidence.
  if completed and checkpoint and completed["route_id"] == checkpoint["route_id"]:
    status = completed
  else:
    candidates = (candidate for candidate in (completed, checkpoint) if candidate)
    status = max(candidates, key=lambda candidate: float(
      candidate["completed_at_unix"] if candidate["route_complete"] else candidate["published_at_unix"]
    ), default={})
  if not status:
    return {}

  age_timestamp = status.get("completed_at_unix") if status["route_complete"] else status.get("published_at_unix")
  raw_age_seconds = time.time() - float(age_timestamp)
  if raw_age_seconds < -1.0:
    return {}
  status["age_seconds"] = round(max(raw_age_seconds, 0.0), 2)
  status["provenance_valid"] = True
  return status


def _get_toadpilot_last_stop_go_route():
  status = read_json_status(TOADPILOT_STOP_GO_LAST_ROUTE_KEY, params)
  if not valid_toadpilot_stop_go_summary(status) or status.get("route_complete") is not True:
    return {}
  raw_age_seconds = time.time() - float(status["completed_at_unix"])
  if raw_age_seconds < -1.0:
    return {}
  status["age_seconds"] = round(max(raw_age_seconds, 0.0), 2)
  status["provenance_valid"] = True
  return status

def _get_toadpilot_road_state():
  persistent_onroad = params.get_bool("IsOnroad")
  persistent_offroad = params.get_bool("IsOffroad")
  shared_onroad = params_memory.get_bool("IsOnroad")
  onroad = bool(persistent_onroad or shared_onroad)
  offroad_confirmed = persistent_offroad and not persistent_onroad and not shared_onroad
  return {
    "onroad": onroad,
    "offroad_confirmed": bool(offroad_confirmed),
    "road_state": "onroad" if onroad else "offroad" if offroad_confirmed else "unknown",
    "settings_writable": not onroad,
  }


def _require_toadpilot_not_onroad():
  road_state = _get_toadpilot_road_state()
  if road_state["onroad"]:
    raise ValueError("TOADPILOT changes are blocked while the device is onroad; go offroad first.")
  return road_state


def _require_toadpilot_offroad():
  road_state = _get_toadpilot_road_state()
  if not road_state["offroad_confirmed"]:
    raise ValueError("This TOADPILOT operation requires a confirmed offroad device state.")
  return road_state


def _get_start_accel_ramp_capability():
  capability = {
    "supported": False,
    "vehicle_known": False,
    "fingerprint": "Not detected",
    "openpilot_longitudinal": False,
  }

  car_params = params.get("CarParams") or params.get("CarParamsPersistent")
  if not car_params:
    capability["reason"] = "No connected or saved vehicle is identified yet."
    return capability

  try:
    with car.CarParams.from_bytes(car_params) as CP:
      capability["vehicle_known"] = True
      capability["fingerprint"] = CP.carFingerprint
      capability["openpilot_longitudinal"] = bool(CP.openpilotLongitudinalControl)
      capability["supported"] = (CP.carName == "toyota" and CP.carFingerprint == TOADPILOT_FINGERPRINT and
                                 CP.openpilotLongitudinalControl)
      capability["reason"] = ("Camry TSS2 openpilot longitudinal control is available." if capability["supported"] else
                              "The start acceleration ramp is limited to Camry TSS2 openpilot longitudinal control.")
  except Exception as error:
    capability["reason"] = f"Saved CarParams could not be decoded: {error}"

  return capability


def get_start_accel_ramp_state():
  value = parse_start_accel_ramp(params.get(START_ACCEL_RAMP_PARAM))
  capability = _get_start_accel_ramp_capability()
  road_state = _get_toadpilot_road_state()
  return {
    "schema_version": 1,
    "value": value,
    "effective_value": START_ACCEL_RAMP_DEFAULT,
    "retired": True,
    "replacement": "responsive_takeoff",
    "default": START_ACCEL_RAMP_DEFAULT,
    "minimum": START_ACCEL_RAMP_MIN,
    "maximum": START_ACCEL_RAMP_MAX,
    "step": START_ACCEL_RAMP_STEP,
    "enabled": False,
    "capability": capability,
    "environment": road_state,
    "runtime": _get_toadpilot_status(TOADPILOT_STOP_GO_STATUS_KEY),
    "last_route": _get_toadpilot_last_stop_go_route(),
  }


def set_start_accel_ramp(request_data):
  road_state = _get_toadpilot_road_state()
  if road_state["onroad"]:
    raise ValueError("Start acceleration ramp changes are blocked while the device is onroad; go offroad first.")
  if set(request_data) != {"value"}:
    raise ValueError("Start acceleration ramp requires exactly one numeric value.")

  value = validate_start_accel_ramp(request_data["value"])

  # Recheck immediately before the persistent write in case ignition changed
  # while the request was being validated.
  if _get_toadpilot_road_state()["onroad"]:
    raise ValueError("Start acceleration ramp changes are blocked while the device is onroad; go offroad first.")
  params.put(START_ACCEL_RAMP_PARAM, f"{value:.2f}")
  if parse_start_accel_ramp(params.get(START_ACCEL_RAMP_PARAM)) != value:
    raise RuntimeError("Start acceleration ramp did not persist correctly.")

  update_frogpilot_toggles()
  return get_start_accel_ramp_state()


def _get_toadpilot_steering_experiment(capability, road_state):
  stored_profile = read_toadpilot_steering_response_profile(params)
  requested = read_toadpilot_steering_response_request(params)
  boot_latched = read_toadpilot_steering_response_latch()
  supported_vehicle = capability.get("supported") is True
  controller_flag_present = capability.get("steering_authority_flag") is True
  panda_firmware_matched = capability.get("panda_firmware_matched") is True
  stale_profile = stored_profile if stored_profile not in (None, TOADPILOT_STEERING_RESPONSE_PROFILE) else None
  legacy_profile = stale_profile in TOADPILOT_LEGACY_STEERING_RESPONSE_PROFILES
  pending_reboot = requested != boot_latched
  effective = bool(boot_latched and supported_vehicle and controller_flag_present and panda_firmware_matched)

  if stale_profile is not None:
    state = "stale_profile"
    reason = ("A saved request targets an older steering profile and cannot activate this envelope. "
              "Confirm the new profile offroad, then reboot.")
  elif effective and pending_reboot:
    state = "pending_disable"
    reason = "The experimental envelope remains effective for this boot; stock limits return after reboot."
  elif effective:
    state = "effective"
    reason = "The exact Camry controller/Panda authority flag is active for this boot."
  elif pending_reboot and requested:
    state = "pending_enable"
    reason = "The new profile is saved but cannot become active until reboot."
  elif pending_reboot:
    state = "pending_disable"
    reason = "The boot latch remains set; the saved request is off and takes effect after reboot."
  elif requested and boot_latched and not panda_firmware_matched:
    state = "armed_waiting_for_firmware"
    reason = ("The profile is boot-latched, but the connected Panda does not report the exact paired firmware "
              "signature. Stock limits remain effective.")
  elif requested and boot_latched:
    state = "armed_waiting_for_controller"
    reason = ("The profile is boot-latched, but the saved/current CarParams do not report the matching "
              "controller/Panda flag. Stock limits remain effective.")
  else:
    state = "off"
    reason = "Not requested; the stock controller/Panda steering envelope remains effective."

  if (requested or boot_latched) and not supported_vehicle:
    reason += " A supported Camry TSS2 torque-control fingerprint has not been confirmed."

  return {
    "profile": TOADPILOT_STEERING_RESPONSE_PROFILE,
    "stored_profile": stored_profile,
    "stale_profile": stale_profile,
    "legacy_profile": legacy_profile,
    "requested": requested,
    "boot_latched": boot_latched,
    "controller_flag_present": controller_flag_present,
    "panda_firmware_matched": panda_firmware_matched,
    "available": True,
    "effective": effective,
    "pending_reboot": pending_reboot,
    "requires_reboot": True,
    "request_writable": road_state["offroad_confirmed"],
    "state": state,
    "reason": reason,
  }



def get_toadpilot_state():
  config = read_toadpilot_config(params)
  capability = _get_toadpilot_capability()
  road_state = _get_toadpilot_road_state()
  return {
    "config": asdict(config),
    "capability": capability,
    "effective_enabled": bool(config.enabled and capability["supported"]),
    "learner": _get_toadpilot_learner(),
    "runtime": _get_toadpilot_status(TOADPILOT_STATUS_KEY),
    "steering_status": _get_toadpilot_status(TOADPILOT_STEERING_STATUS_KEY),
    "stop_go_status": _get_toadpilot_status(TOADPILOT_STOP_GO_STATUS_KEY),
    "stop_go_last_route": _get_toadpilot_last_stop_go_route(),
    "navigation_status": _get_toadpilot_status(TOADPILOT_NAVIGATION_STATUS_KEY),
    "tune_status": _get_toadpilot_status(TOADPILOT_TUNE_STATUS_KEY),
    "torque_status": _get_toadpilot_status(TOADPILOT_TORQUE_STATUS_KEY),
    "torque_last_route": _get_toadpilot_last_torque_route(),
    "steering_experiment": _get_toadpilot_steering_experiment(capability, road_state),
    "environment": {
      "experimental_mode": params.get_bool("ExperimentalMode"),
      "conditional_experimental": params.get_bool("ConditionalExperimental"),
      "conditional_curves": params.get_bool("CECurves"),
      "nnff": params.get_bool("NNFF"),
      "nnff_lite": params.get_bool("NNFFLite"),
      "legacy_prototype_enabled": params.get_bool("ToyotaPerformanceMode"),
      **road_state,
    },
  }


def set_toadpilot_config(request_data):
  _require_toadpilot_not_onroad()

  request_data = dict(request_data)
  steering_request_present = "experimental_steering_response_requested" in request_data
  steering_request = request_data.pop("experimental_steering_response_requested", None)
  if steering_request_present and type(steering_request) is not bool:
    raise ValueError("Experimental Camry steering response request must be a boolean.")

  stored_steering_profile = read_toadpilot_steering_response_profile(params)
  steering_request_changed = steering_request_present and (
    (steering_request and stored_steering_profile != TOADPILOT_STEERING_RESPONSE_PROFILE) or
    (not steering_request and stored_steering_profile is not None)
  )
  if steering_request_changed:
    _require_toadpilot_offroad()

  required_fields = {
    "version", "enabled", "profile", "auto_tune", "lane_centering_assist", "smooth_lane_transitions", "straight_line_smoothing", "experimental_max_torque",
    "improved_stop_and_go", "responsive_takeoff", "smooth_stop_finish", "automatic_traffic_mode", "traffic_mode_entry_time", "traffic_mode_max_speed_mph", "map_stop_assist",
    "curve_assist_in_experimental", "smart_saturation_warning", "telemetry",
  }
  missing_fields = required_fields - set(request_data)
  if missing_fields:
    raise ValueError(f"Missing TOADPILOT setting(s): {', '.join(sorted(missing_fields))}")

  config = normalize_toadpilot_config(request_data)
  _require_toadpilot_not_onroad()
  if steering_request_changed:
    _require_toadpilot_offroad()
  write_toadpilot_config(config, params)
  if steering_request_changed:
    write_toadpilot_steering_response_request(steering_request, params)
  if not config.enabled or not config.telemetry:
    params_memory.remove(TOADPILOT_STATUS_KEY)
    params_memory.remove(TOADPILOT_STEERING_STATUS_KEY)
    params_memory.remove(TOADPILOT_STOP_GO_STATUS_KEY)
    params_memory.remove(TOADPILOT_NAVIGATION_STATUS_KEY)
    params_memory.remove(TOADPILOT_TUNE_STATUS_KEY)
  update_frogpilot_toggles()
  return get_toadpilot_state()


def reset_toadpilot_tune():
  _require_toadpilot_offroad()

  params.remove("LiveTorqueParameters")
  params.remove(TOADPILOT_TORQUE_CHECKPOINT_KEY)
  params.remove(TOADPILOT_TORQUE_LAST_ROUTE_KEY)
  params.put_bool(TOADPILOT_RESET_TUNE_KEY, True)
  params_memory.remove(TOADPILOT_STATUS_KEY)
  params_memory.remove(TOADPILOT_TUNE_STATUS_KEY)
  params_memory.remove(TOADPILOT_TORQUE_STATUS_KEY)
  return {
    "message": "Cached learning data was cleared and an in-memory reset was queued for torqued.",
    "state": get_toadpilot_state(),
  }

def reset_toggle_values():
  params.put_bool("DoToggleReset", True)
  HARDWARE.reboot()

def store_toggle_values(request_data):
  allowed_keys = {key for key, _, _ in frogpilot_default_params}
  allowed_keys -= EXCLUDED_KEYS

  toggle_values = decode_parameters(request_data['data'])
  for key, value in toggle_values.items():
    if key not in allowed_keys:
      continue
    params.put(key, value)

  update_frogpilot_toggles()

def capture_tmux_log():
  os.makedirs(TMUX_LOGS_PATH, exist_ok=True)

  timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
  log_filename = f"tmux_log_{timestamp}.txt"
  log_path = os.path.join(TMUX_LOGS_PATH, log_filename)

  try:
    subprocess.run(["tmux", "capture-pane", "-J", "-S", "-"], check=True)
    result = subprocess.run(["tmux", "show-buffer"], capture_output=True, text=True, check=True)

    with open(log_path, "w", encoding="utf-8") as log_file:
      log_file.write(result.stdout)

    subprocess.run(["tmux", "delete-buffer"], check=True)

    return log_filename

  except subprocess.CalledProcessError as e:
    raise Exception(f"Error capturing tmux log: {e}")

def lock_doors():
  try:
    print("Attempting to lock doors...")

    panda = Panda()
    print(f"Panda connected: {panda.get_usb_serial()}")
    print(f"Firmware Version: {panda.get_version()}")
    print(f"Hardware Type: {panda.get_type()}")
    initial_safety = panda.health()["safety_mode"]
    print(f"Initial Safety Mode: {initial_safety}")

    os.system("pkill -STOP -f pandad")

    print("Setting safety mode to SAFETY_TOYOTA...")
    panda.set_safety_mode(panda.SAFETY_TOYOTA)

    new_safety = panda.health()["safety_mode"]
    print(f"Safety Mode after change: {new_safety}")

    if new_safety != panda.SAFETY_TOYOTA:
      print("Failed to set SAFETY_TOYOTA! Aborting.")
      return

    can_health = panda.can_health(0)
    print(f"CAN0 Health: {can_health}")

    print(f"Sending CAN message to lock doors (ID=0x750, CMD={LOCK_CMD.hex()})...")
    panda.can_send(0x750, LOCK_CMD, 0)

    new_can_health = panda.can_health(0)
    print(f"CAN0 Health after send: {new_can_health}")

    final_safety = panda.health()["safety_mode"]
    print(f"Final Safety Mode: {final_safety}")

    print("Sending Panda heartbeat...")
    panda.send_heartbeat()

    print("Closing Panda connection.")
    panda.close()

    print("Lock doors operation completed successfully!")

    print("Starting the pandad process...")
    os.system("pkill -CONT -f pandad")

  except Exception as error:
    print(f"Error in lock_doors(): {error}")
    print(traceback.format_exc())

def unlock_doors():
  try:
    print("Attempting to unlock doors...")

    panda = Panda()
    print(f"Panda connected: {panda.get_usb_serial()}")
    print(f"Firmware Version: {panda.get_version()}")
    print(f"Hardware Type: {panda.get_type()}")
    initial_safety = panda.health()["safety_mode"]
    print(f"Initial Safety Mode: {initial_safety}")

    os.system("pkill -STOP -f pandad")

    print("Setting safety mode to SAFETY_TOYOTA...")
    panda.set_safety_mode(panda.SAFETY_TOYOTA)

    new_safety = panda.health()["safety_mode"]
    print(f"Safety Mode after change: {new_safety}")

    if new_safety != panda.SAFETY_TOYOTA:
      print("Failed to set SAFETY_TOYOTA! Aborting.")
      return

    can_health = panda.can_health(0)
    print(f"CAN0 Health: {can_health}")

    print(f"Sending CAN message to unlock doors (ID=0x750, CMD={UNLOCK_CMD.hex()})...")
    panda.can_send(0x750, UNLOCK_CMD, 0)

    new_can_health = panda.can_health(0)
    print(f"CAN0 Health after send: {new_can_health}")

    final_safety = panda.health()["safety_mode"]
    print(f"Final Safety Mode: {final_safety}")

    print("Sending Panda heartbeat...")
    panda.send_heartbeat()

    print("Closing Panda connection.")
    panda.close()

    print("Unlock doors operation completed successfully!")

    print("Starting the pandad process...")
    os.system("pkill -CONT -f pandad")

  except Exception as error:
    print(f"Error in unlock_doors(): {error}")
    print(traceback.format_exc())

def reboot_device():
  HARDWARE.reboot()
