#!/usr/bin/env python3
"""Exercise onroad controls using the native UI objects and isolated Params."""
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import shutil
import subprocess
from threading import Thread
import time
import uuid

from openpilot.common.params import Params

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / '.native-build'



class SearchHandler(BaseHTTPRequestHandler):
  def do_POST(self):
    query = json.loads(self.rfile.read(int(self.headers['Content-Length'])))['query']
    if self.path != '/navigation/search':
      self.send_error(404)
      return
    if query == '100':
      time.sleep(1.1)
    body = json.dumps({'results': [{'place_name': query + ' Street, Example City', 'latitude': 37., 'longitude': -122.}]}).encode()
    self.send_response(200)
    self.send_header('Content-Type', 'application/json')
    self.send_header('Content-Length', str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def log_message(self, *_):
    pass

def main():
  makefile = BUILD / 'Makefile.onroad-test'
  makefile.write_text('''include Makefile
test_onroad_controls.o: ../tools/native_ui/test_onroad_controls.cc ../selfdrive/ui/qt/onroad/curve_status.h ../selfdrive/ui/qt/onroad/speed_limit_button.h ../selfdrive/ui/qt/onroad/annotated_camera.h ../selfdrive/ui/qt/sidebar.h ../selfdrive/ui/qt/widgets/cameraview.h
\t$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<
test-onroad-controls: test_onroad_controls.o $(filter-out main.o,$(OBJECTS))
\t$(LINK) $(LFLAGS) -o $@ $(filter-out main.o,$(OBJECTS)) test_onroad_controls.o $(LIBS) -lQt5Test
''')
  subprocess.run(['make', '-f', makefile.name, '-j1', 'test-onroad-controls'], cwd=BUILD, check=True)
  os.environ['OPENPILOT_PREFIX'] = 'onroad-test-' + uuid.uuid4().hex[:8]
  queue_dir = Path('/dev/shm') / os.environ['OPENPILOT_PREFIX']
  queue_dir.mkdir(mode=0o700)
  aliases = []
  try:
    for base in ['/data/params', '/dev/shm/params', '/dev/shm/params_default', '/cache/params', '/cache/tracking']:
      alias = Path(Params(base).get_param_path())
      aliases.append((alias, alias.resolve()))
    Params('/dev/shm/params').put('FrogPilotToggles', '{"signal_icons":"arrows","wheel_image":"stock","color_scheme":"stock"}')
    with ThreadingHTTPServer(('127.0.0.1', 0), SearchHandler) as server:
      Thread(target=server.serve_forever, daemon=True).start()
      env = dict(os.environ, QT_QPA_PLATFORM='offscreen', QT_QUICK_BACKEND='software', ABRASAX_UI_PREVIEW='1',
                 TOADPILOT_BOUNDED_IPC='1', TOADPILOT_API=f'http://127.0.0.1:{server.server_port}')
      try:
        subprocess.run([str(BUILD / 'test-onroad-controls')], cwd=BUILD, env=env, check=True, timeout=30)
      finally:
        server.shutdown()
  finally:
    for alias, target in aliases:
      alias.unlink(missing_ok=True)
      shutil.rmtree(target)
    shutil.rmtree(queue_dir)


if __name__ == '__main__':
  main()
