#!/usr/bin/env python3
"""Exercise onroad controls using the native UI objects and isolated Params."""
import os
from pathlib import Path
import shutil
import subprocess
import uuid

from openpilot.common.params import Params

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / '.native-build'


def main():
  makefile = BUILD / 'Makefile.onroad-test'
  makefile.write_text('''include Makefile
test_onroad_controls.o: ../tools/native_ui/test_onroad_controls.cc ../selfdrive/ui/qt/onroad/curve_status.h ../selfdrive/ui/qt/onroad/speed_limit_button.h ../selfdrive/ui/qt/widgets/cameraview.h
\t$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<
test-onroad-controls: test_onroad_controls.o
\t$(LINK) $(LFLAGS) -o $@ $(filter-out main.o,$(OBJECTS)) test_onroad_controls.o $(LIBS) -lQt5Test
''')
  subprocess.run(['make', '-f', makefile.name, '-j1', 'test-onroad-controls'], cwd=BUILD, check=True)
  os.environ['OPENPILOT_PREFIX'] = 'onroad-test-' + uuid.uuid4().hex[:8]
  aliases = []
  try:
    for base in ['/data/params', '/dev/shm/params']:
      alias = Path(Params(base).get_param_path())
      aliases.append((alias, alias.resolve()))
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen', QT_QUICK_BACKEND='software')
    subprocess.run([str(BUILD / 'test-onroad-controls')], cwd=BUILD, env=env, check=True)
  finally:
    for alias, target in aliases:
      alias.unlink(missing_ok=True)
      shutil.rmtree(target)


if __name__ == '__main__':
  main()
