Abrasax native UI upgrade

The installed branch was a stripped release. Native source was recovered from
MrXelious/FrogPilot at 834a37e6b5b881e7af2866ca30f8202a8f376774. Its original
cereal/car.capnp and cereal/log.capnp match the initial ToadPilot snapshot byte
for byte. Native sources retain their original licenses; the manifest records
Git blob hashes. Existing Python code, current schemas, vehicle settings and
model artifacts are retained. The C++ generator uses the CURRENT schemas.

Build from the repository root:
  python3 tools/native_ui/restore_sources.py
  python3 tools/native_ui/build.py

Output is .native-build/ui-abrasax. The installed executable is selfdrive/ui/ui-abrasax; the original selfdrive/ui/ui remains available for rollback. The build never replaces either executable automatically.
It uses AGNOS 10.1 Qt 5.12, capnp 1.0.2, QCOM OpenCL, the existing MapLibre
library and the pinned libyuv archive. Missing source downloads are verified
against the manifest before writing. Local source modifications are preserved.

Panel tests (mock HTTP server; never write vehicle settings):
  mkdir -p .native-preview
  cd .native-preview
  qmake ../tools/native_ui/test_panel.pro -o Makefile.test
  make -f Makefile.test -j2
  QT_QPA_PLATFORM=offscreen ./test-panel

Panel preview:
  cd selfdrive/ui
  QT_QPA_PLATFORM=offscreen ABRASAX_SCREENSHOT=/tmp/abrasax-settings.png \
    ../../.native-build/ui-abrasax --abrasax-preview
TOADPILOT_API may point at an isolated Fleet Manager while developing.

The ToadPilot settings panel shares the existing validated Fleet Manager API.
It preserves untouched configuration fields, verifies save responses, and
pauses polling when hidden. Model changes require confirmed offroad state and
an installed matching runtime. Explicit model selection disables randomization.

The onroad status strip shows the model, mode, assistance status and recording
state. It hides during alerts and never accepts input. Vehicle controllers,
model inference and safety limits are not changed by the UI upgrade.

Trip diagnostics run at 5 Hz onroad and 1 Hz offroad, retain only compact scalar
samples and rotate local files within a 32 MiB budget. Controller diagnostics
must be enabled to record. Recordings contain no camera video or GPS locations.
Fleet Manager /diagnostics offers bounded trace previews and full downloads.
Steering-input counts are observations, not automatic classifications of a
necessary intervention. These traces supplement, rather than replace, normal
full-rate route logs. They cannot establish road behavior from offroad tests.

Native queue tests:
  clang++ -std=c++17 -O2 -I msgq_repo tools/native_ui/test_msgq.cc \
    msgq_repo/msgq/msgq.cc -o /tmp/test-msgq
  /tmp/test-msgq
The tests exercise 10,000 ring-buffer messages across many wraps and mapping
failure cleanup. The service-size backport is restricted to isolated namespaces
with TOADPILOT_BOUNDED_IPC=1. Production queues retain their legacy size.

The old loggerd remains disabled. Its memory problem has not been established
as solely a queue-sizing problem. All legacy native producers and consumers
still require matching rebuilds before smaller shared-memory queues can be
activated. Never truncate a live queue or mix queue sizes across processes.

Logo: original unmodified PNG from
https://abrasax.technology/static/img/abrasax_logo.png?v=transparent-20261012
Used in the native settings, offroad home/sidebar and Fleet Manager.
