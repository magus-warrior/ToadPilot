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

API tests (mock HTTP server; never write vehicle settings):
  mkdir -p .native-preview
  cd .native-preview
  qmake ../tools/native_ui/test_panel.pro -o Makefile.test
  make -f Makefile.test -j2
  QT_QPA_PLATFORM=offscreen ./test-panel

Full UI previews (isolated queues and parameter copies; no model/config saves):
  python3 tools/native_ui/smoke.py
  ABRASAX_PREVIEW_PANEL=models python3 tools/native_ui/smoke.py
  ABRASAX_PREVIEW_PANEL=gas python3 tools/native_ui/smoke.py
TOADPILOT_API may point at an isolated Fleet Manager while developing.

The extra ToadPilot page was removed. FrogPilot's Driving Controls / Driving
Model page uses its original full-screen touch dialog, with the validated Fleet
Manager installed-model API. Direct selection requires confirmed offroad state
and a matching runtime. Explicit model selection disables randomization. The dialog
uses the same Wayland rotation as the main window and never rebuilds on a timer.
Onroad selection opens a reboot confirmation. It accepts either a tap on Reboot
or one second of fresh cruise-off / control-inactive data. The request is then
queued in Fleet Manager, whose independent watcher verifies device state,
cruise-off, controls disabled, and both lateral/longitudinal actuation inactive
for a continuous second before saving the model and setting the normal DoReboot
request. Stale/invalid/future data or sampling gaps reset that confirmation.
It sends no cruise-cancel commands and makes no live model change. A pending
request can be cancelled in the model row and expires after 120 seconds. Fleet
Manager restart also cancels pending requests. No moving-vehicle reboot has been
performed during validation; tests inject all control states and mock writes.
The Gas / Brake menu exposes accelerator-override learning. Its switch updates
learning and its required coordination option together, preserving other config
fields and confirming the saved response. Below 10 mph the original blended
planner remains active, preserving the existing hybrid launch / stop-go path.
Legacy steering assistance remains disabled in the saved device configuration.

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
Used on white backgrounds in offroad home/sidebar and Fleet Manager.
The boot surface uses assets/other_images/abrasax_boot_logo.png, rendered from
the original artwork by tools/native_ui/render_boot_logo.cc with a white
background and the compositor's portrait orientation. setup_frogpilot installs
it to /usr/comma/bg.jpg. The previous boot image and executable are backed up
under /data/toadpilot/ui-fix-backup-20261105-151819.
