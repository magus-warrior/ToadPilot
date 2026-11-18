#include <QApplication>
#include <QPainter>
#include <QtTest/QTest>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "selfdrive/ui/qt/onroad/speed_limit_button.h"
#include "selfdrive/ui/qt/onroad/curve_status.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

static void check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

class TestCameraWidget : public CameraWidget {
public:
  TestCameraWidget() : CameraWidget("test", VISION_STREAM_ROAD, false) {}

  void checkFrameSelection() {
    check(selectFrame(0) == frames.crend(), "An empty stream must have no selected frame");
    frames = {{0, nullptr}, {1, nullptr}, {2, nullptr}};
    check(selectFrame(0)->first == 0, "Frame zero must match");
    check(selectFrame(1)->first == 1, "An older matching frame must be selected");
    check(selectFrame(3)->first == 2, "A missing model frame must fall back to the newest camera frame");
    check(selectFrame(std::nullopt)->first == 2, "A camera without overlays must show the newest frame");
  }
};

int main(int argc, char **argv) {
  // The caller must isolate both persistent and shared Params before starting.
  check(qEnvironmentVariable("OPENPILOT_PREFIX").startsWith("onroad-test-"), "Isolated Params required");
  QApplication app(argc, argv);
  TestCameraWidget().checkFrameSelection();
  Params params;
  Params memory("/dev/shm/params");
  params.putBool("SpeedLimitController", true);
  params.putBool("VisionTurnControl", true);
  memory.putBool("FrogPilotTogglesUpdated", false);

  SpeedLimitButton button;
  button.updateState(true, true);
  button.show();
  app.processEvents();
  check(button.isChecked() && button.isEnabled(), "Initial ON state");
  QPixmap preview(440, 230);
  preview.fill(QColor(40, 40, 40));
  QPainter painter(&preview);
  painter.drawPixmap(10, 10, button.grab());
  QTest::mouseClick(&button, Qt::LeftButton);
  check(!params.getBool("SpeedLimitController"), "OFF must persist");
  check(memory.getBool("FrogPilotTogglesUpdated"), "OFF must refresh the planner");
  button.updateState(true, true);
  check(!button.isChecked(), "Waiting for the planner must not flash back ON");
  button.updateState(true, false);
  check(!button.isChecked(), "Effective OFF state");
  painter.drawPixmap(230, 10, button.grab());
  painter.end();
  preview.save("onroad-speed-limit-button.png");

  memory.putBool("FrogPilotTogglesUpdated", false);
  QTest::mouseClick(&button, Qt::LeftButton);
  check(params.getBool("SpeedLimitController"), "ON must persist");
  check(memory.getBool("FrogPilotTogglesUpdated"), "ON must refresh the planner");
  check(params.getBool("VisionTurnControl"), "Curve controller preference must be preserved");
  button.updateState(true, true);
  button.updateState(false, true);
  QTest::mouseClick(&button, Qt::LeftButton);
  check(params.getBool("SpeedLimitController"), "Unsupported button must ignore taps");
  button.updateState(true, false);
  check(!button.isChecked(), "External setting changes must refresh the button");

  // Captured case: dashboard 80.467 km/h, internal cruise and both targets 78.
  check(!curveSpeedLimited(78, true, 78 / 3.6f, true, 78 / 3.6f), "No phantom cluster reduction");
  check(curveSpeedLimited(78, false, 78 / 3.6f, true, 15), "Vision reduction must show");
  check(curveSpeedLimited(78, true, 15, false, 78 / 3.6f), "Map reduction must show");
  check(!curveSpeedLimited(78, false, 15, false, 15), "Disabled controllers must not show");
  check(!curveSpeedLimited(78, true, 0, true, NAN), "Missing targets must not show");
  check(!curveSpeedLimited(255, true, 15, true, 15), "Unset cruise must not show");
  check(!curveSpeedLimited(78, true, 78 / 3.6f - .1f, false, 0), "Rounding noise must not show");
  std::cout << "Onroad camera selection, button taps, setting refresh and curve display regression checks passed\n";
}
