#include <QApplication>
#include <QDialog>
#include <QDir>
#include <QFontDatabase>
#include <QPainter>
#include <QLineEdit>
#include <QListWidget>
#include <QTimer>
#include <QtTest/QTest>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "selfdrive/ui/qt/maps/map_settings.h"
#include "selfdrive/ui/qt/onroad/annotated_camera.h"
#include "selfdrive/ui/qt/sidebar.h"
#include "selfdrive/ui/qt/onroad/speed_limit_button.h"
#include "selfdrive/ui/qt/onroad/curve_status.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

static void check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

class HudPreview : public AnnotatedCameraWidget {
public:
  HudPreview() : AnnotatedCameraWidget(VISION_STREAM_ROAD) {}
  QImage renderHud(const UIState &s, int alertHeight = 0, const QImage &road = {}) {
    updateState(alertHeight, s);
    QImage frame(size(), QImage::Format_RGB32);
    frame.fill(QColor("#2c465a"));
    QPainter p(&frame);
    p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing | QPainter::SmoothPixmapTransform);
    if (!road.isNull()) {
      const QSize crop = size().scaled(road.size(), Qt::KeepAspectRatio);
      QRect source(QPoint(), crop);
      source.moveCenter(road.rect().center());
      p.drawImage(rect(), road, source);
    }
    drawHud(p);
    auto *sign = findChild<QPushButton *>("speedLimitControllerButton");
    if (!sign->isHidden()) sign->render(&p, sign->pos(), QRegion(), QWidget::DrawChildren);
    return frame;
  }
};

static void checkOnroadAppearance() {
  const auto savedCwd = QDir::currentPath();
  check(QDir::setCurrent(QCoreApplication::applicationDirPath() + "/../selfdrive/ui"), "Driving UI assets must be available");
  for (const auto &style : {"Regular", "SemiBold", "Bold"})
    QFontDatabase::addApplicationFont(QString("../assets/fonts/Inter-%1.ttf").arg(style));
  Params().putBool("SpeedLimitController", true);
  auto *s = uiState();
  MessageBuilder carMsg, controlsMsg, planMsg, deviceMsg, locationMsg;
  auto carEvent = carMsg.initEvent(), controlsEvent = controlsMsg.initEvent(), planEvent = planMsg.initEvent(), deviceEvent = deviceMsg.initEvent();
  carEvent.initCarState().setVEgo(42. / MS_TO_MPH);
  auto controls = controlsEvent.initControlsState();
  controls.setVCruise(50. / KM_TO_MILE);
  controls.setVCruiseCluster(50. / KM_TO_MILE);
  planEvent.initFrogpilotPlan();
  auto dev = deviceEvent.initDeviceState();
  dev.setNetworkType(cereal::DeviceState::NetworkType::WIFI);
  dev.setNetworkStrength(cereal::DeviceState::NetworkStrength::GOOD);
  dev.setLastAthenaPingTime(nanos_since_boot());
  auto locationEvent = locationMsg.initEvent();
  locationEvent.initLiveLocationKalman().setGpsOK(true);
  s->sm->update_msgs(nanos_since_boot(), {{"carState", carEvent}, {"controlsState", controlsEvent},
                                          {"frogpilotPlan", planEvent}, {"deviceState", deviceEvent}, {"liveLocationKalman", locationEvent}});
  s->status = STATUS_ENGAGED;
  auto &scene = s->scene;
  scene.started = true;
  scene.pandaType = cereal::PandaState::PandaType::TRES;
  scene.enabled = scene.show_speed_limits = scene.speed_limit_controller = scene.longitudinal_control = true;
  scene.speed_limit_source = "Dashboard";
  scene.speed_limit = 40. / MS_TO_MPH;
  scene.speed_limit_offset = 10. / MS_TO_MPH;
  scene.show_speed_limit_offset = true;
  const QColor amber("#ffcb67"), red("#ff626a");
  HudPreview hud;
  hud.resize(1800, 1020);
  const QPoint left(900 - 222, 125), right(900 + 222, 125);
  scene.turn_signal_left = true;
  auto frame = hud.renderHud(*s);
  check(frame.pixelColor(left) == amber && frame.pixelColor(right) != amber, "Left signal must show only on the left");
  scene.turn_signal_right = scene.blind_spot_left = true;
  frame = hud.renderHud(*s);
  check(frame.pixelColor(left) == red && frame.pixelColor(right) == amber, "Hazards must preserve each side's blind-spot warning");
  frame = hud.renderHud(*s, 200);
  check(frame.pixelColor(left) != red && frame.pixelColor(right) != amber, "Alerts must take priority over the arrows");
  hud.resize(940, 1020);
  frame = hud.renderHud(*s);
  check(frame.pixelColor(470 + 122, 272) == amber, "Arrow must fit beside speed units on a narrow camera view");
  scene.turn_signal_left = scene.blind_spot_left = false;
  scene.frogpilot_toggles["signal_icons"] = "none";
  check(hud.renderHud(*s).pixelColor(470 + 122, 272) != amber, "Turning signals off must hide the native arrows");

  scene.frogpilot_toggles["signal_icons"] = "arrows";
  hud.resize(1800, 1020);
  frame = hud.renderHud(*s, 0, QImage(qEnvironmentVariable("TOADPILOT_PREVIEW_FRAME")));
  Sidebar sidebar;
  sidebar.resize(300, 1080);
  sidebar.offroadTransition(false);
  sidebar.show();
  sidebar.updateState(*s);
  const auto rail = sidebar.grab().toImage();
  check(rail.pixelColor(8, 16) == QColor("#0a1b2b") && rail.pixelColor(150, 911) == QColor("#86eedc"),
        "Driving sidebar must render its navy background and bookmark");
  QImage screen(2160, 1080, QImage::Format_RGB32);
  screen.fill(bg_colors[STATUS_ENGAGED].rgb());
  QPainter p(&screen);
  p.drawImage(0, 0, rail);
  p.drawImage(330, 30, frame);
  p.end();
  check(QDir::setCurrent(savedCwd) && QDir().mkpath("../.native-preview") && screen.save("../.native-preview/onroad-hud.jpg", nullptr, 88),
        "Save driving HUD preview");
}

class TestCameraWidget : public CameraWidget {
public:
  TestCameraWidget() : CameraWidget("test", VISION_STREAM_ROAD, false) {}

  void checkFrameSelection() {
    const auto [empty, empty_overlay] = selectFrame(0);
    check(empty == frames.crend() && !empty_overlay, "An empty stream has no video or overlay");
    frames = {{0, nullptr}};
    check(selectFrame(0).second, "Frame zero must match");
    frames.emplace_back(1, nullptr);
    const auto [video_before_model, overlay_before_model] = selectFrame(1);
    check(video_before_model->first == 1 && overlay_before_model, "An exact match shows the overlay");
    frames.emplace_back(2, nullptr);
    const auto [video_with_late_model, overlay_with_late_model] = selectFrame(0);
    check(video_with_late_model->first == 2 && overlay_with_late_model, "A late model must not rewind the video");
    frames.emplace_back(3, nullptr);
    const auto [video_with_stale_model, overlay_with_stale_model] = selectFrame(0);
    check(video_with_stale_model->first == 3 && !overlay_with_stale_model, "A stale model hides its overlay");
    frames.emplace_back(5, nullptr);
    const auto [video_with_missing_model, overlay_with_missing_model] = selectFrame(4);
    check(video_with_missing_model->first == 5 && !overlay_with_missing_model, "A dropped camera frame cannot match the model");
    check(!selectFrame(6).second, "A future model frame must not show an overlay");
    check(selectFrame(std::nullopt).first->first == 5 && !selectFrame(std::nullopt).second, "Video advances without overlays");
  }
};

static void checkDestinationSearch() {
  MapSettings panel;
  panel.resize(2160, 1080);
  panel.show();
  auto *search = panel.findChild<QPushButton *>("navigationSearch");
  check(search, "Navigation exposes address search");
  Params params;
  bool typed = false, matched = false, confirmed = false;
  QElapsedTimer elapsed;
  QTimer drive;
  QObject::connect(&drive, &QTimer::timeout, [&] {
    QWidget *dialog = QApplication::activeModalWidget();
    if (!dialog) {
      for (auto *top : QApplication::topLevelWidgets()) {
        if (top->isVisible() && qobject_cast<QDialog *>(top)) dialog = top;
      }
    }
    if (!dialog) return;
    if (auto *entry = dialog->findChild<QLineEdit *>("destinationInput")) {
      if (!typed) {
        entry->setText("100");
        elapsed.start();
        typed = true;
      } else if (entry->text() == "100" && elapsed.elapsed() > 500) {
        QTest::keyClicks(entry, " Main");
      } else if (elapsed.elapsed() > 1700) {
        auto *matches = dialog->findChild<QListWidget *>("addressSuggestions");
        if (matches->count() == 1 && matches->item(0)->data(Qt::AccessibleTextRole).toString() == "100 Main Street, Example City") {
          auto *item = matches->item(0);
          const auto labels = matches->itemWidget(item)->findChildren<QLabel *>();
          check(labels.size() == 2 && labels[0]->font().pixelSize() >= 60 && labels[1]->font().pixelSize() >= 44
                && matches->visualItemRect(item).height() >= 150, "Address lines and tap area must be readable on the comma");

          matched = true;
          QTest::mouseClick(matches->viewport(), Qt::LeftButton, {}, matches->visualItemRect(matches->item(0)).center());
        }
      }
    } else if (auto *start = dialog->findChild<QPushButton *>("confirm_btn")) {
      check(params.get("NavDestination").empty(), "Typing and selecting must not start navigation");
      confirmed = start->text() == "Start route";
      start->click();
    }
  });
  drive.start(50);
  search->click();
  const auto place = QJsonDocument::fromJson(QByteArray::fromStdString(params.get("NavDestination"))).object();
  check(typed && matched && confirmed && place.value("place_name").toString() == "100 Main Street, Example City",
        "Latest autocomplete result must reach the route engine after confirmation");
}

int main(int argc, char **argv) {
  // The caller must isolate both persistent and shared Params before starting.
  check(qEnvironmentVariable("OPENPILOT_PREFIX").startsWith("onroad-test-"), "Isolated Params required");
  QApplication app(argc, argv);
  checkDestinationSearch();
  TestCameraWidget().checkFrameSelection();
  Params params;
  Params memory("/dev/shm/params");
  params.putBool("SpeedLimitController", true);
  params.putBool("VisionTurnControl", true);
  memory.putBool("FrogPilotTogglesUpdated", false);

  SpeedLimitButton button;
  button.updateState(true, true, false);
  button.resize(166, 186);
  button.show();
  app.processEvents();
  check(button.isChecked() && button.isEnabled(), "Initial ON state");
  QPixmap preview(440, 230);
  preview.fill(QColor(40, 40, 40));
  QPainter painter(&preview);
  const QImage active_sign = button.grab().toImage();
  painter.drawImage(10, 10, active_sign);
  QTest::mouseClick(&button, Qt::LeftButton);
  check(!params.getBool("SpeedLimitController"), "OFF must persist");
  check(memory.getBool("FrogPilotTogglesUpdated"), "OFF must refresh the planner");
  button.updateState(true, true, false);
  check(!button.isChecked(), "Waiting for the planner must not flash back ON");
  button.updateState(true, false, false);
  check(!button.isChecked(), "Effective OFF state");
  const QImage idle_sign = button.grab().toImage();
  check(active_sign != idle_sign, "The active sign must have a visible outline");
  painter.drawImage(230, 10, idle_sign);
  painter.end();
  preview.save("onroad-speed-limit-button.png");

  memory.putBool("FrogPilotTogglesUpdated", false);
  QTest::mouseClick(&button, Qt::LeftButton);
  check(params.getBool("SpeedLimitController"), "ON must persist");
  check(memory.getBool("FrogPilotTogglesUpdated"), "ON must refresh the planner");
  check(params.getBool("VisionTurnControl"), "Curve controller preference must be preserved");
  button.updateState(true, true, false);
  button.updateState(false, true, false);
  QTest::mouseClick(&button, Qt::LeftButton);
  check(params.getBool("SpeedLimitController"), "Unsupported button must ignore taps");
  button.updateState(true, false, false);
  check(!button.isChecked(), "External setting changes must refresh the button");

  // Captured case: dashboard 80.467 km/h, internal cruise and both targets 78.
  check(!curveSpeedLimited(78, true, 78 / 3.6f, true, 78 / 3.6f), "No phantom cluster reduction");
  check(curveSpeedLimited(78, false, 78 / 3.6f, true, 15), "Vision reduction must show");
  check(curveSpeedLimited(78, true, 15, false, 78 / 3.6f), "Map reduction must show");
  check(!curveSpeedLimited(78, false, 15, false, 15), "Disabled controllers must not show");
  check(!curveSpeedLimited(78, true, 0, true, NAN), "Missing targets must not show");
  check(!curveSpeedLimited(255, true, 15, true, 15), "Unset cruise must not show");
  check(!curveSpeedLimited(78, true, 78 / 3.6f - .1f, false, 0), "Rounding noise must not show");
  checkOnroadAppearance();
  std::cout << "Driving HUD, turn and blind-spot arrows, sidebar bookmark, sign glow and taps, camera and autocomplete passed\n";
}
