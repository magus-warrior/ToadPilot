#pragma once

#include <QElapsedTimer>
#include <QPainter>
#include <QPushButton>

#include "common/params.h"

class SpeedLimitButton : public QPushButton {
public:
  explicit SpeedLimitButton(QWidget *parent = nullptr);
  void updateState(bool available, bool active);

private:
  void paintEvent(QPaintEvent *event) override;
  Params params;
  Params params_memory{"/dev/shm/params"};
  QElapsedTimer pending_change;
};

inline SpeedLimitButton::SpeedLimitButton(QWidget *parent) : QPushButton(parent) {
  setObjectName("speedLimitControllerButton");
  setFixedSize(192, 202);
  setCheckable(true);
  setChecked(params.getBool("SpeedLimitController"));
  setAccessibleName(tr("Speed limit controller"));
  connect(this, &QPushButton::clicked, this, [this](bool active) {
    if (params.putBool("SpeedLimitController", active) == 0) {
      params_memory.putBool("FrogPilotTogglesUpdated", true);
      pending_change.start();
    } else {
      setChecked(params.getBool("SpeedLimitController"));
    }
    update();
  });
}

inline void SpeedLimitButton::updateState(bool available, bool active) {
  setEnabled(available);
  if (active == isChecked()) pending_change.invalidate();
  if (!pending_change.isValid() || pending_change.elapsed() > 2000) setChecked(active);
  setAccessibleDescription(isChecked() ? tr("On. Tap to turn off.") : tr("Off. Tap to turn on."));
  update();
}

inline void SpeedLimitButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(isDown() || !isEnabled() ? 0.6 : 1.0);
  const QRect box(6, 14, width() - 12, height() - 28);
  p.setPen(QPen(isChecked() ? QColor(80, 220, 130) : QColor(180, 180, 180), 4));
  p.setBrush(isChecked() ? QColor(23, 134, 68, 220) : QColor(0, 0, 0, 166));
  p.drawRoundedRect(box, 24, 24);
  p.setPen(Qt::white);
  QFont font("Inter");
  font.setPixelSize(25);
  font.setWeight(QFont::DemiBold);
  p.setFont(font);
  p.drawText(box.adjusted(4, 16, -4, -70), Qt::AlignCenter, tr("SPEED\nLIMIT"));
  font.setPixelSize(42);
  font.setBold(true);
  p.setFont(font);
  p.drawText(box.adjusted(4, 95, -4, -6), Qt::AlignCenter, isChecked() ? tr("ON") : tr("OFF"));
}
