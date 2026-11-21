#pragma once

#include <QElapsedTimer>
#include <QPainter>
#include <QPushButton>

#include "common/params.h"

class SpeedLimitButton : public QPushButton {
public:
  explicit SpeedLimitButton(QWidget *parent = nullptr);
  void updateState(bool available, bool active, bool vienna);

private:
  void paintEvent(QPaintEvent *event) override;
  bool circular = false;
  Params params;
  Params params_memory{"/dev/shm/params"};
  QElapsedTimer pending_change;
};

inline SpeedLimitButton::SpeedLimitButton(QWidget *parent) : QPushButton(parent) {
  setObjectName("speedLimitControllerButton");
  setFocusPolicy(Qt::NoFocus);
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

inline void SpeedLimitButton::updateState(bool available, bool active, bool vienna) {
  circular = vienna;
  setEnabled(available);
  if (active == isChecked()) pending_change.invalidate();
  if (!pending_change.isValid() || pending_change.elapsed() > 2000) setChecked(active);
  setAccessibleDescription(isChecked() ? tr("On. Tap to turn off.") : tr("Off. Tap to turn on."));
  update();
}

inline void SpeedLimitButton::paintEvent(QPaintEvent *event) {
  if (!isChecked() || !isEnabled()) return;
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.setBrush(Qt::NoBrush);
  auto outline = [&](int inset, int width, QColor color) {
    p.setPen(QPen(color, width));
    const QRect box = rect().adjusted(inset, inset, -inset, -inset);
    if (circular) p.drawEllipse(box);
    else p.drawRoundedRect(box, 24 - inset, 24 - inset);
  };
  outline(6, 12, QColor(20, 224, 126, 100));
  outline(3, 5, QColor(62, 245, 155));
}
