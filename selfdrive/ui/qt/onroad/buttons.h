#pragma once

#include <QLabel>
#include <QMovie>
#include <QPushButton>

#include "selfdrive/ui/ui.h"

const int btn_size = 192;
const int img_size = (btn_size / 4) * 3;

class MapSettingsButton : public QPushButton {
  Q_OBJECT

public:
  explicit MapSettingsButton(QWidget *parent = 0);

private:
  void paintEvent(QPaintEvent *event) override;

  QPixmap settings_img;
};

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity, const int angle = 0);

// FrogPilot buttons
class DistanceButton : public QPushButton {
  Q_OBJECT

public:
  explicit DistanceButton(QWidget *parent = 0);
  ~DistanceButton();
  void updateIcon();
  void updateState(const UIScene &scene);

  bool road_name_ui;

private:
  void paintEvent(QPaintEvent *event) override;

  Params params_memory{"/dev/shm/params"};

  QLabel *gif_label;
  QPixmap profile_image;

  QVector<QPixmap> profile_data_png;
  QVector<QMovie*> profile_data_gif;

  bool traffic_mode_active;
  bool use_gif;

  int personality;
};
