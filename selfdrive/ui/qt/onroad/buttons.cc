#include "selfdrive/ui/qt/onroad/buttons.h"

#include <QPainter>

#include "selfdrive/ui/qt/util.h"

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity, const int angle) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.save();
  p.translate(center);
  p.rotate(angle);
  p.setOpacity(opacity);
  p.drawPixmap(-QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
  p.restore();
}

// MapSettingsButton
MapSettingsButton::MapSettingsButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size + 20);
  setAccessibleName(tr("Navigation"));
  settings_img = loadPixmap("../assets/navigation/icon_directions_outlined.svg", {img_size, img_size});

  // hidden by default, made visible if map is created (has prime or mapbox token)
  setVisible(false);
  setEnabled(false);
}

void MapSettingsButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), settings_img, QColor(0, 0, 0, 166), isDown() ? 0.6 : 1.0);
}

// FrogPilot buttons

// DistanceButton
DistanceButton::DistanceButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size * 1.5, btn_size * 1.5);

  gif_label = new QLabel(this);
  gif_label->setScaledContents(true);

  connect(this, &QPushButton::pressed, [this] {params_memory.putBool("OnroadDistanceButtonPressed", true);});
  connect(this, &QPushButton::released, [this] {params_memory.putBool("OnroadDistanceButtonPressed", false);});
}

DistanceButton::~DistanceButton() {
  qDeleteAll(profile_data_gif);

  profile_data_gif.clear();
  profile_data_png.clear();
}

void DistanceButton::updateState(const UIScene &scene) {
  bool state_changed = (traffic_mode_active != scene.traffic_mode_active) ||
                       (personality != static_cast<int>(scene.personality) + 1 && !traffic_mode_active);

  if (!state_changed) {
    return;
  }

  personality = static_cast<int>(scene.personality) + 1;
  traffic_mode_active = scene.traffic_mode_active;

  int profile_index = traffic_mode_active ? 0 : personality;

  if (QMovie *gif = profile_data_gif.value(profile_index)) {
    gif_label->setMovie(gif);
    gif_label->resize(btn_size, btn_size);
    gif_label->move(UI_BORDER_SIZE, btn_size / 2 - (road_name_ui ? 30 : 0));
    gif_label->show();

    gif->start();

    use_gif = true;
  } else {
    gif_label->hide();

    profile_image = profile_data_png.value(profile_index);

    use_gif = false;
  }

  update();
}

void DistanceButton::updateIcon() {
  qDeleteAll(profile_data_gif);

  profile_data_gif.clear();
  profile_data_png.clear();

  static const QVector<QString> file_names = {
    "../frogpilot/assets/active_theme/distance_icons/traffic",
    "../frogpilot/assets/active_theme/distance_icons/aggressive",
    "../frogpilot/assets/active_theme/distance_icons/standard",
    "../frogpilot/assets/active_theme/distance_icons/relaxed"
  };

  for (int i = 0; i < file_names.size(); ++i) {
    const QString &file_name = file_names[i];
    QString gif_file = file_name + ".gif";
    QString png_file = file_name + ".png";
    QString fallback_file = QString("../frogpilot/assets/stock_theme/distance_icons/%1.png").arg(QFileInfo(file_name).baseName().toLower());

    if (QFile::exists(gif_file)) {
      QMovie *movie = new QMovie(gif_file);
      profile_data_gif.push_back(movie);
      profile_data_png.push_back(QPixmap());
    } else {
      QPixmap pixmap = loadPixmap(QFile::exists(png_file) ? png_file : fallback_file, QSize(btn_size * 1.25, btn_size * 1.25));
      profile_data_gif.push_back(nullptr);
      profile_data_png.push_back(pixmap);
    }
  }

  personality = 0;
}

void DistanceButton::paintEvent(QPaintEvent *event) {
  if (use_gif) {
    return;
  }

  QPainter p(this);
  p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

  drawIcon(p, QPoint((btn_size / 2) + UI_BORDER_SIZE, btn_size - UI_BORDER_SIZE - (road_name_ui ? 30 : 0)), profile_image, Qt::transparent, 1.0);
}
