#include "selfdrive/ui/qt/offroad/experimental_mode.h"

#include <QDebug>
#include <QHBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QStyle>

#include "selfdrive/ui/ui.h"

ExperimentalModeButton::ExperimentalModeButton(QWidget *parent) : QPushButton(parent) {
  chill_pixmap = QPixmap("../assets/img_couch.svg").scaledToWidth(img_width, Qt::SmoothTransformation);
  experimental_pixmap = QPixmap("../assets/img_experimental_grey.svg").scaledToWidth(img_width, Qt::SmoothTransformation);

  // go to toggles and expand experimental mode description
  connect(this, &QPushButton::clicked, [=]() { emit openSettings(2, "ExperimentalMode"); });

  setFixedHeight(96);
  QHBoxLayout *main_layout = new QHBoxLayout;
  main_layout->setContentsMargins(horizontal_padding, 0, horizontal_padding, 0);

  mode_label = new QLabel;
  mode_icon = new QLabel;
  mode_icon->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));

  main_layout->addWidget(mode_label, 1, Qt::AlignLeft);
  main_layout->addWidget(mode_icon, 0, Qt::AlignRight);

  setLayout(main_layout);

  setStyleSheet(R"(
    QPushButton {
      border: none;
    }

    QLabel {
      font-size: 37px;
      font-weight: 600;
      text-align: left;
      color: #000000;
    }
  )");
}

void ExperimentalModeButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setPen(Qt::NoPen);
  p.setRenderHint(QPainter::Antialiasing);

  QPainterPath path;
  path.addRoundedRect(rect(), 16, 16);

  // gradient
  bool pressed = isDown();
  QLinearGradient gradient(rect().left(), 0, rect().right(), 0);
  if (experimental_mode) {
    gradient.setColorAt(0, QColor(255, 196, 109, pressed ? 0xcc : 0xff));
    gradient.setColorAt(1, QColor(255, 121, 156, pressed ? 0xcc : 0xff));
  } else {
    gradient.setColorAt(0, QColor(134, 245, 214, pressed ? 0xcc : 0xff));
    gradient.setColorAt(1, QColor(123, 191, 255, pressed ? 0xcc : 0xff));
  }
  p.fillPath(path, gradient);

  // vertical line
  p.setPen(QPen(QColor(0, 0, 0, 0x4d), 3, Qt::SolidLine));
  int line_x = rect().right() - img_width - (2 * horizontal_padding);
  p.drawLine(line_x, rect().bottom(), line_x, rect().top());
}

void ExperimentalModeButton::showEvent(QShowEvent *event) {
  experimental_mode = params.getBool("ExperimentalMode");
  mode_icon->setPixmap(experimental_mode ? experimental_pixmap : chill_pixmap);
  mode_label->setText(experimental_mode ? tr("Experimental mode") : tr("Chill mode"));
}
