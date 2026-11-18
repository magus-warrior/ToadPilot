#include "selfdrive/ui/qt/widgets/wifi.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>

WiFiPromptWidget::WiFiPromptWidget(QWidget *parent) : QFrame(parent) {
  stack = new QStackedLayout(this);

  // Setup Wi-Fi
  QFrame *setup = new QFrame;
  QVBoxLayout *setup_layout = new QVBoxLayout(setup);
  setup_layout->setContentsMargins(28, 24, 28, 24);
  setup_layout->setSpacing(20);
  {
    QHBoxLayout *title_layout = new QHBoxLayout;
    title_layout->setSpacing(18);
    {
      QLabel *icon = new QLabel;
      QPixmap pixmap("../assets/offroad/icon_wifi_strength_full.svg");
      icon->setPixmap(pixmap.scaledToWidth(48, Qt::SmoothTransformation));
      title_layout->addWidget(icon);

      QLabel *title = new QLabel(tr("Setup Wi-Fi"));
      title->setStyleSheet("font-size: 40px; font-weight: 600;");
      title_layout->addWidget(title);
      title_layout->addStretch();
    }
    setup_layout->addLayout(title_layout);

    QLabel *desc = new QLabel(tr("Connect to Wi-Fi to upload driving data and help improve openpilot"));
    desc->setStyleSheet("font-size: 30px; color: #d4c8e5; font-weight: 400;");
    desc->setWordWrap(true);
    setup_layout->addWidget(desc);

    QPushButton *settings_btn = new QPushButton(tr("Open Settings"));
    connect(settings_btn, &QPushButton::clicked, [=]() { emit openSettings(1); });
    settings_btn->setStyleSheet(R"(
      QPushButton {
        font-size: 30px;
        font-weight: 500;
        border-radius: 16px;
        background-color: #65428b;
        padding: 15px;
      }
      QPushButton:pressed {
        background-color: #815aa7;
      }
    )");
    setup_layout->addWidget(settings_btn);
  }
  stack->addWidget(setup);

  // Uploading data
  QWidget *uploading = new QWidget;
  QVBoxLayout *uploading_layout = new QVBoxLayout(uploading);
  uploading_layout->setContentsMargins(28, 24, 28, 24);
  uploading_layout->setSpacing(18);
  {
    QHBoxLayout *title_layout = new QHBoxLayout;
    {
      QLabel *title = new QLabel(tr("Ready to upload"));
      title->setStyleSheet("font-size: 40px; font-weight: 600;");
      title->setWordWrap(true);
      title->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
      title_layout->addWidget(title);
      title_layout->addStretch();

      QLabel *icon = new QLabel;
      QPixmap pixmap("../assets/offroad/icon_wifi_uploading.svg");
      icon->setPixmap(pixmap.scaledToWidth(64, Qt::SmoothTransformation));
      title_layout->addWidget(icon);
    }
    uploading_layout->addLayout(title_layout);

    QLabel *desc = new QLabel(tr("Training data will be pulled periodically while your device is on Wi-Fi"));
    desc->setStyleSheet("font-size: 30px; color: #d4c8e5; font-weight: 400;");
    desc->setWordWrap(true);
    uploading_layout->addWidget(desc);
  }
  stack->addWidget(uploading);

  // not uploading data
  QWidget *notUploading = new QWidget;
  QVBoxLayout *not_uploading_layout = new QVBoxLayout(notUploading);
  not_uploading_layout->setContentsMargins(28, 24, 28, 24);
  not_uploading_layout->setSpacing(18);
  {
    QHBoxLayout *title_layout = new QHBoxLayout;
    {
      QLabel *title = new QLabel(tr("Uploading disabled"));
      title->setStyleSheet("font-size: 40px; font-weight: 600;");
      title->setWordWrap(true);
      title->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
      title_layout->addWidget(title);
      title_layout->addStretch();

      QLabel *icon = new QLabel;
      QPixmap pixmap("../frogpilot/assets/other_images/icon_wifi_uploading_disabled.svg");
      icon->setPixmap(pixmap.scaledToWidth(64, Qt::SmoothTransformation));
      title_layout->addWidget(icon);
    }
    not_uploading_layout->addLayout(title_layout);

    QLabel *desc = new QLabel(tr("Toggle off the 'Turn Off Data Uploads' toggle to re-enable uploads."));
    desc->setStyleSheet("font-size: 30px; color: #d4c8e5; font-weight: 400;");
    desc->setWordWrap(true);
    not_uploading_layout->addWidget(desc);
  }
  stack->addWidget(notUploading);

  setStyleSheet(R"(
    WiFiPromptWidget {
      background-color: #201930;
      border-radius: 16px;
    }
  )");

  QObject::connect(uiState(), &UIState::uiUpdate, this, &WiFiPromptWidget::updateState);
}

void WiFiPromptWidget::updateState(const UIState &s) {
  if (!isVisible()) return;

  auto &sm = *(s.sm);

  auto network_type = sm["deviceState"].getDeviceState().getNetworkType();
  auto uploading = network_type == cereal::DeviceState::NetworkType::WIFI ||
      network_type == cereal::DeviceState::NetworkType::ETHERNET;
  stack->setCurrentIndex(s.scene.no_uploads ? 2 : uploading ? 1 : 0);
}
