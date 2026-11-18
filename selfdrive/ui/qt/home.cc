#include "selfdrive/ui/qt/home.h"

#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QStackedWidget>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/offroad/experimental_mode.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/prime.h"

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "selfdrive/frogpilot/ui/qt/widgets/drive_stats.h"
#include "selfdrive/frogpilot/ui/qt/widgets/model_reviewer.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome(this);
  QObject::connect(home, &OffroadHome::openSettings, this, &HomeWindow::openSettings);
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  QObject::connect(onroad, &OnroadWindow::mapPanelRequested, this, [=] { sidebar->hide(); });
  slayout->addWidget(onroad);

  body = new BodyWindow(this);
  slayout->addWidget(body);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &HomeWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &HomeWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::offroadTransition, sidebar, &Sidebar::offroadTransition);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::showMapPanel(bool show) {
  onroad->showMapPanel(show);
}

void HomeWindow::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // switch to the generic robot UI
  if (onroad->isVisible() && !body->isEnabled() && sm["carParams"].getCarParams().getNotCar()) {
    body->setEnabled(true);
    slayout->setCurrentWidget(body);
  }

  if (s.scene.started) {
    showDriverView(s.scene.driver_camera_timer >= 10, true);
    if (s.scene.map_open) {
      showSidebar(false);
    }
  }
}

void HomeWindow::offroadTransition(bool offroad) {
  body->setEnabled(false);
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    showSidebar(params.getBool("Sidebar"));
    slayout->setCurrentWidget(onroad);
  }
}

void HomeWindow::showDriverView(bool show, bool started) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
    sidebar->setVisible(show == false);
  } else {
    if (started) {
      slayout->setCurrentWidget(onroad);
      sidebar->setVisible(params.getBool("Sidebar"));
    } else {
      slayout->setCurrentWidget(home);
      sidebar->setVisible(show == false);
    }
  }
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Handle sidebar collapsing
  if ((onroad->isVisible() || body->isVisible()) && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
    params.putBool("Sidebar", sidebar->isVisible());
  }
}

void HomeWindow::mouseDoubleClickEvent(QMouseEvent* e) {
  HomeWindow::mousePressEvent(e);
  const SubMaster &sm = *(uiState()->sm);
  if (sm["carParams"].getCarParams().getNotCar()) {
    if (onroad->isVisible()) {
      slayout->setCurrentWidget(body);
    } else if (body->isVisible()) {
      slayout->setCurrentWidget(onroad);
    }
    showSidebar(false);
  }
}

// OffroadHome: the offroad home page

namespace {
class DashboardBanner : public QFrame {
public:
  explicit DashboardBanner(QWidget *parent) : QFrame(parent) {
    connect(uiState(), &UIState::offroadTransition, this, [this](bool offroad) {
      if (!offroad) art = QPixmap();
    });
  }

protected:
  void showEvent(QShowEvent *event) override {
    QFrame::showEvent(event);
    if (art.isNull()) art = QPixmap::fromImage(QImage("../frogpilot/assets/toad_neon/rainbow-circuit.jpg"));
  }

  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    QPainterPath outline;
    outline.addRoundedRect(rect(), 24, 24);
    p.setClipPath(outline);
    p.fillRect(rect(), QColor("#0b091f"));
    const int artWidth = art.isNull() ? width() : qRound(height() * art.width() / double(art.height()));
    const int artLeft = width() - artWidth;
    if (!art.isNull()) {
      p.setRenderHint(QPainter::SmoothPixmapTransform);
      p.drawPixmap(QRect(artLeft, 0, artWidth, height()), art);
    }
    QLinearGradient shade(qMax(0, artLeft), 0, qMax(0, artLeft) + artWidth * .36, 0);
    shade.setColorAt(0, QColor(11, 9, 31));
    shade.setColorAt(.55, QColor(11, 9, 31, 155));
    shade.setColorAt(1, QColor(11, 9, 31, 0));
    p.fillRect(rect(), shade);
    QLinearGradient spectrum(0, 0, width(), 0);
    spectrum.setColorAt(0, QColor("#ff58b3"));
    spectrum.setColorAt(.23, QColor("#ffae51"));
    spectrum.setColorAt(.42, QColor("#e5f775"));
    spectrum.setColorAt(.62, QColor("#48f5cf"));
    spectrum.setColorAt(.82, QColor("#56a6ff"));
    spectrum.setColorAt(1, QColor("#b676ff"));
    p.fillRect(QRect(0, height() - 5, width(), 5), spectrum);
  }

private:
  QPixmap art;
};
}

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 28, 40, 28);
  main_layout->setSpacing(22);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(16);

  auto *heading = new QLabel(tr("Dashboard"), this);
  heading->setObjectName("dashboardTitle");
  header_layout->addWidget(heading);
  header_layout->addStretch();
  update_notif = new QPushButton(tr("UPDATE"));
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  date = new ElidedLabel();
  date->setObjectName("dashboardDate");
  header_layout->addWidget(date);
  main_layout->addLayout(header_layout);

  center_layout = new QStackedLayout();
  QWidget *home_widget = new QWidget(this);
  {
    auto *dashboard = new QVBoxLayout(home_widget);
    dashboard->setContentsMargins(0, 0, 0, 0);
    dashboard->setSpacing(22);
    auto *banner = new DashboardBanner(home_widget);
    banner->setFixedHeight(364);
    auto *hero = new QVBoxLayout(banner);
    hero->setContentsMargins(44, 24, 44, 24);
    hero->setSpacing(5);
    auto *modelLabel = new QLabel(tr("Driving model"), banner);
    modelLabel->setObjectName("modelCaption");
    hero->addWidget(modelLabel);
    modelName = new ElidedLabel();
    modelName->setObjectName("modelDisplayName");
    modelName->setMinimumWidth(1);
    modelName->setMaximumWidth(1040);
    hero->addWidget(modelName);
    vehicleName = new QLabel(banner);
    vehicleName->setObjectName("vehicleName");
    hero->addWidget(vehicleName);
    hero->addStretch();
    gasBrake = new QPushButton(tr("Gas / Brake  →"), banner);
    gasBrake->setObjectName("dashboardGasBrake");
    connect(gasBrake, &QPushButton::clicked, this, [this] { emit openSettings(4, "DrivingResponse"); });
    auto *driving = new QPushButton(tr("Driving controls"), banner);
    driving->setObjectName("dashboardDriving");
    connect(driving, &QPushButton::clicked, this, [this] { emit openSettings(4); });
    auto *shortcuts = new QHBoxLayout;
    shortcuts->setSpacing(16);
    shortcuts->addWidget(gasBrake);
    shortcuts->addWidget(driving);
    shortcuts->addStretch();
    hero->addLayout(shortcuts);
    dashboard->addWidget(banner);
    auto *home_layout = new QHBoxLayout;
    home_layout->setSpacing(24);
    dashboard->addLayout(home_layout, 1);

    // left: MapSettings
    QStackedWidget *left_widget = new QStackedWidget(this);
#ifdef ENABLE_MAPS
    left_widget->addWidget(new MapSettings);
#else
    left_widget->addWidget(new QWidget);
#endif
    left_widget->addWidget(new DriveStats);

    ModelReview *modelReview = new ModelReview(this);
    left_widget->addWidget(modelReview);

    left_widget->setCurrentIndex(1);

    connect(modelReview, &ModelReview::driveRated, [=]() {
      left_widget->setCurrentIndex(1);
    });
    connect(uiState(), &UIState::reviewModel, [=]() {
      left_widget->setCurrentIndex(2);
    });

    home_layout->addWidget(left_widget, 6);

    // right: ExperimentalModeButton, SetupWidget
    QWidget* right_widget = new QWidget(this);
    QVBoxLayout* right_column = new QVBoxLayout(right_widget);
    right_column->setContentsMargins(0, 0, 0, 0);
    right_column->setSpacing(14);

    ExperimentalModeButton *experimental_mode = new ExperimentalModeButton(this);
    QObject::connect(experimental_mode, &ExperimentalModeButton::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(experimental_mode);

    SetupWidget *setup_widget = new SetupWidget;
    QObject::connect(setup_widget, &SetupWidget::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(setup_widget, 1);

    home_layout->addWidget(right_widget, 4);
  }
  center_layout->addWidget(home_widget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  QObject::connect(uiState(), &UIState::togglesUpdated, this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
      color: white;
    }
    OffroadHome { background-color: #0c0a1a; }
    QLabel#dashboardTitle { font-size: 52px; font-weight: 650; color: #f9f6ff; }
    QLabel#dashboardDate { font-size: 30px; color: #c4bbda; }
    QLabel#modelCaption { font-size: 29px; color: #8cfff1; font-weight: 600; }
    QLabel#modelDisplayName { font-size: 70px; color: white; font-weight: 700; }
    QLabel#vehicleName { font-size: 25px; color: #d4cde8; }
    QPushButton#dashboardGasBrake, QPushButton#dashboardDriving {
      padding: 14px 26px; min-height: 46px; border-radius: 14px; font-size: 32px; font-weight: 600;
    }
    QPushButton#dashboardGasBrake { background: #8df5ec; color: #101529; border: 2px solid #b2fff6; }
    QPushButton#dashboardDriving { background: #221a39; color: #f6eeff; border: 2px solid #81719d; }
    QPushButton#dashboardGasBrake:pressed, QPushButton#dashboardDriving:pressed { color: #130c24; background: #d9b0ff; }
    OffroadHome > QPushButton { padding: 12px 24px; border-radius: 12px; font-size: 32px; font-weight: 600; }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  const std::string cp = params.get("CarParamsPersistent");
  if (!cp.empty()) {
    AlignedBuffer buffer;
    capnp::FlatArrayMessageReader reader(buffer.align(cp.data(), cp.size()));
    gasBrake->setVisible(hasLongitudinalControl(reader.getRoot<cereal::CarParams>()));
  }
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  QString model = processModelName(uiState()->scene.model_name);

  date->setText(QLocale(uiState()->language.mid(5)).toString(QDateTime::currentDateTime(), "dddd, MMMM d"));
  modelName->setText(model.isEmpty() ? tr("Loading model…") : model);

  const QString vehicle = QString::fromStdString(params.get("CarModelName")).trimmed();
  vehicleName->setText(vehicle);
  vehicleName->setVisible(!vehicle.isEmpty());
  date->setVisible(util::system_time_valid());

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + (alerts > 1 ? tr(" ALERTS") : tr(" ALERT")));
  }
}
