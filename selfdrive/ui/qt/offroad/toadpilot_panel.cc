#include "selfdrive/ui/qt/offroad/toadpilot_panel.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QNetworkReply>
#include <QPixmap>
#include <QSignalBlocker>
#include <QVBoxLayout>

ToadPilotPanel::ToadPilotPanel(QWidget *parent) : QWidget(parent) {
  setObjectName("ToadPilotPanel");
  setStyleSheet(R"(
    #ToadPilotPanel { background: #090619; color: #f7f4ff; }
    QLabel { color: #f7f4ff; font-size: 30px; }
    QCheckBox { color: #f7f4ff; font-size: 34px; spacing: 22px; padding: 16px 0; }
    QCheckBox::indicator { width: 52px; height: 52px; }
    QCheckBox::indicator:unchecked { background: #171023; border: 2px solid #8d75ab; border-radius: 10px; }
    QCheckBox::indicator:checked { background: #21dcff; border: 4px solid #f7f4ff; border-radius: 10px; }
    QComboBox, QDoubleSpinBox { font-size: 32px; padding: 16px; min-height: 54px; color: white; background: #241638; border: 2px solid #57427b; border-radius: 12px; }
    QComboBox::drop-down { width: 65px; border-left: 1px solid #57427b; }
    QComboBox QAbstractItemView { color: #f7f4ff; background: #241638; selection-background-color: #57427b; }
    QPushButton { color: #f7f4ff; font-size: 34px; padding: 22px; min-height: 54px; border-radius: 14px; background: #33204a; }
    QPushButton:disabled { color: #92879f; }
  )");
  auto *layout = new QVBoxLayout(this);
  layout->setSpacing(20);
  auto *logo = new QLabel;
  logo->setPixmap(QPixmap("../frogpilot/fleetmanager/static/abrasax-logo.png").scaled(600, 200, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  logo->setAlignment(Qt::AlignCenter);
  layout->addWidget(logo);
  auto *heading = new QLabel(tr("ToadPilot · Driving settings"));
  heading->setStyleSheet("font-size: 48px; font-weight: bold;");
  layout->addWidget(heading);
  message = new QLabel(tr("Connecting to device settings…"));
  message->setWordWrap(true);
  layout->addWidget(message);
  health = new QLabel;
  health->setWordWrap(true);
  health->setStyleSheet("font-size: 27px; color: #9edbed;");
  layout->addWidget(health);
  selectedModel = new QLabel;
  selectedModel->setWordWrap(true);
  layout->addWidget(selectedModel);
  models = new QComboBox;
  models->setMinimumContentsLength(18);
  models->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  layout->addWidget(models);
  modelButton = new QPushButton(tr("Use selected model"));
  modelButton->setEnabled(false);
  layout->addWidget(modelButton);
  connect(modelButton, &QPushButton::clicked, this, [this] {
    modelButton->setEnabled(false);
    request("/toadpilot/models", {{"model", models->currentData().toString()}}, true, [this](const QJsonObject &) {
      message->setText(tr("Model saved. It will load on the next drive."));
      refresh();
    });
  });
  profile = new QComboBox;
  profile->addItem(tr("Observe — diagnostics"), "observe");
  profile->addItem(tr("Adaptive — lane and curve assistance"), "adaptive");
  profile->addItem(tr("Full — existing steering authority"), "full");
  layout->addWidget(profile);
  connect(profile, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] { markDirty(); });
  const QList<QPair<QString, QString>> options = {
    {"enabled", tr("Enable ToadPilot")}, {"auto_tune", tr("Use learned steering tune")},
    {"lane_centering_assist", tr("Lane centering assistance")},
    {"smooth_lane_transitions", tr("Smooth lane transitions")},
    {"straight_line_smoothing", tr("Straight line smoothing")},
    {"improved_stop_and_go", tr("Improved stop and go")},
    {"responsive_takeoff", tr("Responsive takeoff")}, {"smooth_stop_finish", tr("Smooth stop finish")},
    {"smooth_braking", tr("Smooth braking")}, {"automatic_traffic_mode", tr("Automatic Traffic Mode")},
    {"map_stop_assist", tr("Mapped stop anticipation")},
    {"curve_assist_in_experimental", tr("Curve assistance in Experimental Mode")},
    {"smart_saturation_warning", tr("Smart steering limit warning")},
    {"telemetry", tr("Controller diagnostics")},
    {"live_steering_delay", tr("Research: learned steering delay")},
    {"modern_longitudinal", tr("Research: longitudinal coordination")},
    {"accel_boost", tr("Research: accelerator override learning")},
    {"hill_compensation", tr("Research: hill compensation")},
    {"camera_timing_checks", tr("Camera timing checks")},
  };
  for (const auto &item : options) {
    auto *toggle = new QCheckBox(item.second);
    toggles[item.first] = toggle;
    layout->addWidget(toggle);
    connect(toggle, &QCheckBox::toggled, this, [this] { markDirty(); });
  }
  auto *trafficLabel = new QLabel(tr("Traffic Mode entry time (seconds) / maximum speed (mph)"));
  trafficLabel->setWordWrap(true);
  layout->addWidget(trafficLabel);
  auto *row = new QHBoxLayout;
  trafficTime = new QDoubleSpinBox;
  trafficTime->setRange(.5, 30.); trafficTime->setSingleStep(.5); trafficTime->setDecimals(1);
  trafficSpeed = new QDoubleSpinBox;
  trafficSpeed->setRange(5., 60.); trafficSpeed->setSingleStep(5.); trafficSpeed->setDecimals(0);
  row->addWidget(trafficTime); row->addWidget(trafficSpeed);
  layout->addLayout(row);
  connect(trafficTime, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] { markDirty(); });
  connect(trafficSpeed, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] { markDirty(); });
  saveButton = new QPushButton(tr("Save driving settings"));
  saveButton->setEnabled(false);
  layout->addWidget(saveButton);
  connect(saveButton, &QPushButton::clicked, this, &ToadPilotPanel::save);
  auto *note = new QLabel(tr("Settings lock while driving. Research features require individual evaluation. Trip traces and downloads are available in Fleet Manager → Diagnostics."));
  note->setWordWrap(true); layout->addWidget(note);
  layout->addStretch();
  timer.setInterval(2000);
  connect(&timer, &QTimer::timeout, this, &ToadPilotPanel::refresh);
}

void ToadPilotPanel::showEvent(QShowEvent *event) { QWidget::showEvent(event); refresh(); timer.start(); }
void ToadPilotPanel::hideEvent(QHideEvent *event) { timer.stop(); QWidget::hideEvent(event); }

void ToadPilotPanel::markDirty() {
  if (loading) return;
  dirty = true;
  saveButton->setEnabled(writable && !pending.contains("/toadpilot"));
  message->setText(tr("Unsaved changes"));
}

void ToadPilotPanel::request(const QString &path, const QJsonObject &data, bool post,
                            std::function<void(const QJsonObject &)> done) {
  if (pending.contains(path)) return;
  pending.insert(path);
  QString base = qEnvironmentVariable("TOADPILOT_API", "http://127.0.0.1:8082");
  QNetworkRequest req{QUrl(base + path)};
  req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
  auto *reply = post ? network.post(req, QJsonDocument(data).toJson(QJsonDocument::Compact)) : network.get(req);
  QTimer::singleShot(3000, reply, [reply] { if (!reply->isFinished()) reply->abort(); });
  connect(reply, &QNetworkReply::finished, this, [this, reply, path, done] {
    pending.remove(path);
    auto object = QJsonDocument::fromJson(reply->readAll()).object();
    if (reply->error() != QNetworkReply::NoError || !object.value("error").toString().isEmpty() || object.isEmpty()) {
      message->setText(object.value("error").toString(tr("Settings service unavailable. Your changes have not been confirmed.")));
      writable = false; saveButton->setEnabled(false); modelButton->setEnabled(false);
    } else {
      done(object);
    }
    reply->deleteLater();
  });
}

void ToadPilotPanel::refresh() {
  request("/toadpilot", {}, false, [this](const QJsonObject &data) { setState(data); });
  request("/toadpilot/models", {}, false, [this](const QJsonObject &data) {
    const QString chosen = models->currentData().toString();
    const QString selected = data.value("selected").toString();
    QSignalBlocker block(models);
    models->clear();
    QString name = selected;
    for (const auto &value : data.value("models").toArray()) {
      auto model = value.toObject();
      if (model.value("id").toString() == selected) name = model.value("name").toString();
      if (model.value("installed").toBool()) models->addItem(model.value("name").toString(), model.value("id"));
    }
    int index = models->findData(chosen.isEmpty() ? selected : chosen);
    models->setCurrentIndex(index < 0 ? models->findData(selected) : index);
    selectedModel->setText(tr("Selected model: %1").arg(name));
    modelButton->setEnabled(data.value("environment").toObject().value("offroad_confirmed").toBool() && models->count());
  });
  request("/toadpilot/health", {}, false, [this](const QJsonObject &data) {
    if (!data.value("fresh").toBool()) { health->setText(tr("Diagnostics waiting for fresh device data")); return; }
    health->setText(tr("CPU temperature %1°C · Memory %2% · %3")
      .arg(data.value("max_cpu_temp_c").toDouble(), 0, 'f', 0).arg(data.value("memory_percent").toInt())
      .arg(data.value("recording").toBool() ? tr("Recording trip diagnostics") :
           !data.value("recording_enabled").toBool() ? tr("Recording disabled") :
           data.value("onroad").toBool() ? tr("Waiting for valid driving data") : tr("Offroad / recorder ready")));
  });
}

void ToadPilotPanel::setState(const QJsonObject &data, bool saved) {
  writable = data.value("environment").toObject().value("settings_writable").toBool();
  if (!dirty || saved || config.isEmpty()) {
    loading = true;
    config = data.value("config").toObject();
    for (auto it = toggles.begin(); it != toggles.end(); ++it) it.value()->setChecked(config.value(it.key()).toBool());
    profile->setCurrentIndex(profile->findData(config.value("profile").toString()));
    trafficTime->setValue(config.value("traffic_mode_entry_time").toDouble());
    trafficSpeed->setValue(config.value("traffic_mode_max_speed_mph").toDouble());
    loading = false; dirty = false;
    message->setText(saved ? tr("Settings saved and verified") : (writable ? tr("Ready · settings apply to your next drive") : tr("Driving · settings locked")));
  }
  profile->setEnabled(writable); trafficTime->setEnabled(writable); trafficSpeed->setEnabled(writable);
  for (auto *toggle : toggles) toggle->setEnabled(writable);
  saveButton->setEnabled(writable && dirty && !pending.contains("/toadpilot"));
}

void ToadPilotPanel::save() {
  if (!writable || config.isEmpty()) return;
  QJsonObject edited = config;
  for (auto it = toggles.begin(); it != toggles.end(); ++it) edited[it.key()] = it.value()->isChecked();
  edited["profile"] = profile->currentData().toString();
  edited["traffic_mode_entry_time"] = trafficTime->value();
  edited["traffic_mode_max_speed_mph"] = trafficSpeed->value();
  saveButton->setEnabled(false);
  message->setText(tr("Saving…"));
  request("/toadpilot", edited, true, [this, edited](const QJsonObject &data) {
    if (data.value("config").toObject() != edited) {
      message->setText(tr("The returned settings differ. Refresh before retrying."));
      writable = false;
      return;
    }
    setState(data, true);
  });
}
