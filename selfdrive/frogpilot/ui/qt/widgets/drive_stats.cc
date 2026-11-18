#include "selfdrive/ui/qt/request_repeater.h"
#include "selfdrive/ui/qt/util.h"

#include "selfdrive/frogpilot/ui/qt/widgets/drive_stats.h"

static QLabel *newLabel(const QString &text, const QString &type) {
  QLabel *label = new QLabel(text);
  label->setProperty("type", type);
  return label;
}

DriveStats::DriveStats(QWidget *parent) : QFrame(parent) {
  metric = params.getBool("IsMetric");

  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(34, 24, 34, 28);
  layout->setSpacing(16);
  layout->addWidget(newLabel(tr("Trip history"), "heading"));
  auto *table = new QGridLayout;
  table->setHorizontalSpacing(20);
  table->setVerticalSpacing(10);
  table->setColumnStretch(0, 2);
  for (int column = 1; column < 4; ++column) table->setColumnStretch(column, 1);
  table->addWidget(newLabel(tr("Drives"), "unit"), 0, 1, Qt::AlignRight);
  table->addWidget(distanceUnit = newLabel(getDistanceUnit(), "unit"), 0, 2, Qt::AlignRight);
  table->addWidget(newLabel(tr("Hours"), "unit"), 0, 3, Qt::AlignRight);
  auto addPeriod = [&](int row, const QString &period, StatsLabels &labels) {
    auto *rule = new QFrame(this);
    rule->setObjectName("statsRule");
    rule->setFixedHeight(2);
    table->addWidget(rule, row, 0, 1, 4);
    table->addWidget(newLabel(period, "period"), row + 1, 0);
    table->addWidget(labels.routes = newLabel("—", "number"), row + 1, 1, Qt::AlignRight);
    table->addWidget(labels.distance = newLabel("—", "number"), row + 1, 2, Qt::AlignRight);
    table->addWidget(labels.hours = newLabel("—", "number"), row + 1, 3, Qt::AlignRight);
  };
  addPeriod(1, tr("All time"), all);
  addPeriod(3, tr("Past week"), week);
  addPeriod(5, tr("FrogPilot"), frogPilot);
  layout->addLayout(table);
  layout->addStretch();

  stats = QJsonDocument::fromJson(QByteArray::fromStdString(params.get("ApiCache_DriveStats")));
  std::optional<QString> dongleId = getDongleId();
  if (dongleId.has_value()) {
    QString url = CommaApi::BASE_URL + "/v1.1/devices/" + dongleId.value() + "/stats";
    RequestRepeater *repeater = new RequestRepeater(this, url, "ApiCache_DriveStats", 30);
    QObject::connect(repeater, &RequestRepeater::requestDone, this, &DriveStats::parseResponse);
  }

  setStyleSheet(R"(
    DriveStats { background: #1b1731; border: 2px solid #443951; border-radius: 20px; }
    QFrame#statsRule { background: #443951; }
    QLabel[type="heading"] { font-size: 42px; font-weight: 600; color: #f4eaff; }
    QLabel[type="period"] { font-size: 31px; color: #e3d9ef; }
    QLabel[type="number"] { font-size: 41px; font-weight: 600; color: #f7f5ff; }
    QLabel[type="unit"] { font-size: 27px; color: #c4b2da; }
  )");
}

void DriveStats::updateStatsForLabel(const QJsonObject &obj, StatsLabels &labels) {
  labels.routes->setText(QString::number((int)obj["routes"].toDouble()));
  labels.distance->setText(QString::number(int(obj["distance"].toDouble() * (metric ? MILE_TO_KM : 1))));
  labels.hours->setText(QString::number((int)(obj["minutes"].toDouble() / 60)));
}

void DriveStats::updateFrogPilotStats() {
  frogPilot.routes->setText(QString::number(paramsTracking.getInt("FrogPilotDrives")));
  frogPilot.distance->setText(QString::number(int(paramsTracking.getFloat("FrogPilotKilometers") * (metric ? 1 : KM_TO_MILE))));
  frogPilot.hours->setText(QString::number(int(paramsTracking.getFloat("FrogPilotMinutes") / 60)));
}

void DriveStats::updateStats() {
  QJsonObject json = stats.object();

  distanceUnit->setText(getDistanceUnit());
  updateFrogPilotStats();
  const QJsonObject allTime = json["all"].toObject();
  if (!allTime.isEmpty()) updateStatsForLabel(allTime, all);
  if (json["week"].isObject()) updateStatsForLabel(json["week"].toObject(), week);
  if (allTime["minutes"].isDouble()) {
    params.putInt("openpilotMinutes", static_cast<int>(allTime["minutes"].toDouble()));
  }
}

void DriveStats::parseResponse(const QString &response, bool success) {
  if (!success) {
    return;
  }

  QJsonDocument doc = QJsonDocument::fromJson(response.trimmed().toUtf8());
  if (doc.isNull()) {
    qDebug() << "JSON Parse failed on getting past drives statistics";
    return;
  }
  stats = doc;
  updateStats();
}

void DriveStats::showEvent(QShowEvent *event) {
  metric = params.getBool("IsMetric");
  updateStats();
}
