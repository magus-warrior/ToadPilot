#pragma once

#include "selfdrive/ui/ui.h"

class DriveStats : public QFrame {
  Q_OBJECT

public:
  explicit DriveStats(QWidget *parent = 0);

private:
  inline QString getDistanceUnit() const { return metric ? tr("KM") : tr("Miles"); }

  struct StatsLabels {
    QLabel *routes;
    QLabel *distance;
    QLabel *hours;
  };

  QLabel *distanceUnit;
  void showEvent(QShowEvent *event) override;
  void updateStats();
  void updateStatsForLabel(const QJsonObject &obj, StatsLabels &labels);
  void updateFrogPilotStats();

  Params params;
  Params paramsTracking{"/cache/tracking"};

  bool metric;

  QJsonDocument stats;

  StatsLabels all, week, frogPilot;

private slots:
  void parseResponse(const QString &response, bool success);
};
