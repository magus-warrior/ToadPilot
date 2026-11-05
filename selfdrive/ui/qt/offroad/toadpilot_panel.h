#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QJsonObject>
#include <QLabel>
#include <QMap>
#include <QNetworkAccessManager>
#include <QPushButton>
#include <QSet>
#include <QTimer>
#include <QWidget>
#include <functional>

class ToadPilotPanel : public QWidget {
  Q_OBJECT
public:
  explicit ToadPilotPanel(QWidget *parent = nullptr);
protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
private:
  void refresh();
  void setState(const QJsonObject &data, bool saved = false);
  void save();
  void request(const QString &path, const QJsonObject &data, bool post,
               std::function<void(const QJsonObject &)> done);
  void markDirty();
  QNetworkAccessManager network;
  QTimer timer;
  QSet<QString> pending;
  QJsonObject config;
  QMap<QString, QCheckBox *> toggles;
  QComboBox *profile, *models;
  QDoubleSpinBox *trafficTime, *trafficSpeed;
  QPushButton *saveButton, *modelButton;
  QLabel *message, *health, *selectedModel;
  bool dirty = false, loading = false, writable = false;
};
