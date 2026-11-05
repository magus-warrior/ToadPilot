#pragma once

#include <QJsonObject>
#include <QNetworkAccessManager>
#include <functional>

// Asynchronous access to the same validated API used by Fleet Manager.
class ToadPilotApi : public QObject {
public:
  using Callback = std::function<void(const QJsonObject &, const QString &)>;
  explicit ToadPilotApi(QObject *parent = nullptr) : QObject(parent), network(this) {}
  void request(const QString &path, const QJsonObject &data, bool post, Callback done);
  void setAccelerationLearning(bool enabled, Callback done);
private:
  QNetworkAccessManager network;
};
