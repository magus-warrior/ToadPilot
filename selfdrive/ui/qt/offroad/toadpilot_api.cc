#include "selfdrive/ui/qt/offroad/toadpilot_api.h"

#include <QJsonDocument>
#include <QNetworkReply>
#include <QTimer>

void ToadPilotApi::request(const QString &path, const QJsonObject &data, bool post, Callback done) {
  QNetworkRequest req{QUrl(qEnvironmentVariable("TOADPILOT_API", "http://127.0.0.1:8082") + path)};
  req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
  auto *reply = post ? network.post(req, QJsonDocument(data).toJson(QJsonDocument::Compact)) : network.get(req);
  QTimer::singleShot(3000, reply, [reply] { if (!reply->isFinished()) reply->abort(); });
  connect(reply, &QNetworkReply::finished, this, [reply, done] {
    auto object = QJsonDocument::fromJson(reply->readAll()).object();
    QString error = object.value("error").toString();
    if (error.isEmpty() && (reply->error() != QNetworkReply::NoError || object.isEmpty()))
      error = tr("Settings service unavailable. No change has been confirmed.");
    reply->deleteLater();
    done(object, error);
  });
}

void ToadPilotApi::setDrivingPreference(const QString &key, const QJsonValue &choice, Callback done) {
  request("/toadpilot/driving-preferences", {{key, choice}}, true, [key, choice, done](const QJsonObject &state, QString error) {
    const auto config = state.value("config").toObject();
    bool confirmed = config.contains(key) && config.value(key) == choice;
    if (key == "pickup" && config.value("accel_boost").isBool()) {
      confirmed = choice.toString() == "off" ? !config.value("accel_boost").toBool()
          : config.value("accel_boost").toBool() && config.value("modern_longitudinal").toBool() && config.value("pickup_strength") == choice;
    }
    if (error.isEmpty() && !confirmed) error = tr("Could not confirm the saved choice.");
    done(state, error);
  });
}
