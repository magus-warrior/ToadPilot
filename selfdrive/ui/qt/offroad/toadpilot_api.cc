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

void ToadPilotApi::setAccelerationLearning(bool enabled, Callback done) {
  request("/toadpilot", {}, false, [this, enabled, done](const QJsonObject &state, const QString &error) {
    if (!error.isEmpty()) { done(state, error); return; }
    if (!state.value("environment").toObject().value("offroad_confirmed").toBool()) {
      done(state, tr("Park before changing acceleration learning."));
      return;
    }
    auto config = state.value("config").toObject();
    if (config.isEmpty()) { done(state, tr("No saved configuration was received.")); return; }
    config["accel_boost"] = enabled;
    config["modern_longitudinal"] = enabled;
    request("/toadpilot", config, true, [config, done](const QJsonObject &saved, const QString &saveError) {
      done(saved, !saveError.isEmpty() ? saveError : saved.value("config").toObject() == config ? QString() : tr("The saved settings were not confirmed."));
    });
  });
}
