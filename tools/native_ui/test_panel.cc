#include <QJsonDocument>
#include <QTcpServer>
#include <QTcpSocket>
#include <QtTest>
#include <memory>

#include "selfdrive/ui/qt/offroad/toadpilot_api.h"

class ApiTest : public QObject {
  Q_OBJECT
private slots:
  void learningPreservesConfigAndRequiresOffroad() {
    QTcpServer server;
    QVERIFY(server.listen(QHostAddress::LocalHost));
    qputenv("TOADPILOT_API", QString("http://127.0.0.1:%1").arg(server.serverPort()).toUtf8());
    QJsonObject config{{"version", 13}, {"profile", "observe"}, {"takeoff_style", "planner"}, {"enabled", true},
      {"experimental_max_torque", 1500}, {"responsive_takeoff", true}, {"smooth_stop_finish", true},
      {"modern_longitudinal", false}, {"accel_boost", false}, {"pickup_strength", "brisk"}, {"matched_lead_prediction", true}};
    QJsonObject posted;
    bool offroad = true;
    int posts = 0;
    connect(&server, &QTcpServer::newConnection, this, [&] {
      auto *socket = server.nextPendingConnection();
      auto buffer = std::make_shared<QByteArray>();
      connect(socket, &QTcpSocket::readyRead, socket, [&, socket, buffer] {
        buffer->append(socket->readAll());
        const int separator = buffer->indexOf("\r\n\r\n");
        if (separator < 0) return;
        int length = 0;
        for (const auto &line : buffer->left(separator).split('\n'))
          if (line.toLower().startsWith("content-length:")) length = line.mid(15).trimmed().toInt();
        if (buffer->size() < separator + 4 + length) return;
        bool rejected = false;
        if (buffer->startsWith("POST /toadpilot/driving-preferences ")) {
          posted = QJsonDocument::fromJson(buffer->mid(separator + 4, length)).object();
          rejected = !offroad;
          if (!rejected) {
            if (posted.contains("pickup")) {
              const auto choice = posted.value("pickup").toString();
              config["accel_boost"] = choice != "off";
              if (choice != "off") {
                config["modern_longitudinal"] = true;
                config["pickup_strength"] = choice;
              }
            } else if (posted.contains("takeoff_style")) {
              config["takeoff_style"] = posted.value("takeoff_style");
            } else {
              config["matched_lead_prediction"] = posted.value("matched_lead_prediction");
            }
            ++posts;
          }
        }
        QJsonObject state{{"config", config}, {"environment", QJsonObject{{"offroad_confirmed", offroad}}}};
        if (rejected) state["error"] = "Park to edit";
        const auto body = QJsonDocument(state).toJson();
        socket->write("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: " +
                      QByteArray::number(body.size()) + "\r\n\r\n" + body);
        socket->disconnectFromHost();
      });
      connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
    });
    ToadPilotApi api;
    bool finished = false;
    QString error;
    auto done = [&](const QJsonObject &, const QString &message) { error = message; finished = true; };
    api.setDrivingPreference("pickup", "gentle", done);
    QTRY_VERIFY(finished);
    QVERIFY2(error.isEmpty(), qPrintable(error));
    QCOMPARE(posts, 1);
    QCOMPARE(posted.size(), 1);
    QCOMPARE(config.value("pickup_strength").toString(), QString("gentle"));
    finished = false;
    offroad = false;
    api.setDrivingPreference("matched_lead_prediction", false, done);
    QTRY_VERIFY(finished);
    QVERIFY(!error.isEmpty());
    QCOMPARE(posts, 1);
    offroad = true;
    finished = false;
    api.setDrivingPreference("matched_lead_prediction", false, done);
    QTRY_VERIFY(finished);
    QVERIFY(error.isEmpty());
    QVERIFY(!config.value("matched_lead_prediction").toBool());
    QVERIFY(config.value("accel_boost").toBool());
    finished = false;
    api.setDrivingPreference("pickup", "off", done);
    QTRY_VERIFY(finished);
    QVERIFY(error.isEmpty());
    QVERIFY(!config.value("accel_boost").toBool());
    QVERIFY(config.value("modern_longitudinal").toBool());
    QCOMPARE(config.value("experimental_max_torque").toInt(), 1500);
    QCOMPARE(posts, 3);
    const auto beforeTakeoff = config;
    for (const QString &style : {QString("existing"), QString("planner")}) {
      finished = false;
      api.setDrivingPreference("takeoff_style", style, done);
      QTRY_VERIFY(finished);
      QVERIFY2(error.isEmpty(), qPrintable(error));
      auto expected = beforeTakeoff;
      expected["takeoff_style"] = style;
      QCOMPARE(config, expected);
      QCOMPARE(posted, QJsonObject({{"takeoff_style", style}}));
    }
  }
};

QTEST_GUILESS_MAIN(ApiTest)
#include "test_panel.moc"
