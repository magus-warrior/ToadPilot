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
    QJsonObject config{{"version", 11}, {"profile", "observe"}, {"enabled", true},
      {"experimental_max_torque", 1500}, {"responsive_takeoff", true}, {"smooth_stop_finish", true},
      {"modern_longitudinal", false}, {"accel_boost", false}};
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
        if (buffer->startsWith("POST")) {
          posted = QJsonDocument::fromJson(buffer->mid(separator + 4, length)).object();
          config = posted;
          ++posts;
        }
        QJsonObject state{{"config", config}, {"error", QJsonValue::Null},
          {"environment", QJsonObject{{"offroad_confirmed", offroad}}}};
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
    api.setAccelerationLearning(true, done);
    QTRY_VERIFY(finished);
    QVERIFY2(error.isEmpty(), qPrintable(error));
    QCOMPARE(posts, 1);
    QVERIFY(posted.value("accel_boost").toBool());
    QVERIFY(posted.value("modern_longitudinal").toBool());
    QVERIFY(posted.value("responsive_takeoff").toBool());
    QVERIFY(posted.value("smooth_stop_finish").toBool());
    QCOMPARE(posted.value("experimental_max_torque").toInt(), 1500);
    QCOMPARE(posted.value("profile").toString(), QString("observe"));
    finished = false;
    offroad = false;
    api.setAccelerationLearning(false, done);
    QTRY_VERIFY(finished);
    QVERIFY(!error.isEmpty());
    QCOMPARE(posts, 1);
    offroad = true;
    finished = false;
    api.setAccelerationLearning(false, done);
    QTRY_VERIFY(finished);
    QVERIFY(error.isEmpty());
    QVERIFY(!posted.value("accel_boost").toBool());
    QVERIFY(!posted.value("modern_longitudinal").toBool());
    QVERIFY(posted.value("responsive_takeoff").toBool());
    QCOMPARE(posts, 2);
  }
};

QTEST_GUILESS_MAIN(ApiTest)
#include "test_panel.moc"
