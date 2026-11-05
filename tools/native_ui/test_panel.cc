#include <QApplication>
#include <QCheckBox>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QPushButton>
#include <QTcpServer>
#include <QTcpSocket>
#include <QtTest>
#include <memory>

#include "selfdrive/ui/qt/offroad/toadpilot_panel.h"

class PanelTest : public QObject {
  Q_OBJECT
private slots:
  void savePreservesConfigAndHealthNullIsNotAnError() {
    QTcpServer server;
    QVERIFY(server.listen(QHostAddress::LocalHost));
    qputenv("TOADPILOT_API", QString("http://127.0.0.1:%1").arg(server.serverPort()).toUtf8());
    QJsonObject config{{"version", 11}, {"profile", "observe"}, {"enabled", true},
      {"experimental_max_torque", 1500}, {"traffic_mode_entry_time", 1.},
      {"traffic_mode_max_speed_mph", 30}, {"telemetry", true}, {"hill_compensation", true}};
    QJsonObject posted;
    bool locked = false;
    auto state = [&] { return QJsonObject{{"config", config},
      {"environment", QJsonObject{{"settings_writable", !locked}}}}; };
    connect(&server, &QTcpServer::newConnection, this, [&] {
      auto *socket = server.nextPendingConnection();
      auto buffer = std::make_shared<QByteArray>();
      connect(socket, &QTcpSocket::readyRead, socket, [&, socket, buffer] {
        buffer->append(socket->readAll());
        int separator = buffer->indexOf("\r\n\r\n");
        if (separator < 0) return;
        int length = 0;
        for (const auto &line : buffer->left(separator).split('\n'))
          if (line.toLower().startsWith("content-length:")) length = line.mid(15).trimmed().toInt();
        if (buffer->size() < separator + 4 + length) return;
        auto first = buffer->split('\n').first().split(' ');
        QJsonObject result;
        if (first[1] == "/toadpilot") {
          if (first[0] == "POST") {
            posted = QJsonDocument::fromJson(buffer->mid(separator + 4, length)).object();
            config = posted;
          }
          result = state();
        } else if (first[1] == "/toadpilot/models") {
          result = {{"selected", "test-model"},
            {"models", QJsonArray{QJsonObject{{"id", "test-model"}, {"name", "Test model"}, {"installed", true}}}},
            {"environment", QJsonObject{{"offroad_confirmed", true}}}};
        } else {
          result = {{"fresh", false}, {"error", QJsonValue::Null}};
        }
        QByteArray body = QJsonDocument(result).toJson();
        socket->write("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: " +
                      QByteArray::number(body.size()) + "\r\n\r\n" + body);
        socket->disconnectFromHost();
      });
      connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
    });
    ToadPilotPanel panel;
    panel.resize(1400, 4000);
    panel.show();
    auto *enable = panel.findChildren<QCheckBox *>().first();
    QPushButton *save = nullptr;
    for (auto *button : panel.findChildren<QPushButton *>())
      if (button->text() == "Save driving settings") save = button;
    QVERIFY(save);
    QTRY_VERIFY(enable->isChecked());
    enable->setChecked(false);
    QTRY_VERIFY(save->isEnabled());
    save->click();
    QTRY_VERIFY(!posted.isEmpty());
    QCOMPARE(posted.value("experimental_max_torque").toInt(), 1500);
    QCOMPARE(posted.value("version").toInt(), 11);
    QCOMPARE(posted.value("profile").toString(), QString("observe"));
    QVERIFY(posted.value("hill_compensation").toBool());
    QVERIFY(!posted.value("enabled").toBool());
    QTRY_VERIFY(!save->isEnabled());
    locked = true;
    QTRY_VERIFY(!enable->isEnabled());
    QVERIFY(!save->isEnabled());
  }
};

QTEST_MAIN(PanelTest)
#include "test_panel.moc"
