#include <sys/resource.h>

#include <QApplication>
#include <QTranslator>
#include <QScrollArea>
#include <QTimer>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/window.h"
#include "selfdrive/ui/qt/offroad/toadpilot_panel.h"

int main(int argc, char *argv[]) {
  if (argc > 1 && QString(argv[1]) == "--abrasax-offroad-preview") {
    qputenv("ABRASAX_UI_PREVIEW", "1");
    QApplication app(argc, argv);
    MainWindow window;
    window.resize(2160, 1080);
    window.show();
    QTimer::singleShot(3500, &app, [&] {
      window.grab().save(qEnvironmentVariable("ABRASAX_SCREENSHOT"));
      app.quit();
    });
    return app.exec();
  }
  if (argc > 1 && QString(argv[1]) == "--abrasax-preview") {
    QApplication app(argc, argv);
    QScrollArea preview;
    preview.setWidgetResizable(true);
    preview.setWidget(new ToadPilotPanel);
    preview.resize(1920, 1080);
    preview.show();
    const QString screenshot = qEnvironmentVariable("ABRASAX_SCREENSHOT");
    if (!screenshot.isEmpty()) QTimer::singleShot(2500, &app, [&] {
      preview.grab().save(screenshot);
      app.quit();
    });
    return app.exec();
  }
  setpriority(PRIO_PROCESS, 0, -20);

  qInstallMessageHandler(swagLogMessageHandler);
  initApp(argc, argv);

  QTranslator translator;
  QString translation_file = QString::fromStdString(Params().get("LanguageSetting"));
  if (!translator.load(QString(":/%1").arg(translation_file)) && translation_file.length()) {
    qCritical() << "Failed to load translation file:" << translation_file;
  }

  QApplication a(argc, argv);
  a.installTranslator(&translator);

  MainWindow w;
  setMainWindow(&w);
  a.installEventFilter(&w);
  return a.exec();
}
