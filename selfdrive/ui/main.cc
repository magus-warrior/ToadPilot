#include <sys/resource.h>

#include <QApplication>
#include <QTranslator>
#include <QTimer>
#include <QPushButton>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/window.h"

int main(int argc, char *argv[]) {
  if (argc > 1 && QString(argv[1]) == "--abrasax-offroad-preview") {
    qputenv("ABRASAX_UI_PREVIEW", "1");
    QApplication app(argc, argv);
    MainWindow window;
    window.setFixedSize(DEVICE_SCREEN_SIZE);
    window.show();
    const QString previewPanel = qEnvironmentVariable("ABRASAX_PREVIEW_PANEL");
    if (!previewPanel.isEmpty() && previewPanel != "offroad") {
      QTimer::singleShot(500, &window, [&window, previewPanel] {
        if (previewPanel == "gas") {
          auto *shortcut = window.findChild<QPushButton *>("dashboardGasBrake");
          if (shortcut && shortcut->isVisible()) shortcut->click();
          return;
        }
        window.findChild<HomeWindow *>()->openSettings(4, QString());
        if (previewPanel == "controls") return;
        const QString text = "DRIVING MODEL";
        for (auto *button : window.findChildren<QPushButton *>()) {
          if (button->isVisible() && button->text() == text) { button->click(); break; }
        }
      });
      if (previewPanel == "models") {
        auto *selectTimer = new QTimer(&window);
        selectTimer->setInterval(200);
        QObject::connect(selectTimer, &QTimer::timeout, &window, [&window, selectTimer] {
          if (auto *selector = window.findChild<QWidget *>("installedModelSelector")) {
            for (auto *button : selector->findChildren<QPushButton *>()) {
              if (button->isVisible() && button->isEnabled() && button->text() == "SELECT") {
                fprintf(stderr, "Preview clicking model SELECT\n");
                selectTimer->stop();
                button->click();
                break;
              }
            }
          }
        });
        selectTimer->start();
      }
    }
    QTimer::singleShot(previewPanel == "models" ? 7000 : 3500, &app, [&] {
      QWidget *dialog = QApplication::activeModalWidget();
      for (auto *top : QApplication::topLevelWidgets()) {
        fprintf(stderr, "Preview window %s visible=%d modal=%d %dx%d\n", top->metaObject()->className(), top->isVisible(), top->isWindow(), top->width(), top->height());
        if (top->isVisible() && qobject_cast<QDialog *>(top)) dialog = top;
      }
      const QString expected = previewPanel == "gas" ? "drivingResponseCard" :
                               previewPanel == "controls" ? "drivingMenu" : "dashboardGasBrake";
      if (previewPanel != "models") {
        QWidget *target = window.findChild<QWidget *>(expected);
        if (!target || !target->isVisible() || !target->visibleRegion().intersects(target->rect())) {
          qCritical() << "Preview did not reach" << expected;
          app.exit(1);
          return;
        }
      }
      const bool saved = (dialog ? dialog : &window)->grab().save(qEnvironmentVariable("ABRASAX_SCREENSHOT"));
      app.exit(saved ? 0 : 1);
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
