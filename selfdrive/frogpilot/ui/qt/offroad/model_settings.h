#pragma once

#include <set>

#include "selfdrive/frogpilot/ui/qt/offroad/frogpilot_settings.h"
#include "selfdrive/ui/qt/offroad/toadpilot_api.h"

class FrogPilotModelPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotModelPanel(FrogPilotSettingsWindow *parent);

signals:
  void openParentToggle();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  void updateModelLabels(FrogPilotListWidget *labelsList);
  void updateState(const UIState &s);
  void updateToggles();
  void loadInstalledModels(bool choose);
  void queueModelReboot(const QString &id, const QString &name);
  void refreshModelReboot();
  ToadPilotApi modelApi{this};
  bool modelRequestPending = false;
  bool rebootPending = false;
  bool rebootStatusPending = false;
  QTimer rebootTimer;

  bool allModelsDownloaded = false;
  bool allModelsDownloading = false;
  bool cancellingDownload = false;
  bool finalizingDownload = false;
  bool modelDownloading = false;
  bool noModelsDownloaded = false;
  bool started = false;

  int tuningLevel;

  std::map<QString, AbstractControl*> toggles;

  ButtonControl *selectModelBtn;

  FrogPilotButtonsControl *deleteModelBtn;
  FrogPilotButtonsControl *downloadModelBtn;

  FrogPilotSettingsWindow *parent;

  Params params;
  Params params_cache{"/cache/params"};
  Params params_default{"/dev/shm/params_default"};
  Params params_memory{"/dev/shm/params"};

  QDir modelDir{"/data/models/"};

  QJsonObject frogpilotToggleLevels;

  QMap<QString, QString> modelFileToNameMap;
  QMap<QString, QString> modelFileToNameMapProcessed;

  QString currentModel;

  QStringList availableModels;
  QStringList availableModelNames;
  QStringList deletableModels;
  QStringList downloadableModels;
};
