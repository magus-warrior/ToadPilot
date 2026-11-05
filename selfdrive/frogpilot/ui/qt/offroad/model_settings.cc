#include "selfdrive/frogpilot/ui/qt/offroad/model_settings.h"
#include <QJsonArray>
#include "common/timing.h"

FrogPilotModelPanel::FrogPilotModelPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent), parent(parent) {
  QStackedLayout *modelLayout = new QStackedLayout();
  addItem(modelLayout);

  FrogPilotListWidget *modelList = new FrogPilotListWidget(this);

  ScrollView *modelPanel = new ScrollView(modelList, this);

  modelLayout->addWidget(modelPanel);

  FrogPilotListWidget *modelLabelsList = new FrogPilotListWidget(this);

  ScrollView *modelLabelsPanel = new ScrollView(modelLabelsList, this);

  modelLayout->addWidget(modelLabelsPanel);

  const std::vector<std::tuple<QString, QString, QString, QString>> modelToggles {
    {"AutomaticallyDownloadModels", tr("Automatically Download New Models"), tr("Automatically download new driving models as they become available."), ""},
    {"DeleteModel", tr("Delete Driving Models"), tr("Delete driving models from the device."), ""},
    {"DownloadModel", tr("Download Driving Models"), tr("Download driving models to the device."), ""},
    {"ModelRandomizer", tr("Model Randomizer"), tr("Driving models are chosen at random each drive and feedback prompts are used to find the model that best suits your needs."), ""},
    {"ManageBlacklistedModels", tr("Manage Model Blacklist"), tr("Add or remove models from the <b>Model Randomizer</b>'s blacklist list."), ""},
    {"ManageScores", tr("Manage Model Ratings"), tr("Reset or view the saved ratings for the driving models."), ""},
    {"SelectModel", tr("Select Driving Model"), tr("Select an installed model. Offroad changes apply next drive. While driving, confirm a reboot manually or by switching cruise off; reboot waits until all driving control is inactive."), ""}
  };

  for (const auto &[param, title, desc, icon] : modelToggles) {
    AbstractControl *modelToggle;

    if (param == "DeleteModel") {
      deleteModelBtn = new FrogPilotButtonsControl(title, desc, icon, {tr("DELETE"), tr("DELETE ALL")});
      QObject::connect(deleteModelBtn, &FrogPilotButtonsControl::buttonClicked, [this](int id) {
        for (const QString &file : modelDir.entryList(QDir::Files)) {
          QString modelName = modelFileToNameMapProcessed.value(QFileInfo(file).baseName());
          if (!modelName.isEmpty()) {
            deletableModels.append(modelName);
          }
        }
        deletableModels.removeAll(processModelName(currentModel));
        deletableModels.removeAll(modelFileToNameMapProcessed.value(QString::fromStdString(params_default.get("Model"))));

        if (id == 0) {
          QString modelToDelete = MultiOptionDialog::getSelection(tr("Select a driving model to delete"), deletableModels, "", this);
          if (!modelToDelete.isEmpty() && ConfirmationDialog::confirm(tr("Are you sure you want to delete the \"%1\" model?").arg(modelToDelete), tr("Delete"), this)) {
            QString modelFile = modelFileToNameMapProcessed.key(modelToDelete);
            for (const QString &file : modelDir.entryList(QDir::Files)) {
              if (QFileInfo(file).baseName() == modelFile) {
                QFile::remove(modelDir.filePath(file));
                break;
              }
            }
            downloadableModels.append(modelFileToNameMap.value(modelFile));
            downloadableModels.sort();

            allModelsDownloaded = false;
          }
        } else if (id == 1) {
          if (ConfirmationDialog::confirm(tr("Are you sure you want to delete all of your downloaded driving models?"), tr("Delete"), this)) {
            for (const QString &file : modelDir.entryList(QDir::Files)) {
              QString modelName = modelFileToNameMapProcessed.value(QFileInfo(file).baseName());
              if (deletableModels.contains(modelName)) {
                QFile::remove(modelDir.filePath(file));
              }
            }
            downloadableModels = availableModelNames;

            allModelsDownloaded = false;
            noModelsDownloaded = true;
          }
        }
      });
      modelToggle = deleteModelBtn;
    } else if (param == "DownloadModel") {
      downloadModelBtn = new FrogPilotButtonsControl(title, desc, icon, {tr("DOWNLOAD"), tr("DOWNLOAD ALL")});
      QObject::connect(downloadModelBtn, &FrogPilotButtonsControl::buttonClicked, [this](int id) {
        if (id == 0) {
          if (modelDownloading) {
            params_memory.putBool("CancelModelDownload", true);

            cancellingDownload = true;
          } else {
            for (const QString &file : modelDir.entryList(QDir::Files)) {
              downloadableModels.removeAll(modelFileToNameMap.value(QFileInfo(file).baseName()));
            }

            QString modelToDownload = MultiOptionDialog::getSelection(tr("Select a driving model to download"), downloadableModels, "", this);
            if (!modelToDownload.isEmpty()) {
              params_memory.put("ModelToDownload", modelFileToNameMap.key(modelToDownload).toStdString());
              params_memory.put("ModelDownloadProgress", "Downloading...");

              downloadModelBtn->setText(0, tr("CANCEL"));

              downloadModelBtn->setValue("Downloading...");

              downloadModelBtn->setVisibleButton(1, false);

              modelDownloading = true;
            }
          }
        } else if (id == 1) {
          if (allModelsDownloading) {
            params_memory.putBool("CancelModelDownload", true);

            cancellingDownload = true;
          } else {
            params_memory.putBool("DownloadAllModels", true);
            params_memory.put("ModelDownloadProgress", "Downloading...");

            downloadModelBtn->setText(1, tr("CANCEL"));

            downloadModelBtn->setValue("Downloading...");

            downloadModelBtn->setVisibleButton(0, false);

            allModelsDownloading = true;
          }
        }
      });
      modelToggle = downloadModelBtn;
    } else if (param == "ManageBlacklistedModels") {
      FrogPilotButtonsControl *blacklistBtn = new FrogPilotButtonsControl(title, desc, icon, {tr("ADD"), tr("REMOVE"), tr("REMOVE ALL")});
      QObject::connect(blacklistBtn, &FrogPilotButtonsControl::buttonClicked, [this](int id) {
        QStringList blacklistedModels = QString::fromStdString(params.get("BlacklistedModels")).split(",");
        blacklistedModels.removeAll("");

        if (id == 0) {
          QStringList blacklistableModels;
          for (const QString &model : modelFileToNameMapProcessed.keys()) {
            if (!blacklistedModels.contains(model)) {
              blacklistableModels.append(modelFileToNameMapProcessed.value(model));
            }
          }

          if (blacklistableModels.size() <= 1) {
            ConfirmationDialog::alert(tr("There are no more models to blacklist! The only available model is \"%1\"!").arg(blacklistableModels.first()), this);
          } else {
            QString modelToBlacklist = MultiOptionDialog::getSelection(tr("Select a model to add to the blacklist"), blacklistableModels, "", this);
            if (!modelToBlacklist.isEmpty()) {
              if (ConfirmationDialog::confirm(tr("Are you sure you want to add the \"%1\" model to the blacklist?").arg(modelToBlacklist), tr("Add"), this)) {
                blacklistedModels.append(modelFileToNameMapProcessed.key(modelToBlacklist));

                params.put("BlacklistedModels", blacklistedModels.join(",").toStdString());
              }
            }
          }
        } else if (id == 1) {
          QStringList whitelistableModels;
          for (const QString &model : blacklistedModels) {
            QString modelName = modelFileToNameMapProcessed.value(model);
            if (!modelName.isEmpty()) {
              whitelistableModels.append(modelName);
            }
          }
          whitelistableModels.sort();

          QString modelToWhitelist = MultiOptionDialog::getSelection(tr("Select a model to remove from the blacklist"), whitelistableModels, "", this);
          if (!modelToWhitelist.isEmpty()) {
            if (ConfirmationDialog::confirm(tr("Are you sure you want to remove the \"%1\" model from the blacklist?").arg(modelToWhitelist), tr("Remove"), this)) {
              blacklistedModels.removeAll(modelFileToNameMapProcessed.key(modelToWhitelist));

              params.put("BlacklistedModels", blacklistedModels.join(",").toStdString());
            }
          }
        } else if (id == 2) {
          if (FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to remove all of your blacklisted models?"), this)) {
            params.remove("BlacklistedModels");
            params_cache.remove("BlacklistedModels");
          }
        }
      });
      modelToggle = blacklistBtn;
    } else if (param == "ManageScores") {
      FrogPilotButtonsControl *manageScoresBtn = new FrogPilotButtonsControl(title, desc, icon, {tr("RESET"), tr("VIEW")});
      QObject::connect(manageScoresBtn, &FrogPilotButtonsControl::buttonClicked, [this, modelLayout, modelLabelsList, modelLabelsPanel](int id) {
        if (id == 0) {
          if (FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to reset all of your model drives and scores?"), this)) {
            params.remove("ModelDrivesAndScores");
            params_cache.remove("ModelDrivesAndScores");
          }
        } else if (id == 1) {
          openParentToggle();

          updateModelLabels(modelLabelsList);

          modelLayout->setCurrentWidget(modelLabelsPanel);
        }
      });
      modelToggle = manageScoresBtn;
    } else if (param == "SelectModel") {
      selectModelBtn = new ButtonControl(title, tr("SELECT"), desc);
      selectModelBtn->setObjectName("installedModelSelector");
      QObject::connect(selectModelBtn, &ButtonControl::clicked, [this]() {
        if (rebootPending) {
          modelApi.request("/toadpilot/model-reboot", {{"cancel", true}}, true, [this](const QJsonObject &, const QString &error) {
            if (!error.isEmpty() && isVisible()) ConfirmationDialog::alert(error, this);
            refreshModelReboot();
          });
          return;
        }
        loadInstalledModels(true);
      });
      modelToggle = selectModelBtn;

    } else {
      modelToggle = new ParamControl(param, title, desc, icon);
    }

    toggles[param] = modelToggle;

    modelList->addItem(modelToggle);

    QObject::connect(modelToggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });
  }

  QObject::connect(static_cast<ToggleControl*>(toggles["ModelRandomizer"]), &ToggleControl::toggleFlipped, [this](bool state) {
    updateToggles();

    if (state && !allModelsDownloaded) {
      if (FrogPilotConfirmationDialog::yesorno(tr("The \"Model Randomizer\" only works with downloaded models. Do you want to download all the driving models?"), this)) {
        params_memory.putBool("DownloadAllModels", true);
        params_memory.put("ModelDownloadProgress", "Downloading...");

        downloadModelBtn->setValue("Downloading...");

        allModelsDownloading = true;
      }
    }
  });

  QObject::connect(parent, &FrogPilotSettingsWindow::closeParentToggle, [modelLayout, modelPanel] {modelLayout->setCurrentWidget(modelPanel);});
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotModelPanel::updateState);
  rebootTimer.setInterval(1000);
  QObject::connect(&rebootTimer, &QTimer::timeout, this, &FrogPilotModelPanel::refreshModelReboot);
}

void FrogPilotModelPanel::showEvent(QShowEvent *event) {
  UIState *s = uiState();

  frogpilotToggleLevels = parent->frogpilotToggleLevels;
  tuningLevel = parent->tuningLevel;

  availableModels = QString::fromStdString(params.get("AvailableModels")).split(",");
  availableModelNames = QString::fromStdString(params.get("AvailableModelNames")).split(",");

  modelFileToNameMap.clear();
  modelFileToNameMapProcessed.clear();
  deletableModels.clear();

  int size = qMin(availableModels.size(), availableModelNames.size());
  for (int i = 0; i < size; ++i) {
    modelFileToNameMap.insert(availableModels[i], availableModelNames[i]);
    modelFileToNameMapProcessed.insert(availableModels[i], processModelName(availableModelNames[i]));
  }

  downloadableModels = availableModelNames;
  for (const QString &file : modelDir.entryList(QDir::Files)) {
    downloadableModels.removeAll(modelFileToNameMap.value(QFileInfo(file).baseName()));
  }
  allModelsDownloaded = downloadableModels.isEmpty();

  for (const QString &file : modelDir.entryList(QDir::Files)) {
    QString modelName = modelFileToNameMapProcessed.value(QFileInfo(file).baseName());
    if (!modelName.isEmpty()) {
      deletableModels.append(modelName);
    }
  }
  deletableModels.removeAll(processModelName(currentModel));
  deletableModels.removeAll(modelFileToNameMapProcessed.value(QString::fromStdString(params_default.get("Model"))));
  noModelsDownloaded = deletableModels.isEmpty();

  currentModel = modelFileToNameMap.value(QString::fromStdString(params.get("Model")));
  selectModelBtn->setValue(currentModel);
  loadInstalledModels(false);
  refreshModelReboot();
  rebootTimer.start();

  bool parked = !s->scene.started || s->scene.parked;

  deleteModelBtn->setEnabled(!(allModelsDownloading || modelDownloading || noModelsDownloaded));

  downloadModelBtn->setEnabledButtons(0, !allModelsDownloaded && !allModelsDownloading && !cancellingDownload && s->scene.online && parked);
  downloadModelBtn->setEnabledButtons(1, !allModelsDownloaded && !modelDownloading && !cancellingDownload && s->scene.online && parked);

  started = s->scene.started;

  updateToggles();
}

void FrogPilotModelPanel::updateState(const UIState &s) {
  if (!isVisible() || finalizingDownload) {
    return;
  }

  if (allModelsDownloading || modelDownloading) {
    QString progress = QString::fromStdString(params_memory.get("ModelDownloadProgress"));
    bool downloadFailed = progress.contains(QRegularExpression("cancelled|exists|failed|offline", QRegularExpression::CaseInsensitiveOption));

    if (progress != "Downloading...") {
      downloadModelBtn->setValue(progress);
    }

    if (progress == "All models downloaded!" && allModelsDownloading || progress == "Downloaded!" && modelDownloading || downloadFailed) {
      finalizingDownload = true;

      QTimer::singleShot(2500, [this, progress]() {
        allModelsDownloaded = progress == "All models downloaded!";
        allModelsDownloading = false;
        cancellingDownload = false;
        finalizingDownload = false;
        modelDownloading = false;
        noModelsDownloaded = false;

        params_memory.remove("CancelModelDownload");
        params_memory.remove("DownloadAllModels");
        params_memory.remove("ModelDownloadProgress");
        params_memory.remove("ModelToDownload");

        downloadModelBtn->setEnabled(true);
        downloadModelBtn->setValue("");
      });
    }
  }

  bool parked = !started || s.scene.parked || s.scene.frogs_go_moo;

  deleteModelBtn->setEnabled(!(allModelsDownloading || modelDownloading || noModelsDownloaded));

  downloadModelBtn->setText(0, modelDownloading ? tr("CANCEL") : tr("DOWNLOAD"));
  downloadModelBtn->setText(1, allModelsDownloading ? tr("CANCEL") : tr("DOWNLOAD ALL"));

  downloadModelBtn->setEnabledButtons(0, !allModelsDownloaded && !allModelsDownloading && !cancellingDownload && s.scene.online && parked);
  downloadModelBtn->setEnabledButtons(1, !allModelsDownloaded && !modelDownloading && !cancellingDownload && s.scene.online && parked);

  downloadModelBtn->setVisibleButton(0, !allModelsDownloading);
  downloadModelBtn->setVisibleButton(1, !modelDownloading);

  started = s.scene.started;
  selectModelBtn->setEnabled(!modelRequestPending);

  parent->keepScreenOn = allModelsDownloading || modelDownloading;
}

void FrogPilotModelPanel::loadInstalledModels(bool choose) {
  if (qEnvironmentVariableIsSet("ABRASAX_UI_PREVIEW")) fprintf(stderr, "models request choose=%d pending=%d visible=%d\n", choose, modelRequestPending, isVisible());
  if (modelRequestPending) return;
  modelRequestPending = true;
  selectModelBtn->setEnabled(false);
  modelApi.request("/toadpilot/models", {}, false, [this, choose](const QJsonObject &data, const QString &error) {
    modelRequestPending = false;
    const bool offroad = data.value("environment").toObject().value("offroad_confirmed").toBool();
    if (qEnvironmentVariableIsSet("ABRASAX_UI_PREVIEW")) fprintf(stderr, "models response choose=%d offroad=%d visible=%d error=%s\n", choose, offroad, isVisible(), qPrintable(error));
    const bool onroad = data.value("environment").toObject().value("onroad").toBool();
    selectModelBtn->setEnabled(offroad || onroad);
    if (!error.isEmpty()) {
      selectModelBtn->setValue(tr("Unavailable"));
      // Allow a retry after a transient failure; the server still checks road state.
      selectModelBtn->setEnabled(true);
      if (choose && isVisible()) ConfirmationDialog::alert(error, this);
      return;
    }
    QMap<QString, QString> nameToId;
    QString selectedName;
    const QString selected = data.value("selected").toString();
    for (const auto &value : data.value("models").toArray()) {
      auto model = value.toObject();
      if (model.value("id").toString() == selected) selectedName = model.value("name").toString();
      if (model.value("installed").toBool()) nameToId[model.value("name").toString()] = model.value("id").toString();
    }
    currentModel = selectedName.isEmpty() ? selected : selectedName;
    selectModelBtn->setValue(currentModel);
    if (!choose || !(offroad || onroad) || !isVisible() || rebootPending) return;
    // FrogPilot's full-screen dialog sets the Wayland rotation and uses touch-sized rows.
    const QString name = MultiOptionDialog::getSelection(tr("Select an installed driving model"), nameToId.keys(), currentModel, this);
    if (name.isEmpty() || !nameToId.contains(name) || !isVisible()) return;
    const QString id = nameToId.value(name);
    if (uiState()->scene.started || onroad) {
      queueModelReboot(id, name);
      return;
    }
    modelRequestPending = true;
    selectModelBtn->setEnabled(false);
    selectModelBtn->setValue(tr("Saving…"));
    modelApi.request("/toadpilot/models", {{"model", id}}, true, [this, id, name](const QJsonObject &saved, const QString &saveError) {
      modelRequestPending = false;
      selectModelBtn->setEnabled(!uiState()->scene.started);
      if (!saveError.isEmpty() || saved.value("selected").toString() != id) {
        selectModelBtn->setValue(tr("Not confirmed"));
        if (isVisible()) ConfirmationDialog::alert(saveError.isEmpty() ? tr("Model selection was not confirmed.") : saveError, this);
        return;
      }
      currentModel = name;
      selectModelBtn->setValue(name);
      // The backend also disables randomization when a model is explicitly chosen.
      static_cast<ParamControl *>(toggles["ModelRandomizer"])->refresh();
      updateToggles();
    });
  });
}

void FrogPilotModelPanel::hideEvent(QHideEvent *event) {
  rebootTimer.stop();
  FrogPilotListWidget::hideEvent(event);
}

void FrogPilotModelPanel::queueModelReboot(const QString &id, const QString &name) {
  ConfirmationDialog dialog(tr("Reboot with %1?\n\nTap Reboot, or switch cruise off to confirm automatically. Reboot waits for cruise, steering control and acceleration control to be inactive. Cancel keeps the current model.").arg(name),
                            tr("Reboot"), tr("Cancel"), false, this, true);
  QTimer watcher(&dialog);
  watcher.setInterval(100);
  uint64_t readySince = 0;
  QObject::connect(&watcher, &QTimer::timeout, &dialog, [&dialog, &readySince] {
    auto &sm = *uiState()->sm;
    const uint64_t now = nanos_since_boot();
    bool fresh = true;
    for (const char *service : {"carState", "carControl", "controlsState"}) {
      const uint64_t received = sm.rcv_time(service);
      const uint64_t timestamp = sm[service].getLogMonoTime();
      fresh &= sm.alive(service) && sm.valid(service) && received <= now && now - received < 500000000ULL &&
               timestamp <= now && now - timestamp < 500000000ULL;
    }
    bool ready = false;
    if (fresh) {
      const auto command = sm["carControl"].getCarControl();
      ready = !sm["carState"].getCarState().getCruiseState().getEnabled() &&
              !sm["controlsState"].getControlsState().getEnabled() &&
              !command.getEnabled() && !command.getLatActive() && !command.getLongActive();
    }
    if (!ready) readySince = 0;
    else if (!readySince) readySince = now;
    else if (now - readySince >= 1000000000ULL) dialog.accept();
  });
  watcher.start();
  if (!dialog.exec()) return;
  modelRequestPending = true;
  selectModelBtn->setEnabled(false);
  modelApi.request("/toadpilot/model-reboot", {{"model", id}}, true, [this](const QJsonObject &state, const QString &error) {
    modelRequestPending = false;
    if (!error.isEmpty() && isVisible()) ConfirmationDialog::alert(error, this);
    rebootPending = state.value("pending").toBool();
    refreshModelReboot();
  });
}

void FrogPilotModelPanel::refreshModelReboot() {
  if (rebootStatusPending) return;
  rebootStatusPending = true;
  modelApi.request("/toadpilot/model-reboot", {}, false, [this](const QJsonObject &state, const QString &error) {
    rebootStatusPending = false;
    if (!error.isEmpty()) return;
    const bool wasPending = rebootPending;
    rebootPending = state.value("pending").toBool();
    selectModelBtn->setText(rebootPending ? tr("CANCEL") : tr("SELECT"));
    if (rebootPending || state.value("state").toString() == "rebooting")
      selectModelBtn->setValue(state.value("message").toString());
    else if (wasPending) loadInstalledModels(false);
  });
}

void FrogPilotModelPanel::updateModelLabels(FrogPilotListWidget *labelsList) {
  labelsList->clear();

  QJsonObject modelDrivesAndScores = QJsonDocument::fromJson(QString::fromStdString(params.get("ModelDrivesAndScores")).toUtf8()).object();

  for (const QString &modelName : availableModelNames) {
    QJsonObject modelData = modelDrivesAndScores.value(processModelName(modelName)).toObject();

    int drives = modelData.value("Drives").toInt(0);
    int score = modelData.value("Score").toInt(0);

    QString drivesDisplay = drives == 1 ? QString("%1 Drive").arg(drives) : drives > 0 ? QString("%1 Drives").arg(drives) : "N/A";
    QString scoreDisplay = drives > 0 ? QString("Score: %1%").arg(score) : "N/A";

    QString labelTitle = processModelName(modelName);
    QString labelText = QString("%1 (%2)").arg(scoreDisplay, drivesDisplay);

    LabelControl *labelControl = new LabelControl(labelTitle, labelText, "", this);
    labelsList->addItem(labelControl);
  }
}

void FrogPilotModelPanel::updateToggles() {
  for (auto &[key, toggle] : toggles) {
    bool setVisible = tuningLevel >= frogpilotToggleLevels[key].toDouble();

    if (key == "ManageBlacklistedModels" || key == "ManageScores") {
      setVisible &= params.getBool("ModelRandomizer");
    }

    toggle->setVisible(setVisible);
  }

  update();
}
