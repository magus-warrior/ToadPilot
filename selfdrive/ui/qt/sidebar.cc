#include "selfdrive/ui/qt/sidebar.h"

#include <algorithm>
#include <QMouseEvent>
#include <QPainterPath>

#include "selfdrive/ui/qt/util.h"

void Sidebar::drawMetric(QPainter &p, const QPair<QString, QString> &label, QColor c, int y) {
  const QRect rect = {30, y, 240, 126};

  p.setPen(QPen(onroad ? QColor("#35566d") : QColor("#463958"), 2));
  p.setBrush(onroad ? QColor("#122c40") : QColor("#211b32"));
  p.drawRoundedRect(rect, 18, 18);
  p.setPen(Qt::NoPen);
  p.setBrush(c);
  p.drawEllipse(QPoint(rect.right() - 24, rect.top() + 26), onroad ? 10 : 7, onroad ? 10 : 7);
  p.setPen(onroad ? QColor("#b6d5e5") : QColor("#c8bcdc"));
  p.setFont(InterFont(onroad ? 28 : 24, QFont::DemiBold));
  p.drawText(rect.adjusted(19, 10, onroad ? -40 : -27, -68), Qt::AlignLeft | Qt::AlignVCenter, label.first);
  p.setPen(Qt::white);
  const QRect valueRect = rect.adjusted(19, 48, -10, -8);
  p.setFont(InterFont(onroad ? 41 : 34, QFont::DemiBold));
  if (onroad && p.fontMetrics().horizontalAdvance(label.second) > valueRect.width()) {
    p.setFont(InterFont(std::max(24, 41 * valueRect.width() / p.fontMetrics().horizontalAdvance(label.second)), QFont::DemiBold));
  }
  p.drawText(valueRect, Qt::AlignLeft | Qt::AlignVCenter, label.second);
}

Sidebar::Sidebar(QWidget *parent) : QFrame(parent), onroad(false), flag_pressed(false), settings_pressed(false) {
  connect(this, &Sidebar::valueChanged, [=] { update(); });

  setAttribute(Qt::WA_OpaquePaintEvent);
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
  setFixedWidth(300);

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Sidebar::updateState);

  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"userFlag"});

  // FrogPilot variables
  home_label = new QLabel(this);
  settings_label = new QLabel(this);

  onroad_settings_img = loadPixmap("../assets/navigation/icon_settings.svg", {80, 80});
  homeGifPath = "../frogpilot/assets/active_theme/icons/button_home.gif";
  homePngPath = "../frogpilot/assets/active_theme/icons/button_home.png";
  settingsGifPath = "../frogpilot/assets/active_theme/icons/button_settings.gif";
  settingsPngPath = "../frogpilot/assets/active_theme/icons/button_settings.png";

  randomEventGifPath = "../frogpilot/assets/random_events/icons/button_home.gif";

  QObject::connect(uiState(), &UIState::themeUpdated, this, &Sidebar::updateIcons);
}

void Sidebar::showEvent(QShowEvent *event) {
  updateIcons();
}

void Sidebar::updateIcons() {
  updateIcon(home_label, home_gif, QString(), home_btn, "../frogpilot/fleetmanager/static/abrasax-logo.png", isHomeGif);
  updateIcon(settings_label, settings_gif, settingsGifPath, settings_btn, settingsPngPath, isSettingsGif);
}

void Sidebar::updateIcon(QLabel *&label, QMovie *&gif, const QString &gifPath, const QRect &btnRect, const QString &pngPath, bool &isGif) {
  QString selectedGifPath = gifPath;
  if (!gifPath.isEmpty() && util::random_int(1, 100 * UI_FREQ) == 100 * UI_FREQ && btnRect == home_btn && isRandomEvents) {
    selectedGifPath = randomEventGifPath;
  }

  if (gif != nullptr) {
    gif->stop();
    delete gif;
    gif = nullptr;
    if (label) {
      label->hide();
    }
  }

  if (QFile::exists(selectedGifPath)) {
    gif = new QMovie(selectedGifPath);

    if (gif->isValid()) {
      gif->setScaledSize(btnRect.size());

      if (label) {
        label->setGeometry(btnRect);
        label->setMovie(gif);
        label->setVisible(!onroad);
      }

      gif->start();
      isGif = true;
    } else {
      delete gif;
      gif = nullptr;
      isGif = false;
    }
  } else {
    if (btnRect == home_btn) {
      home_img = QPixmap(btnRect.size());
      home_img.fill(Qt::transparent);
      QPainter logoPainter(&home_img);
      logoPainter.setRenderHint(QPainter::Antialiasing);
      logoPainter.setPen(Qt::NoPen);
      logoPainter.setBrush(Qt::white);
      logoPainter.drawRoundedRect(home_img.rect().adjusted(0, 40, 0, -40), 12, 12);
      auto logo = loadPixmap(pngPath, QSize(160, 80));
      logoPainter.drawPixmap((180 - logo.width()) / 2, (180 - logo.height()) / 2, logo);
    } else {
      settings_img = loadPixmap(pngPath, btnRect.size(), Qt::IgnoreAspectRatio);
    }

    isGif = false;
  }
}

void Sidebar::mousePressEvent(QMouseEvent *event) {
  UIState *s = uiState();
  UIScene &scene = s->scene;

  QPoint pos = event->pos();

  QRect cpuRect = {30, 496, 240, 126};
  QRect memoryRect = {30, 654, 240, 126};
  QRect tempRect = {30, 338, 240, 126};

  static int showChip = 0;
  static int showMemory = 0;
  static int showTemp = 0;

  if (cpuRect.contains(pos) && isSidebarMetrics) {
    showChip = (showChip + 1) % 3;

    isCPU = (showChip == 1);
    isGPU = (showChip == 2);

    scene.cpu_metrics = isCPU;
    scene.gpu_metrics = isGPU;

    params.putBoolNonBlocking("ShowCPU", isCPU);
    params.putBoolNonBlocking("ShowGPU", isGPU);

    update();
    return;
  }

  if (memoryRect.contains(pos) && isSidebarMetrics) {
    showMemory = (showMemory + 1) % 4;

    isMemoryUsage = (showMemory == 1);
    isStorageLeft = (showMemory == 2);
    isStorageUsed = (showMemory == 3);

    scene.memory_metrics = isMemoryUsage;
    scene.storage_left_metrics = isStorageLeft;
    scene.storage_used_metrics = isStorageUsed;

    params.putBoolNonBlocking("ShowMemoryUsage", isMemoryUsage);
    params.putBoolNonBlocking("ShowStorageLeft", isStorageLeft);
    params.putBoolNonBlocking("ShowStorageUsed", isStorageUsed);

    update();
    return;
  }

  if (tempRect.contains(pos) && isSidebarMetrics) {
    showTemp = (showTemp + 1) % 3;

    isFahrenheit = showTemp == 2;

    scene.fahrenheit = isFahrenheit;
    scene.numerical_temp = showTemp != 0;

    params.putBoolNonBlocking("Fahrenheit", showTemp == 2);
    params.putBoolNonBlocking("NumericalTemp", showTemp != 0);

    update();
    return;
  }

  if (onroad && home_btn.contains(pos)) {
    flag_pressed = true;
    update();
    return;
  }

  if (settings_btn.contains(pos)) {
    settings_pressed = true;
    update();
    return;
  }
}

void Sidebar::mouseReleaseEvent(QMouseEvent *event) {
  if (flag_pressed || settings_pressed) {
    flag_pressed = settings_pressed = false;
    update();
  }
  if (onroad && home_btn.contains(event->pos())) {
    MessageBuilder msg;
    msg.initEvent().initUserFlag();
    pm->send("userFlag", msg);
  } else if (settings_btn.contains(event->pos())) {
    emit openSettings();
  }
}

void Sidebar::offroadTransition(bool offroad) {
  onroad = !offroad;
  settings_label->setVisible(offroad && isSettingsGif);
  update();
}

void Sidebar::updateState(const UIState &s) {
  if (!isVisible()) {
    if (home_gif != nullptr) {
      home_gif->stop();
      delete home_gif;
      home_gif = nullptr;
      home_label->hide();
    }

    if (settings_gif != nullptr) {
      settings_gif->stop();
      delete settings_gif;
      settings_gif = nullptr;
      settings_label->hide();
    }
    return;
  }

  auto &sm = *(s.sm);

  auto deviceState = sm["deviceState"].getDeviceState();
  // FrogPilot variables
  const UIScene &scene = s.scene;

  isCPU = scene.cpu_metrics;
  isFahrenheit = scene.fahrenheit;
  isGPU = scene.gpu_metrics;
  isIP = scene.ip_metrics;
  isMemoryUsage = scene.memory_metrics;
  isNumericalTemp = scene.numerical_temp;
  isRandomEvents = scene.random_events;
  isSidebarMetrics = scene.sidebar_metrics;
  isStorageLeft = scene.storage_left_metrics;
  isStorageUsed = scene.storage_used_metrics;

  bool useStockColors = scene.use_stock_colors;
  sidebar_color1 = useStockColors ? good_color : scene.sidebar_color1;
  sidebar_color2 = useStockColors ? good_color : scene.sidebar_color2;
  sidebar_color3 = useStockColors ? good_color : scene.sidebar_color3;

  const cereal::FrogPilotDeviceState::Reader &frogpilotDeviceState = sm["frogpilotDeviceState"].getFrogpilotDeviceState();

  int maxTempC = deviceState.getMaxTempC();
  max_temp = isFahrenheit ? QString::number(maxTempC * 9 / 5 + 32) + "°F" : QString::number(maxTempC) + "°C";

  setProperty("netType", network_type[deviceState.getNetworkType()]);
  int strength = (int)deviceState.getNetworkStrength();
  setProperty("netStrength", strength > 0 ? strength + 1 : 0);

  ItemStatus connectStatus;
  auto last_ping = deviceState.getLastAthenaPingTime();
  if (last_ping == 0) {
    connectStatus = ItemStatus{{tr("CONNECT"), tr("OFFLINE")}, warning_color};
  } else {
    connectStatus = nanos_since_boot() - last_ping < 80e9
                        ? ItemStatus{{tr("CONNECT"), tr("ONLINE")}, sidebar_color3}
                        : ItemStatus{{tr("CONNECT"), tr("ERROR")}, danger_color};
  }
  setProperty("connectStatus", QVariant::fromValue(connectStatus));

  ItemStatus tempStatus = {{tr("TEMP"), isNumericalTemp ? max_temp : tr("HIGH")}, danger_color};
  auto ts = deviceState.getThermalStatus();
  if (ts == cereal::DeviceState::ThermalStatus::GREEN) {
    tempStatus = {{tr("TEMP"), isNumericalTemp ? max_temp : tr("GOOD")}, sidebar_color1};
  } else if (ts == cereal::DeviceState::ThermalStatus::YELLOW) {
    tempStatus = {{tr("TEMP"), isNumericalTemp ? max_temp : tr("OK")}, warning_color};
  }
  setProperty("tempStatus", QVariant::fromValue(tempStatus));

  ItemStatus pandaStatus = {{tr("VEHICLE"), tr("ONLINE")}, sidebar_color2};
  if (s.scene.pandaType == cereal::PandaState::PandaType::UNKNOWN) {
    pandaStatus = {{tr("NO"), tr("PANDA")}, danger_color};
  } else if (s.scene.started && !sm["liveLocationKalman"].getLiveLocationKalman().getGpsOK()) {
    pandaStatus = {{tr("GPS"), tr("SEARCH")}, warning_color};
  }
  setProperty("pandaStatus", QVariant::fromValue(pandaStatus));

  if (isCPU || isGPU) {
    capnp::List<int8_t>::Reader cpu_loads = deviceState.getCpuUsagePercent();
    int cpu_usage = cpu_loads.size() != 0 ? std::accumulate(cpu_loads.begin(), cpu_loads.end(), 0) / cpu_loads.size() : 0;
    int gpu_usage = deviceState.getGpuUsagePercent();
    int usage = isGPU ? gpu_usage : cpu_usage;

    QString chip_usage = QString::number(usage) + "%";

    ItemStatus cpuStatus = {{isGPU ? tr("GPU") : tr("CPU"), chip_usage}, sidebar_color2};
    if (usage >= 85) {
      cpuStatus = {{isGPU ? tr("GPU") : tr("CPU"), chip_usage}, danger_color};
    } else if (usage >= 70) {
      cpuStatus = {{isGPU ? tr("GPU") : tr("CPU"), chip_usage}, warning_color};
    }
    setProperty("cpuStatus", QVariant::fromValue(cpuStatus));
  }

  if (isMemoryUsage || isStorageLeft || isStorageUsed) {
    int memory_usage = deviceState.getMemoryUsagePercent();
    int storage_left = frogpilotDeviceState.getFreeSpace();
    int storage_used = frogpilotDeviceState.getUsedSpace();

    QString memory = QString::number(memory_usage) + "%";
    QString storage = QString::number(isStorageLeft ? storage_left : storage_used) + tr(" GB");

    if (isMemoryUsage) {
      ItemStatus memoryStatus = {{tr("MEMORY"), memory}, sidebar_color3};
      if (memory_usage >= 85) {
        memoryStatus = {{tr("MEMORY"), memory}, danger_color};
      } else if (memory_usage >= 70) {
        memoryStatus = {{tr("MEMORY"), memory}, warning_color};
      }
      setProperty("memoryStatus", QVariant::fromValue(memoryStatus));
    } else {
      ItemStatus storageStatus = {{isStorageLeft ? tr("LEFT") : tr("USED"), storage}, sidebar_color3};
      if (25 > storage_left && storage_left >= 10) {
        storageStatus = {{isStorageLeft ? tr("LEFT") : tr("USED"), storage}, warning_color};
      } else if (10 > storage_left) {
        storageStatus = {{isStorageLeft ? tr("LEFT") : tr("USED"), storage}, danger_color};
      }
      setProperty("storageStatus", QVariant::fromValue(storageStatus));
    }
  }
}

void Sidebar::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setPen(Qt::NoPen);
  p.setRenderHint(QPainter::Antialiasing);

  p.fillRect(rect(), onroad ? QColor("#0a1b2b") : QColor("#151123"));

  // buttons
  if (onroad) {
    p.setPen(QPen(QColor("#35566d"), 2));
    p.setBrush(settings_pressed ? QColor("#1e455a") : QColor("#122c40"));
    p.drawRoundedRect(settings_btn, 22, 22);
    p.drawPixmap(settings_btn.center() - QPoint(onroad_settings_img.width() / 2, onroad_settings_img.height() / 2), onroad_settings_img);

    p.setBrush(flag_pressed ? QColor("#215467") : QColor("#103446"));
    p.setPen(QPen(QColor("#408497"), 3));
    p.drawRoundedRect(home_btn, 26, 26);
    const QPoint center = home_btn.center();
    QPainterPath bookmark;
    bookmark.moveTo(center.x() - 25, center.y() - 52);
    bookmark.lineTo(center.x() + 25, center.y() - 52);
    bookmark.lineTo(center.x() + 25, center.y() + 12);
    bookmark.lineTo(center.x(), center.y() - 4);
    bookmark.lineTo(center.x() - 25, center.y() + 12);
    bookmark.closeSubpath();
    p.setPen(Qt::NoPen);
    p.setBrush(QColor("#86eedc"));
    p.drawPath(bookmark);
    p.setPen(Qt::white);
    p.setFont(InterFont(27, QFont::DemiBold));
    p.drawText(home_btn.adjusted(3, 105, -3, -8), Qt::AlignCenter, tr("Mark event"));
  } else if (!isSettingsGif) {
    p.setOpacity(settings_pressed ? 0.65 : 1.0);
    p.drawPixmap(settings_btn.x(), settings_btn.y(), settings_img);
  }
  if (!onroad && !isHomeGif) {
    p.setOpacity(1.);
    p.drawPixmap(home_btn.x(), home_btn.y(), home_img);
  }
  p.setOpacity(1.0);

  // network
  int x = 58;
  const QColor inactiveBar = onroad ? QColor("#335269") : QColor("#545454");
  p.setPen(Qt::NoPen);
  p.setFont(InterFont(35));

  if (isIP) {
    p.setPen(QColor(0xff, 0xff, 0xff));
    p.save();
    p.setFont(InterFont(30));
    QRect ipBox = QRect(50, 196, 225, 27);
    p.drawText(ipBox, Qt::AlignLeft | Qt::AlignVCenter, uiState()->wifi->getIp4Address());
    p.restore();
  } else {
    for (int i = 0; i < 5; ++i) {
      p.setBrush(i < net_strength ? Qt::white : inactiveBar);
      p.drawEllipse(x, 196, 27, 27);
      x += 37;
    }
    p.setPen(QColor(0xff, 0xff, 0xff));
  }

  const QRect r = QRect(50, 247, 100, 50);
  p.drawText(r, Qt::AlignCenter, net_type);

  // metrics
  drawMetric(p, temp_status.first, temp_status.second, 338);

  if (isCPU || isGPU) {
    drawMetric(p, cpu_status.first, cpu_status.second, 496);
  } else {
    drawMetric(p, panda_status.first, panda_status.second, 496);
  }

  if (isMemoryUsage) {
    drawMetric(p, memory_status.first, memory_status.second, 654);
  } else if (isStorageLeft || isStorageUsed) {
    drawMetric(p, storage_status.first, storage_status.second, 654);
  } else {
    drawMetric(p, connect_status.first, connect_status.second, 654);
  }
}
