#pragma once

#include <cstdlib>
#include <set>
#include <string>

#include "msgq/msgq.h"

// Adapted from upstream's 250 KiB / 2 MiB / 10 MiB service classes.
// Legacy binaries hard-code 10 MiB and ftruncate on every connection. Until all
// producers/consumers are rebuilt, smaller queues are restricted to explicitly
// isolated test namespaces. The installed shared queues keep their original ABI.
inline size_t msgq_configured_size(const std::string &endpoint) {
  const char *enabled = std::getenv("TOADPILOT_BOUNDED_IPC");
  const char *prefix = std::getenv("OPENPILOT_PREFIX");
  if (!enabled || std::string(enabled) != "1" || !prefix || !*prefix) return DEFAULT_SEGMENT_SIZE;
  if (endpoint == "controlsState" || endpoint == "sendcan" || endpoint == "navRoute") return 2 * 1024 * 1024;
  static const std::set<std::string> small = {
    "gyroscope", "gyroscope2", "accelerometer", "accelerometer2", "magnetometer", "lightSensor",
    "temperatureSensor", "temperatureSensor2", "gpsNMEA", "deviceState", "pandaStates", "peripheralState",
    "radarState", "liveTracks", "logMessage", "errorLogMessage", "liveCalibration", "liveTorqueParameters",
    "liveDelay", "androidLog", "carState", "carControl", "carOutput", "longitudinalPlan", "gpsLocationExternal",
    "gpsLocation", "clocks", "liveLocationKalman", "liveParameters", "cameraOdometry", "carParams",
    "roadCameraState", "driverCameraState", "wideRoadCameraState", "driverMonitoringState", "managerState",
    "navInstruction", "uiPlan", "userFlag", "frogpilotCarState", "frogpilotDeviceState", "frogpilotNavigation", "frogpilotPlan"
  };
  return small.count(endpoint) ? 250 * 1024 : DEFAULT_SEGMENT_SIZE;
}
