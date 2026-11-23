#pragma once

#include "WallFollower.h"
#include <Arduino.h>

namespace ir_autonomy {
// Enable only after verifying sensor wiring, calibration, mounting and stopping distances.
constexpr bool kAutonomousActuationEnabled = false;

inline wall_follow::Config makeWallFollowConfig() {
  wall_follow::Config config;
  config.corridorMm = 610.0f;
  // TODO: replace with measured sensor-face offsets from vehicle centre.
  config.leftOffsetMm = 0.0f;
  config.rightOffsetMm = 0.0f;
  // Other provisional thresholds/gains/PWM settings are in WallFollower.h.
  return config;
}

void cancel();
bool isDriving();
void setTestRanges(float left, float front, float right, Print &out);
void start(bool preview, Print &out);
void printStatus(Print &out);
void printSensors(Print &out);
void service(Print *statusOutput);
} // namespace ir_autonomy
