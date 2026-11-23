#ifndef POWERON_DISTANCE_SENSORS_H
#define POWERON_DISTANCE_SENSORS_H

#include "../hardware/VehicleProfile.h"
#include "../ir_wall_following/WallFollower.h"
#include "SharpIr.h"
#include <Arduino.h>

constexpr uint8_t kDistanceSensorPins[] = {A0, A1, A2}; // left, front, right
constexpr bool kDistanceSensorConnected[] = {
    vehicle_profile::ir::kLeftInstalled,
    vehicle_profile::ir::kFrontInstalled,
    vehicle_profile::ir::kRightInstalled,
};
constexpr uint32_t kDistanceScanPeriodMs = 60; // Sensor cycle 38.3 +/- 9.6 ms + output delay.
constexpr uint32_t kDistanceWarmupMs = 200;

struct DistanceSensorState {
  bool connected[3] = {kDistanceSensorConnected[0], kDistanceSensorConnected[1], kDistanceSensorConnected[2]};
  sharp_ir::Calibration calibration[3];
  sharp_ir::Reading reading[3];
  uint32_t startedMs = 0;
  uint32_t lastScanMs = 0;
  uint32_t capturedMs = 0;
  bool available = false;
};

inline DistanceSensorState &distanceSensorState() {
  static DistanceSensorState state;
  return state;
}

inline void beginDistanceSensors() {
  analogReference(AR_DEFAULT); // SAMD 3.3 V full-scale ADC range.
  analogReadResolution(12);
  for (uint8_t pin : kDistanceSensorPins) pinMode(pin, INPUT);
  DistanceSensorState &state = distanceSensorState();
  state.startedMs = state.lastScanMs = millis();
  state.available = false;
  for (uint8_t i = 0; i < 3; ++i) {
    state.reading[i] = sharp_ir::Reading();
    if (!state.connected[i]) state.reading[i].status = sharp_ir::Status::NotConnected;
  }
  // Each channel's calibration table/reference/divider can be set here.
  // User confirmed Vo directly to A0/A1/A2: dividerRatio=1.0, not 0.5.
  // This conversion setting does NOT protect Nano inputs from overvoltage.
  // Low output remains FarOrUnknown, unusable by default; do not silently turn
  // a disconnected sensor or an ambiguous near object into an open corridor.
}

inline bool pollDistanceSensors(wall_follow::Sample &sample, uint32_t now) {
  DistanceSensorState &state = distanceSensorState();
  if (now - state.startedMs < kDistanceWarmupMs ||
      now - state.lastScanMs < kDistanceScanPeriodMs) return false;
  state.lastScanMs = now;
  for (uint8_t channel = 0; channel < 3; ++channel) {
    if (!state.connected[channel]) {
      state.reading[channel] = sharp_ir::Reading();
      state.reading[channel].status = sharp_ir::Status::NotConnected;
      continue; // Never convert floating inputs into valid-looking distances.
    }
    const uint8_t pin = kDistanceSensorPins[channel];
    analogRead(pin); // Discard first conversion after ADC channel switching.
    uint16_t values[5];
    for (uint8_t i = 0; i < 5; ++i) values[i] = static_cast<uint16_t>(analogRead(pin));
    for (uint8_t i = 1; i < 5; ++i) {
      const uint16_t value = values[i];
      uint8_t j = i;
      while (j && values[j-1] > value) { values[j] = values[j-1]; --j; }
      values[j] = value;
    }
    // This median reduces ADC spikes; it is NOT five independent sensor frames.
    state.reading[channel] = sharp_ir::decode(values[2], state.calibration[channel]);
  }
  state.capturedMs = now; // Conservative ADC scan start; internal sensor age is additional.
  state.available = true;
  sample.leftMm = state.reading[0].mm;
  sample.frontMm = state.reading[1].mm;
  sample.rightMm = state.reading[2].mm;
  sample.leftValid = state.reading[0].usable;
  sample.frontValid = state.reading[1].usable;
  sample.rightValid = state.reading[2].usable;
  sample.capturedMs = state.capturedMs;
  sample.available = true;
  return true; // Includes new INVALID readings; never keep old valid data on error.
}

#endif
