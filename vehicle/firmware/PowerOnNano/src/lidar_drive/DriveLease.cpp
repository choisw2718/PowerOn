#include "DriveLease.h"

#include "../motor/Motor.h"

namespace lidar_drive {
namespace {
bool driveArmed = false;
bool leaseExpired = false;
uint32_t lastCommandMs = 0;
uint32_t leaseMs = kWatchdogMs;
} // namespace

void arm(uint32_t now) {
  renew(now, kWatchdogMs);
  driveArmed = true;
}

void cancel() {
  driveArmed = false;
  leaseExpired = false;
}

bool armed() { return driveArmed; }
bool expired() { return leaseExpired; }

void renew(uint32_t now, uint32_t durationMs) {
  lastCommandMs = now;
  leaseMs = durationMs;
}

void service(uint32_t now) {
  if (driveArmed && now - lastCommandMs >= leaseMs) {
    driveArmed = false;
    leaseExpired = true;
    motor::stop();
  }
}
} // namespace lidar_drive
