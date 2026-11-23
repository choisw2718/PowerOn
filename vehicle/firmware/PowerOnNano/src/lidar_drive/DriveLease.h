#pragma once

#include "../hardware/VehicleProfile.h"

namespace lidar_drive {
using vehicle_profile::lidar::kWatchdogMs;
using vehicle_profile::lidar::kMaxPwm;
using vehicle_profile::lidar::kMaxSteerDeg;

void arm(uint32_t now);
void cancel();
bool armed();
bool expired();
void renew(uint32_t now, uint32_t durationMs);
void service(uint32_t now);
} // namespace lidar_drive
