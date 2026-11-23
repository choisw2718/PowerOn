#pragma once

#include <Arduino.h>
#include "../hardware/VehicleProfile.h"

namespace dual_rear {
constexpr bool kSelected = vehicle_profile::dual_rear::kSelected;
constexpr float kMaxCenterSteer = vehicle_profile::lidar::kMaxSteerDeg;
void begin();
void stop();
void service();
void printStatus(Print &out);
bool arm(Print &out);
bool command(float centerMps, Print &out);
bool benchPulse(bool left, float signedDuty, Print &out);
} // namespace dual_rear
