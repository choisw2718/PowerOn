// Generated from hardware/control_profile.json. Do not edit.
#pragma once

#include <stdint.h>

namespace vehicle_profile {

namespace steering {
constexpr float kWheelbaseM = 0.13324f;
constexpr float kSteeringPivotTrackM = 0.1212f;
constexpr float kMinWheelSteerDeg = -45.0f;
constexpr float kMaxWheelSteerDeg = 55.0f;
constexpr bool kServoChannelsCrossed = true;
constexpr float kLeftServoDirection = 1.0f;
constexpr float kRightServoDirection = 1.0f;
constexpr float kLeftServoUsPerDegree = 10.0f;
constexpr float kRightServoUsPerDegree = 10.0f;
constexpr uint16_t kRightAlignedD5Us = 1597;
constexpr uint16_t kRightAlignedD6Us = 1952;
constexpr uint16_t kLeftAlignedD5Us = 1001;
constexpr uint16_t kLeftAlignedD6Us = 1416;
constexpr uint16_t kLeftServoMinUs = 948;
constexpr uint16_t kLeftServoMaxUs = 1948;
constexpr uint16_t kRightServoMinUs = 952;
constexpr uint16_t kRightServoMaxUs = 1952;
} // namespace steering

namespace motor {
constexpr float kMotorSupplyVoltage = 12.0f;
constexpr float kMinimumReliableMotorVoltage = 7.0f;
} // namespace motor

namespace dual_rear {
constexpr bool kSelected = true;
constexpr int8_t kLeftMotorSign = 0;
constexpr int8_t kRightMotorSign = 0;
constexpr int8_t kLeftEncoderSign = 0;
constexpr int8_t kRightEncoderSign = 0;
constexpr float kRearTrackM = 0.0f;
constexpr float kLeftRolloutM = 0.0f;
constexpr float kRightRolloutM = 0.0f;
constexpr float kLeftCountsPerRev = 0.0f;
constexpr float kRightCountsPerRev = 0.0f;
constexpr float kBenchPwm = 0.0f;
constexpr float kMaxPwm = 0.0f;
constexpr float kMaxWheelSpeed = 0.0f;
constexpr float kTargetAcceleration = 0.0f;
constexpr float kMotorBusV = 0.0f;
constexpr float kLeftFeedForward = 0.0f;
constexpr float kRightFeedForward = 0.0f;
constexpr float kLeftKp = 0.0f;
constexpr float kRightKp = 0.0f;
constexpr float kLeftKi = 0.0f;
constexpr float kRightKi = 0.0f;
} // namespace dual_rear

namespace lidar {
constexpr uint32_t kWatchdogMs = 500;
constexpr float kMaxPwm = 65.0f;
constexpr float kMaxSteerDeg = 28.0f;
} // namespace lidar

namespace ir {
constexpr bool kLeftInstalled = false;
constexpr bool kFrontInstalled = false;
constexpr bool kRightInstalled = false;
} // namespace ir

} // namespace vehicle_profile
