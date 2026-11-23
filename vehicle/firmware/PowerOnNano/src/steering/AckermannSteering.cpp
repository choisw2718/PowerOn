#include "AckermannSteering.h"

#include "../hardware/VehicleProfile.h"
#include <Servo.h>
#include <math.h>

namespace steering {
namespace {

using namespace vehicle_profile::steering;

constexpr uint8_t kLeftServoPin = 5;      // D5 -> servo channel L, assumed physical right
constexpr uint8_t kRightServoPin = 6;     // D6 -> servo channel R, assumed physical left

// The ~40 mm wheel-carrying arm rotates rigidly with the wheel. With the arm
// along the wheel axle (zero fore/aft offset), its lateral length cancels from
// the no-side-slip constraint. Use SERVO PIVOT spacing, not tire-center spacing.
// Nominal straight-ahead tire-center spacing would be 121.20 + 2*40 = 201.20 mm.

// Select the aligned wheel's raw pulse, independently of the new mirror direction.
// Physical D5/D6 mapping is still an assumption; confirm it with STATUS/wiring.
constexpr uint16_t kLeftServoNeutralUs =
    kServoChannelsCrossed ? kRightAlignedD5Us : kLeftAlignedD5Us;
constexpr uint16_t kRightServoNeutralUs =
    kServoChannelsCrossed ? kLeftAlignedD6Us : kRightAlignedD6Us;
static_assert((kLeftServoDirection == 1.0f || kLeftServoDirection == -1.0f) &&
              (kRightServoDirection == 1.0f || kRightServoDirection == -1.0f),
              "Each servo channel direction must be +1 or -1.");
static_assert(kLeftServoNeutralUs > kLeftServoMinUs &&
              kLeftServoNeutralUs < kLeftServoMaxUs &&
              kRightServoNeutralUs > kRightServoMinUs &&
              kRightServoNeutralUs < kRightServoMaxUs,
              "Neutral sample hits a pulse limit; remeasure actual channel mapping.");
static_assert(kWheelbaseM > 0.0f && kSteeringPivotTrackM > 0.0f,
              "Ackermann geometry must have positive axle distances.");
static_assert(kMinWheelSteerDeg < 0.0f && kMinWheelSteerDeg > -89.0f &&
              kMaxWheelSteerDeg > 0.0f && kMaxWheelSteerDeg < 89.0f &&
              kLeftServoUsPerDegree > 0.0f && kRightServoUsPerDegree > 0.0f,
              "Steering limits and servo gains must allow both turn directions.");

constexpr float kEpsilon = 0.001f;
constexpr float kDegToRad = 0.0174532925f;
constexpr float kRadToDeg = 57.2957795f;

Servo leftServo;
Servo rightServo;
float steeringRequestedDeg = 0.0f;
float steeringCenterDeg = 0.0f;
bool steeringLimited = false;
float frontLeftSteerDeg = 0.0f;
float frontRightSteerDeg = 0.0f;
uint16_t leftServoUs = kLeftServoNeutralUs;
uint16_t rightServoUs = kRightServoNeutralUs;

float clampFloat(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float servoWheelLimit(bool positive, uint16_t minimumUs, uint16_t neutralUs,
                     uint16_t maximumUs, float direction, float usPerDegree) {
  const float angleAtMin =
      (static_cast<float>(minimumUs) - neutralUs) / (direction * usPerDegree);
  const float angleAtMax =
      (static_cast<float>(maximumUs) - neutralUs) / (direction * usPerDegree);
  // Return a positive magnitude in the requested physical steering direction.
  return positive ? fminf(kMaxWheelSteerDeg, fmaxf(angleAtMin, angleAtMax))
                  : fminf(-kMinWheelSteerDeg, -fminf(angleAtMin, angleAtMax));
}

float steeringCenterLimit(bool positive) {
  const float channelL = servoWheelLimit(
      positive, kLeftServoMinUs, kLeftServoNeutralUs, kLeftServoMaxUs,
      kLeftServoDirection, kLeftServoUsPerDegree);
  const float channelR = servoWheelLimit(
      positive, kRightServoMinUs, kRightServoNeutralUs, kRightServoMaxUs,
      kRightServoDirection, kRightServoUsPerDegree);
  const float physicalLeft = kServoChannelsCrossed ? channelR : channelL;
  const float physicalRight = kServoChannelsCrossed ? channelL : channelR;
  const float innerLimit = positive ? physicalLeft : physicalRight;
  const float outerLimit = positive ? physicalRight : physicalLeft;
  const float halfTrack = kSteeringPivotTrackM * 0.5f;
  // Either wheel can be the limiting channel, including the OUTER wheel.
  // Restrict the common turn radius before generating either wheel angle.
  const float innerRadius = halfTrack + kWheelbaseM / tanf(innerLimit * kDegToRad);
  const float outerRadius = -halfTrack + kWheelbaseM / tanf(outerLimit * kDegToRad);
  const float radius = fmaxf(innerRadius, outerRadius);
  return atanf(kWheelbaseM / radius) * kRadToDeg;
}

void setServosFromWheelAngles(float leftDeg, float rightDeg) {
  // apply limits the pair together; do not independently clip wheel angles.
  const float channelL = kServoChannelsCrossed ? rightDeg : leftDeg;
  const float channelR = kServoChannelsCrossed ? leftDeg : rightDeg;
  leftServoUs = static_cast<uint16_t>(
      clampFloat(kLeftServoNeutralUs + channelL * kLeftServoDirection *
                     kLeftServoUsPerDegree,
                 kLeftServoMinUs, kLeftServoMaxUs) + 0.5f);
  rightServoUs = static_cast<uint16_t>(
      clampFloat(kRightServoNeutralUs + channelR * kRightServoDirection *
                     kRightServoUsPerDegree,
                 kRightServoMinUs, kRightServoMaxUs) + 0.5f);
  leftServo.writeMicroseconds(leftServoUs);
  rightServo.writeMicroseconds(rightServoUs);
}

} // namespace

void apply(float centerDeg) {
  const float minimum = -steeringCenterLimit(false);
  const float maximum = steeringCenterLimit(true);
  steeringRequestedDeg = centerDeg;
  steeringCenterDeg = clampFloat(centerDeg, minimum, maximum);
  steeringLimited = fabsf(steeringRequestedDeg - steeringCenterDeg) > kEpsilon;
  if (fabsf(steeringCenterDeg) <= kEpsilon) {
    steeringCenterDeg = frontLeftSteerDeg = frontRightSteerDeg = 0.0f;
  } else {
    const float signedRadius = kWheelbaseM /
                               tanf(steeringCenterDeg * kDegToRad);
    const float radius = fabsf(signedRadius);
    const float halfTrack = kSteeringPivotTrackM * 0.5f;
    const float inner = atanf(kWheelbaseM / (radius - halfTrack)) * kRadToDeg;
    const float outer = atanf(kWheelbaseM / (radius + halfTrack)) * kRadToDeg;
    frontLeftSteerDeg = signedRadius > 0.0f ? inner : -outer;
    frontRightSteerDeg = signedRadius > 0.0f ? outer : -inner;
  }
  setServosFromWheelAngles(frontLeftSteerDeg, frontRightSteerDeg);
}

void begin() {
  leftServo.attach(kLeftServoPin);
  rightServo.attach(kRightServoPin);
  apply(0.0f);
}

float center() { return steeringCenterDeg; }

void printStatus(Print &out) {
  out.print(F(" steer_requested=")); out.print(steeringRequestedDeg, 2);
  out.print(F(" steer_center=")); out.print(steeringCenterDeg, 3);
  out.print(F("deg steer_limited=")); out.print(steeringLimited ? 1 : 0);
  out.print(F(" wheel_left_deg=")); out.print(frontLeftSteerDeg, 2);
  out.print(F(" wheel_right_deg=")); out.print(frontRightSteerDeg, 2);
  out.print(F(" servo_us_l=")); out.print(leftServoUs);
  out.print(F(" servo_us_r=")); out.print(rightServoUs);
  out.print(F(" steering_cal=ackermann-body2-v4 neutral_us_d5=")); out.print(kLeftServoNeutralUs);
  out.print(F(" neutral_us_d6=")); out.print(kRightServoNeutralUs);
  out.print(F(" direction_d5=")); out.print(kLeftServoDirection, 0);
  out.print(F(" direction_d6=")); out.print(kRightServoDirection, 0);
  out.print(F(" channels_crossed=")); out.print(kServoChannelsCrossed ? 1 : 0);
  out.print(F(" geometry=body2-hs311-nominal wheelbase_mm="));
  out.print(kWheelbaseM * 1000.0f, 2);
  out.print(F(" pivot_track_mm=")); out.print(kSteeringPivotTrackM * 1000.0f, 2);
  out.print(F(" center_min_deg=")); out.print(-steeringCenterLimit(false), 2);
  out.print(F(" center_max_deg=")); out.print(steeringCenterLimit(true), 2);
  out.print(F(" pulse_limit_d5="));
  out.print(leftServoUs == kLeftServoMinUs || leftServoUs == kLeftServoMaxUs ? 1 : 0);
  out.print(F(" pulse_limit_d6="));
  out.println(rightServoUs == kRightServoMinUs || rightServoUs == kRightServoMaxUs ? 1 : 0);
}

} // namespace steering
