#include "Motor.h"

#include "DualRear.h"
#include "../hardware/VehicleProfile.h"
#include <math.h>

namespace motor {
namespace {

using vehicle_profile::motor::kMinimumReliableMotorVoltage;
using vehicle_profile::motor::kMotorSupplyVoltage;

constexpr char kMotorChannel = 'A';
constexpr uint8_t kMotorPwmPin = 9;       // D9 -> ENA signal pin (not its 5V jumper pin)
constexpr uint8_t kMotorIn1Pin = 2;       // D2 -> IN1
constexpr uint8_t kMotorIn2Pin = 4;       // D4 -> IN2
constexpr uint8_t kUnusedFormerEnablePin = 3; // Disconnect the former MAI ENABLE wire.
constexpr bool kForwardIn1High = true;
constexpr uint32_t kDirectionPauseMs = 50;
constexpr float kEpsilon = 0.001f;

bool motorEnabled = false;
bool motorForward = true;
bool directionPauseActive = false;
float targetDutyPercent = 0.0f;
float appliedDutyPercent = 0.0f;
uint32_t directionPauseStartedMs = 0;

float clampFloat(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

void disableMotor() {
  // L298N ENA LOW disables the bridge (coast). Cut PWM before direction inputs.
  analogWrite(kMotorPwmPin, 0);
  motorEnabled = false;
  digitalWrite(kMotorIn1Pin, LOW);
  digitalWrite(kMotorIn2Pin, LOW);
  appliedDutyPercent = 0.0f;
}

void setMotorDirection(bool forward) {
  const bool in1High = forward ? kForwardIn1High : !kForwardIn1High;
  digitalWrite(kMotorIn1Pin, in1High ? HIGH : LOW);
  digitalWrite(kMotorIn2Pin, in1High ? LOW : HIGH);
}

} // namespace

void stop() {
  if (dual_rear::kSelected) {
    dual_rear::stop();
    return;
  }
  targetDutyPercent = 0.0f;
  directionPauseActive = false;
  disableMotor();
}

void commandPercent(float requestedPercent) {
  if (dual_rear::kSelected) return; // Legacy open-loop commands are disabled.
  const float limited = clampFloat(requestedPercent, -100.0f, 100.0f);
  if (fabsf(limited) <= kEpsilon) {
    stop();
    return;
  }
  targetDutyPercent = limited;
  const bool requestedForward = limited > 0.0f;
  if (requestedForward != motorForward) {
    disableMotor();
    motorForward = requestedForward;
    directionPauseStartedMs = millis();
    directionPauseActive = true;
  }
}

float commandReliableVoltage(float requestedVoltage) {
  float voltage = clampFloat(requestedVoltage, -kMotorSupplyVoltage,
                             kMotorSupplyVoltage);
  if (fabsf(voltage) > kEpsilon &&
      fabsf(voltage) < kMinimumReliableMotorVoltage) {
    voltage = voltage > 0.0f ? kMinimumReliableMotorVoltage
                             : -kMinimumReliableMotorVoltage;
  }
  commandPercent(voltage * 100.0f / kMotorSupplyVoltage);
  return voltage;
}

void service() {
  if (dual_rear::kSelected) {
    dual_rear::service();
    return;
  }
  if (fabsf(targetDutyPercent) <= kEpsilon) return;
  if (directionPauseActive) {
    if (millis() - directionPauseStartedMs < kDirectionPauseMs) return;
    directionPauseActive = false;
  }
  const float duty = fabsf(targetDutyPercent);
  if (motorEnabled && fabsf(appliedDutyPercent - duty) <= kEpsilon) return;
  analogWrite(kMotorPwmPin, 0);
  setMotorDirection(motorForward);
  analogWrite(kMotorPwmPin, static_cast<int>(duty * 255.0f / 100.0f + 0.5f));
  motorEnabled = true;
  appliedDutyPercent = duty;
}

void begin() {
  if (dual_rear::kSelected) {
    dual_rear::begin();
    return;
  }
  // The old D3 MAI ENABLE signal must not be wired to L298N ENA.
  pinMode(kUnusedFormerEnablePin, INPUT);
  digitalWrite(kMotorIn1Pin, LOW); pinMode(kMotorIn1Pin, OUTPUT);
  digitalWrite(kMotorIn2Pin, LOW); pinMode(kMotorIn2Pin, OUTPUT);
  digitalWrite(kMotorPwmPin, LOW); pinMode(kMotorPwmPin, OUTPUT);
  stop();
}

void commandVoltage(float requestedVoltage) {
  commandPercent(clampFloat(requestedVoltage, -kMotorSupplyVoltage,
                            kMotorSupplyVoltage) * 100.0f / kMotorSupplyVoltage);
}

void printStatus(Print &out) {
  if (dual_rear::kSelected) {
    dual_rear::printStatus(out);
    out.print(F("STEERING"));
    return;
  }
  out.print(F("STATUS driver=L298N channel=")); out.print(kMotorChannel);
  out.print(F(" duty_cmd=")); out.print(targetDutyPercent, 1);
  out.print(F("% duty_applied=")); out.print(appliedDutyPercent, 1);
  out.print(F("% enabled=")); out.print(motorEnabled ? 1 : 0);
}

} // namespace motor
