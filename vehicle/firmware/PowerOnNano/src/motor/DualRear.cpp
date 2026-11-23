#include "DualRear.h"

#include "RearSpeedControl.h"
#include "../steering/AckermannSteering.h"
#include <initializer_list>
#include <stdio.h>

namespace dual_rear {
namespace {
using namespace vehicle_profile::dual_rear;
using vehicle_profile::steering::kWheelbaseM;
using rear_speed::clamp;

constexpr uint32_t kControlPeriodUs = 10000;
constexpr uint32_t kControlGapUs = 100000;
constexpr uint32_t kFeedbackTimeoutMs = 350;
constexpr uint32_t kCommandLeaseMs = 300;
constexpr uint32_t kBenchPulseMs = 250;
constexpr uint32_t kCoastWaitMs = 500;
constexpr float kMovingThreshold = 0.025f;

struct WheelConfig {
  uint8_t pwm, in1, in2, encoderA, encoderB;
  int8_t motorSign, encoderSign;
  float rollout, countsPerRev, feedForward, kp, ki;
};

constexpr WheelConfig kLeft{9, 2, 4, 3, 11, kLeftMotorSign, kLeftEncoderSign,
                            kLeftRolloutM, kLeftCountsPerRev, kLeftFeedForward,
                            kLeftKp, kLeftKi};
constexpr WheelConfig kRight{10, 7, 8, A5, A7, kRightMotorSign, kRightEncoderSign,
                             kRightRolloutM, kRightCountsPerRev, kRightFeedForward,
                             kRightKp, kRightKi};

struct Wheel {
  const WheelConfig &config;
  volatile uint8_t phase = 0;
  volatile uint32_t count = 0, invalid = 0;
  uint32_t lastCount = 0, lastInvalid = 0, progressAtMs = 0;
  int32_t progress = 0;
  float speed = 0, target = 0, voltage = 0, duty = 0;
  int pwm = 0;
  rear_speed::SpeedPI pi;
  explicit Wheel(const WheelConfig &config) : config(config) {}
};

Wheel left(kLeft), right(kRight);
bool armed = false, bench = false;
float requestedSpeed = 0, rampedSpeed = 0;
int8_t commandedSign = 0;
uint32_t lastSampleUs = 0, lastCommandMs = 0, stoppedAtMs = 0;
const char *fault = "none";

uint8_t readPhase(const WheelConfig &config) {
  return (digitalRead(config.encoderA) == HIGH ? 2 : 0) |
         (digitalRead(config.encoderB) == HIGH ? 1 : 0);
}

void countEdge(Wheel &wheel) {
  const uint8_t phase = readPhase(wheel.config);
  const uint8_t previous = wheel.phase;
  wheel.phase = phase;
  if ((previous ^ phase) == 3) ++wheel.invalid;
  wheel.count += rear_speed::quadratureStep(previous, phase);
}

void leftEdge() { countEdge(left); }
void rightEdge() { countEdge(right); }

void writeDuty(Wheel &wheel, float duty) {
  const int pwm = static_cast<int>(duty * 2.55f);
  if (pwm != wheel.pwm) {
    if (pwm == 0 || wheel.pwm == 0 || (pwm > 0) != (wheel.pwm > 0)) {
      digitalWrite(wheel.config.pwm, LOW);
      pinMode(wheel.config.pwm, OUTPUT);
      digitalWrite(wheel.config.in1, pwm > 0 ? HIGH : LOW);
      digitalWrite(wheel.config.in2, pwm < 0 ? HIGH : LOW);
    }
    if (pwm != 0) analogWrite(wheel.config.pwm, abs(pwm));
    wheel.pwm = pwm;
  }
  wheel.duty = pwm * (100.0f / 255.0f);
}

void beginWheel(Wheel &wheel, void (*edge)()) {
  const WheelConfig &c = wheel.config;
  for (uint8_t pin : {c.pwm, c.in1, c.in2}) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  pinMode(c.encoderA, INPUT);
  pinMode(c.encoderB, INPUT);
  wheel.phase = readPhase(c);
  attachInterrupt(digitalPinToInterrupt(c.encoderA), edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c.encoderB), edge, CHANGE);
}

bool calibrated(const WheelConfig &c) {
  return c.rollout > 0 && c.countsPerRev > 0 && abs(c.motorSign) == 1 &&
         abs(c.encoderSign) == 1 && c.kp > 0 && c.ki > 0 && c.feedForward >= 0;
}

bool ready() {
  const float curvatureLimit = tanf(kMaxCenterSteer * 0.01745329252f);
  return calibrated(kLeft) && calibrated(kRight) && kWheelbaseM > 0 &&
         kRearTrackM > 0 && 0.5f * kRearTrackM * curvatureLimit < kWheelbaseM &&
         kMotorBusV > 0 && kMotorBusV <= 24 && kMaxPwm >= 1 && kMaxPwm <= 100 &&
         kTargetAcceleration > 0 && kMaxWheelSpeed > kMovingThreshold;
}

bool stopped(uint32_t now) {
  return now - stoppedAtMs >= kCoastWaitMs &&
         fabsf(left.speed) < kMovingThreshold && fabsf(right.speed) < kMovingThreshold;
}

void resetWheel(Wheel &wheel) {
  writeDuty(wheel, 0);
  wheel.target = wheel.voltage = 0;
  wheel.progress = 0;
  wheel.pi.reset();
}

void trip(const char *reason) {
  stop();
  fault = reason;
}

int32_t sample(Wheel &wheel, uint32_t count, float dt) {
  const int32_t ticks = static_cast<int32_t>(count - wheel.lastCount);
  wheel.lastCount = count;
  const auto &c = wheel.config;
  const int32_t signedTicks = c.encoderSign == -1 ? -ticks : ticks;
  if (c.countsPerRev > 0) {
    const float mps = signedTicks * c.rollout / (c.countsPerRev * dt);
    wheel.speed += dt / (0.04f + dt) * (mps - wheel.speed);
  }
  return signedTicks;
}

bool driveWheel(Wheel &wheel, float target, int32_t ticks, float dt, uint32_t now) {
  const auto &c = wheel.config;
  wheel.target = target;
  if (fabsf(wheel.speed) > 1.25f * kMaxWheelSpeed) return false;
  const float magnitude = fabsf(target);
  const int direction = target > 0 ? 1 : -1;
  wheel.voltage = direction * wheel.pi.voltage(magnitude, direction * wheel.speed,
      dt, c.feedForward, c.kp, c.ki, kMotorBusV * kMaxPwm / 100.0f);
  writeDuty(wheel, wheel.voltage * c.motorSign * 100.0f / kMotorBusV);
  if (wheel.pwm == 0) {
    wheel.progress = 0;
    wheel.progressAtMs = now;
    return true;
  }
  wheel.progress += direction * ticks;
  if (wheel.progress <= -4) return false;
  if (wheel.progress >= 2) {
    wheel.progress = 0;
    wheel.progressAtMs = now;
  }
  return now - wheel.progressAtMs < kFeedbackTimeoutMs;
}

void printWheel(Print &out, const Wheel &wheel) {
  out.print(F(" count=")); out.print(static_cast<long>(static_cast<int32_t>(wheel.count)));
  out.print(F(" invalid=")); out.print(static_cast<unsigned long>(wheel.invalid));
  out.print(F(" target_mps=")); out.print(wheel.target, 3);
  out.print(F(" speed_mps=")); out.print(wheel.speed, 3);
  out.print(F(" cmd_nominal_v=")); out.print(wheel.voltage, 2);
  out.print(F(" signed_pwm_pct=")); out.print(wheel.duty, 1);
}
} // namespace

void begin() {
  beginWheel(left, leftEdge);
  beginWheel(right, rightEdge);
  stoppedAtMs = millis();
  lastSampleUs = micros();
}

void stop() {
  if (armed || bench) stoppedAtMs = millis();
  resetWheel(left);
  resetWheel(right);
  armed = bench = false;
  requestedSpeed = rampedSpeed = 0;
  commandedSign = 0;
}

bool arm(Print &out) {
  const uint32_t now = millis();
  if (!ready() || armed || bench || !stopped(now)) {
    out.println(F("ERR DUAL arm requires wheel calibration and stopped outputs"));
    return false;
  }
  fault = "none";
  armed = true;
  lastCommandMs = left.progressAtMs = right.progressAtMs = now;
  out.println(F("OK DUAL ARM"));
  return true;
}

bool command(float centerMps, Print &out) {
  if (centerMps == 0) {
    stop();
    out.println(F("OK SPEED STOP"));
    return true;
  }
  if (!armed || !isfinite(centerMps) || fabsf(centerMps) > kMaxWheelSpeed) {
    trip("speed_rejected");
    out.println(F("ERR SPEED requires arm and configured speed range"));
    return false;
  }
  const int8_t sign = centerMps > 0 ? 1 : -1;
  if (commandedSign != 0 && sign != commandedSign) {
    trip("direction_change");
    out.println(F("ERR SPEED direction change requires STOP, coast and ARM"));
    return false;
  }
  if (commandedSign == 0) left.progressAtMs = right.progressAtMs = millis();
  commandedSign = sign;
  requestedSpeed = centerMps;
  lastCommandMs = millis();
  // One TCP write. Values refer to the most recent completed control sample.
  char telemetry[150];
  snprintf(telemetry, sizeof(telemetry),
           "OK SPEED target_mmps=%ld,%ld speed_mmps=%ld,%ld motor_pwm_permille=%ld,%ld",
           lroundf(left.target * 1000), lroundf(right.target * 1000),
           lroundf(left.speed * 1000), lroundf(right.speed * 1000),
           lroundf(left.duty * 10), lroundf(right.duty * 10));
  out.println(telemetry);
  return true;
}

bool benchPulse(bool leftWheel, float signedDuty, Print &out) {
  const uint32_t now = millis();
  if (armed || bench || !stopped(now) || !isfinite(signedDuty) ||
      kBenchPwm <= 0 || kBenchPwm > 10 || fabsf(signedDuty) < 1 ||
      fabsf(signedDuty) > kBenchPwm) {
    out.println(F("ERR DUAL TEST requires stopped outputs and 1..configured bench PWM"));
    return false;
  }
  bench = true;
  lastCommandMs = now;
  writeDuty(leftWheel ? left : right, signedDuty);
  out.println(F("OK DUAL TEST"));
  return true;
}

void service() {
  const uint32_t now = millis();
  if (armed && now - lastCommandMs >= kCommandLeaseMs) trip("command_timeout");
  if (bench && now - lastCommandMs >= kBenchPulseMs) stop();
  const uint32_t sampleUs = micros();
  const uint32_t elapsed = sampleUs - lastSampleUs;
  if (elapsed < kControlPeriodUs) return;
  lastSampleUs = sampleUs;
  if ((armed || bench) && elapsed > kControlGapUs) trip("control_gap");

  // Copy both counters and transition diagnostics at one sampling instant.
  noInterrupts();
  const uint32_t leftCount = left.count, rightCount = right.count;
  const uint32_t leftErrors = left.invalid, rightErrors = right.invalid;
  interrupts();
  const float dt = elapsed * 0.000001f;
  const int32_t leftTicks = sample(left, leftCount, dt);
  const int32_t rightTicks = sample(right, rightCount, dt);
  const bool noisy = leftErrors - left.lastInvalid >= 4 || rightErrors - right.lastInvalid >= 4;
  left.lastInvalid = leftErrors;
  right.lastInvalid = rightErrors;
  if (!armed || requestedSpeed == 0) return;
  if (noisy) {
    trip("encoder_transitions");
    return;
  }
  // Ramp increases in magnitude. Respond immediately to a lower speed command.
  rampedSpeed = commandedSign * fminf(fabsf(requestedSpeed),
                                      fabsf(rampedSpeed) + kTargetAcceleration * dt);
  const auto targets = rear_speed::targets(rampedSpeed, steering::center(),
                                          kWheelbaseM, kRearTrackM, kMaxWheelSpeed);
  if (!driveWheel(left, targets.left, leftTicks, dt, now)) trip("left_encoder_motion");
  else if (!driveWheel(right, targets.right, rightTicks, dt, now)) trip("right_encoder_motion");
}

void printStatus(Print &out) {
  out.print(F("STATUS driver=L298N channel=AB mode=dual_rear ready="));
  out.print(ready() ? 1 : 0);
  out.print(F(" armed=")); out.print(armed ? 1 : 0);
  out.print(F(" enabled=")); out.print(left.pwm || right.pwm ? 1 : 0);
  out.print(F(" max_mps=")); out.print(kMaxWheelSpeed, 3);
  out.print(F(" fault=")); out.println(fault);
  out.print(F("REAR left")); printWheel(out, left); out.println();
  out.print(F("REAR right")); printWheel(out, right); out.println();
}
} // namespace dual_rear
