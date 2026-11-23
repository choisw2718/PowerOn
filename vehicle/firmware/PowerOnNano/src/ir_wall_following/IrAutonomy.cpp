#include "IrAutonomy.h"

#include "AutonomousSteering.h"
#include "../ir_sensors/DistanceSensors.h"
#include "../motor/Motor.h"
#include "../steering/AckermannSteering.h"
#include <math.h>

namespace ir_autonomy {
namespace {

wall_follow::Controller wallFollower(makeWallFollowConfig());
AutonomousSteering autonomousSteering;
wall_follow::Sample liveRanges;
wall_follow::Sample testRanges;
bool autonomyActive = false;
bool autonomyPreview = false;
uint32_t lastAutonomyTickMs = 0;
constexpr uint32_t kAutonomyPeriodMs = 20; // Servo targets at 50 Hz, new sensor scans at ~16.7 Hz.

} // namespace

bool isDriving() { return autonomyActive && !autonomyPreview; }

void cancel() {
  wallFollower.stop();
  autonomousSteering.reset(steering::center(), millis());
  autonomyActive = false;
  autonomyPreview = false;
}

void setTestRanges(float left, float front, float right, Print &out) {
  if (autonomyActive && !autonomyPreview) {
    out.println(F("ERR injected ranges are forbidden during AUTO RUN"));
    return;
  }
  testRanges.leftMm = left;
  testRanges.frontMm = front;
  testRanges.rightMm = right;
  testRanges.capturedMs = millis();
  testRanges.available = true;
  testRanges.leftValid = left > 0.0f;
  testRanges.frontValid = front > 0.0f;
  testRanges.rightValid = right > 0.0f;
  out.println(F("OK RANGES TEST ONLY"));
}

void start(bool preview, Print &out) {
  if (!preview && !kAutonomousActuationEnabled) {
    out.println(F("ERR AUTO START disabled; calibrate sensors, far policy and vehicle first"));
    return;
  }
  if (!preview) pollDistanceSensors(liveRanges, millis());
  autonomyPreview = preview;
  autonomyActive = true;
  const bool started = wallFollower.start(preview ? testRanges : liveRanges, millis());
  autonomousSteering.reset(steering::center(), millis());
  lastAutonomyTickMs = millis();
  out.println(started ? F("OK AUTO") : F("ERR AUTO cannot start"));
  printStatus(out);
}

void printStatus(Print &out) {
  const wall_follow::Output &result = wallFollower.output();
  out.print(F("AUTO mode="));
  out.print(!autonomyActive ? F("OFF") : (autonomyPreview ? F("TEST") : F("RUN")));
  out.print(F(" state=")); out.print(wall_follow::stateName(result.state));
  out.print(F(" fault=")); out.print(wall_follow::faultName(result.fault));
  out.print(F(" duty_target=")); out.print(result.duty, 1);
  out.print(F(" steer_target=")); out.print(result.steerDeg, 3);
  out.print(F(" steer_smoothed=")); out.print(autonomousSteering.angle(), 3);
  out.print(F(" error_mm=")); out.print(result.errorMm, 1);
  out.print(F(" wall_ref=")); out.print(result.wallReference);
  out.print(F(" turns=")); out.print(result.turnsCompleted);
  out.print(F(" actuation_enabled=")); out.println(kAutonomousActuationEnabled ? 1 : 0);
}

void printSensors(Print &out) {
  pollDistanceSensors(liveRanges, millis());
  const DistanceSensorState &state = distanceSensorState();
  out.print(F("SENSORS model=GP2Y0A21YK0F pins=A0,A1,A2 available=")); out.print(state.available ? 1 : 0);
  out.print(F(" age_ms=")); out.print(state.available ? millis() - state.capturedMs : 0);
  const char *names[] = {" left", " front", " right"};
  for (uint8_t i = 0; i < 3; ++i) {
    const sharp_ir::Reading &reading = state.reading[i];
    const sharp_ir::Calibration &calibration = state.calibration[i];
    out.print(names[i]); out.print(F("_adc=")); out.print(reading.adc);
    out.print(names[i]); out.print(F("_adc_V="));
    out.print(reading.adc * calibration.adcReferenceVolts / 4095.0f, 3);
    out.print(names[i]); out.print(F("_divider=")); out.print(calibration.dividerRatio, 3);
    out.print(names[i]); out.print(F("_sensor_V=")); out.print(reading.sensorVolts, 3);
    out.print(names[i]); out.print(F("_mm=")); out.print(reading.mm, 1);
    out.print(names[i]); out.print(F("_status=")); out.print(sharp_ir::statusName(reading.status));
    out.print(names[i]); out.print(F("_usable=")); out.print(reading.usable ? 1 : 0);
  }
  out.println();
}

void service(Print *statusOutput) {
  const uint32_t now = millis();
  // A physical driver supplies only liveRanges; injected ranges never reach RUN.
  pollDistanceSensors(liveRanges, now);
  if (!autonomyActive || now - lastAutonomyTickMs < kAutonomyPeriodMs) return;
  lastAutonomyTickMs = now;
  const wall_follow::State oldState = wallFollower.output().state;
  wallFollower.update(autonomyPreview ? testRanges : liveRanges, now);
  const wall_follow::Output &result = wallFollower.output();
  const bool turning = result.state == wall_follow::State::TurnLeft ||
                       result.state == wall_follow::State::TurnRight;
  if (result.state != wall_follow::State::Fault)
    autonomousSteering.update(result.steerDeg, now, turning || result.state == wall_follow::State::Settle);
  if (!autonomyPreview) {
    if (result.state == wall_follow::State::Fault) motor::stop();
    else {
      steering::apply(autonomousSteering.angle());
      // Only the commanded trajectory is checked, not actual servo position.
      const bool steeringReady = !turning ||
          fabsf(autonomousSteering.angle() - result.steerDeg) <= 0.05f;
      motor::commandPercent(steeringReady ? result.duty : 0.0f);
    }
  }
  if (result.state != oldState && statusOutput) printStatus(*statusOutput);
}

} // namespace ir_autonomy
