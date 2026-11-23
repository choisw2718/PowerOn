#include "ir_sensors/DistanceSensors.h"
#include "ir_wall_following/IrAutonomy.h"
#include <cstdio>
#include <cstdlib>

#define CHECK(condition) do { if (!(condition)) { \
  std::fprintf(stderr, "FAIL ADC driver line %d: %s\n", __LINE__, #condition); std::exit(1); \
} } while (0)

uint32_t fakeMs = 0;
int bits = 0, modes = 0, references = 0;
int calls[3] = {0,0,0};
uint16_t adc[3];
uint32_t millis() { return fakeMs; }
void analogReference(uint8_t reference) { CHECK(reference == AR_DEFAULT); ++references; }
void analogReadResolution(int resolution) { bits = resolution; }
void pinMode(uint8_t pin, uint8_t mode) { CHECK(pin >= A0 && pin <= A2 && mode == INPUT); ++modes; }
int analogRead(uint8_t pin) {
  const uint8_t channel = pin - A0;
  const int index = calls[channel]++ % 6;
  // First value is a mux-switch outlier, second is a spike; median rejects both.
  if (index == 0) return 4095;
  if (index == 1) return adc[channel] / 2;
  return adc[channel];
}
uint16_t atVolts(float volts) { return static_cast<uint16_t>(volts / 3.3f * 4095 + 0.5f); }

int main() {
  beginDistanceSensors();
  CHECK(bits == 12 && modes == 3 && references == 1);
  CHECK(!ir_autonomy::kAutonomousActuationEnabled);
  for (const sharp_ir::Calibration &calibration : distanceSensorState().calibration)
    CHECK(calibration.dividerRatio == 1.0f);
  for (uint8_t channel = 0; channel < 3; ++channel)
    CHECK(distanceSensorState().connected[channel] == kDistanceSensorConnected[channel]);

  for (bool &connected : distanceSensorState().connected) connected = false;
  beginDistanceSensors();
  wall_follow::Sample unmounted;
  CHECK(pollDistanceSensors(unmounted, 200));
  CHECK(!unmounted.leftValid && !unmounted.frontValid && !unmounted.rightValid);
  for (uint8_t channel = 0; channel < 3; ++channel) {
    CHECK(calls[channel] == 0);
    CHECK(distanceSensorState().reading[channel].status == sharp_ir::Status::NotConnected);
  }
  std::puts("PASS unmounted sensors are never sampled");

  // Simulate the earlier bench configuration with only A2 wired.
  distanceSensorState().connected[2] = true;
  beginDistanceSensors();
  adc[2] = atVolts(0.30f);
  wall_follow::Sample rightOnly;
  CHECK(pollDistanceSensors(rightOnly, 200));
  CHECK(!rightOnly.leftValid && !rightOnly.frontValid);
  CHECK(distanceSensorState().reading[0].status == sharp_ir::Status::NotConnected);
  CHECK(distanceSensorState().reading[1].status == sharp_ir::Status::NotConnected);
  CHECK(calls[0] == 0 && calls[1] == 0 && calls[2] == 6);
  CHECK(!rightOnly.rightValid); // Open-space low signal remains unknown.
  std::puts("PASS right-only wiring: unused ADC pins are not sampled or treated as distances");
  // The remaining driver scenarios deliberately simulate three wired sensors.
  for (bool &connected : distanceSensorState().connected) connected = true;
  calls[0] = calls[1] = calls[2] = 0;
  beginDistanceSensors();
  adc[0] = atVolts(0.92f); adc[1] = atVolts(0.43f); adc[2] = atVolts(0.92f);
  wall_follow::Sample sample;
  CHECK(!pollDistanceSensors(sample, 199));
  CHECK(!sample.available && calls[0] == 0);
  CHECK(pollDistanceSensors(sample, 200));
  CHECK(sample.available && sample.capturedMs == 200);
  CHECK(sample.leftValid && sample.frontValid && sample.rightValid);
  CHECK(fabsf(sample.leftMm - 300) < 2);
  CHECK(calls[0] == 6 && calls[1] == 6 && calls[2] == 6);
  CHECK(!pollDistanceSensors(sample, 259));
  CHECK(sample.capturedMs == 200);
  adc[1] = 0;
  CHECK(pollDistanceSensors(sample, 260));
  CHECK(!sample.frontValid && sample.capturedMs == 260); // Invalid replaces old valid reading.
  adc[1] = atVolts(0.30f);
  CHECK(pollDistanceSensors(sample, 320));
  CHECK(!sample.frontValid);
  CHECK(distanceSensorState().reading[1].status == sharp_ir::Status::FarOrUnknown);
  distanceSensorState().calibration[1].allowLowSignalAsFar = true;
  CHECK(pollDistanceSensors(sample, 380));
  CHECK(sample.frontValid && sample.frontMm == 800);
  // The exact same sensor-to-controller configuration runs at 50 Hz service
  // ticks but only new 60 ms ADC scans count towards geometric confirmation.
  wall_follow::Controller controller(ir_autonomy::makeWallFollowConfig());
  CHECK(controller.start(sample, 380));
  controller.update(sample, 681);
  CHECK(controller.output().state == wall_follow::State::Fault);
  CHECK(controller.output().fault == wall_follow::Fault::Stale);
  fakeMs = UINT32_MAX - 100;
  beginDistanceSensors();
  CHECK(!pollDistanceSensors(sample, 98));
  CHECK(pollDistanceSensors(sample, 99));
  std::puts("PASS Sharp driver warmup, mux discard, median, 60 ms scans, invalid replacement, far policy and rollover");
}
