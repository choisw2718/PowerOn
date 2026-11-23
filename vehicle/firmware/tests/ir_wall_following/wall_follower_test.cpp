#include "ir_wall_following/WallFollower.h"
#include "ir_sensors/SharpIr.h"
#include "ir_wall_following/AutonomousSteering.h"
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace wall_follow;
#define CHECK(condition) do { if (!(condition)) { \
  std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); std::exit(1); \
} } while (0)

Sample scan(uint32_t at, float left = 305, float front = 750, float right = 305) {
  Sample sample;
  sample.leftMm = left; sample.frontMm = front; sample.rightMm = right;
  sample.capturedMs = at; sample.available = true;
  sample.leftValid = sample.frontValid = sample.rightValid = true;
  return sample;
}

void faulted(const Controller &controller, Fault reason) {
  CHECK(controller.output().state == State::Fault);
  CHECK(controller.output().fault == reason);
  CHECK(controller.output().duty == 0);
  CHECK(controller.output().steerDeg == 0);
}

void beginTurn(Controller &controller, bool left) {
  CHECK(controller.start(scan(0), 0));
  for (uint32_t t = 50; t <= 150; t += 50) {
    controller.update(scan(t, left ? 750 : 305, 350, left ? 305 : 750), t);
    if (t < 150) CHECK(controller.output().duty == 0);
  }
  CHECK(controller.output().state == (left ? State::TurnLeft : State::TurnRight));
  CHECK(left ? controller.output().steerDeg > 0 : controller.output().steerDeg < 0);
}

void replayCsv(const char *path) {
  FILE *file = std::fopen(path, "r");
  CHECK(file != nullptr);
  char line[200];
  CHECK(std::fgets(line, sizeof(line), file) != nullptr);
  Controller preview;
  uint32_t now = 0;
  bool started = false;
  State previous = State::Off;
  std::puts("Offline example replay (targets only, no vehicle connection):");
  while (std::fgets(line, sizeof(line), file)) {
    float duration, left, front, right;
    CHECK(std::sscanf(line, "%f,%f,%f,%f", &duration, &left, &front, &right) == 4);
    const int ticks = static_cast<int>(duration * 20.0f + 0.5f);
    for (int tick = 0; tick < ticks; ++tick) {
      const Sample sample = scan(now, left, front, right);
      if (!started) { CHECK(preview.start(sample, now)); started = true; }
      else preview.update(sample, now);
      const Output &output = preview.output();
      if (output.state != previous) {
        std::printf("%5u ms  %-10s duty=%4.1f steer=%6.2f fault=%s turns=%u\n",
                    now, stateName(output.state), output.duty, output.steerDeg,
                    faultName(output.fault), output.turnsCompleted);
        previous = output.state;
      }
      now += 50;
    }
  }
  std::fclose(file);
  CHECK(preview.output().turnsCompleted == 2);
  faulted(preview, Fault::FrontClose);
}

int main(int argc, char **argv) {
  sharp_ir::Calibration calibration;
  const auto adcAt = [&](float volts) {
    return static_cast<uint16_t>(volts * calibration.dividerRatio /
        calibration.adcReferenceVolts * 4095 + 0.5f);
  };
  float previousDistance = 1000;
  for (float volts = 0.41f; volts <= 2.29f; volts += 0.01f) {
    const sharp_ir::Reading reading = sharp_ir::decode(adcAt(volts), calibration);
    CHECK(reading.usable && reading.status == sharp_ir::Status::InRange);
    CHECK(reading.mm >= 100 && reading.mm <= 800);
    CHECK(reading.mm < previousDistance);
    previousDistance = reading.mm;
  }
  CHECK(fabsf(sharp_ir::decode(adcAt(0.92f), calibration).mm - 300) < 2);
  CHECK(fabsf(sharp_ir::decode(adcAt(1.30f), calibration).mm - 200) < 2);
  // Real reported ADC ~1600 must be ~1.29 V / 20 cm in DIRECT wiring,
  // not the earlier erroneous ~2.58 V / too_near classification.
  const sharp_ir::Reading direct = sharp_ir::decode(1600, calibration);
  CHECK(calibration.dividerRatio == 1.0f);
  CHECK(fabsf(direct.sensorVolts - 1.2894f) < 0.002f);
  CHECK(direct.usable && direct.mm > 195 && direct.mm < 210);
  sharp_ir::Calibration divided = calibration;
  divided.dividerRatio = 0.5f;
  const sharp_ir::Reading viaDivider = sharp_ir::decode(800, divided);
  CHECK(fabsf(viaDivider.mm - direct.mm) < 0.001f);
  CHECK(sharp_ir::decode(1600, divided).status == sharp_ir::Status::TooNear);
  CHECK(!sharp_ir::decode(0, calibration).usable);
  CHECK(!sharp_ir::decode(4095, calibration).usable);
  CHECK(sharp_ir::decode(adcAt(2.8f), calibration).status == sharp_ir::Status::TooNear);
  CHECK(!sharp_ir::decode(adcAt(2.8f), calibration).usable);
  CHECK(sharp_ir::decode(adcAt(0.3f), calibration).status == sharp_ir::Status::FarOrUnknown);
  CHECK(!sharp_ir::decode(adcAt(0.3f), calibration).usable);
  calibration.allowLowSignalAsFar = true;
  CHECK(sharp_ir::decode(adcAt(0.3f), calibration).usable);
  CHECK(!sharp_ir::decode(0, calibration).usable);
  calibration.dividerRatio = 0;
  CHECK(!sharp_ir::decode(800, calibration).usable);
  std::puts("PASS Sharp ADC divider, nonlinear curve, range limits and explicit far policy");

  AutonomousSteering servo;
  servo.reset(0, 0);
  CHECK(fabsf(servo.update(0.1f, 20, false) - 0.1f) < 0.00001f);
  CHECK(fabsf(servo.update(-0.1f, 40, false) + 0.1f) < 0.00001f);
  servo.reset(0, 0);
  CHECK(fabsf(servo.update(18, 20, false) - 0.4f) < 0.00001f);
  CHECK(fabsf(servo.update(18, 1000, false) - 1.2f) < 0.00001f);
  servo.reset(0, UINT32_MAX - 10);
  CHECK(fabsf(servo.update(0.1f, 9, false) - 0.1f) < 0.00001f);
  servo.reset(0, 0);
  for (uint32_t t = 20; t <= 400; t += 20) servo.update(30, t, true);
  CHECK(fabsf(servo.angle() - 30) < 0.00001f);
  std::puts("PASS sub-degree steering, 50 Hz slew, delayed tick cap and millis rollover");

  Controller straight;
  CHECK(straight.start(scan(0), 0));
  CHECK(straight.output().state == State::Follow);
  CHECK(straight.output().wallReference == 3);
  CHECK(straight.output().steerDeg == 0);
  straight.update(scan(50, 355, 750, 255), 50);
  CHECK(straight.output().steerDeg > 0); // Too far from left -> steer left.
  Controller rightCorrection;
  CHECK(rightCorrection.start(scan(0, 255, 750, 355), 0));
  CHECK(rightCorrection.output().steerDeg < 0);
  std::puts("PASS centre control and steering sign");

  Config offsets;
  offsets.leftOffsetMm = 100; offsets.rightOffsetMm = 80;
  Controller mounted(offsets);
  CHECK(mounted.start(scan(0, 205, 750, 225), 0));
  CHECK(mounted.output().steerDeg == 0);
  std::puts("PASS asymmetric sensor mounting offsets");

  Controller openings;
  CHECK(openings.start(scan(0), 0));
  openings.update(scan(50, 750, 750, 305), 50);
  CHECK(openings.output().state == State::Follow);
  CHECK(openings.output().wallReference == 2);
  CHECK(openings.output().steerDeg == 0);
  openings.update(scan(100, 305, 750, 750), 100);
  CHECK(openings.output().wallReference == 1);
  CHECK(openings.output().steerDeg == 0);
  openings.update(scan(150, 305, 750, 355), 150);
  CHECK(openings.output().steerDeg < 0);
  CHECK(openings.output().turnsCompleted == 0);
  std::puts("PASS shortcuts on either side ignored while front is clear");

  Controller approach;
  CHECK(approach.start(scan(0, 305, 500, 305), 0));
  CHECK(approach.output().state == State::Approach);
  CHECK(approach.output().duty < Config().straightDuty);
  Controller leftTurn, rightTurn;
  beginTurn(leftTurn, true); beginTurn(rightTurn, false);
  std::puts("PASS approach, stationary confirmation, left and right turns");

  // A single clear reading and elapsed time alone cannot complete a turn.
  leftTurn.update(scan(200), 200);
  CHECK(leftTurn.output().state == State::TurnLeft);
  leftTurn.update(scan(250, 750, 350, 305), 250);
  leftTurn.update(scan(500), 500);
  CHECK(leftTurn.output().state == State::TurnLeft);
  for (uint32_t t = 550; t <= 650; t += 50) leftTurn.update(scan(t), t);
  CHECK(leftTurn.output().state == State::Settle);
  CHECK(leftTurn.output().turnsCompleted == 1);
  for (uint32_t t = 700; t <= 1150; t += 50) leftTurn.update(scan(t), t);
  CHECK(leftTurn.output().state == State::Follow);
  std::puts("PASS turn exits only after stable new-corridor samples and settles");

  Controller duplicate;
  CHECK(duplicate.start(scan(0), 0));
  Sample corner = scan(50, 750, 350, 305);
  for (uint32_t t = 50; t <= 150; t += 50) duplicate.update(corner, t);
  CHECK(duplicate.output().state == State::Approach);
  CHECK(duplicate.output().duty == 0);
  duplicate.update(corner, 351);
  faulted(duplicate, Fault::Stale);
  std::puts("PASS duplicate frames do not confirm a corner; missing updates stop");

  for (int variant = 0; variant < 4; ++variant) {
    Controller invalid;
    CHECK(invalid.start(scan(0), 0));
    Sample bad = scan(50);
    if (variant == 0) bad.leftValid = false;
    if (variant == 1) bad.frontMm = 0;
    if (variant == 2) bad.rightMm = std::numeric_limits<float>::quiet_NaN();
    if (variant == 3) bad.frontMm = 7000;
    invalid.update(bad, 50);
    faulted(invalid, Fault::InvalidRange);
    invalid.update(scan(100), 100);
    faulted(invalid, Fault::InvalidRange); // No automatic restart.
    CHECK(invalid.start(scan(150), 150));
  }
  Controller unavailable;
  CHECK(!unavailable.start(Sample(), 0));
  faulted(unavailable, Fault::Stale);
  std::puts("PASS invalid/no-echo/NaN/out-of-range/missing samples and latched faults");

  Controller emergency;
  CHECK(!emergency.start(scan(0, 305, 150, 305), 0));
  faulted(emergency, Fault::FrontClose);
  CHECK(!emergency.start(scan(0, 120, 750, 305), 0));
  faulted(emergency, Fault::SideClose);
  Controller turnEmergency;
  beginTurn(turnEmergency, true);
  turnEmergency.update(scan(200, 750, 140, 305), 200);
  faulted(turnEmergency, Fault::FrontClose);
  std::puts("PASS emergency clearance stops in straight and turn states");

  Controller lost, width;
  CHECK(!lost.start(scan(0, 750, 750, 750), 0));
  faulted(lost, Fault::NoWall);
  CHECK(!width.start(scan(0, 180, 750, 180), 0));
  faulted(width, Fault::WidthMismatch);
  for (int variant = 0; variant < 2; ++variant) {
    Controller ambiguous;
    CHECK(ambiguous.start(scan(0), 0));
    for (uint32_t t = 50; t <= 150; t += 50)
      ambiguous.update(scan(t, variant ? 305 : 750, 350, variant ? 305 : 750), t);
    faulted(ambiguous, Fault::CornerUnclear);
  }
  std::puts("PASS lost walls, inconsistent width, dead end and ambiguous junction stop");

  Controller timeout;
  beginTurn(timeout, false);
  for (uint32_t t = 200; t <= 4650; t += 50)
    timeout.update(scan(t, 305, 350, 750), t);
  faulted(timeout, Fault::TurnTimeout);
  timeout.stop();
  CHECK(timeout.output().state == State::Off && timeout.output().duty == 0);
  timeout.update(scan(4700), 4700);
  CHECK(timeout.output().state == State::Off);
  std::puts("PASS turn timeout and explicit stop");

  Controller wrap;
  CHECK(wrap.start(scan(UINT32_MAX - 25), UINT32_MAX - 25));
  wrap.update(scan(24), 24);
  CHECK(wrap.output().state == State::Follow);
  wrap.update(scan(24), 325);
  faulted(wrap, Fault::Stale);
  Config badConfig;
  badConfig.corridorMm = -1;
  Controller bad(badConfig);
  CHECK(!bad.start(scan(0), 0));
  faulted(bad, Fault::BadConfig);
  std::puts("PASS millis rollover and rejected invalid configuration");
  std::puts("All wall-follower scenario tests passed (synthetic inputs only).");
  if (argc > 1) replayCsv(argv[1]);
}
