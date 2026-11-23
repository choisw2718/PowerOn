#ifndef POWERON_AUTONOMOUS_STEERING_H
#define POWERON_AUTONOMOUS_STEERING_H

#include <math.h>
#include <stdint.h>

// Keep fractional degrees throughout; only Servo::writeMicroseconds rounds to
// integer us. With the existing nominal 10 us/degree this is ~0.1 wheel degree,
// not a guarantee about HS-311 mechanical resolution/backlash/deadband.
class AutonomousSteering {
 public:
  void reset(float angle, uint32_t now) { angle_ = angle; lastMs_ = now; }
  float angle() const { return angle_; }
  float update(float target, uint32_t now, bool manoeuvring) {
    // 50 Hz output, independently of the 60 ms distance scans. Cap a delayed
    // tick so a Wi-Fi stall does not produce a large catch-up steering jump.
    const uint32_t elapsed = now - lastMs_;
    const float seconds = (elapsed > 40 ? 40 : elapsed) * 0.001f;
    lastMs_ = now;
    const float rate = manoeuvring ? 90.0f : 20.0f;
    const float step = rate * seconds;
    const float error = target - angle_;
    angle_ += fmaxf(-step, fminf(step, error));
    return angle_;
  }
 private:
  float angle_ = 0;
  uint32_t lastMs_ = 0;
};
#endif
