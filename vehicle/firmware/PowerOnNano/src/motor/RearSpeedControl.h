#pragma once

#include <math.h>
#include <stdint.h>

namespace rear_speed {

inline float clamp(float value, float minimum, float maximum) {
  return fminf(fmaxf(value, minimum), maximum);
}

struct Targets { float left; float right; };

inline Targets targets(float centerMps, float steerDeg, float wheelbaseM,
                       float rearTrackM, float maxWheelMps) {
  // Positive steering is left. The rear axle midpoint travels on radius L/tan(delta).
  const float halfTrackCurvature =
      0.5f * rearTrackM * tanf(steerDeg * 0.01745329252f) / wheelbaseM;
  Targets result{centerMps * (1.0f - halfTrackCurvature),
                 centerMps * (1.0f + halfTrackCurvature)};
  const float peak = fmaxf(fabsf(result.left), fabsf(result.right));
  if (peak > maxWheelMps) {
    result.left *= maxWheelMps / peak;
    result.right *= maxWheelMps / peak;
  }
  return result;
}

inline int8_t quadratureStep(uint8_t previous, uint8_t next) {
  constexpr int8_t steps[] = {0, 1, -1, 0, -1, 0, 0, 1,
                              1, 0, 0, -1, 0, -1, 1, 0};
  return steps[4 * previous + next];
}

struct SpeedPI {
  float integral = 0.0f;

  void reset() { integral = 0.0f; }

  float voltage(float target, float measured, float dt, float feedForward,
                float kp, float ki, float maxVoltage) {
    const float error = target - measured;
    const float base = feedForward * target + kp * error;
    const float candidate = clamp(integral + ki * error * dt, -maxVoltage, maxVoltage);
    // Block integration further into saturation. Zero output lets the wheel coast.
    if ((base + candidate <= maxVoltage || error < 0.0f) &&
        (base + candidate >= 0.0f || error > 0.0f)) {
      integral = candidate;
    }
    return clamp(base + integral, 0, maxVoltage);
  }
};

} // namespace rear_speed
