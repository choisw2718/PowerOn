#include "motor/RearSpeedControl.h"

#include <cassert>
#include <cmath>

int main() {
  using rear_speed::targets;
  const auto straight = targets(0.2f, 0, 0.2f, 0.16f, 0.5f);
  assert(straight.left == 0.2f && straight.right == 0.2f);
  const auto left = targets(0.2f, 25, 0.2f, 0.16f, 0.5f);
  const auto right = targets(0.2f, -25, 0.2f, 0.16f, 0.5f);
  assert(left.left < 0.2f && left.right > 0.2f);
  assert(left.left == right.right && left.right == right.left);
  assert(fabsf((left.left + left.right) / 2 - 0.2f) < 0.00001f);
  const auto reverse = targets(-0.2f, 25, 0.2f, 0.16f, 0.5f);
  assert(reverse.left == -left.left && reverse.right == -left.right);
  const auto limited = targets(0.2f, 25, 0.2f, 0.16f, 0.2f);
  assert(fabsf(limited.right - 0.2f) < 0.00001f);
  assert(fabsf(limited.left / limited.right - left.left / left.right) < 0.00001f);

  using rear_speed::quadratureStep;
  assert(quadratureStep(0, 1) + quadratureStep(1, 3) +
         quadratureStep(3, 2) + quadratureStep(2, 0) == 4);
  assert(quadratureStep(0, 2) + quadratureStep(2, 3) +
         quadratureStep(3, 1) + quadratureStep(1, 0) == -4);
  assert(quadratureStep(0, 3) == 0 && quadratureStep(2, 2) == 0);
  assert(quadratureStep(0, 1) + quadratureStep(1, 0) == 0);

  rear_speed::SpeedPI slow, fast;
  assert(slow.voltage(0.2f, 0.1f, 0.01f, 4, 8, 2, 6) >
         fast.voltage(0.2f, 0.2f, 0.01f, 4, 8, 2, 6));
  for (int sample = 0; sample < 100; ++sample) {
    assert(slow.voltage(0.5f, 0.0f, 0.01f, 10, 10, 20, 3) == 3.0f);
  }
  assert(slow.integral < 0.01f);
  assert(slow.voltage(0.2f, 0.5f, 0.01f, 4, 8, 2, 6) == 0.0f);
  slow.reset();
  assert(slow.integral == 0);
}
