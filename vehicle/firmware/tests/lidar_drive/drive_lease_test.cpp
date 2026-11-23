#include "lidar_drive/DriveLease.h"
#include "motor/Motor.h"

#include <cassert>
#include <cstdio>
#include <limits>

namespace {
int stops = 0;
}

void motor::stop() { ++stops; }

int main() {
  using namespace lidar_drive;

  arm(100);
  service(100 + kWatchdogMs - 1);
  assert(armed() && !expired() && stops == 0);
  service(100 + kWatchdogMs);
  assert(!armed() && expired() && stops == 1);
  service(100 + kWatchdogMs + 1);
  assert(stops == 1);

  cancel();
  const uint32_t nearWrap = std::numeric_limits<uint32_t>::max() - 8;
  arm(nearWrap - 1);
  renew(nearWrap, 12);
  service(2);
  assert(armed() && !expired() && stops == 1);
  service(3);
  assert(!armed() && expired() && stops == 2);

  cancel();
  arm(100);
  cancel();
  service(100 + kWatchdogMs);
  assert(!armed() && !expired() && stops == 2);

  std::puts("Drive lease tests passed.");
}
