"""Deterministic timing faults and local grip for simulator experiments."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FaultWindow:
    kind: str
    start_ms: int
    end_ms: int
    value: int = 1


@dataclass(frozen=True)
class GripZone:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    scale: float

    def contains(self, x, y):
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass(frozen=True)
class StressSettings:
    faults: tuple[FaultWindow, ...] = ()
    grip_zones: tuple[GripZone, ...] = ()
    combined_grip: bool = False
    scan_max_delivery_ms: int | None = None

    def fault(self, kind, stamp):
        return max((window.value for window in self.faults
                    if window.kind == kind and window.start_ms <= stamp < window.end_ms), default=0)

    def grip_at(self, x, y):
        return min((zone.scale for zone in self.grip_zones if zone.contains(x, y)), default=1.0)


def contact_ratio(longitudinal, lateral, longitudinal_limit, lateral_limit, grip):
    """Normalized demand in a simple longitudinal/lateral friction ellipse."""
    return math.hypot(longitudinal / longitudinal_limit, lateral / lateral_limit) / grip


SENSOR_FAULTS = (
    FaultWindow("front-gap", 4_650, 4_820),
    FaultWindow("scan-drop", 23_000, 23_220),
    FaultWindow("front-gap", 56_400, 57_350),
)
TRANSPORT_FAULTS = (
    FaultWindow("scan-delay", 4_650, 4_820, 135),
    FaultWindow("command-drop", 29_600, 29_770),
    FaultWindow("command-delay", 54_100, 54_250, 100),
)
LOW_GRIP_CORNERS = (
    GripZone(8.65, 10.60, -0.60, 1.60, 0.68),
    GripZone(2.50, 3.60, 5.25, 6.95, 0.78),
)
STRESS_PROFILES = {
    "clean": StressSettings(),
    "sensor": StressSettings(faults=SENSOR_FAULTS),
    "transport": StressSettings(faults=TRANSPORT_FAULTS, scan_max_delivery_ms=120),
    "traction": StressSettings(grip_zones=LOW_GRIP_CORNERS, combined_grip=True),
    "mixed": StressSettings(faults=SENSOR_FAULTS + TRANSPORT_FAULTS, grip_zones=LOW_GRIP_CORNERS,
                            combined_grip=True, scan_max_delivery_ms=120),
    "watchdog": StressSettings(faults=(FaultWindow("command-drop", 4_000, 4_850),)),
    "sensor-outage": StressSettings(faults=(FaultWindow("scan-drop", 4_000, 20_000),)),
}
