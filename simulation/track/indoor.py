"""Seeded indoor test conditions. Values are simulation assumptions."""

from dataclasses import dataclass
import random

from .conditions import GripZone


ENVIRONMENTS = ("nominal", "split-grip", "weak-returns", "rough-floor", "indoor-mixed", "blocked",
                "encoder-outage", "motor-stall")


@dataclass(frozen=True)
class IndoorEnvironment:
    name: str
    start_offset: float = 0.0
    start_yaw: float = 0.0  # radians
    left_motor_gain: float = 1.0
    right_motor_gain: float = 1.0
    right_servo_trim: float = 0.0
    right_servo_rate_scale: float = 1.0
    supply_scale: float = 1.0
    load_scale: float = 1.0
    right_rollout_scale: float = 1.0
    grip_zones: tuple = ()
    weak_panel: tuple | None = None  # wall hit x interval
    obstacles: tuple = ()  # rectangles in world coordinates
    uneven_floor: bool = False
    encoder_outage_s: float | None = None
    stall_right_s: float | None = None

    def reflectivity(self, hit_x, hit_y):
        if self.weak_panel and self.weak_panel[0] <= hit_x <= self.weak_panel[1] and hit_y < 0:
            return 0.32
        return 1.0

    def grip_at(self, x, y):
        return min((zone.scale for zone in self.grip_zones if zone.contains(x, y)), default=1.0)

    def grade_at(self, x):
        # A shallow ramp with a raised threshold in the positive-x direction.
        if self.uneven_floor and 1.2 <= x < 1.95:
            return 0.065 if x < 1.25 else 0.018
        return 0.0

    def pitch_at(self, x):
        if self.uneven_floor and 1.25 <= x < 1.32:
            return 0.10  # nose dips after the front wheels roll off the lip
        return -self.grade_at(x)


def indoor_environment(name, seed):
    if name not in ENVIRONMENTS:
        raise ValueError(f"Unknown indoor environment: {name}")
    rng = random.Random(seed)
    grip = ()
    panel = None
    if name in ("split-grip", "indoor-mixed"):
        # Offset strip on the approach loads the two driven tires differently.
        edge = rng.uniform(-0.03, 0.03)
        grip = (GripZone(0.65, 2.15, -0.60, edge, rng.uniform(0.55, 0.75)),)
    if name in ("weak-returns", "indoor-mixed"):
        start = rng.uniform(0.65, 1.1)
        panel = (start, start + rng.uniform(0.6, 1.0))
    if name == "indoor-mixed":
        return IndoorEnvironment(name, start_offset=rng.uniform(-0.04, 0.04),
                                 start_yaw=rng.uniform(-0.035, 0.035),
                                 left_motor_gain=rng.uniform(0.95, 1.01),
                                 right_motor_gain=rng.uniform(0.94, 1.02),
                                 right_servo_trim=rng.uniform(-1.5, 1.5),
                                 right_servo_rate_scale=rng.uniform(0.75, 0.95),
                                 supply_scale=rng.uniform(0.90, 0.98),
                                 load_scale=rng.uniform(1.04, 1.20),
                                 right_rollout_scale=rng.uniform(0.985, 1.015),
                                 grip_zones=grip, weak_panel=panel, uneven_floor=True)
    obstacles = (((1.6, -0.7), (1.70, -0.7), (1.70, 0.7), (1.60, 0.7)),) if name == "blocked" else ()
    return IndoorEnvironment(name, grip_zones=grip, weak_panel=panel, obstacles=obstacles,
                             uneven_floor=name == "rough-floor",
                             encoder_outage_s=4.0 if name == "encoder-outage" else None,
                             stall_right_s=4.0 if name == "motor-stall" else None)
