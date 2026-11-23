"""Plan local Ackermann motion from one complete, clockwise RPLIDAR rotation."""

from dataclasses import dataclass, field
import math
from statistics import median

from hardware import PARAMETERS
from .route import ObservedRoute, RouteVisibilityHistory, observed_route

CHASSIS = PARAMETERS["chassis"]
LIDAR = PARAMETERS["lidar"]
CORNER_CLEARANCE_TARGET_M = 0.14
CORNER_CLEARANCE_WEIGHT = 16.0
PREFERRED_CLEARANCE_M = 0.10
ISOLATED_NEAR_RETURN = "isolated near return"


@dataclass(frozen=True)
class TrackConfig:
    forward_angle: float = LIDAR["forward_angle_deg"]
    wheelbase: float = CHASSIS["wheelbase_m"]
    sensor_ahead: float = LIDAR["ahead_of_rear_axle_m"]
    width: float = CHASSIS["width_m"]
    front: float = CHASSIS["front_from_lidar_m"]
    rear: float = CHASSIS["rear_from_lidar_m"]
    stop_distance: float = LIDAR["stop_distance_m"]
    pwm: float = LIDAR["preview_pwm_pct"]
    margin: float = LIDAR["body_margin_m"]
    min_turn_radius: float = CHASSIS["min_turn_radius_m"]  # Zero uses the steering limit alone.
    prefer_corner_clearance: bool = False
    follow_route: bool = False

    @property
    def ideal_turn_radius(self):
        limit_rad = math.radians(LIDAR["max_center_steer_deg"])
        return max(self.min_turn_radius, self.wheelbase / math.tan(limit_rad))


@dataclass(frozen=True)
class Decision:
    pwm: float = 0.0
    steer: float = 0.0  # Nano uses positive for left; the geometry below uses y-right.
    reason: str = "stop"
    clear_m: float = 0.0
    left_m: float | None = None
    right_m: float | None = None
    curvature: float = 0.0
    turn_in_m: float | None = None
    speed_scale: float = 1.0
    curvature_ahead: tuple[tuple[float, float], ...] = ()
    held_clear_m: float | None = None  # Observed clearance if the latest command is never renewed.


@dataclass
class CorridorMemory:
    """Keep a short-lived width estimate from repeated, measured wall pairs."""

    width: float | None = None
    observed_at: float = -math.inf
    candidate: float | None = None
    candidate_at: float = -math.inf
    confirmations: int = 0
    route_points: tuple = ()
    route_world: tuple = ()
    route_stamp: float = -math.inf
    bilateral_length: float = 0.0
    visibility: RouteVisibilityHistory = field(default_factory=RouteVisibilityHistory, repr=False)

    def update_route(self, points, config, geometry, pose, stamp):
        fresh = observed_route(points, config, geometry=geometry, width=self.width)
        self.bilateral_length = fresh.bilateral_length if fresh is not None else 0.0
        # Register the previous centreline with measured odometry. Its tail
        # guides steering through a brief occlusion; only this scan can clear a command.
        prior = ()
        if pose is not None and 0 <= stamp - self.route_stamp < 0.55:
            prior = tuple(pose.local(*point) for point in self.route_world)
        if fresh is not None:
            line = list(fresh.points)
            if prior:
                index = min(range(len(prior)), key=lambda i: math.dist(line[-1], prior[i]))
                dx, dy = line[-1][0] - line[-2][0], line[-1][1] - line[-2][1]
                if math.dist(line[-1], prior[index]) < 0.09:
                    for x, y in prior[index + 1:]:
                        if (x - line[-1][0]) * dx + (y - line[-1][1]) * dy <= 0:
                            break
                        line.append((x, y))
                        dx, dy = line[-1][0] - line[-2][0], line[-1][1] - line[-2][1]
            # Date only the newly measured prefix. Repeated short scans cannot
            # keep an unseen tail alive indefinitely.
            self.route_world = tuple(pose.world(*p) for p in fresh.points) if pose else ()
            self.route_stamp = stamp
        else:
            line = []
            if prior:
                index = min(range(len(prior)), key=lambda i: math.hypot(*prior[i]))
                if math.hypot(*prior[index]) < 0.22 and len(prior) - index >= 3:
                    line = prior[index:]
        self.route_points = tuple(line)
        return ObservedRoute.from_points(line, config) if len(line) >= 2 else None

    def observe(self, width, now):
        if now <= self.candidate_at:
            return
        if (self.candidate is None or now - self.candidate_at > 0.5
                or abs(width - self.candidate) > max(0.12, self.candidate * 0.18)):
            self.confirmations = 0
        self.candidate, self.candidate_at = width, now
        self.confirmations += 1
        if self.confirmations >= 3:
            self.width = width if self.width is None else 0.8 * self.width + 0.2 * width
            self.observed_at = now


@dataclass(frozen=True)
class Path:
    steer: float
    clear: float
    walls: list[tuple[float | None, float | None]]
    blocked: bool = False
    clearance_cost: float = 0.0
    isolated_block: bool = False
    near_clearance_m: float = math.inf


@dataclass(frozen=True)
class WallEvidence:
    pairs: list[tuple[float, float]]
    degraded: bool
    near_width: float | None = None


def footprint_path(points, rays, config, steer, horizon, geometry=None, motion=None, steering=None,
                   *, check_visibility=True, score_clearance=False, hold_turn=False):
    curvature = math.tan(math.radians(steer)) / config.wheelbase
    half_width = config.width / 2
    side_limit = half_width + config.margin
    front_limit = config.front + config.margin
    rear_limit = config.rear + config.margin
    step_count = math.ceil(horizon / 0.045)
    wall_sections = []
    clear = 0.0
    clearance_deficits = []
    rear_x, rear_y, yaw = -config.sensor_ahead, 0.0, 0.0
    distance_step = horizon / step_count
    margin_goal = CORNER_CLEARANCE_TARGET_M
    wall_exclusions = geometry.wall_exclusions if geometry is not None else frozenset()
    for index in range(step_count + 1):
        travel = horizon * index / step_count
        if steering is not None:
            if index:
                wheel_angle = steering.angle_at(steer, travel - distance_step / 2)
                turn = distance_step * math.tan(math.radians(wheel_angle)) / config.wheelbase
                next_yaw = yaw + turn if hold_turn else max(-math.pi / 2, min(math.pi / 2, yaw + turn))
                midpoint = (yaw + next_yaw) / 2
                rear_x += distance_step * math.cos(midpoint)
                rear_y += distance_step * math.sin(midpoint)
                yaw = next_yaw
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            x = rear_x + config.sensor_ahead * cos_yaw
            y = rear_y + config.sensor_ahead * sin_yaw
        elif curvature:
            arc = travel if hold_turn else min(travel, math.pi / (2 * abs(curvature)))
            yaw = curvature * arc
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            straight = travel - arc
            x = math.sin(yaw) / curvature + (straight + config.sensor_ahead) * cos_yaw - config.sensor_ahead
            y = (1 - cos_yaw) / curvature + (straight + config.sensor_ahead) * sin_yaw
        else:
            x, y, cos_yaw, sin_yaw = travel, 0.0, 1.0, 0.0
        # Require visibility of the nose and both corners of the inflated bumper.
        nose_x = x + front_limit * cos_yaw
        nose_y = y + front_limit * sin_yaw
        if check_visibility and not bumper_visible(
                nose_x, nose_y, cos_yaw, sin_yaw, side_limit, rays, geometry, motion):
            break
        padding = 0.0
        if motion is not None:
            body_radius = math.hypot(max(front_limit, rear_limit), side_limit)
            padding = motion.padding_at(math.hypot(x, y) + body_radius)
        padded_side = side_limit + padding
        padded_front = front_limit + padding
        padded_rear = rear_limit + padding
        left, right = [], []
        nearest = margin_goal ** 2
        # Score the near swept body after the common initial pose, where a steering change can help.
        score_section = score_clearance and 0.08 <= travel <= 0.50
        isolated_block = False
        for point_x, point_y in points:
            dx, dy = point_x - x, point_y - y
            ahead = dx * cos_yaw + dy * sin_yaw
            side = -dx * sin_yaw + dy * cos_yaw
            if -padded_rear <= ahead <= padded_front and abs(side) <= padded_side:
                if wall_exclusions and (point_x, point_y) in wall_exclusions:
                    isolated_block = True
                    continue
                return Path(steer, clear, wall_sections, blocked=True,
                            clearance_cost=clearance_cost(clearance_deficits),
                            near_clearance_m=margin_goal - max(clearance_deficits, default=0.0))
            if (score_section and abs(side) < half_width + margin_goal
                    and -config.rear - margin_goal < ahead < config.front + margin_goal):
                dx_body = max(0.0, ahead - config.front, -config.rear - ahead)
                dy_body = max(0.0, abs(side) - half_width)
                nearest = min(nearest, dx_body ** 2 + dy_body ** 2)
            if (index and abs(ahead) <= 0.09 and abs(side) <= 3.0
                    and (not wall_exclusions or (point_x, point_y) not in wall_exclusions)):
                (right if side > 0 else left).append(abs(side))
        if isolated_block:
            return Path(steer, clear, wall_sections, blocked=True,
                        clearance_cost=clearance_cost(clearance_deficits), isolated_block=True,
                        near_clearance_m=margin_goal - max(clearance_deficits, default=0.0))
        clear = travel
        if score_section:
            clearance_deficits.append(margin_goal - math.sqrt(nearest))
        if index == 0:
            continue
        # Use the closest supported surface on each side of this section.
        wall_sections.append((median(sorted(left)[:5]) if len(left) >= 3 else None,
                              median(sorted(right)[:5]) if len(right) >= 3 else None))
    return Path(steer, clear, wall_sections, clearance_cost=clearance_cost(clearance_deficits),
                near_clearance_m=margin_goal - max(clearance_deficits, default=0.0))


def clearance_cost(deficits):
    return (max(deficits) + sum(deficits) / len(deficits)) / 2 if deficits else 0.0


def bumper_visible(x, y, cosine, sine, half_width, rays, geometry=None, motion=None):
    for offset in (-half_width, 0.0, half_width):
        probe_x, probe_y = x - offset * sine, y + offset * cosine
        if geometry is not None:
            if not geometry.visible(probe_x, probe_y):
                return False
            continue
        bearing = math.degrees(math.atan2(probe_y, probe_x))
        distance = math.hypot(probe_x, probe_y)
        if motion is None:
            if rays[int(bearing // 5) % 72] < distance:
                return False
        else:
            spread = motion.angular_padding_deg(distance)
            first = math.floor((bearing - spread) / 5)
            last = math.floor((bearing + spread) / 5)
            if any(rays[index % 72] < distance + motion.translation_m
                   for index in range(first, last + 1)):
                return False
    return True


def wall_evidence(path, width, can_infer):
    needed = max(3, math.ceil(len(path.walls) * 0.55))
    maximum = min(3.0, width + max(0.22, 0.30 * width)) if width is not None else 3.0
    measured = [pair if None not in pair and sum(pair) <= maximum else None
                for pair in path.walls]
    pairs = [pair for pair in measured if pair is not None]
    near = [sum(pair) for pair in measured[:5] if pair is not None]
    near_width = median(near) if len(near) >= 3 and max(near) - min(near) <= 0.18 else None
    # Width just ahead of the car can stay well measured while a corner hides
    # the far wall. Only refresh from physical pairs, with stronger near support
    # when some of the future path depends on inference.
    measured_near_width = (near_width if len(measured) >= 4
                           and all(pair is not None for pair in measured[:4]) else None)
    if len(pairs) >= needed:
        weak = width is not None and len(pairs) < math.ceil(len(measured) * 0.8)
        return WallEvidence(pairs, weak, measured_near_width if weak else near_width)
    if not can_infer:
        return None

    # Infer the opposite side only from one contiguous wall anchored near the car.
    tolerance = max(0.18, width * 0.27)
    best = []
    for side in (0, 1):
        continuous = []
        previous = None
        for index, section in enumerate(path.walls):
            distance, other = section[side], section[1 - side]
            anchored = distance is not None and abs(distance - width / 2) <= tolerance
            other_plausible = other is not None and abs(other - width / 2) <= tolerance
            usable = anchored and (measured[index] is not None or not other_plausible)
            if not usable or (previous is not None and abs(distance - previous) > 0.13):
                continuous, previous = [], None
                continue
            if not continuous and index >= 5:
                break
            pair = measured[index]
            if pair is None:
                pair = (distance, width - distance) if side == 0 else (width - distance, distance)
            continuous.append(pair)
            previous = distance
            if len(continuous) > len(best):
                best = continuous.copy()
    return WallEvidence(best, True, measured_near_width) if len(best) >= needed else None


def near_wall_evidence(path, width, required_clear):
    """Use a short bilateral corridor to steer through a briefly occluded bend."""
    look = min(path.clear, max(0.44, required_clear))
    count = math.ceil(len(path.walls) * look / path.clear)
    prefix = path.walls[:count]
    tolerance = max(0.14, 0.22 * width)
    measured = [pair if None not in pair and abs(sum(pair) - width) <= tolerance else None
                for pair in prefix]
    pairs = [pair for pair in measured if pair is not None]
    if len(pairs) < max(4, math.ceil(0.65 * count)):
        return None
    if sum(pair is not None for pair in measured[:5]) < 4:
        return None
    widths = [sum(pair) for pair in pairs]
    if max(widths) - min(widths) > tolerance:
        return None
    return WallEvidence(pairs, True, median(sum(pair) for pair in measured[:5] if pair is not None))


def corner_exit_evidence(path, width, required_clear):
    """Require both walls beyond an occluded inside corner before following it."""
    if abs(path.steer) < 10 or path.clear < max(0.58, required_clear + 0.15):
        return None
    tolerance = max(0.14, 0.22 * width)
    step = path.clear / len(path.walls)
    needed = max(5, math.ceil(0.19 / step))
    run = []
    for index, pair in enumerate(path.walls, start=1):
        if index * step < 0.18 or None in pair or abs(sum(pair) - width) > tolerance:
            run.clear()
            continue
        run.append(pair)
        if len(run) > needed:
            run.pop(0)
        if len(run) == needed and index * step >= 0.43:
            widths = [sum(section) for section in run]
            if max(widths) - min(widths) <= tolerance:
                return WallEvidence(run, True)
    return None


def angled_wall_evidence(path, width):
    """Accept wider diagonal sections only when two measured walls agree on their heading."""
    step = path.clear / len(path.walls)
    tolerance = max(0.14, 0.22 * width)
    for start in range(min(3, len(path.walls) - 4)):
        pairs = path.walls[start:start + 5]
        if any(None in pair for pair in pairs):
            continue
        widths = [sum(pair) for pair in pairs]
        observed = median(widths)
        if observed <= width + tolerance or max(widths) - min(widths) > 0.14:
            continue
        left_slope = (pairs[0][0] - pairs[-1][0]) / (4 * step)
        right_slope = (pairs[-1][1] - pairs[0][1]) / (4 * step)
        slope = (left_slope + right_slope) / 2
        if (left_slope * right_slope > 0 and abs(left_slope - right_slope) <= 0.45
                and 0.32 <= abs(slope) <= 1.2
                and abs(observed / math.hypot(1.0, slope) - width) <= tolerance):
            return WallEvidence(pairs, True)
    return None


def plan(samples, config=TrackConfig(), previous_steer=0.0, *, memory=None, now=0.0,
         geometry=None, lookahead=0.76, motion=None, steering=None, command_distance=0.0,
         sensor_pose=None, cruise_horizon=0.0):
    """Return forward PWM and steering, or a zero-PWM decision with a reason."""
    if memory is not None:
        memory.route_points = ()
    if config.follow_route:
        if memory is None:
            memory = CorridorMemory()
        geometry = memory.visibility.update(geometry, sensor_pose, now)
    observed = set()
    rays = [math.inf] * 72
    points = []
    for angle, distance_mm, quality in samples:
        if not math.isfinite(angle) or not 0 <= angle < 360:
            continue
        relative = angle if geometry is not None else (angle - config.forward_angle) % 360
        observed.add(int(relative // 10))
        if quality <= 0 or not math.isfinite(distance_mm) or distance_mm <= 0:
            continue
        distance = distance_mm / 1000
        bin_index = int(relative // 5)
        rays[bin_index] = min(rays[bin_index], distance)
        if geometry is None and distance <= 6.0:
            radians = math.radians(relative)
            points.append((distance * math.cos(radians), distance * math.sin(radians)))
    rays = [0.0 if math.isinf(value) else value for value in rays]
    if geometry is not None:
        points = geometry.points
    if len(observed) < 32 or len(points) < 50:
        return Decision(reason="incomplete scan")
    if geometry is not None:
        front_coverage = len({math.floor(math.degrees(math.atan2(y, x)) / 5)
                              for x, y in points if x > 0 and -x <= y < x})
    else:
        front_coverage = sum(value > 0 for value in rays[:9] + rays[-9:])
    if front_coverage < 10:
        return Decision(reason="insufficient forward returns")

    if steering is not None:
        held_distance = steering.speed_mps * (steering.delay_s + 0.06)
        held = footprint_path(points, rays, config, steering.current, held_distance, geometry, motion)
        if held.clear < held_distance - 1e-9:
            if held.isolated_block:
                reason = ISOLATED_NEAR_RETURN
            else:
                reason = "held steering is unsafe" if held.blocked else "held steering unobserved"
            return Decision(reason=reason, clear_m=held.clear)

    horizon = max(lookahead, config.stop_distance + 0.24)
    if config.follow_route:
        horizon = max(horizon, command_distance + 0.05)
    angle_limit = math.degrees(math.atan(config.wheelbase / config.ideal_turn_radius))
    angles = [angle_limit * i / 14 for i in range(-14, 15)]
    guide = None
    target_steer = 0.0
    if config.follow_route:
        guide = memory.update_route(points, config, geometry, sensor_pose, now)
        if guide is not None:
            pursuit = 0.30 + 0.22 * (steering.speed_mps if steering is not None else 0.0)
            angle = median(guide.steer_at(pursuit + shift) for shift in (-0.05, 0.0, 0.05))
            target_steer = max(-angle_limit, min(angle_limit, angle))
            # Keep straight at the middle index for the junction test below.
            angles.append(target_steer)
    paths = [footprint_path(points, rays, config, steer, horizon, geometry, motion, steering,
                            score_clearance=config.prefer_corner_clearance)
             for steer in angles]
    required_clear = config.stop_distance + 0.055
    straight = paths[angles.index(0.0)]
    width = memory.width if memory is not None else None
    width_age = now - memory.observed_at if width is not None else math.inf
    # Rule out an ambiguous branch using observed space before wall scoring can favor a side.
    turn_side = 0
    left_clear = max((path.clear for path in paths if path.steer <= -10), default=0.0)
    right_clear = max((path.clear for path in paths if path.steer >= 10), default=0.0)
    branch_entry = max(required_clear, straight.clear + 0.12)
    if min(left_clear, right_clear) >= branch_entry and abs(left_clear - right_clear) <= 0.12:
        # A missing ray can shorten the straight path. A junction also needs
        # measured returns across the straight bumper's stopping corridor.
        bumper = config.front + config.margin
        half_width = config.width / 2 + config.margin
        forward_wall = any(bumper <= x <= bumper + straight.clear + 0.12 and abs(y) <= half_width
                           for x, y in points)
        if forward_wall:
            # Both sides can have room for the nose inside a single corner.
            # Check that a branch continues far enough for the whole car.
            branch_horizon = max(0.95, horizon)
            open_sides = set()
            for path in paths:
                if abs(path.steer) < 10 or path.clear < branch_entry:
                    continue
                side = -1 if path.steer < 0 else 1
                if side in open_sides:
                    continue
                longer = (path if horizon >= branch_horizon else
                          footprint_path(points, rays, config, path.steer, branch_horizon,
                                          geometry, motion, steering))
                if longer.clear >= branch_horizon:
                    open_sides.add(side)
            if not open_sides and 0 <= width_age <= 2.0:
                # A real corner can have both short arcs open, yet only one
                # measured outlet with a car-width corridor behind it.
                open_sides = {1 if path.steer > 0 else -1 for path in paths
                              if path.clear >= branch_entry
                              and corner_exit_evidence(path, width, required_clear) is not None}
            if len(open_sides) != 1:
                return Decision(reason="ambiguous turn", clear_m=straight.clear)
            turn_side = open_sides.pop()

    can_infer = 0 <= width_age <= 1.2
    passable = [path for path in paths if path.clear >= required_clear
                and (not turn_side or path.steer * turn_side >= 10)]
    # A short, well-centered path can have too little clearance for even one
    # command. Look for a supported alternative that the drive gate can issue.
    held_distances = {}
    def held_distance(path):
        if path.steer not in held_distances:
            max_steer = abs(path.steer)
            if steering is not None:
                max_steer = max(max_steer, abs(steering.current), abs(steering.wheel_target(path.steer)),
                                abs(steering.wheel_target(steering.previous)))
            max_curve = math.tan(math.radians(max_steer)) / config.wheelbase
            held_distances[path.steer] = (
                footprint_path(points, rays, config, path.steer, path.clear, geometry, motion, steering,
                               hold_turn=True).clear if max_curve * path.clear > math.pi / 2 else path.clear)
        return held_distances[path.steer]
    if command_distance > required_clear:
        authorized = [path for path in passable if path.clear >= command_distance
                      and held_distance(path) >= command_distance]
        if authorized:
            passable = authorized
    candidates = [(path, evidence) for path in passable
                  if (evidence := wall_evidence(path, width, can_infer)) is not None]
    # A curved corridor can have good walls along its centreline even where a
    # single unchanging arc loses a distant wall. Check the nearby measured pair.
    if guide is not None and guide.length >= 0.46 and width is not None:
        supported = {p.steer for p, _ in candidates}
        for path in passable:
            if path.steer not in supported:
                near = near_wall_evidence(path, width, required_clear)
                if near is not None:
                    candidates.append((path, near))
    recovery_reason = None
    if not candidates and can_infer:
        candidates = [(path, evidence) for path in passable
                      if (evidence := near_wall_evidence(path, width, required_clear)) is not None]
        if candidates:
            recovery_reason = "confirmed near corridor"
    if not candidates and 0 <= width_age <= 2.0:
        exits = [(path, evidence) for path in passable
                 if (evidence := corner_exit_evidence(path, width, required_clear)) is not None]
        if exits and any(path.steer > 0 for path, _ in exits) and any(path.steer < 0 for path, _ in exits):
            return Decision(reason="ambiguous turn", clear_m=straight.clear)
        candidates = exits
        if candidates:
            recovery_reason = "confirmed corner exit"
    if not candidates and width is not None:
        candidates = [(path, evidence) for path in passable
                      if (evidence := angled_wall_evidence(path, width)) is not None]
        if candidates:
            recovery_reason = "confirmed angled corridor"
    if not candidates:
        longest = max(path.clear for path in paths)
        reason = ("insufficient clear distance" if longest < required_clear
                  else "insufficient wall support")
        return Decision(reason=reason, clear_m=longest)

    previous_right = -previous_steer
    if config.prefer_corner_clearance and command_distance and guide is None:
        # Prefer a viable swept path that keeps the body 10 cm from measured
        # walls. If none does, favor the largest clearance that is still observed.
        available = max(path.near_clearance_m for path, _ in candidates)
        minimum = min(available, PREFERRED_CLEARANCE_M)
        candidates = [(path, walls) for path, walls in candidates if path.near_clearance_m >= minimum]

    def score(candidate):
        path, walls = candidate
        center_error = sum(abs(left - right) for left, right in walls.pairs) / (2 * len(walls.pairs))
        missing = 1 - len(walls.pairs) / len(path.walls)
        change_penalty = 0.015 if walls.degraded else 0.002
        progress = path.clear
        if config.prefer_corner_clearance and command_distance:
            # Beyond the distance needed for the next command, extra visibility
            # must not draw the car across the inside of a narrowing bend.
            surplus = max(0.0, path.clear - command_distance)
            progress -= 0.80 * surplus
        if guide is not None:
            # Track the changing corridor heading. All candidates already passed
            # the current scan's footprint and visibility checks.
            return (-abs(path.steer - target_steer) - 7.0 * path.clearance_cost
                    - 0.015 * abs(path.steer - previous_right))
        return (2.0 * progress - 3.2 * center_error - 0.25 * missing
                - CORNER_CLEARANCE_WEIGHT * path.clearance_cost
                - (0.30 if walls.degraded else 0)
                - 0.002 * abs(path.steer) - change_penalty * abs(path.steer - previous_right))

    best, walls = max(candidates, key=score)
    if memory is not None and walls.near_width is not None:
        memory.observe(walls.near_width, now)
    left_m, right_m = walls.pairs[-1]
    turn_in = max(0.0, straight.clear - 0.10) if best.clear > straight.clear + 0.045 else None
    curvature = abs(math.tan(math.radians(best.steer)) / config.wheelbase)
    preview = ()
    if steering is not None:
        steps = max(1, math.ceil(best.clear / 0.045))
        distances = (best.clear * index / steps for index in range(steps + 1))
        preview = tuple((distance, abs(math.tan(math.radians(steering.angle_at(best.steer, distance))))
                         / config.wheelbase) for distance in distances)
    limited = walls.degraded or bool(turn_side)
    reason = recovery_reason or ("limited wall support" if limited else "follow walls")
    clear = best.clear
    held = held_distance(best)
    if guide is not None:
        reason = "follow corridor route"
        if not turn_side and memory.bilateral_length >= min(guide.length, max(0.55, command_distance)):
            limited = False
        if not limited and abs(best.steer) < 4 and cruise_horizon > clear:
            extended = footprint_path(points, rays, config, best.steer, cruise_horizon, geometry, motion,
                                      steering, hold_turn=True)
            clear = held = max(held, extended.clear)
        # Preview the actual change of centreline direction ahead. Distant
        # bends affect the target via braking distance, not an instant speed cap.
        ahead = []
        for index in range(2, len(guide.points) - 2):
            a, b, c = guide.points[index - 2], guide.points[index], guide.points[index + 2]
            ab, bc, ac = math.dist(a, b), math.dist(b, c), math.dist(a, c)
            if ab * bc * ac > 1e-6:
                cross = (b[0]-a[0]) * (c[1]-b[1]) - (b[1]-a[1]) * (c[0]-b[0])
                ahead.append((guide.distances[index], 2 * abs(cross) / (ab * bc * ac)))
        preview += tuple(ahead)
    return Decision(config.pwm, -best.steer, reason, clear, left_m,
                    right_m, curvature, turn_in, 0.40 if limited else 1.0, preview, held)
