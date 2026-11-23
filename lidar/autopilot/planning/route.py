"""Trace successive corridor cross sections in lidar coordinates (forward x, right y)."""

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass, field
import math
from statistics import median

from lidar.geometry.scan_geometry import GRID_M, ScanGeometry, cell


class RouteVisibilityHistory:
    """Keep raw sweeps only; a repaired cell is never evidence for a later repair."""

    def __init__(self):
        self.frames = deque(maxlen=2)

    def update(self, current, sensor_pose, stamp):
        if (current is None or sensor_pose is None
                or not all(map(math.isfinite, (stamp, sensor_pose.x, sensor_pose.y, sensor_pose.yaw)))):
            self.frames.clear()
            return current
        if self.frames:
            old_time, old_pose, _ = self.frames[-1]
            dt = stamp - old_time
            shift = math.hypot(sensor_pose.x - old_pose.x, sensor_pose.y - old_pose.y)
            turn = abs(math.remainder(sensor_pose.yaw - old_pose.yaw, math.tau))
            if (not 0 < dt <= 0.45 or shift > 0.75 * dt + 0.02
                    or turn > 2.8 * dt + math.radians(5)):
                self.frames.clear()
        old_views = []
        for old_time, old_pose, frame in self.frames:
            if stamp - old_time <= 0.45:
                x, y = old_pose.local(sensor_pose.x, sensor_pose.y)
                angle = sensor_pose.yaw - old_pose.yaw
                old_views.append((x, y, math.cos(angle), math.sin(angle), frame))
        self.frames.append((stamp, sensor_pose, current))
        return _RouteVisibility(current, old_views) if old_views else current


class _RouteVisibility(ScanGeometry):
    def __init__(self, current, old_views):
        super().__init__(current.samples, current.points, current.ray_segments, current.free,
                         current.occupied, current.wall_exclusions, current.rejected_far)
        self.old_views = old_views
        self.holes = {}
        self.max_origin = 0.0
        # Retain sample slots without a return so their neighbours can never
        # become an apparent pair of adjacent measured rays.
        segments = iter(current.ray_segments)
        rays = []
        for _, distance, quality in current.samples:
            if distance <= 0 or quality <= 0:
                rays.append(None)
                continue
            ox, oy, hx, hy = next(segments)
            dx, dy = hx - ox, hy - oy
            length = math.hypot(dx, dy)
            rays.append((ox, oy, dx / length, dy / length, length))
            self.max_origin = max(self.max_origin, math.hypot(ox, oy))
        neighbours = []
        for a, b in zip(rays, rays[1:] + rays[:1]):
            if a is None or b is None:
                continue
            angle = math.atan2(a[3], a[2])
            gap = (math.atan2(b[3], b[2]) - angle) % math.tau
            if 0 < gap <= math.radians(3):
                neighbours.append((angle, a, b))
        neighbours.sort()
        self.bearings = tuple(t + wrap for wrap in (-math.tau, 0, math.tau) for t, _, _ in neighbours)
        self.pairs = [(a, b) for _, a, b in neighbours] * 3

    def _small_unoccluded_hole(self, index):
        cx, cy = index
        neighbours = ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1))
        if (sum(p in self.free for p in neighbours) < 3
                or any(p in self.occupied for p in neighbours)):
            return False
        # Reject shadow behind a measured return, including a small post between walls.
        steps = 2 * max(abs(cx), abs(cy), 1)
        return not any((round(cx * i / steps), round(cy * i / steps)) in self.occupied
                       for i in range(1, steps))

    def visible(self, x, y):
        index = cell(x, y)
        if index in self.occupied:
            return False
        if index in self.free:
            return True
        if index not in self.holes:
            self.holes[index] = self._small_unoccluded_hole(index)
        if not self.holes[index]:
            return False
        if not any(frame.visible(ox + c * x - s * y, oy + s * x + c * y)
                   for ox, oy, c, s, frame in self.old_views):
            return False
        radius = math.hypot(x, y)
        if not 0.15 <= radius <= 2.6:
            return False
        # The two actual beams must straddle this point no more than one
        # cell apart. Each must be observed beyond it, from its own origin.
        angle = math.atan2(y, x)
        spread = math.asin(min(1.0, (self.max_origin + GRID_M) / radius))
        start, end = (bisect_left(self.bearings, angle + d) for d in (-spread, spread))
        for i in range(start, end):
            a, b = self.pairs[i]
            dx, dy = x - a[0], y - a[1]
            left = a[2] * dy - a[3] * dx
            if not 0 <= left <= GRID_M or not 0 <= a[2] * dx + a[3] * dy < a[4] - GRID_M:
                continue
            dx, dy = x - b[0], y - b[1]
            right = b[3] * dx - b[2] * dy
            if 0 <= right <= GRID_M - left and 0 <= b[2] * dx + b[3] * dy < b[4] - GRID_M:
                return True
        return False


@dataclass(frozen=True)
class ObservedRoute:
    points: tuple
    sensor_ahead: float
    wheelbase: float
    bilateral_length: float = 0.0
    distances: tuple = field(init=False, repr=False)

    def __post_init__(self):
        if len(self.points) < 2:
            raise ValueError("A route needs at least two distinct points")
        distances = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            step = math.dist(a, b)
            if not math.isfinite(step) or step <= 0:
                raise ValueError("Route points must be distinct, finite neighbours")
            distances.append(distances[-1] + step)
        object.__setattr__(self, "distances", tuple(distances))
        if not math.isfinite(self.bilateral_length) or self.bilateral_length < 0:
            raise ValueError("Observed two-wall length must be finite and nonnegative")
        object.__setattr__(self, "bilateral_length", min(distances[-1], self.bilateral_length))

    @classmethod
    def from_points(cls, points, config, *, bilateral_length=0.0):
        return cls(tuple(points), config.sensor_ahead, config.wheelbase, bilateral_length)

    @property
    def length(self):
        return self.distances[-1]

    def point_at(self, distance_m):
        i = bisect_left(self.distances, distance_m)
        if i == 0 or i == len(self.points):
            return self.points[0] if i == 0 else self.points[-1]
        fraction = (distance_m - self.distances[i - 1]) / (self.distances[i] - self.distances[i - 1])
        ax, ay = self.points[i - 1]
        bx, by = self.points[i]
        return ax + fraction * (bx - ax), ay + fraction * (by - ay)

    def steer_at(self, distance_m):
        """Pure pursuit to an arc-length lookahead, from the rear axle; right is positive."""
        x, y = self.point_at(distance_m)
        x += self.sensor_ahead
        return math.degrees(math.atan2(2 * self.wheelbase * y, x * x + y * y))


def _wall_edges(points, exclusions):
    """Join close returns on the same visible surface; leave depth jumps and angular gaps open."""
    ordered = sorted((math.atan2(y, x), x, y, math.hypot(x, y)) for x, y in points
                     if math.isfinite(x) and math.isfinite(y))
    edges = []
    for (angle, x, y, radius), (other, bx, by, other_radius) in zip(ordered, ordered[1:] + ordered[:1]):
        if min(radius, other_radius) > 3.5 or (x, y) in exclusions or (bx, by) in exclusions:
            continue
        gap = (other - angle) % math.tau
        if gap > math.radians(6) or abs(other_radius - radius) > min(0.28, 0.07 + 0.14 * radius):
            continue
        if gap > 0:
            edges.append((x, y, bx - x, by - y))
    return edges, ordered


def _section(edges, x, y, hx, hy):
    """Intersect a line perpendicular to the local road heading with measured surfaces."""
    left, right = -math.inf, math.inf
    for ax, ay, dx, dy in edges:
        along = dx * hx + dy * hy
        if abs(along) < 1e-8:
            continue
        ax -= x
        ay -= y
        t = -(ax * hx + ay * hy) / along
        if 0 <= t <= 1:
            side = -(ax + t * dx) * hy + (ay + t * dy) * hx
            if side >= 0:
                right = min(side, right)
            else:
                left = max(side, left)
    return left, right


def observed_route(points, config, *, geometry=None, width=None):
    """Return a guide through successive pairs of visible walls in the current sweep.

    This guides steering selection. Collision and held-command travel remain the
    responsibility of the footprint and command-lease checks.
    """
    exclusions = geometry.wall_exclusions if geometry is not None else ()
    edges, ordered = _wall_edges(points, exclusions)
    if len(edges) < 8:
        return None
    bearings = [beam[0] for beam in ordered]

    def visible(x, y):
        if geometry is not None:
            return geometry.visible(x, y)
        bearing = math.atan2(y, x)
        i = bisect_left(bearings, bearing)
        a, b = ordered[i - 1], ordered[i % len(ordered)]
        return ((b[0] - a[0]) % math.tau < math.radians(6)
                and min(a[3], b[3]) > math.hypot(x, y))

    step = 0.12
    min_width = config.width + 2 * config.margin
    centers = []
    widths = []
    bilateral_length = 0.0
    bilateral = True
    one_wall_steps = 0
    x = y = 0.0
    hx, hy = 1.0, 0.0
    for _ in range(16):
        cuts = [_section(edges, x + shift * hx, y + shift * hy, hx, hy)
                for shift in (-0.025, 0.0, 0.025)]
        pairs = [(l, r) for l, r in cuts if min_width <= r - l <= 2.4]
        if len(pairs) >= 2:
            left, right = (median(values) for values in zip(*pairs))
            span = right - left
            # Abrupt expansion often denotes a junction or a missing side wall.
            if width is not None and abs(span - width) > max(0.18, 0.34 * width):
                break
            offset = (right + left) / 2
            widths.append(span)
            width = span
            one_wall_steps = 0
        else:
            # Around a bend the inner wall disappears behind its tangent. Follow
            # the still-visible outer wall only as far as the centre is observed.
            if len(widths) < 2 or one_wall_steps >= 6:
                break
            half = median(widths[-3:]) / 2
            lefts = [l for l, _ in cuts if abs(l + half) <= max(0.085, half * 0.30)]
            rights = [r for _, r in cuts if abs(r - half) <= max(0.085, half * 0.30)]
            if (len(lefts) >= 2) == (len(rights) >= 2):
                break
            offset = median(lefts) + half if len(lefts) >= 2 else median(rights) - half
            one_wall_steps += 1
            bilateral = False
        cx, cy = x - offset * hy, y + offset * hx
        if centers:
            dx, dy = cx - centers[-1][0], cy - centers[-1][1]
            samples = math.ceil(math.hypot(dx, dy) / GRID_M)
            if not all(visible(centers[-1][0] + dx * i / samples, centers[-1][1] + dy * i / samples)
                       for i in range(1, samples + 1)):
                break
            direction = math.atan2(dy, dx)
            turn = math.remainder(direction - math.atan2(hy, hx), math.tau)
            if abs(turn) > math.radians(38):
                break
            hx, hy = math.cos(direction), math.sin(direction)
            if bilateral:
                bilateral_length += math.hypot(dx, dy)
        elif abs(offset) > GRID_M and not visible(cx, cy):
            break
        centers.append((cx, cy))
        x, y = cx + step * hx, cy + step * hy
    if len(centers) >= 4:
        return ObservedRoute.from_points(centers, config, bilateral_length=bilateral_length)
    return None
