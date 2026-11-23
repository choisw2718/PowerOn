"""Planar lidar calibration, pose interpolation and moving-scan geometry."""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math
from statistics import median
import time


def corrected_range(distance_mm, quality, offset_mm=0.0):
    if quality <= 0 or not math.isfinite(distance_mm) or distance_mm <= 0:
        return 0.0
    return max(0.0, distance_mm - offset_mm)


def estimate_ray_times(samples, start, end):
    """Approximate per-ray times from clockwise phase between rotation markers."""
    if not samples:
        return []
    first = samples[0][0]
    phase = 0.0
    times = []
    for angle, _, _ in samples:
        advance = (angle - first) % 360
        if advance > 355 and phase < 5:
            advance = 0.0  # Small backward jitter at the first marker.
        phase = max(phase, advance)
        times.append(start + (end - start) * phase / 360)
    return times


def sample_clock_times(count, start, end):
    """Use the ordered standard-scan samples between two rotation markers."""
    return tuple(start + (end - start) * (index / count) for index in range(count))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    steer_deg: float | None = None

    def interpolate(self, other, fraction):
        turn = math.remainder(other.yaw - self.yaw, math.tau)
        steer = self.steer_deg
        if steer is not None and other.steer_deg is not None:
            steer += fraction * (other.steer_deg - steer)
        return Pose(self.x + fraction * (other.x - self.x),
                    self.y + fraction * (other.y - self.y),
                    self.yaw + fraction * turn, steer)

    def local(self, x, y):
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def world(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y


class PoseUnavailable(RuntimeError):
    """No trustworthy pose interpolation is possible for this sweep."""


class PoseLog:
    """Read timestamped rear-axle poses from a replay or append-only local CSV."""

    def __init__(self, file, *, live=False, on_sample=None):
        self.file = file
        self.live = live
        self.on_sample = on_sample
        self.times = []
        self.poses = []
        self.read_header(file.readline().strip())

    def read_header(self, header):
        base_header = "timestamp_s,x_m,y_m,yaw_deg"
        if header not in (base_header, base_header + ",steer_deg"):
            raise ValueError("Pose CSV needs timestamp_s,x_m,y_m,yaw_deg[,steer_deg]")
        self.steering = header.endswith(",steer_deg")

    def parse_row(self, line):
        values = [float(value) for value in line.split(",")]
        if len(values) != (5 if self.steering else 4) or not all(map(math.isfinite, values)):
            raise ValueError("Invalid row in pose CSV")
        stamp, x, y, yaw = values[:4]
        steer = values[4] if self.steering else None
        if steer is not None and abs(steer) > 45:
            raise ValueError("Measured steering angle must be within ±45 degrees")
        return stamp, Pose(x, y, math.radians(yaw), steer)

    def window(self, start, end, *, wait_until=0.0):
        if not math.isfinite(start) or not math.isfinite(end) or end <= start:
            raise PoseUnavailable("invalid lidar scan time")
        while not self.times or self.times[-1] < end:
            position = self.file.tell()
            line = self.file.readline(257)
            if len(line) > 256:
                raise ValueError("Pose CSV row is too long")
            if not line or self.live and not line.endswith("\n"):
                self.file.seek(position)
                remaining = wait_until - time.monotonic()
                if not self.live or remaining <= 0:
                    raise PoseUnavailable("poses do not cover the end of the scan")
                time.sleep(min(0.003, remaining))
                continue
            stamp, pose = self.parse_row(line)
            if self.times and stamp <= self.times[-1]:
                raise ValueError("Pose CSV timestamps must increase strictly")
            if self.on_sample is not None:
                self.on_sample(stamp, pose, line.rstrip())
            if stamp <= start:
                self.times.clear()
                self.poses.clear()
            self.times.append(stamp)
            self.poses.append(pose)

        first = bisect_right(self.times, start) - 1
        last = bisect_left(self.times, end)
        if first < 0:
            raise PoseUnavailable("poses do not cover the start of the scan")
        if first:
            del self.times[:first]
            del self.poses[:first]
            last -= first
        times, poses = self.times[:last + 1], self.poses[:last + 1]
        if any(b - a > 0.06 for a, b in zip(times, times[1:])):
            raise PoseUnavailable("pose gap exceeds 60 ms during the scan")
        return times, poses


def pose_at(times, poses, stamp):
    index = bisect_left(times, stamp)
    if index < len(times) and times[index] == stamp:
        return poses[index]
    if not 0 < index < len(times):
        raise PoseUnavailable("cannot extrapolate a pose outside the measured interval")
    fraction = (stamp - times[index - 1]) / (times[index] - times[index - 1])
    return poses[index - 1].interpolate(poses[index], fraction)


def deskew_rotation(samples, start, end, times, poses, *, sensor_ahead=0.0,
                    yaw_deg=0.0, offset_mm=0.0, sample_times=None):
    def sensor_at(stamp):
        rear = pose_at(times, poses, stamp)
        return Pose(*rear.world(sensor_ahead, 0.0), rear.yaw)

    if sample_times is None:
        sample_times = estimate_ray_times(samples, start, end)
    if (len(sample_times) != len(samples) or any(not start <= stamp < end for stamp in sample_times)
            or any(b < a for a, b in zip(sample_times, sample_times[1:]))):
        raise PoseUnavailable("invalid per-ray timestamps")
    beams = (Beam(angle, distance, quality, stamp * 1000, sensor_at(stamp))
             for (angle, distance, quality), stamp in zip(samples, sample_times))
    return project_sweep(beams, sensor_at(end), yaw_deg=yaw_deg, offset_mm=offset_mm, guard_spikes=True)


@dataclass(frozen=True)
class MotionUncertainty:
    translation_m: float
    heading_rad: float

    def padding_at(self, radius):
        return self.translation_m + 2 * radius * math.sin(min(math.pi, self.heading_rad) / 2)

    def angular_padding_deg(self, radius):
        if radius <= self.translation_m:
            return 180.0
        return min(180.0, math.degrees(self.heading_rad + math.asin(self.translation_m / radius)))


@dataclass(frozen=True)
class Beam:
    angle: float
    distance_mm: float
    quality: int
    measured_ms: float
    odometry: Pose
    hit: tuple[float, float] | None = None  # Simulation truth, used only for scoring.


GRID_M = 0.04


def cell(x, y):
    return round(x / GRID_M), round(y / GRID_M)


@dataclass
class ScanGeometry:
    samples: list
    points: list
    ray_segments: list
    free: set
    occupied: set
    wall_exclusions: frozenset = frozenset()
    rejected_far: int = 0

    def visible(self, x, y):
        index = cell(x, y)
        return index in self.free and index not in self.occupied


@dataclass(frozen=True)
class ProjectedRay:
    angle: float
    distance: float
    quality: int
    x: float
    y: float
    dx: float
    dy: float

    @property
    def hit(self):
        return self.x + self.dx * self.distance, self.y + self.dy * self.distance


def isolated_returns(rays):
    """Compare one return with a locally straight surface supported on both sides."""
    def sample_neighbors(index, direction):
        points = []
        angle = rays[index].angle
        for offset in range(1, 6):
            neighbor = index + direction * offset
            if not 0 <= neighbor < len(rays):
                break
            ray = rays[neighbor]
            if not 0 < (direction * (ray.angle - angle)) % 360 <= 8:
                break
            if ray.distance > 0:
                points.append(ray.hit)
            if len(points) == 3:
                break
        return points

    far, near = set(), set()
    for index in range(2, len(rays) - 2):
        ray = rays[index]
        if ray.distance <= 0:
            continue
        before, after = sample_neighbors(index, -1), sample_neighbors(index, 1)
        if len(before) < 2 or len(after) < 2:
            continue
        ax, ay = (median(axis) for axis in zip(*before))
        bx, by = (median(axis) for axis in zip(*after))
        sx, sy = bx - ax, by - ay
        span = math.hypot(sx, sy)
        if not 0.008 <= span <= 0.32:
            continue
        ox, oy = ray.x, ray.y
        cross = ray.dx * sy - ray.dy * sx
        if abs(cross) < 0.30 * span:
            continue
        distance = ((ax - ox) * sy - (ay - oy) * sx) / cross
        along = ((ax - ox) * ray.dy - (ay - oy) * ray.dx) / cross
        if distance <= 0 or not 0 <= along <= 1:
            continue
        tolerance = (0.021 + 0.008 * distance) * span
        if any(sum(abs((hx - ax) * sy - (hy - ay) * sx) <= tolerance for hx, hy in side) < 2
               for side in (before, after)):
            continue
        difference = ray.distance - distance
        if abs(difference) > max(0.065, 0.035 * distance):
            (far if difference > 0 else near).add(index)
    return far, near


def guard_raw_returns(samples):
    """Without odometry reject suspicious far rays, while retaining all near hazards."""
    rays = []
    for angle, distance, quality in samples:
        if math.isfinite(angle) and 0 <= angle < 360:
            bearing = math.radians(angle)
            rays.append(ProjectedRay(angle, corrected_range(distance, quality) / 1000, quality, 0, 0,
                                     math.cos(bearing), math.sin(bearing)))
        else:
            rays.append(ProjectedRay(0, 0, 0, 0, 0, 0, 0))
    far, _ = isolated_returns(rays)
    return [(sample[0], 0.0, 0) if index in far else sample for index, sample in enumerate(samples)]


def project_sweep(beams, current, *, yaw_deg=0.0, offset_mm=0.0, deskew=True, guard_spikes=False):
    rays = []
    for beam in beams:
        if not math.isfinite(beam.angle):
            rays.append(ProjectedRay(beam.angle, 0, 0, 0, 0, 0, 0))
            continue
        angle = (beam.angle + yaw_deg) % 360
        distance = corrected_range(beam.distance_mm, beam.quality, offset_mm) / 1000
        origin = beam.odometry if deskew else current
        x, y = current.local(origin.x, origin.y) if deskew else (0.0, 0.0)
        bearing = math.radians(angle) + origin.yaw - current.yaw
        rays.append(ProjectedRay(angle, distance, beam.quality, x, y, math.cos(bearing), math.sin(bearing)))
    far, near = isolated_returns(rays) if guard_spikes else (set(), set())
    samples, points, segments, free, occupied = [], [], [], set(), set()
    wall_exclusions = set()
    for index, ray in enumerate(rays):
        if not ray.distance or index in far:
            samples.append((ray.angle, 0.0, 0))
            continue
        samples.append((ray.angle, ray.distance * 1000, ray.quality))
        hit = ray.hit
        points.append(hit)
        segments.append((ray.x, ray.y, *hit))
        occupied.add(cell(*hit))
        if index in near:
            wall_exclusions.add(hit)
        # Only traverse the observed interval from this ray's own origin.
        # The cap need only cover the local planner's maximum lookahead.
        count = max(0, int(min(ray.distance - GRID_M, 2.6) / (GRID_M / 2)))
        for step in range(1, count + 1):
            travel = step * (GRID_M / 2)
            free.add(cell(ray.x + ray.dx * travel, ray.y + ray.dy * travel))
    return ScanGeometry(samples, points, segments, free, occupied, frozenset(wall_exclusions), len(far))
