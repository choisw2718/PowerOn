"""Follow ordered, surveyed sensor-center waypoints in a shared odometry frame."""

from bisect import bisect_right
import csv
from dataclasses import dataclass
import math
from pathlib import Path


@dataclass(frozen=True)
class ReferenceView:
    status: str
    points: tuple[tuple[float, float], ...]
    progress_m: float
    remaining_m: float | None


class ReferenceRoute:
    """The first waypoint and first segment define the expected starting pose."""

    def __init__(self, points, *, closed=False):
        vertices = []
        for x, y in points:
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("Route coordinates must be finite meters")
            if not vertices or math.dist(vertices[-1], (x, y)) > 1e-6:
                vertices.append((x, y))
        if closed and len(vertices) > 1 and math.dist(vertices[-1], vertices[0]) <= 1e-6:
            vertices.pop()
        if len(set(vertices)) < (3 if closed else 2):
            raise ValueError("Route needs two distinct waypoints, or three for a closed loop")
        if closed:
            vertices.append(vertices[0])
        self.closed = closed
        self.points = tuple(vertices)
        self.stations = [0.0]
        self.tangents = []
        for a, b in zip(vertices, vertices[1:]):
            length = math.dist(a, b)
            self.stations.append(self.stations[-1] + length)
            self.tangents.append(((b[0] - a[0]) / length, (b[1] - a[1]) / length))
        self.length_m = self.stations[-1]
        self.reset()

    @classmethod
    def from_csv(cls, source, *, closed=False):
        if not hasattr(source, "read"):
            with Path(source).open(encoding="utf-8-sig", newline="") as stream:
                return cls.from_csv(stream, closed=closed)
        rows = csv.DictReader(source)
        if rows.fieldnames is None or not {"x_m", "y_m"} <= set(rows.fieldnames):
            raise ValueError("Route CSV needs x_m,y_m columns in meters")
        points = []
        for row in rows:
            try:
                points.append((float(row["x_m"]), float(row["y_m"])))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid route waypoint on CSV line {rows.line_num}") from exc
        return cls(points, closed=closed)

    def reset(self):
        self.progress_m = 0.0
        self.previous_xy = None
        self.previous_stamp = None
        self.terminal = None

    def _sections(self, start, end):
        """Yield clipped sections in order, including passage across a closed loop's seam."""
        local = start % self.length_m if self.closed else start
        index = min(bisect_right(self.stations, local) - 1, len(self.tangents) - 1)
        origin = start - local + self.stations[index]
        while origin < end - 1e-9:
            next_origin = origin + self.stations[index + 1] - self.stations[index]
            yield index, origin, max(start, origin), min(end, next_origin)
            origin = next_origin
            index += 1
            if index == len(self.tangents):
                if not self.closed:
                    break
                index = 0

    def _point(self, index, distance_from_vertex):
        x, y = self.points[index]
        dx, dy = self.tangents[index]
        return x + dx * distance_from_vertex, y + dy * distance_from_vertex

    def _view(self, status, points=()):
        remaining = None if self.closed else max(0.0, self.length_m - self.progress_m)
        return ReferenceView(status, points, self.progress_m, remaining)

    def update(self, sensor_pose, *, stamp_s, max_speed_mps, horizon_m=2.0):
        """Project onto neighboring segments; this produces guidance, never free-space evidence."""
        if self.terminal is not None:
            return self._view(self.terminal)
        if not all(math.isfinite(v) and v > 0 for v in (max_speed_mps, horizon_m)):
            raise ValueError("Route speed and lookahead must be finite and positive")
        if (sensor_pose is None or not all(map(math.isfinite, (sensor_pose.x, sensor_pose.y,
                                                              sensor_pose.yaw, stamp_s)))
                or self.previous_stamp is not None and stamp_s <= self.previous_stamp):
            return self._view("pose_unavailable")
        xy = (sensor_pose.x, sensor_pose.y)
        hx, hy = math.cos(sensor_pose.yaw), math.sin(sensor_pose.yaw)
        if self.previous_xy is None:
            dx, dy = self.tangents[0]
            if (math.dist(xy, self.points[0]) > 0.25
                    or hx * dx + hy * dy < math.cos(math.radians(55))):
                return self._view("start_mismatch")
            start, end = 0.0, min(0.25, self.stations[1])
        else:
            moved = math.dist(xy, self.previous_xy)
            speed_bound = max_speed_mps * min(stamp_s - self.previous_stamp, 0.75) + 0.05
            if moved > speed_bound:
                self.terminal = "off_route"
                return self._view(self.terminal)
            # Compare only the interval reachable from the previous segment.
            # Similar coordinates at a crossing or hairpin can be meters apart along the route.
            start = max(0.0, self.progress_m - 0.08)
            end = self.progress_m + min(1.5 * moved + 0.04, speed_bound)
            if not self.closed:
                end = min(end, self.length_m)

        best = None
        for index, origin, lo, hi in self._sections(start, end):
            dx, dy = self.tangents[index]
            alignment = hx * dx + hy * dy
            if alignment < math.cos(math.radians(85)):
                continue
            x, y = self.points[index]
            projection = origin + (xy[0] - x) * dx + (xy[1] - y) * dy
            station = max(lo, min(hi, projection))
            offset = math.dist(xy, self._point(index, station - origin))
            candidate = (offset, -alignment, station)
            if best is None or candidate < best:
                best = candidate
        if best is None or best[0] > 0.28:
            self.terminal = "off_route"
            return self._view(self.terminal)
        self.progress_m = max(self.progress_m, best[2])
        self.previous_xy, self.previous_stamp = xy, stamp_s
        if not self.closed and self.length_m - self.progress_m < 0.08:
            if math.dist(xy, self.points[-1]) <= 0.10:
                self.terminal = "complete"
                return self._view(self.terminal)
            if self.progress_m >= self.length_m - 1e-9:
                self.terminal = "off_route"
                return self._view(self.terminal)
        end = self.progress_m + min(horizon_m, self.length_m) if self.closed else min(
            self.length_m, self.progress_m + horizon_m)
        local_points = []
        for index, origin, lo, hi in self._sections(self.progress_m, end):
            if not local_points:
                local_points.append(sensor_pose.local(*self._point(index, lo - origin)))
            # Keep every real corner while giving long straight segments enough pursuit samples.
            count = max(1, math.ceil((hi - lo) / 0.12))
            for i in range(1, count + 1):
                along = lo + (hi - lo) * i / count - origin
                local_points.append(sensor_pose.local(*self._point(index, along)))
        return self._view("tracking", tuple(local_points))
