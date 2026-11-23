"""Rear-axle pose inferred from synchronized wheel counts, clockwise yaw positive."""

import math

from .scan_geometry import Pose, PoseLog


def integrate_wheel_distances(pose, left_m, right_m, rear_track_m):
    turn = (left_m - right_m) / rear_track_m
    half = turn / 2
    arc = (left_m + right_m) / 2
    chord = arc * math.sin(half) / half if abs(half) > 1e-9 else arc
    return Pose(pose.x + chord * math.cos(pose.yaw + half),
                pose.y + chord * math.sin(pose.yaw + half),
                math.remainder(pose.yaw + turn, math.tau))


class EncoderLog(PoseLog):
    """Use signed raw Nano counters and the measured wheel calibration."""

    def __init__(self, file, calibration, *, live=False, on_sample=None):
        self.track = calibration["rear_track_m"]
        self.max_speed = calibration["max_wheel_speed_mps"]
        if not all(math.isfinite(v) and v > 0 for v in (self.track, self.max_speed)):
            raise ValueError("Encoder feedback needs measured rear track and maximum wheel speed")
        self.scales = []
        for wheel in ("left", "right"):
            rollout = calibration[wheel + "_rollout_m"]
            counts = calibration[wheel + "_counts_per_rev"]
            sign = calibration[wheel + "_encoder_sign"]
            if not all(math.isfinite(v) and v > 0 for v in (rollout, counts)) or sign not in (-1, 1):
                raise ValueError(f"Encoder feedback needs measured {wheel} rollout, counts/rev and sign")
            self.scales.append(sign * rollout / counts)
        self.previous = None
        self.pose = Pose(0.0, 0.0, 0.0)
        super().__init__(file, live=live, on_sample=on_sample)

    def read_header(self, header):
        if header != "timestamp_s,left_count,right_count":
            raise ValueError("Encoder CSV needs timestamp_s,left_count,right_count")

    def parse_row(self, line):
        raw_stamp, raw_left, raw_right = line.split(",")
        stamp = float(raw_stamp)
        counts = (int(raw_left), int(raw_right))
        if not math.isfinite(stamp) or any(not -(2**31) <= v < 2**32 for v in counts):
            raise ValueError("Invalid encoder timestamp or 32-bit count")
        if self.previous is not None:
            old_stamp, old_counts = self.previous
            dt = stamp - old_stamp
            if dt <= 0:
                raise ValueError("Encoder timestamps must increase strictly")
            travel = []
            for current, old, scale in zip(counts, old_counts, self.scales):
                delta = ((current - old + 2**31) % 2**32) - 2**31
                distance = delta * scale
                # Allow one tick of quantization at each window boundary.
                if abs(distance) > self.max_speed * dt * 1.25 + 2 * abs(scale):
                    raise ValueError("Encoder counter discontinuity or wheel overspeed")
                travel.append(distance)
            self.pose = integrate_wheel_distances(self.pose, *travel, self.track)
        self.previous = stamp, counts
        return stamp, self.pose
