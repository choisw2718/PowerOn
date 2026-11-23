from io import StringIO
from contextlib import ExitStack
import json
import math
from pathlib import Path
import tempfile
import unittest

from lidar.autopilot.runner import recorded_session_motion, session_rotations
from lidar.autopilot.motion.feedback import ActuationMonitor, MotionFeedback
from lidar.autopilot.motion.speed_control import SpeedConfig
from lidar.geometry.scan_geometry import Pose, PoseUnavailable
from lidar.geometry.wheel_odometry import EncoderLog, integrate_wheel_distances


class EncoderLogTest(unittest.TestCase):
    calibration = dict(rear_track_m=0.15, max_wheel_speed_mps=0.8,
                       left_counts_per_rev=200, right_counts_per_rev=200,
                       left_rollout_m=0.2, right_rollout_m=0.2,
                       left_encoder_sign=1, right_encoder_sign=-1)

    def stream(self, rows):
        return StringIO("timestamp_s,left_count,right_count\n" + rows)

    def test_same_wheel_travel_and_wrapped_counters(self):
        log = EncoderLog(self.stream("0,2147483642,-7\n0.05,-2147483644,-17\n"), self.calibration)
        _, poses = log.window(0, 0.05)
        self.assertAlmostEqual(poses[-1].x, 0.010)
        self.assertAlmostEqual(poses[-1].yaw, 0)
        self.assertIsNone(poses[-1].steer_deg)

    def test_inner_outer_distances_describe_one_arc(self):
        for direction in (1, -1):
            radius, track, angle = 0.45, 0.15, math.pi / 4
            left = angle * (radius + direction * track / 2)
            right = angle * (radius - direction * track / 2)
            pose = integrate_wheel_distances(Pose(0, 0, 0), left, right, track)
            self.assertAlmostEqual(pose.yaw, direction * angle)
            self.assertAlmostEqual(pose.x, radius * math.sin(angle))
            self.assertAlmostEqual(pose.y, direction * radius * (1 - math.cos(angle)))

    def test_missing_calibration_discontinuous_count_and_time_gap(self):
        with self.assertRaises(ValueError):
            EncoderLog(self.stream(""), {**self.calibration, "rear_track_m": 0})
        with self.assertRaises(ValueError):
            EncoderLog(self.stream("0,500,900\n0.05,0,0\n"), self.calibration).window(0, 0.05)
        with self.assertRaises(PoseUnavailable):
            EncoderLog(self.stream("0,0,0\n0.1,0,0\n"), self.calibration).window(0, 0.10)

    def test_normal_corner_and_stalled_inner_wheel(self):
        monitor = ActuationMonitor(SpeedConfig())
        self.assertIsNone(monitor.check(MotionFeedback(0.20, -0.20 / 0.26), 0.133, 0)[0])
        for stamp in (0.2, 0.4, 0.6, 0.8):
            reason, cap = monitor.check(MotionFeedback(0.20, 0.40 / 0.145), 0.133, stamp)
        self.assertEqual(reason, "wheel motion exceeds steering range")
        self.assertEqual(cap, 0)

    def test_run_replays_recorded_motion_and_survives_interrupted_final_write(self):
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            rows = [dict(type="motion_sample", timestamp_s=t, x_m=t / 5, y_m=0, yaw_deg=0,
                         steer_deg=None) for t in (0, 0.025, 0.05)]
            (Path(directory) / "events.jsonl").write_text(
                "\n".join(json.dumps(row) for row in rows) + '\n{"type":', encoding="utf-8")
            log = recorded_session_motion(Path(directory), stack)
            _, poses = log.window(0, 0.05)
            self.assertAlmostEqual(poses[-1].x, 0.01)
        frame = dict(type="scan", start=0, end=0.15, samples=[[10, 350, 8]])
        self.assertEqual(len(list(session_rotations(StringIO(json.dumps(frame) + '\n{"start":')))), 1)
