import contextlib
import io
import math
import threading
import time
from types import SimpleNamespace
import unittest
from dataclasses import replace
from unittest.mock import patch

from lidar.autopilot.motion.feedback import MotionFeedback

from lidar.autopilot.runner import (DriveProgress, Rotation, drive_lease_ms, live_rotations,
                                    prepare_scan, recorded_rotations, renew_drive, run, run_live)
from lidar.autopilot.planning.continuity import BRIDGEABLE, RECOVERABLE, lease_floor_ms
from lidar.geometry.scan_geometry import Beam, Pose, PoseLog, PoseUnavailable, project_sweep, sample_clock_times
from simulation.track.simulator import (SCENARIOS, T_JUNCTION, Simulation, ray_hit,
                                     ray_scan as virtual_scan, wall_segments)
from lidar.autopilot.motion.speed_control import SpeedConfig
from lidar.autopilot.planning.wall_track import (ISOLATED_NEAR_RETURN, CorridorMemory, Decision, Path, TrackConfig,
                                      angled_wall_evidence, corner_exit_evidence, footprint_path, plan, wall_evidence)
from lidar.acquisition.latest_scan import latest_scans
from lidar.acquisition.serial_scan import ScanDecoder, ScanInterrupted, UART_BYTE_S


STRAIGHT = [(-3, -0.50), (5, -0.50), (5, 0.50), (-3, 0.50)]
RIGHT_BEND = [(-3, -0.45), (1.15, -0.45), (1.15, 4), (0.25, 4),
              (0.25, 0.45), (-3, 0.45)]
DEAD_END = [(-3, -0.50), (0.47, -0.50), (0.47, 0.50), (-3, 0.50)]


def ray_scan(polygon, x=0.0, y=0.0, yaw=0.0, forward=0):
    return virtual_scan(wall_segments((polygon,)), x, y, yaw, forward)


class WallTrackTest(unittest.TestCase):
    def test_steers_towards_center_and_corrects_heading(self):
        centered = plan(ray_scan(STRAIGHT))
        self.assertEqual(centered.steer, 0)
        self.assertGreater(centered.pwm, 0)
        self.assertLess(plan(ray_scan(STRAIGHT, y=-0.13)).steer, 0)
        self.assertGreater(plan(ray_scan(STRAIGHT, y=0.13)).steer, 0)
        self.assertGreater(plan(ray_scan(STRAIGHT, yaw=math.radians(12))).steer, 0)
        rotated = replace(TrackConfig(), forward_angle=105)
        self.assertEqual(plan(ray_scan(STRAIGHT, forward=105), rotated).steer, 0)

    def test_right_and_left_bends(self):
        right = plan(ray_scan(RIGHT_BEND, 0.39, 0.02, math.radians(20)))
        mirrored = [(x, -y) for x, y in RIGHT_BEND]
        left = plan(ray_scan(mirrored, 0.39, -0.02, math.radians(-20)))
        self.assertLess(right.steer, 0)
        self.assertGreater(left.steer, 0)
        self.assertGreater(right.pwm, 0)
        self.assertGreater(left.pwm, 0)

    def test_drives_through_a_right_angle_with_successive_scans(self):
        config = TrackConfig()
        rear_x, rear_y, yaw = -config.sensor_ahead, 0.0, 0.0
        previous = 0.0
        for _ in range(46):
            x = rear_x + config.sensor_ahead * math.cos(yaw)
            y = rear_y + config.sensor_ahead * math.sin(yaw)
            decision = plan(ray_scan(RIGHT_BEND, x, y, yaw), config, previous)
            self.assertGreater(decision.pwm, 0, (x, y, yaw, decision))
            previous = decision.steer
            turn = 0.037 * math.tan(math.radians(-previous)) / config.wheelbase
            rear_x += 0.037 * math.cos(yaw + turn / 2)
            rear_y += 0.037 * math.sin(yaw + turn / 2)
            yaw += turn
        self.assertGreater(rear_y, 0.6)
        self.assertTrue(0.25 < rear_x < 1.15)

    def test_stops_for_near_wall_unknown_front_and_incomplete_rotation(self):
        self.assertEqual(plan(ray_scan(DEAD_END)).pwm, 0)
        unknown = [(a, 0, 0) if a <= 45 or a >= 315 else (a, d, q)
                   for a, d, q in ray_scan(STRAIGHT)]
        self.assertEqual(plan(unknown).reason, "insufficient forward returns")
        self.assertEqual(plan(list(ray_scan(STRAIGHT))[:90]).reason, "incomplete scan")

    def test_short_one_wall_loss_uses_recent_width_without_renewing_it(self):
        memory = CorridorMemory()
        wall_gap = [(a, 0, 0) if 43 < a < 143 else (a, d, q) for a, d, q in ray_scan(STRAIGHT)]
        self.assertEqual(plan(wall_gap, memory=memory).pwm, 0)
        for now in (0.0, 0.16, 0.32):
            plan(ray_scan(STRAIGHT), memory=memory, now=now)
        result = plan(wall_gap, memory=memory, now=0.48)
        self.assertGreater(result.pwm, 0)
        self.assertEqual(result.speed_scale, 0.4)
        self.assertLessEqual(abs(result.steer), 2)
        self.assertEqual(memory.observed_at, 0.32)
        self.assertEqual(plan(wall_gap, memory=memory, now=1.8).pwm, 0)

    def test_measured_near_walls_refresh_width_while_the_far_wall_is_hidden(self):
        near = [(0.43, 0.50)] * 5
        hidden = [(0.43, None)] * 12
        measured = wall_evidence(Path(0, 0.76, near + hidden), width=0.94, can_infer=True)
        self.assertTrue(measured.degraded)
        self.assertAlmostEqual(measured.near_width, 0.93)
        missing = wall_evidence(Path(0, 0.76, [(0.43, None)] * 17), width=0.94, can_infer=True)
        self.assertTrue(missing.degraded)
        self.assertIsNone(missing.near_width)

    def test_corner_exit_requires_continuous_bilateral_support_beyond_occlusion(self):
        hidden = [(None, 0.42)] * 7
        measured = [(0.45, 0.49)] * 5
        path = Path(-24, 0.76, hidden + measured + hidden[:5])
        evidence = corner_exit_evidence(path, 0.95, 0.355)
        self.assertTrue(evidence.degraded)
        self.assertEqual(evidence.pairs, measured)
        unsupported = replace(path, walls=hidden + measured[:2] + hidden[:1] + measured[2:] + hidden[:4])
        self.assertIsNone(corner_exit_evidence(unsupported, 0.95, 0.355))
        self.assertIsNone(corner_exit_evidence(path, 0.65, 0.355))

    def test_wider_diagonal_sections_require_two_coherent_wall_slopes(self):
        slope, width, spacing = 0.72, 1.0, 0.045
        diagonal = width * math.hypot(1, slope)
        pairs = [(0.60 - index * slope * spacing, diagonal - 0.60 + index * slope * spacing)
                 for index in range(10)]
        path = Path(4, 0.45, pairs)
        self.assertTrue(angled_wall_evidence(path, width).degraded)
        widened = replace(path, walls=[pairs[0]] * 10)
        self.assertIsNone(angled_wall_evidence(widened, width))
        missing = replace(path, walls=[(left, None) for left, _ in pairs])
        self.assertIsNone(angled_wall_evidence(missing, width))

    def test_opening_does_not_pull_car_toward_new_far_wall(self):
        memory = CorridorMemory()
        for now in (0.0, 0.16, 0.32):
            plan(ray_scan(STRAIGHT), memory=memory, now=now)
        widened = [(x, 1.65 if y > 0 else y) for x, y in STRAIGHT]
        result = plan(ray_scan(widened), memory=memory, now=0.48)
        self.assertEqual(result.speed_scale, 0.4)
        self.assertLessEqual(abs(result.steer), 2)
        ambiguous = [(x, math.copysign(1.12, y)) for x, y in STRAIGHT]
        self.assertEqual(plan(ray_scan(ambiguous), memory=memory, now=0.64).pwm, 0)

    def test_occluded_bumper_corner_stops_even_with_visible_center_ray(self):
        samples = [(a, 0, 0) if 35 <= a < 40 else (a, d, q)
                   for a, d, q in ray_scan(STRAIGHT)]
        self.assertEqual(plan(samples).reason, "insufficient clear distance")

    def test_symmetric_junction_stops_when_both_turns_are_plausible(self):
        result = plan(ray_scan(T_JUNCTION, x=0.45), previous_steer=18)
        self.assertEqual(result.reason, "ambiguous turn")
        self.assertEqual(result.pwm, 0)

        sim = Simulation(SCENARIOS["junction"])
        for _ in range(50):
            if not sim.step():
                break
        self.assertIn("ambiguous turn", sim.stop_reason)
        self.assertFalse(sim.collision)
        self.assertLess(abs(sim.sensor[1]), 0.45)

    def test_wall_width_can_adapt_to_consistent_small_changes(self):
        memory = CorridorMemory()
        for now in (0.0, 0.15, 0.30):
            plan(ray_scan(STRAIGHT), memory=memory, now=now)
        slightly_wider = [(x, y * 1.12) for x, y in STRAIGHT]
        for now in (0.45, 0.60, 0.75, 0.90, 1.05):
            result = plan(ray_scan(slightly_wider), memory=memory, now=now)
            self.assertEqual(result.speed_scale, 1.0)
        self.assertGreater(memory.width, 1.05)


class LiveCommandTest(unittest.TestCase):
    def test_decoder_keeps_rotation_markers_over_split_reads(self):
        marker = bytes((0x65, 1, 0, 0xA0, 0x0F))
        decoder = ScanDecoder()
        self.assertEqual(list(decoder.feed(marker[:3])), [])
        self.assertEqual(list(decoder.feed(marker[3:])), [(True, 0.0, 1000.0, 25)])
        with self.assertRaisesRegex(ScanInterrupted, "buffered"):
            next(live_rotations([marker * 2], require_fresh=True))

    def test_isolated_buffered_rotation_is_discarded_between_fresh_rotations(self):
        marker = bytes((0x65, 1, 0, 0xA0, 0x0F))
        clock = [100.0]

        def chunks():
            for stamp in (100.0, 100.16, 100.18, 100.34):
                clock[0] = stamp
                yield marker

        with patch("lidar.autopilot.runner.time.monotonic", side_effect=lambda: clock[0]):
            with contextlib.closing(live_rotations(chunks(), require_fresh=True)) as received:
                first, second = next(received), next(received)
        self.assertEqual((first.start, first.end), (100.0, 100.16))
        self.assertEqual((second.start, second.end), (100.18, 100.34))

    def test_serial_rotation_markers_account_for_following_packet_bytes(self):
        marker = bytes((0x65, 1, 0, 0xA0, 0x0F))
        point = bytes((0x66, 129, 0, 0xA0, 0x0F))
        clock = [100.0]

        def chunks():
            yield marker + point
            clock[0] += 0.16
            yield marker + point

        with patch("lidar.autopilot.runner.time.monotonic", side_effect=lambda: clock[0]):
            with contextlib.closing(live_rotations(chunks(), require_fresh=True)) as received:
                frame = next(received)
        self.assertAlmostEqual(frame.start, 100 - 5 * UART_BYTE_S)
        self.assertAlmostEqual(frame.end - frame.start, 0.16)
        self.assertEqual(frame.timing_source, "sample-clock")
        self.assertAlmostEqual(frame.sample_times[1] - frame.sample_times[0], 0.08)

    def test_rotation_reader_keeps_newest_after_controller_pause(self):
        stop, reader_stop, release, published = (threading.Event() for _ in range(4))

        def rotations():
            yield "first"
            release.wait(1)
            yield "older"
            yield "newest"
            published.set()
            reader_stop.wait(1)

        with contextlib.closing(latest_scans(rotations(), stop, reader_stop, time.monotonic() + 2)) as frames:
            self.assertEqual(next(frames), "first")
            release.set()
            self.assertTrue(published.wait(1))
            self.assertEqual(next(frames), "newest")
        self.assertTrue(reader_stop.is_set())

    def test_disconnect_waits_for_lease_and_recovers_only_with_new_scans(self):
        actions = []
        samples = list(ray_scan(STRAIGHT))
        attempt = 0

        class Nano:
            def __init__(self, _host, _port):
                actions.append("nano connected")

            def request(self, command, _expected):
                actions.append(command.split()[1])

            def close(self):
                actions.append("nano closed")

        def fake_rotations(_chunks, fresh, **_):
            nonlocal attempt
            self.assertTrue(fresh)
            attempt += 1
            actions.append(f"scan {attempt}")
            if attempt == 2:
                raise ScanInterrupted("USB still disconnected")
            for index in range(3 if attempt == 1 else 4):
                actions.append(f"frame {attempt}:{index + 1}")
                now = time.monotonic()
                yield Rotation(now - 0.16, now, samples)
            if attempt == 1:
                raise ScanInterrupted("USB unplugged")

        args = SimpleNamespace(ssh=None, port="COM19", drive=True, host="nano",
                               nano_port=5000, seconds=0)
        with patch("lidar.autopilot.runner.NanoDrive", Nano), \
             patch("lidar.autopilot.runner.LidarPort.resolve", return_value="COM19"), \
             patch("lidar.autopilot.runner.live_rotations", fake_rotations), \
             patch("lidar.autopilot.runner.time.sleep", side_effect=lambda _: actions.append("wait")), \
             patch("lidar.autopilot.runner.time.monotonic", return_value=100.0), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(run_live(args, TrackConfig(), SpeedConfig()), 0)
        self.assertEqual(actions, ["nano connected", "scan 1", "frame 1:1", "frame 1:2", "frame 1:3",
                                   "START", "DRIVE", "wait", "nano closed", "wait", "scan 2", "wait",
                                   "scan 3", "frame 3:1", "nano connected", "frame 3:2", "frame 3:3",
                                   "frame 3:4", "START", "DRIVE", "nano closed"])
        self.assertIn("Waiting for motor coast", output.getvalue())

    def test_disconnect_before_first_drive_never_arms_during_recovery(self):
        actions = []
        samples = list(ray_scan(STRAIGHT))

        class Nano:
            def __init__(self, *_):
                actions.append("connect")

            def request(self, *_):
                raise AssertionError("Unexpected movement command")

            def close(self):
                actions.append("close")

        def frames(_chunks, fresh, **_):
            if fresh:
                raise ScanInterrupted("No lidar at startup")
            now = time.monotonic()
            yield Rotation(now - 0.16, now, samples)

        args = SimpleNamespace(ssh=None, port="COM19", drive=True, host="nano",
                               nano_port=5000, seconds=0)
        with patch("lidar.autopilot.runner.NanoDrive", Nano), \
             patch("lidar.autopilot.runner.LidarPort.resolve", return_value="COM19"), \
             patch("lidar.autopilot.runner.live_rotations", frames), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(run_live(args, TrackConfig(), SpeedConfig()), 1)
        self.assertEqual(actions, ["connect", "close"])
        self.assertIn("preview only", output.getvalue())

    def test_lease_keeps_distance_for_delay_and_motor_coast(self):
        track = TrackConfig()
        motion = SpeedConfig()
        frame = Rotation(10.0, 10.16, [])
        clear = Decision(pwm=45, clear_m=0.76)
        self.assertEqual(drive_lease_ms(clear, frame, track, motion, 10.16), 500)
        self.assertLess(drive_lease_ms(clear, frame, track, motion, 10.56), 500)
        self.assertEqual(drive_lease_ms(replace(clear, clear_m=0.40), frame, track, motion, 10.16), 0)
        self.assertEqual(drive_lease_ms(replace(clear, curvature=4), frame, track, motion, 10.16), 0)
        slow_frame = Rotation(10.0, 10.25, [])
        limited = replace(clear, speed_scale=0.4)
        self.assertGreaterEqual(drive_lease_ms(limited, slow_frame, track, motion, 10.25),
                                lease_floor_ms(0.25))
        tight = replace(clear, clear_m=0.60)
        self.assertLess(drive_lease_ms(tight, slow_frame, track, motion, 10.25), lease_floor_ms(0.25))
        self.assertGreaterEqual(
            drive_lease_ms(tight, slow_frame, track, motion, 10.25, feedback=MotionFeedback(0.25, 0)),
            lease_floor_ms(0.25))

    def test_scan_gap_slows_then_recovers_without_rearming(self):
        class Nano:
            def __init__(self):
                self.commands = []

            def request(self, command, _expected):
                self.commands.append(command)

        samples = list(ray_scan(STRAIGHT))
        gap = [(angle, 0.0, 0) if angle <= 45 or angle >= 315 else (angle, mm, quality)
               for angle, mm, quality in samples]
        for missing in (1, 5):
            with self.subTest(missing=missing):
                nano = Nano()
                now = [100.0]

                def frames():
                    for index, scan in enumerate([samples] * 3 + [gap] * missing + [samples] * 2):
                        now[0] = 100.0 + index * 0.16
                        yield Rotation(now[0] - 0.16, now[0], scan)

                with (patch("lidar.autopilot.runner.time.monotonic", side_effect=lambda: now[0]),
                      contextlib.redirect_stdout(io.StringIO()) as log):
                    run(frames(), TrackConfig(), nano, speed_config=SpeedConfig(max_speed=0.20))
                self.assertEqual(nano.commands.count("LIDAR START"), 1)
                drive_pwm = [float(command.split()[2]) for command in nano.commands[1:]]
                self.assertGreater(drive_pwm[1], 0)
                self.assertIn("bridging scan gap", log.getvalue())
                if missing == 1:
                    self.assertTrue(all(pwm > 0 for pwm in drive_pwm))
                else:
                    self.assertEqual(drive_pwm[-2], 0)  # First good scan still holds output off.
                    self.assertGreater(drive_pwm[-1], 0)

    def test_fresh_obstacle_in_a_sparse_scan_overrides_cached_clearance(self):
        class Nano:
            def __init__(self):
                self.commands = []

            def request(self, command, _expected):
                self.commands.append(command)

        samples = list(ray_scan(STRAIGHT))
        nano = Nano()
        now = [100.0]

        def frames():
            for index, scan in enumerate([samples] * 3 + [[(0, 180, 25)]]):
                now[0] = 100.0 + index * 0.16
                yield Rotation(now[0] - 0.16, now[0], scan)

        with (patch("lidar.autopilot.runner.time.monotonic", side_effect=lambda: now[0]),
              contextlib.redirect_stdout(io.StringIO())):
            with self.assertRaisesRegex(RuntimeError, "obstacle on last confirmed path"):
                run(frames(), TrackConfig(), nano, speed_config=SpeedConfig(max_speed=0.20))
        self.assertEqual(len(nano.commands), 2)

    def test_lost_drive_ack_preserves_possible_powered_interval(self):
        class Nano:
            def request(self, *_):
                raise ScanInterrupted("lost ACK")

        progress = DriveProgress()
        with patch("lidar.autopilot.runner.time.monotonic", return_value=10.0):
            with self.assertRaises(ScanInterrupted):
                renew_drive(Nano(), 12, 0, 300, progress)
        self.assertGreaterEqual(progress.lease_ends_at, 10.30)

    def test_obstacle_after_reconnect_leaves_drive_session_closed(self):
        samples = list(ray_scan(STRAIGHT))
        obstacle = list(ray_scan(DEAD_END))
        commands = []
        attempts = 0

        class Nano:
            def __init__(self, *_):
                commands.append("connect")

            def request(self, command, _expected):
                commands.append(command.split()[1])

            def close(self):
                commands.append("close")

        def frames(_chunks, _fresh, **_):
            nonlocal attempts
            attempts += 1
            for scan in ([samples] * 3 if attempts == 1 else [obstacle]):
                now = time.monotonic()
                yield Rotation(now - 0.16, now, scan)
            if attempts == 1:
                raise ScanInterrupted("USB unplugged")

        args = SimpleNamespace(ssh=None, port="COM19", drive=True, host="nano",
                               nano_port=5000, seconds=0)
        with patch("lidar.autopilot.runner.NanoDrive", Nano), \
             patch("lidar.autopilot.runner.LidarPort.resolve", return_value="COM19"), \
             patch("lidar.autopilot.runner.live_rotations", frames), \
             patch("lidar.autopilot.runner.time.sleep"), \
             patch("lidar.autopilot.runner.time.monotonic", return_value=100.0), \
             contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "Stopped after lidar recovery"):
                run_live(args, TrackConfig(), SpeedConfig())
        self.assertEqual(commands, ["connect", "START", "DRIVE", "close"])

    def test_unsafe_frame_ends_the_session_and_stale_frame_sends_no_drive(self):
        class Nano:
            def __init__(self):
                self.commands = []

            def request(self, command, _expected):
                self.commands.append(command)

        clock = [100.0]

        def frames(samples):
            for index in range(3):
                now = 100.0 + index * 0.16
                clock[0] = now
                yield Rotation(now - 0.16, now, samples)
            yield Rotation(0, 0, list(ray_scan(DEAD_END)))

        samples = list(ray_scan(RIGHT_BEND, 0.39, 0.02, math.radians(20)))
        nano = Nano()
        with patch("lidar.autopilot.runner.time.monotonic", side_effect=lambda: clock[0]), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "Stopped"):
                run(frames(samples), TrackConfig(), nano)
        self.assertEqual(nano.commands[0], "LIDAR START")
        self.assertEqual(len(nano.commands), 2)
        self.assertTrue(nano.commands[1].startswith("LIDAR DRIVE"))
        self.assertLess(float(nano.commands[1].split()[-2]), 0)
        self.assertIn(int(nano.commands[1].split()[-1]), range(220, 501))

        stale = Nano()
        with patch("lidar.autopilot.runner.time.monotonic", side_effect=lambda: clock[0]), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(TimeoutError, "expired"):
                run([Rotation(0, 0, samples)] * 3, TrackConfig(), stale)
        self.assertEqual(stale.commands, [])


class MovingScanTest(unittest.TestCase):
    def test_front_wall_projection_matches_distance_and_bearing_with_nonuniform_spin(self):
        track = TrackConfig()
        walls = wall_segments((((-0.6, -0.7), (1.4, -0.7), (1.4, 0.7), (-0.6, 0.7)),))
        count, start, end = 240, 4.0, 4.24
        timeline = sample_clock_times(count, start, end)
        pose_csv = "timestamp_s,x_m,y_m,yaw_deg\n" + "".join(
            f"{start + 0.024 * i:.6f},{0.012 * i},{0.003 * i},{1.8 * i}\n" for i in range(11))
        samples, targets = [], []
        final = Pose(0.12, 0.03, math.radians(18))
        reference = Pose(*final.world(track.sensor_ahead, 0), final.yaw)
        for i in range(count):
            f = i / count
            angle = (f + 0.35 * math.sin(math.tau * f) / math.tau) * 360
            rear = Pose(0.12 * f, 0.03 * f, math.radians(18 * f))
            x, y = rear.world(track.sensor_ahead, 0)
            bearing = rear.yaw + math.radians(angle)
            dx, dy = math.cos(bearing), math.sin(bearing)
            distance, _ = ray_hit(walls, x, y, dx, dy)
            samples.append((angle, distance * 1000, 25))
            targets.append(reference.local(x + distance * dx, y + distance * dy))
        frame = Rotation(start, end, samples, timeline, "sample-clock")
        front_errors = []
        for estimated_times in (timeline, None):
            _, geometry, _, _ = prepare_scan(replace(frame, sample_times=estimated_times), track,
                                              SpeedConfig(), end - start, 0,
                                              poses=PoseLog(io.StringIO(pose_csv)))
            angle_error, range_error = [], []
            points = iter(geometry.points)
            for sample, target in zip(geometry.samples, targets):
                if not sample[2]:
                    continue
                estimate = next(points)
                if target[0] > 0 and abs(target[1]) < 0.4:
                    angle_error.append(abs(math.atan2(estimate[1], estimate[0]) - math.atan2(target[1], target[0])))
                    range_error.append(abs(math.hypot(*estimate) - math.hypot(*target)))
            front_errors.append((max(range_error), max(angle_error)))
        self.assertLess(max(front_errors[0]), 1e-8)
        self.assertGreater(front_errors[1][0], 0.003)
        self.assertGreater(front_errors[1][1], math.radians(0.1))

    def test_replay_uses_explicit_sample_timestamps_when_present(self):
        capture = io.StringIO("start_of_rotation,host_receive_elapsed_s,sample_timestamp_s,angle_deg,distance_mm,quality\n"
                              "1,5,4,0,900,25\n0,5.01,4.03,40,900,25\n1,5.2,4.2,0,900,25\n")
        frame, = recorded_rotations(capture)
        self.assertEqual((frame.start, frame.end, frame.sample_times, frame.timing_source),
                         (4, 4.2, (4, 4.03), "recorded-time"))

    def test_far_spike_never_clears_wall_and_near_return_still_blocks(self):
        pose = Pose(0, 0, 0)

        def sweep(spike, guard, *, missing_neighbor=False):
            beams = [Beam(angle, spike if not angle else 800 / math.cos(math.radians(angle)),
                          0 if missing_neighbor and angle == -2 else 25, i, pose)
                     for i, angle in enumerate((-6, -4, -2, 0, 2, 4, 6))]
            return project_sweep(beams, pose, guard_spikes=guard)

        self.assertTrue(sweep(1100, False).visible(0.90, 0))
        far = sweep(1100, True)
        self.assertEqual(far.rejected_far, 1)
        self.assertFalse(far.visible(0.90, 0))
        near = sweep(270, True, missing_neighbor=True)
        self.assertIn((0.27, 0.0), near.wall_exclusions)
        self.assertIn((0.27, 0.0), near.points)
        self.assertFalse(near.visible(0.27, 0))
        route = footprint_path(near.points, (), TrackConfig(), 0, 0.60, geometry=near, check_visibility=False)
        self.assertTrue(route.blocked and route.isolated_block)
        corroborated = footprint_path([*near.points, (0.27, 0.02)], (), TrackConfig(), 0, 0.60,
                                       geometry=near, check_visibility=False)
        self.assertTrue(corroborated.blocked)
        self.assertFalse(corroborated.isolated_block)
        self.assertIn(ISOLATED_NEAR_RETURN, RECOVERABLE)
        self.assertNotIn(ISOLATED_NEAR_RETURN, BRIDGEABLE)

    def test_live_planner_deskews_moving_returns_and_their_origins(self):
        track = TrackConfig(forward_angle=25)
        sensor_walls = wall_segments((STRAIGHT,))
        pose_csv = io.StringIO()
        pose_csv.write("timestamp_s,x_m,y_m,yaw_deg\n")
        for step in range(11):
            fraction = step / 10
            pose_csv.write(f"{10 + fraction / 5:.6f},{0.1 * fraction},{0.03 * fraction},{18 * fraction}\n")
        pose_csv.seek(0)
        samples, expected = [], []
        rear_end = Pose(0.1, 0.03, math.radians(18))
        end = Pose(*rear_end.world(track.sensor_ahead, 0), rear_end.yaw)
        for angle in range(360):
            fraction = angle / 360
            rear = Pose(0.1 * fraction, 0.03 * fraction, math.radians(18 * fraction))
            origin = rear.world(track.sensor_ahead, 0)
            bearing = rear.yaw + math.radians(angle - track.forward_angle)
            dx, dy = math.cos(bearing), math.sin(bearing)
            distance, _ = ray_hit(sensor_walls, *origin, dx, dy)
            samples.append((angle, distance * 1000 + 9, 25))
            expected.append((end.local(*origin),
                             end.local(origin[0] + distance * dx, origin[1] + distance * dy)))
        frame = Rotation(10.0, 10.2, samples)
        _, geometry, motion, mode = prepare_scan(frame, track, SpeedConfig(), 0.2, 9,
                                                 poses=PoseLog(pose_csv))
        self.assertEqual(mode, "deskew")
        self.assertIsNone(motion)
        self.assertEqual(len(geometry.ray_segments), 360)
        for recorded, (origin, hit) in zip(geometry.ray_segments, expected):
            self.assertLess(math.dist(recorded[:2], origin), 1e-8)
            self.assertLess(math.dist(recorded[2:], hit), 1e-8)

    def test_unmeasured_motion_shrinks_observed_clearance_and_long_sweep_can_stop(self):
        track = TrackConfig(min_turn_radius=0.25)
        samples = list(ray_scan(STRAIGHT))
        raw = plan(samples, track)
        for duration in (0.166, 0.25):
            frame = Rotation(10.0, 10.0 + duration, samples)
            _, _, motion, mode = prepare_scan(frame, track, SpeedConfig(), duration, 0)
            result = plan(samples, track, motion=motion)
            self.assertEqual(mode, "bounded")
            self.assertLess(result.clear_m, raw.clear_m)
            if duration == 0.25:
                self.assertEqual(result.pwm, 0)

    def test_pose_gap_cannot_be_bridged(self):
        poses = PoseLog(io.StringIO("timestamp_s,x_m,y_m,yaw_deg\n"
                                    "10,0,0,0\n10.05,0.01,0,0\n10.2,0.04,0,0\n"))
        with self.assertRaisesRegex(PoseUnavailable, "gap"):
            poses.window(10.0, 10.18)

    def test_missing_poses_do_not_continue_powering_past_the_confirmed_path(self):
        class Nano:
            def __init__(self):
                self.commands = []

            def request(self, command, _expected):
                self.commands.append(command)

        start = 100.0
        now = [start]
        pose_csv = "timestamp_s,x_m,y_m,yaw_deg\n" + "".join(
            f"{start + index * 0.02!r},0,0,0\n" for index in range(26))
        samples = list(ray_scan(STRAIGHT))
        def frames():
            for index in range(24):
                now[0] = start + (index + 1) * 0.16
                yield Rotation(start + index * 0.16, now[0], samples)

        nano = Nano()
        with (patch("lidar.autopilot.runner.time.monotonic", side_effect=lambda: now[0]),
              contextlib.redirect_stdout(io.StringIO())):
            with self.assertRaisesRegex(RuntimeError, "scan recovery timed out"):
                run(frames(), TrackConfig(), nano, poses=PoseLog(io.StringIO(pose_csv)),
                    speed_config=SpeedConfig(max_speed=0.20))
        self.assertEqual(nano.commands.count("LIDAR START"), 1)
        self.assertTrue(any(float(command.split()[2]) > 0 for command in nano.commands[1:3]))
        self.assertTrue(all(float(command.split()[2]) == 0 for command in nano.commands[4:]))
