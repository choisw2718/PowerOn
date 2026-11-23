from contextlib import ExitStack, redirect_stdout
from io import StringIO
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from lidar.acquisition.serial_scan import ScanInterrupted
from lidar.autopilot.interactive import AttemptCancelled, AutoAttempt, DriveConsole
from lidar.autopilot.motion.speed_control import SpeedConfig
from lidar.autopilot.planning.wall_track import TrackConfig
from lidar.autopilot.runner import Rotation, renew_drive
from simulation.track.simulator import SCENARIOS, ray_scan, wall_segments


class Nano:
    def __init__(self):
        self.commands = []
        self.disconnected = False
        self.closed = False

    def request(self, command, expected, *, prefix=False):
        self.commands.append(command)
        if self.disconnected:
            raise ScanInterrupted("no acknowledgement")
        return expected

    def close(self):
        self.closed = True


class InteractiveDriveTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.speed = SpeedConfig()
        self.console = DriveConsole("nano", 5000, self.speed)
        self.nano = self.console.nano = Nano()
        self.output = StringIO()
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(redirect_stdout(self.output))
        stack.enter_context(patch("lidar.autopilot.interactive.time.monotonic",
                                  side_effect=lambda: self.now))
        self.addCleanup(self.console.close)

    def activate(self):
        self.console.handle_key("1")
        attempt = self.console.attempt = AutoAttempt(self.console)
        attempt.request("LIDAR START", "OK LIDAR START")
        renew_drive(attempt, 20, 0, 400, attempt.progress)
        self.assertTrue(self.console.autonomous)
        return attempt

    def test_zero_and_minus_stop_cancel_and_revoke_old_attempt(self):
        for key in ("0", "-"):
            with self.subTest(key=key):
                attempt = self.activate()
                self.console.handle_key(key)
                self.assertEqual(self.nano.commands[-1], "IDLE")
                self.assertFalse(self.console.auto_requested)
                self.assertTrue(attempt.stop.is_set())
                with self.assertRaises(AttemptCancelled):
                    attempt.request("LIDAR START", "OK LIDAR START")
                self.now = self.console.settled_at

    def test_manual_input_takes_control_in_fallback_without_replay(self):
        attempt = self.activate()
        self.console._end_attempt()
        self.console.handle_key("r")
        self.assertTrue(self.console.auto_requested)
        self.console.handle_key("w")  # Latches manual, but the car may still be coasting.
        self.assertFalse(self.console.auto_requested)
        self.assertEqual(self.nano.commands[-1], "STATUS")
        self.now = self.console.settled_at
        self.console.handle_key("a")
        self.assertEqual(self.nano.commands[-1], "DRIVE 0.0 2.0")
        with self.assertRaises(AttemptCancelled):
            renew_drive(attempt, 20, 0, 400, attempt.progress)
        self.console.handle_key("w")
        self.assertGreater(self.console.keys.motor_voltage, 0)
        self.console.handle_key("1")
        self.assertTrue(self.console.auto_requested)
        self.assertEqual(self.nano.commands[-1], "IDLE")
        self.assertGreater(self.console.settled_at, self.now)

    def test_connected_manual_input_can_take_over_while_recovery_waits_for_scans(self):
        self.activate()
        self.console._end_attempt()
        self.now = self.console.settled_at
        pending = self.console.attempt = AutoAttempt(self.console)
        self.console.handle_key("w")
        self.assertEqual(self.nano.commands[-2], "IDLE")
        self.assertTrue(self.nano.commands[-1].startswith("DRIVE "))
        self.assertFalse(self.console.auto_requested)
        self.assertTrue(pending.stop.is_set())

    def test_nano_disconnect_retains_auto_intent_and_waits_for_command_expiry(self):
        attempt = self.activate()
        self.nano.disconnected = True
        self.console._end_attempt()
        self.assertIsNone(self.console.nano)
        self.assertTrue(self.nano.closed)
        self.assertTrue(self.console.auto_requested)
        self.assertGreaterEqual(self.console.settled_at, attempt.progress.lease_ends_at + self.console.coast_s)
        replacement = Nano()
        with patch("lidar.autopilot.interactive.NanoDrive", return_value=replacement):
            self.assertTrue(self.console._connect())
        self.assertIs(self.console.nano, replacement)
        self.assertEqual(replacement.commands, [])

    def test_worker_stops_on_scan_disconnect_then_rearms_only_on_fresh_scans(self):
        self.console.handle_key("1")
        scenario = SCENARIOS["straight"]
        samples = list(ray_scan(wall_segments(scenario.outlines), 0, scenario.start_y))
        attempts = []
        deadline = self.now + 20

        def open_rotations(fresh, stop):
            self.assertTrue(fresh)
            attempts.append(stop)
            first_attempt = len(attempts) == 1
            if not first_attempt:
                self.assertFalse(self.console.autonomous)
                self.assertTrue(self.console.auto_requested)
                self.assertEqual(self.nano.commands[-1], "IDLE")
                self.assertGreaterEqual(self.now, self.console.settled_at)
            for index in range(3):
                if index < 2 and not first_attempt:
                    self.assertEqual(self.nano.commands.count("LIDAR START"), 1)
                self.now += 0.16
                yield Rotation(self.now - 0.16, self.now, samples)
            if first_attempt:
                self.assertTrue(self.console.autonomous)
                raise ScanInterrupted("USB disconnected")
            self.assertTrue(self.console.autonomous)
            self.now = deadline

        def advance(seconds):
            self.now += seconds

        args = SimpleNamespace(range_offset_mm=0.0, scan_timing=False)
        with patch.object(self.console.stop, "wait", side_effect=advance):
            self.console.work(args, TrackConfig(), self.speed, open_rotations, deadline, None)
        self.assertFalse(self.console.worker_failed, self.output.getvalue())
        self.assertEqual(len(attempts), 2)
        self.assertEqual(self.nano.commands.count("LIDAR START"), 2)
        self.assertIn("MODE MANUAL (automatic recovery pending): USB disconnected", self.output.getvalue())
        self.assertEqual(self.nano.commands[-1], "IDLE")
