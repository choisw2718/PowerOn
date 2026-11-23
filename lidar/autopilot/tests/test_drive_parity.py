"""Replay timing and command permission at the Nano and simulation entry points."""

import contextlib
import io
import unittest
from dataclasses import replace
from unittest.mock import patch

from lidar.autopilot.motion.feedback import MotionFeedback
from lidar.autopilot.motion.speed_control import SpeedConfig
from lidar.autopilot.planning.continuity import (
    CommandMotionBound, TrackContinuity, command_clearance, lease_budget_ms,
)
from lidar.autopilot.planning.wall_track import Decision, TrackConfig
from lidar.autopilot.runner import Rotation, ordered_sweep, run
from lidar.acquisition.latest_scan import OVERDUE_SCAN
from simulation.track.simulator import SCENARIOS, Simulation


class DriveParityTest(unittest.TestCase):
    def test_no_return_placeholders_do_not_corrupt_rotation_order(self):
        scan = [(352, 1000, 15), (0.45, 0, 0), (354.4, 1000, 15),
                (10, 900, 12), (120, 900, 10), (0.42, 0, 0), (240, 900, 10)]
        self.assertTrue(ordered_sweep(scan))
        self.assertFalse(ordered_sweep(scan + [(150, 1000, 15)]))
        self.assertFalse(ordered_sweep([(0.45, 0, 0)]))

    def test_speed_and_age_cannot_increase_lease(self):
        track = TrackConfig()
        speed = SpeedConfig(max_speed=0.6, acceleration=0.8, deceleration=0.4)
        route = Decision(pwm=45, clear_m=0.61)
        fresh = MotionFeedback(0.1, 0)
        fast = MotionFeedback(0.5, 0)
        lease = lease_budget_ms(route, 1.0, track, speed, 1.05, feedback=fresh)
        self.assertGreater(lease, lease_budget_ms(route, 1.0, track, speed, 1.05, feedback=fast))
        self.assertGreater(lease, lease_budget_ms(route, 1.0, track, speed, 1.30, feedback=fresh))
        self.assertGreater(lease, lease_budget_ms(route, 1.0, track, speed, 1.05, feedback=fresh,
                                                  measured_at_reference=False))
        self.assertLessEqual(command_clearance(1.0, track, speed, 1.05, lease, fresh), route.clear_m)
        uncalibrated = replace(speed, speed_error_mps=speed.max_speed)
        self.assertEqual(lease_budget_ms(route, 1.0, track, uncalibrated, 1.05, feedback=fresh),
                         lease_budget_ms(route, 1.0, track, uncalibrated, 1.05))
        self.assertEqual(lease_budget_ms(replace(route, held_clear_m=0.1), 1.0, track, speed, 1.05,
                                         feedback=fresh), 0)

    def test_unmeasured_bounds_allow_full_speed_and_delayed_coast(self):
        settings = SpeedConfig(max_speed=0.5, acceleration=0.5, deceleration=0.25, propulsion_delay_ms=100)
        bound = CommandMotionBound(settings)
        bound.applied(True, 10)
        self.assertAlmostEqual(bound.maximum(10, 10.2), 0.1)
        self.assertAlmostEqual(bound.maximum(11, 12), settings.max_speed)
        bound.applied(False, 12)
        self.assertAlmostEqual(bound.maximum(12, 12.1), settings.max_speed)
        self.assertAlmostEqual(bound.maximum(13.1, 13.2), 0.25)
        self.assertAlmostEqual(bound.maximum(15, 16), 0.0)

    def test_loss_of_pose_cannot_bridge_using_old_motion(self):
        gate = TrackContinuity()
        track, speed = TrackConfig(), SpeedConfig()
        gate.remember(Decision(pwm=45, clear_m=0.76), 1.0, 1.0)
        for now, action in ((1.1, "coast"), (3.15, "stop")):
            result = gate.gate(Decision(reason="motion unavailable"), [], track, speed, now-0.16,
                               now, now, driving=True)
            self.assertEqual(result.action, action)
            self.assertIn("motion unavailable", result.route.reason)

    def test_overdue_rotation_cuts_pi_power_until_two_fresh_plans(self):
        clock = [1.0]
        commands = []
        class Nano:
            def request(self, command, expected):
                if command.startswith("LIDAR DRIVE "):
                    commands.append(float(command.split()[2]))
        def frames():
            for index, is_gap in enumerate((False, False, False, True, False, False)):
                clock[0] = 1 + index * 0.15
                yield OVERDUE_SCAN if is_gap else Rotation(clock[0]-0.15, clock[0], [])
        with (patch("lidar.autopilot.runner.plan", return_value=Decision(pwm=45, clear_m=0.76)),
              patch("lidar.autopilot.runner.time.monotonic", side_effect=lambda: clock[0]),
              contextlib.redirect_stdout(io.StringIO())):
            run(frames(), TrackConfig(), Nano())
        self.assertEqual([pwm > 0 for pwm in commands], [True, False, False, True])

    def test_pi_and_four_wheel_gate_send_identical_commands_on_the_same_decisions(self):
        sim = Simulation(SCENARIOS["straight"], feedback=False)
        speed = sim.controller.settings
        good = Decision(pwm=sim.config.pwm, clear_m=0.74, steer=2)
        too_short = replace(good, held_clear_m=0.35)
        cases = [good, good, good, too_short, good, good, good]
        now = [0.0]
        actual = []
        class Nano:
            def request(self, command, expected):
                if command.startswith("LIDAR DRIVE "):
                    _, _, pwm, steer, lease = command.split()
                    actual.append((float(pwm), float(steer), int(lease)))
        def frames():
            for index in range(len(cases)):
                now[0] = 1 + index * 0.15
                yield Rotation(now[0] - 0.15, now[0], [])
        with (patch("lidar.autopilot.runner.plan", side_effect=cases),
              patch("lidar.autopilot.runner.time.monotonic", side_effect=lambda: now[0]),
              contextlib.redirect_stdout(io.StringIO())):
            run(frames(), sim.config, Nano(), speed_config=speed)
        for index, result in enumerate(cases):
            sim.now_ms = 1000 + index * 150
            end = sim.now_ms / 1000
            sim.decision = result
            sim.process_drive_gate(end-0.15, end, 0.15, None, None, 0.20, None, None)
        expected = [(round(pwm, 2), steer, lease) for _, pwm, steer, lease, _cause in sim.commands]
        self.assertEqual(actual, expected)
        self.assertEqual([pwm > 0 for pwm, _, _ in actual], [True, False, False, True, True])


if __name__ == "__main__":
    unittest.main()
