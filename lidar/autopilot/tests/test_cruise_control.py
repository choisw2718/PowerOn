from dataclasses import replace
import unittest

from lidar.autopilot.motion.feedback import ActuationMonitor, MotionFeedback, PENDING_OVERSPEED
from lidar.autopilot.motion.speed_control import SpeedConfig, SpeedController, speed_for_pwm
from lidar.autopilot.planning.continuity import TrackContinuity, command_clearance
from lidar.autopilot.planning.wall_track import Decision, TrackConfig


class CruiseControlTest(unittest.TestCase):
    track = TrackConfig()
    clear = Decision(pwm=45, clear_m=2.0, left_m=0.4, right_m=0.4)

    def test_stopping_allowance_and_physics_are_independent_lower_bounds(self):
        settings = SpeedConfig(max_speed=1.0, acceleration=0.5, deceleration=0.5,
                               delay_ms=200, speed_error_mps=0)
        measured = MotionFeedback(0.30, 0)
        # Sensor reference to expected expiry: .1 s elapsed + .3 s lease.
        # Motor can accelerate another .2 s during the stop response.
        travel_before_expiry = 0.3 * 0.4 + 0.5 * 0.5 * 0.4 ** 2
        travel_before_cut = 0.3 * 0.6 + 0.5 * 0.5 * 0.6 ** 2
        coast = (0.30 + 0.5 * 0.6) ** 2 / (2 * settings.deceleration)
        for allowance in (0.2, 0.65):
            track = replace(self.track, stop_distance=allowance)
            self.assertAlmostEqual(command_clearance(1, track, settings, 1.1, 300, measured),
                                   max(travel_before_expiry + allowance, travel_before_cut + coast) + 0.055)
        unknown = replace(settings, speed_error_mps=settings.max_speed)
        self.assertEqual(command_clearance(1, self.track, unknown, 1.1, 300, measured),
                         command_clearance(1, self.track, unknown, 1.1, 300))

    def test_resume_uses_rolling_speed_after_gate_reopens(self):
        controller = SpeedController(SpeedConfig())
        measured = MotionFeedback(0.25, 0)
        stop = controller.update(Decision(), self.track, 0.16, feedback=measured)
        self.assertEqual(stop.pwm, 0)
        rolling = controller.update(self.clear, self.track, 0.16, feedback=measured)
        self.assertAlmostEqual(rolling.speed, measured.speed_mps)
        self.assertGreater(rolling.pwm, 0)
        controller.update(Decision(), self.track, 0.16)
        capped = controller.update(self.clear, self.track, 0.16, feedback=measured, speed_cap=0.14)
        self.assertEqual(capped.pwm, 0)
        self.assertLessEqual(capped.speed, 0.14)

    def test_observed_straight_allows_maximum_after_a_corner(self):
        settings = SpeedConfig(max_speed=1.0, acceleration=0.7, lateral_accel=0.90)
        controller = SpeedController(settings)
        bend = replace(self.clear, curvature=3.5)
        controller.update(bend, self.track, 1.0)
        corner = controller.target
        # Slight centring corrections are compatible with full speed here.
        straight = replace(self.clear, curvature=0.54)
        first = controller.update(straight, self.track, 0.16)
        self.assertEqual(first.speed, corner)
        for _ in range(12):
            cruise = controller.update(straight, self.track, 0.16)
        self.assertEqual(cruise.speed, settings.max_speed)
        self.assertEqual(cruise.pwm, self.clear.pwm)
        ahead = replace(straight, curvature_ahead=((0.15, 3.5),))
        self.assertLess(controller.update(ahead, self.track, 0.16).speed, cruise.speed)

    def test_slowing_for_previous_corner_uses_lower_pwm_until_actual_limit(self):
        settings = SpeedConfig()
        controller = SpeedController(settings)
        controller.update(replace(self.clear, curvature=4.0), self.track, 1.0)
        measured = MotionFeedback(controller.target + 0.06, 0)
        reduced = controller.update(self.clear, self.track, 0.16, feedback=measured)
        self.assertGreater(reduced.pwm, 0)
        self.assertLess(speed_for_pwm(reduced.pwm, self.clear.pwm, settings.max_speed, 0), reduced.speed)
        fast = MotionFeedback(settings.max_speed + 0.10, 0)
        self.assertEqual(controller.update(self.clear, self.track, 0.16, feedback=fast).pwm, 0)

    def test_reduced_positive_pwm_must_produce_measured_deceleration(self):
        settings = SpeedConfig()
        controller = SpeedController(settings)
        limited = replace(self.clear, speed_scale=0.4)
        measured = MotionFeedback(0.33, 0.0)
        controller.update(self.clear, self.track, 1.0, feedback=measured)
        responding = controller.update(limited, self.track, 0.16, feedback=measured)
        self.assertGreater(responding.pwm, 0)
        self.assertLess(speed_for_pwm(responding.pwm, limited.pwm, settings.max_speed, 0), responding.speed)
        for _ in range(3):
            stuck = controller.update(limited, self.track, 0.16, feedback=measured)
        self.assertEqual(stuck.pwm, 0)

    def test_overspeed_coasts_first_and_preserves_hard_stops(self):
        settings = SpeedConfig()
        monitor = ActuationMonitor(settings)
        controller = SpeedController(settings)
        gate = TrackContinuity()
        near = MotionFeedback(settings.max_speed * 1.15, 0)
        cause, cap = monitor.check(near, self.track.wheelbase, 1.0)
        self.assertEqual((cause, cap), (PENDING_OVERSPEED, 0))
        self.assertEqual(monitor.check(near, self.track.wheelbase, 1.0), (cause, cap))
        route = controller.checked_route(self.clear, near, 0.03, cause, cap)
        first = gate.gate(route, [], self.track, settings, 0.9, 1, 1, driving=True)
        self.assertEqual(first.action, "coast")
        obstacle = Decision(reason="obstacle on last confirmed path")
        self.assertEqual(controller.checked_route(obstacle, near, 0.03, cause, cap), obstacle)
        blocked = controller.checked_route(replace(self.clear, clear_m=0.1), near, 0.03, cause, cap)
        self.assertEqual(blocked.reason, "measured speed exceeds stopping space")
        self.assertEqual(monitor.check(near, self.track.wheelbase, 1.16), ("measured overspeed", 0))

        normal = MotionFeedback(0.35, 0)
        self.assertIsNone(monitor.check(normal, self.track.wheelbase, 1.3)[0])
        for end, expected in ((1.46, "coast"), (1.62, "drive")):
            check = gate.gate(self.clear, [], self.track, settings, end-0.16, end, end, driving=True)
            self.assertEqual(check.action, expected)
        fast = MotionFeedback(settings.max_speed * 1.35, 0)
        self.assertEqual(ActuationMonitor(settings).check(fast, self.track.wheelbase, 3),
                         ("measured overspeed", 0))
