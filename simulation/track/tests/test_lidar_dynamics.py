from dataclasses import replace
import math
import unittest
from unittest.mock import Mock

from lidar.autopilot.motion.feedback import ActuationMonitor, MotionFeedback, SteeringTransition, measure_motion
from simulation.benchmark.stress_benchmark import compare_planners, run_case
from lidar.geometry.scan_geometry import Beam, Pose, estimate_ray_times, project_sweep
from simulation.track.conditions import FaultWindow, GripZone, STRESS_PROFILES, StressSettings
from simulation.track.courses import CHALLENGE_COURSES
from simulation.track.simulator import (LOOP_CHECKPOINTS, MOTION_PRESETS, MotionSettings, SCENARIOS,
                                     SensorSettings, Simulation, SimulatorWindow, segment_body_clearance,
                                     wall_segments)
from lidar.autopilot.motion.speed_control import SpeedCommand, SpeedConfig, SpeedController, braking_limit
from lidar.autopilot.planning.wall_track import Decision, TrackConfig, footprint_path


class SpeedTest(unittest.TestCase):
    def test_braking_envelope_accounts_for_distance_and_delay(self):
        no_delay = braking_limit(0.25, 0, 0.40, 0)
        delayed = braking_limit(0.25, 0, 0.40, 0.20)
        self.assertLess(delayed, no_delay)
        self.assertAlmostEqual(delayed * 0.20 + delayed ** 2 / 0.80, 0.25)

    def test_slows_before_a_corner_and_accelerates_after_it(self):
        track = TrackConfig()
        speed = SpeedController(SpeedConfig())
        straight = Decision(pwm=track.pwm, clear_m=0.76)
        for _ in range(10):
            fast = speed.update(straight, track, 0.15)
        approach = speed.update(replace(straight, turn_in_m=0.03), track, 0.15)
        self.assertEqual(approach.phase, "decelerate")
        self.assertLess(approach.speed, fast.speed)
        corner = speed.update(replace(straight, curvature=3.5), track, 0.15)
        self.assertLessEqual(corner.speed ** 2 * 3.5, speed.settings.lateral_accel + 1e-9)
        still_turning = speed.update(replace(straight, curvature=0.8), track, 0.15)
        self.assertEqual(still_turning.speed, corner.speed)
        self.assertEqual(speed.update(straight, track, 0.15).speed, corner.speed)
        exit_ = speed.update(straight, track, 0.15)
        self.assertEqual(exit_.phase, "accelerate")
        self.assertAlmostEqual(exit_.speed - corner.speed, speed.settings.acceleration * 0.15)
        self.assertEqual(speed.update(Decision(), track, 0.15).pwm, 0)

    def test_wall_uncertainty_limits_speed_until_two_reliable_scans(self):
        track = TrackConfig()
        speed = SpeedController(SpeedConfig(max_speed=1.0))
        straight = Decision(pwm=track.pwm, clear_m=1.50)
        for _ in range(20):
            speed.update(straight, track, 0.15)
        self.assertAlmostEqual(speed.update(replace(straight, speed_scale=0.4), track, 0.15).speed, 0.20)
        self.assertAlmostEqual(speed.update(straight, track, 0.15).speed, 0.20)
        self.assertGreater(speed.update(straight, track, 0.15).speed, 0.20)

    def test_older_scan_extends_lookahead_and_reduces_braking_speed(self):
        track = TrackConfig()
        fresh = SpeedController(SpeedConfig(max_speed=1.0))
        old = SpeedController(fresh.settings)
        straight = Decision(pwm=track.pwm, clear_m=2.0)
        fresh.update(straight, track, 2.0)
        self.assertGreater(fresh.lookahead(0.8), fresh.lookahead(0.1))
        approaching_wall = replace(straight, clear_m=0.55)
        self.assertLess(old.update(approaching_wall, track, 2.0, scan_age=0.8).speed,
                        fresh.update(approaching_wall, track, 2.0, scan_age=0.1).speed)

    def test_curvature_preview_brakes_as_a_confirmed_curve_gets_closer(self):
        track = TrackConfig()
        settings = SpeedConfig(max_speed=0.8)
        future = Decision(pwm=track.pwm, clear_m=1.45, curvature=3.0,
                          curvature_ahead=((0.0, 0.0), (0.65, 3.0)))
        near = replace(future, curvature_ahead=((0.0, 0.0), (0.24, 3.0)))
        now = replace(future, curvature_ahead=((0.0, 3.0),))
        targets = [SpeedController(settings).update(route, track, 2.0).speed
                   for route in (future, near, now)]
        self.assertGreater(targets[0], targets[1])
        self.assertGreater(targets[1], targets[2])
        self.assertAlmostEqual(targets[2] ** 2 * 3.0, 0.85 * settings.lateral_accel)

    def test_transient_actuation_speed_cap_does_not_become_a_corner_cap(self):
        track, settings = TrackConfig(), SpeedConfig()
        speed = SpeedController(settings)
        curve = Decision(pwm=track.pwm, clear_m=0.76, curvature=2.0)
        self.assertAlmostEqual(speed.update(curve, track, 1.0, speed_cap=0.15).speed, 0.15)
        self.assertGreater(speed.update(curve, track, 0.16).speed, 0.15)


class ActuationTest(unittest.TestCase):
    def test_observed_steering_changes_the_checked_footprint(self):
        track = TrackConfig()
        transition = SteeringTransition(24, 24, 0.40, 0.32, 40)
        points, rays = [(0.40, 0.24)], [2.0] * 72
        nominal = footprint_path(points, rays, track, 0, 0.76)
        current = footprint_path(points, rays, track, 0, 0.76, steering=transition)
        self.assertEqual(nominal.clear, 0.76)
        self.assertLess(current.clear, track.stop_distance)

    def test_pose_derivative_and_measured_wheel_angle_are_distinct(self):
        times = [1.0, 1.05, 1.10]
        poses = [Pose(0.0, 0.0, 0.0, 6), Pose(0.02, 0.0, 0.0, 9), Pose(0.04, 0.0, 0.0, 12)]
        feedback = measure_motion(times, poses, 1.075)
        self.assertAlmostEqual(feedback.speed_mps, 0.4)
        self.assertAlmostEqual(feedback.yaw_rate, 0.0)
        self.assertAlmostEqual(feedback.steer_deg, 10.5)
        self.assertAlmostEqual(feedback.observed_steer(0.13324), 10.5)

    def test_actual_speed_reduces_pwm_and_coasts_on_large_error(self):
        track, settings = TrackConfig(), SpeedConfig()
        route = Decision(pwm=track.pwm, clear_m=0.76)
        speed = SpeedController(settings)
        open_loop = speed.update(route, track, 1.0)
        reduced = speed.update(route, track, 0.16, feedback=MotionFeedback(0.45, 0.0))
        self.assertLess(reduced.pwm, open_loop.pwm)
        coast = speed.update(route, track, 0.16, feedback=MotionFeedback(0.49, 0.0))
        self.assertEqual((coast.pwm, coast.phase), (0.0, "coast"))
        self.assertEqual(coast.speed, settings.max_speed)
        self.assertFalse(speed.can_stop(replace(route, clear_m=0.23), MotionFeedback(0.49, 0.0), 0.25))

    def test_steering_and_coast_failures_first_derate_then_stop(self):
        settings = SpeedConfig()
        measured = MotionFeedback(0.28, 0.0, 0.0)
        cases = (
            (18, SpeedCommand(25, 0.28, "hold"), "steering command mismatch"),
            (0, SpeedCommand(0, 0.16, "coast"), "coasting response too weak"),
            (0, SpeedCommand(25, 0.28, "hold"), "no motion under drive command"),
        )
        for angle, command, expected in cases:
            with self.subTest(expected=expected):
                monitor = ActuationMonitor(settings)
                monitor.record(angle, command, measured, 0)
                current = MotionFeedback(0, 0, 0) if expected.startswith("no motion") else measured
                first = monitor.check(current, 0.13324, 0.8)
                self.assertEqual(first, (expected, 0.15))
                for stamp in (0.96, 1.12, 1.28):
                    last = monitor.check(current, 0.13324, stamp)
                self.assertEqual(last, (expected, 0.0))

    def test_settled_feedback_learns_offset_and_gain_in_the_path_model(self):
        monitor = ActuationMonitor(SpeedConfig())
        demand = SpeedCommand(10, 0.1, "hold")
        neutral = MotionFeedback(0.10, 0.0, 2.0)
        monitor.record(0, demand, neutral, 0)
        for index in range(30):
            monitor.check(neutral, 0.13324, 1 + index / 10)
        self.assertAlmostEqual(monitor.transition(neutral, 0.13324, 0.1).wheel_target(0), -2.0,
                               delta=0.04)
        left = MotionFeedback(0.10, 0.0, 18.0)
        monitor.record(20, demand, left, 4)
        for index in range(30):
            monitor.check(left, 0.13324, 5 + index / 10)
        transition = monitor.transition(left, 0.13324, 0.1)
        self.assertAlmostEqual(transition.wheel_target(-20), -18.0, delta=0.10)


class RollingScanTest(unittest.TestCase):
    def test_timed_pose_transform_recovers_flat_wall(self):
        reference = Pose(0.15, 0.02, 0)
        beams = []
        for index in range(6):
            pose = Pose(0.025 * index, 0.003 * index, 0.04 * index)
            hit = (0.3 + index * 0.05, 0.6)
            x, y = pose.local(*hit)
            angle = math.degrees(math.atan2(y, x))
            beams.append(Beam(angle - 2, math.hypot(x, y) * 1000 + 13, 25, index * 25, pose))
        geometry = project_sweep(beams, reference, yaw_deg=2, offset_mm=13)
        simultaneous = project_sweep(beams, reference, yaw_deg=2, offset_mm=13, deskew=False)
        for index, ((x, y), (ax, ay, _, _)) in enumerate(zip(geometry.points, geometry.ray_segments)):
            self.assertAlmostEqual(x, 0.15 + index * 0.05)
            self.assertAlmostEqual(y, 0.58)
            self.assertAlmostEqual(ax, beams[index].odometry.x - reference.x)
            self.assertAlmostEqual(ay, beams[index].odometry.y - reference.y)
        self.assertGreater(max(abs(y - 0.58) for _, y in simultaneous.points), 0.06)

    def test_missing_returns_and_space_behind_hit_are_unobserved(self):
        pose = Pose(0, 0, 0)
        geometry = project_sweep([Beam(0, 800, 25, 0, pose), Beam(90, 800, 0, 1, pose)], pose)
        self.assertTrue(geometry.visible(0.4, 0))
        self.assertFalse(geometry.visible(0.8, 0))
        self.assertFalse(geometry.visible(1.0, 0))
        self.assertFalse(geometry.visible(0, 0.4))

    def test_estimated_times_follow_clockwise_phase_across_zero(self):
        samples = [(angle, 1000, 25) for angle in (350, 349.8, 0, 80, 170, 260)]
        times = estimate_ray_times(samples, 2.0, 2.18)
        for actual, expected in zip(times, (2, 2, 2.005, 2.045, 2.09, 2.135)):
            self.assertAlmostEqual(actual, expected)

    def test_independent_point_clock_and_revolution_clock(self):
        sensor = SensorSettings(period_ms=166, point_rate_hz=1950)
        sim = Simulation(SCENARIOS["straight"], sensor=sensor)
        counts = []
        for _ in range(5):
            prior_count = sim.next_point_index
            self.assertTrue(sim.step())
            counts.append(sim.next_point_index - prior_count)
        self.assertEqual(set(counts), {323, 324})
        self.assertLessEqual(abs(sum(counts) - sensor.point_rate_hz * sim.now_ms / 1000), 0.5)

    def test_long_bend_slows_then_recovers_with_rolling_scans(self):
        sim = Simulation(SCENARIOS["long-bend"], sensor=SensorSettings(), vehicle_model="legacy")
        straight, corner, exit_ = [], [], []
        for _ in range(125):
            alive = sim.step()
            x, y = sim.sensor
            if 1 < x < 3.7:
                straight.append(sim.speed)
            elif x > 4.25 and y < 0.8:
                corner.append(sim.speed)
            elif 1.55 < y < 2.2:
                exit_.append(sim.speed)
            if not alive:
                break
        self.assertFalse(sim.collision)
        self.assertFalse(sim.turn_limited)
        self.assertGreater(max(straight), 0.40)
        self.assertLessEqual(max(corner), 0.30)
        self.assertGreater(max(exit_), 0.40)
        raw, corrected = sim.moving_error_sums
        self.assertLess(corrected, raw * 0.10)


class TrackDrawingTest(unittest.TestCase):
    def test_finish_line_and_vehicle_align_with_the_map(self):
        window = SimulatorWindow.__new__(SimulatorWindow)
        window.sim = Simulation(SCENARIOS["loop"])
        window.timer = None
        window.canvas = Mock()
        window.canvas.winfo_width.return_value = 800
        window.canvas.winfo_height.return_value = 400
        window.split_rows = [(Mock(), Mock()) for _ in LOOP_CHECKPOINTS]
        window.readout = Mock()

        window.draw()

        canvas = window.canvas
        wall_points = [point for segment in window.sim.walls for point in segment]
        left, right = min(x for x, _ in wall_points), max(x for x, _ in wall_points)
        wall_xs = [x for call in canvas.create_line.call_args_list if call.kwargs.get("width") == 3
                   for x in call.args[::2]]
        expected_x = min(wall_xs) - left / (right - left) * (max(wall_xs) - min(wall_xs))
        finish = [call.args for call in canvas.create_line.call_args_list if "dash" in call.kwargs][-1]
        sensor = canvas.create_oval.call_args.args
        self.assertAlmostEqual(finish[0], expected_x)
        self.assertAlmostEqual((sensor[0] + sensor[2]) / 2, expected_x)


class LargeCircuitTest(unittest.TestCase):
    def test_effective_range_controls_returns_down_the_long_straight(self):
        forward_returns = []
        for maximum in (8, 12):
            sensor = SensorSettings(max_range_m=maximum, noise_mm=0, dropout=0)
            sim = Simulation(SCENARIOS["loop"], sensor=sensor)
            sim.step()
            sim.step()
            forward_returns.append(any(quality for angle, _, quality in sim.samples
                                       if abs(math.remainder(angle, 360)) < 1.5))
        self.assertEqual(forward_returns, [False, True])

    def test_lap_requires_forward_crossings_in_course_order(self):
        sim = Simulation(SCENARIOS["loop"], sensor=SensorSettings())

        def cross(checkpoint, *, reverse=False, lateral=0):
            gx, gy = checkpoint.center
            dx, dy = checkpoint.direction
            x, y = gx - lateral * dy, gy + lateral * dx
            sign = -1 if reverse else 1
            sim.rear_x = x + 0.05 * dx * sign - sim.config.sensor_ahead
            sim.rear_y = y + 0.05 * dy * sign
            sim.update_lap((x - 0.05 * dx * sign, y - 0.05 * dy * sign))

        cross(LOOP_CHECKPOINTS[-1])
        cross(LOOP_CHECKPOINTS[0], reverse=True)
        cross(LOOP_CHECKPOINTS[0], lateral=1.0)
        self.assertEqual(sim.checkpoint_index, 0)
        self.assertEqual(sim.completed_laps, 0)
        for index, checkpoint in enumerate(LOOP_CHECKPOINTS, start=1):
            cross(checkpoint)
            self.assertEqual(sim.last_split.checkpoint, index)
        self.assertEqual(sim.completed_laps, 1)
        self.assertEqual(sim.checkpoint_index, 0)


class ChallengeCoursesTest(unittest.TestCase):
    def test_off_time_uses_applied_transitions_and_classifies_safety_stops(self):
        sim = Simulation(SCENARIOS["loop"], vehicle_model="legacy")
        sim.apply_pwm(0, "observation")  # Waiting to start.
        sim.apply_pwm(22, "speed_control")
        sim.speed = 0.28
        sim.queue_command(0, 0, cause="observation")
        self.assertEqual(sim.power_cut_metrics()["count"], 0)
        sim.step(end_ms=sim.motion.command_delay_ms)
        self.assertEqual(sim.power_cut_metrics()["by_cause"]["observation"]["count"], 1)
        sim.step(end_ms=sim.now_ms + 10)
        sim.apply_pwm(0, "terminal")  # Still off, not an additional cut.
        stats = sim.power_cut_metrics()
        self.assertEqual(stats["count"], 1)
        self.assertAlmostEqual(stats["moving_off_s"], 0.010)
        self.assertEqual(stats["by_cause"]["terminal"]["count"], 0)

    def test_wall_clearance_scoring_keeps_the_existing_collision_limit(self):
        points = [(i / 100, side) for i in range(-30, 100, 2) for side in (-0.28, 0.28)]
        config = TrackConfig()
        center = footprint_path(points, (), config, 0, 0.75, check_visibility=False, score_clearance=True)
        near = footprint_path(points, (), config, 9, 0.75, check_visibility=False, score_clearance=True)
        unscored = footprint_path(points, (), config, 9, 0.75, check_visibility=False)
        self.assertEqual(center.clearance_cost, 0)
        self.assertGreater(near.clearance_cost, 0)
        self.assertTrue(near.blocked)
        self.assertEqual((near.clear, near.blocked), (unscored.clear, unscored.blocked))

    def test_course_report_keeps_partial_sector_and_counts_time_below_threshold(self):
        args = dict(scenario="loop-narrow", min_clearance_m=0.50)
        old = run_case("clean", 7, 1, 1, planner="legacy", **args)
        new = run_case("clean", 7, 1, 1, planner="clearance", **args)
        sector, = new.sectors
        self.assertFalse(sector["completed"])
        self.assertEqual(sector["label"], SCENARIOS["loop-narrow"].checkpoints[0].label)
        self.assertEqual(new.below_wall_threshold_s, 1.0)
        self.assertEqual(sector["below_wall_threshold_s"], new.below_wall_threshold_s)
        comparison, = compare_planners((old, new), 1)
        self.assertIsNone(comparison["time_delta_s"])
        self.assertEqual(comparison["sectors"], [])
        self.assertEqual(comparison["legacy_laps"], old.laps)
        route = replace(new, planner="route", power_cuts=dict(count=2, moving_off_s=0.5))
        comparison, = compare_planners((route, new), 1)
        self.assertEqual((comparison["baseline_planner"], comparison["candidate_planner"]), ("clearance", "route"))
        self.assertEqual(comparison["power_cut_delta"], 2 - new.power_cuts["count"])
        self.assertAlmostEqual(comparison["moving_off_delta_s"], 0.5 - new.power_cuts["moving_off_s"])

    def test_course_wall_loops_are_disjoint_and_timing_centers_clear_the_body(self):
        def side(start, end, point):
            dx, dy = end[0] - start[0], end[1] - start[1]
            return dx * (point[1] - start[1]) - dy * (point[0] - start[0])

        for name, scenario in CHALLENGE_COURSES.items():
            with self.subTest(course=name):
                walls = wall_segments(scenario.outlines)
                for index, (a, b) in enumerate(walls):
                    self.assertGreater(math.dist(a, b), 0.01)
                    for c, d in walls[index + 1:]:
                        crossing = side(a, b, c) * side(a, b, d) < -1e-12 and side(c, d, a) * side(c, d, b) < -1e-12
                        self.assertFalse(crossing, (a, b, c, d))
                sim = Simulation(scenario, sensor=SensorSettings())
                for gate in scenario.checkpoints:
                    x, y = gate.center
                    dx, dy = gate.direction
                    sim.rear_x = x - sim.config.sensor_ahead * dx
                    sim.rear_y = y - sim.config.sensor_ahead * dy
                    sim.yaw = math.atan2(dy, dx)
                    self.assertGreater(sim.wall_clearance(), 0.10, gate.label)

    def test_hairpin_gate_does_not_count_the_parallel_return_lane(self):
        entry = SCENARIOS["loop-hairpin"].checkpoints[0]
        self.assertTrue(entry.crossed((4.7, 0), (4.9, 0)))
        self.assertFalse(entry.crossed((4.7, 0.92), (4.9, 0.92)))
        self.assertFalse(entry.crossed((4.9, 0), (4.7, 0)))

    def test_offset_course_omits_wall_and_mixed_uses_local_grip(self):
        scenario = SCENARIOS["loop-offset"]
        sim = Simulation(scenario, sensor=SensorSettings(), stress=STRESS_PROFILES["mixed"])
        gap, = scenario.missing_walls
        self.assertAlmostEqual(math.dist(*gap), 0.22)
        self.assertIn(gap, wall_segments(scenario.outlines))
        self.assertNotIn(gap, sim.walls)
        self.assertEqual(sim.stress.grip_zones, scenario.grip_zones)
        self.assertLess(sim.stress.grip_at(6.85, 1.6), 1.0)


class DynamicsTest(unittest.TestCase):
    def test_sensor_and_command_delays_both_apply(self):
        motion = MotionSettings(sensor_delay_ms=80, command_delay_ms=90)
        sim = Simulation(SCENARIOS["straight"], motion)
        for _ in range(3):
            self.assertTrue(sim.step())
        self.assertEqual(sim.now_ms, 450)
        self.assertGreater(sim.demand.pwm, 0)
        self.assertEqual(sim.applied_pwm, 0)
        self.assertEqual(sim.distance, 0)
        sim.step()
        self.assertGreater(sim.distance, 0)

    def test_speed_ramps_and_turn_radius_caps_applied_steering(self):
        motion = MotionSettings(min_turn_radius=0.55, acceleration=0.6, deceleration=0.3)
        sim = Simulation(SCENARIOS["straight"], motion)
        sim.commands.append((sim.now_ms, motion.pwm_limit, 28))
        sim.process_events()
        self.assertAlmostEqual(sim.target_steer, math.degrees(math.atan(sim.config.wheelbase / 0.55)))
        sim.integrate(0.10)
        self.assertAlmostEqual(sim.speed, 0.06)
        sim.applied_pwm = 0
        position = sim.distance
        sim.integrate(0.10)
        self.assertAlmostEqual(sim.speed, 0.03)
        self.assertGreater(sim.distance, position)

    def test_sensor_loss_delays_shutdown_and_vehicle_coasts_to_stop(self):
        sim = Simulation(SCENARIOS["straight"], MOTION_PRESETS["delayed"][1], vehicle_model="legacy")
        for _ in range(16):
            self.assertTrue(sim.step())
        sim.drop_front = True
        for _ in range(40):
            if not sim.step():
                break
        self.assertFalse(sim.collision)
        self.assertEqual(sim.stop_reason, "insufficient forward returns")
        self.assertEqual(sim.speed, 0)
        self.assertGreater(sim.distance - sim.stop_request_distance, 0.25)

    def test_single_bad_scan_does_not_stop_the_car(self):
        sim = Simulation(SCENARIOS["straight"], sensor=SensorSettings(), feedback=True)
        for _ in range(12):
            self.assertTrue(sim.step())
        sim.drop_front = True
        self.assertTrue(sim.step())
        sim.drop_front = False
        for _ in range(8):
            self.assertTrue(sim.step())
            self.assertGreater(sim.speed, 0)
        self.assertGreater(sim.bridged_scans, 0)
        self.assertEqual(sim.waiting_scans, 0)
        self.assertIsNone(sim.stop_reason)

    def test_longer_scan_gap_coasts_then_resumes_after_observations_return(self):
        sim = Simulation(SCENARIOS["straight"], sensor=SensorSettings(), feedback=True)
        for _ in range(12):
            self.assertTrue(sim.step())
        sim.drop_front = True
        for _ in range(8):
            self.assertTrue(sim.step())
        self.assertEqual(sim.speed, 0)
        sim.drop_front = False
        for _ in range(9):
            self.assertTrue(sim.step())
        self.assertGreater(sim.speed, 0)
        self.assertGreater(sim.waiting_scans, 0)
        self.assertIsNone(sim.stop_reason)


class StressSimulationTest(unittest.TestCase):
    def test_overdue_rotation_cuts_power_and_requires_two_fresh_plans_to_resume(self):
        stress = StressSettings(faults=(FaultWindow("scan-drop", 2000, 2001),))
        sensor = SensorSettings(period_ms=250, point_rate_hz=1500)
        sim = Simulation(SCENARIOS["loop"], sensor=sensor, feedback=True, stress=stress,
                         coast_on_scan_gap=True, vehicle_model="legacy")
        for _ in range(9):
            self.assertTrue(sim.step())
        self.assertEqual(sim.applied_pwm, 0)
        self.assertEqual(sim.scan_gaps, 1)
        self.assertTrue(sim.step())
        self.assertEqual(sim.applied_pwm, 0)
        self.assertTrue(sim.step())
        self.assertGreater(sim.applied_pwm, 0)
        self.assertEqual(sim.recovered_scan_gaps, 1)
        self.assertEqual(sim.watchdog_trips, 0)

    def test_watchdog_still_checks_delivery_when_missing_scan_has_already_cut_power(self):
        stress = StressSettings(faults=(FaultWindow("scan-drop", 2000, 4000),
                                       FaultWindow("command-drop", 2300, 3300)))
        sim = Simulation(SCENARIOS["loop"], sensor=SensorSettings(period_ms=250),
                         feedback=True, stress=stress, coast_on_scan_gap=True)
        for _ in range(9):
            self.assertTrue(sim.step())
        self.assertEqual(sim.applied_pwm, 0)
        for _ in range(8):
            if not sim.step():
                break
        self.assertEqual(sim.watchdog_trips, 1)
        self.assertEqual(sim.stop_reason, "command watchdog")
        self.assertEqual(sim.speed, 0)

    def test_benchmark_reports_clearance_gate_even_for_an_expected_stop(self):
        result = run_case("watchdog", 7, 1, 8, min_clearance_m=0.45)
        self.assertFalse(result.passed)
        self.assertEqual(result.failed_checks, ("wall clearance",))

    def test_dropped_and_overdue_scans_are_not_used_as_fresh_observations(self):
        stress = StressSettings(faults=(FaultWindow("scan-drop", 150, 151),
                                       FaultWindow("scan-delay", 300, 301, 130)),
                                scan_max_delivery_ms=120)
        sim = Simulation(SCENARIOS["straight"], stress=stress, vehicle_model="legacy")
        for _ in range(4):
            self.assertTrue(sim.step())
        self.assertEqual((sim.dropped_scans, sim.stale_scans), (1, 1))
        self.assertEqual(sim.valid_scans, 2)
        self.assertEqual(sim.applied_pwm, 0)

    def test_scheduled_gap_does_not_consume_different_random_samples(self):
        sensor = SensorSettings()
        healthy = Simulation(SCENARIOS["straight"], sensor=sensor)
        gap = StressSettings(faults=(FaultWindow("front-gap", 0, 166),))
        faulty = Simulation(SCENARIOS["straight"], sensor=sensor, stress=gap)
        healthy.step()
        faulty.step()
        self.assertEqual(faulty.front_gap_scans, 1)
        self.assertEqual(healthy.random.getstate(), faulty.random.getstate())

    def test_grip_patch_reduces_turn_and_acceleration_together(self):
        zone = GripZone(-1.0, 1.0, -0.5, 0.5, 0.5)
        healthy = Simulation(SCENARIOS["straight"])
        low_grip = StressSettings(grip_zones=(zone,), combined_grip=True)
        reduced = Simulation(SCENARIOS["straight"], stress=low_grip)
        for sim in (healthy, reduced):
            sim.speed = 0.30
            sim.actual_steer = sim.target_steer = 17
            sim.applied_pwm = sim.motion.pwm_limit
            sim.integrate(0.005)
        self.assertLess(reduced.speed, healthy.speed)
        self.assertLess(abs(reduced.yaw_rate), abs(healthy.yaw_rate))
        self.assertGreater(reduced.traction_limited_ms, 0)

    def test_clearance_and_watchdog_are_scored_separately_from_lap_completion(self):
        track = TrackConfig()
        self.assertAlmostEqual(segment_body_clearance((0.30, -1), (0.30, 1), track), 0.15)
        self.assertEqual(segment_body_clearance((0, -1), (0, 1), track), 0)
        result = run_case("watchdog", 7, 1, 8)
        self.assertTrue(result.passed)
        self.assertEqual(result.laps, 0)
        self.assertFalse(result.collision)
        self.assertGreater(result.dropped_commands, 0)
