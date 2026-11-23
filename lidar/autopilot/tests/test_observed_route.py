import math
import unittest

from lidar.autopilot.planning.route import ObservedRoute, RouteVisibilityHistory, observed_route
from lidar.autopilot.planning.wall_track import TrackConfig
from lidar.geometry.scan_geometry import Beam, Pose, cell, project_sweep
from simulation.track.simulator import SCENARIOS, ray_scan, wall_segments


class ObservedRouteTests(unittest.TestCase):
    config = TrackConfig()

    def scan(self, name, x=0.0, y=0.0, yaw=0.0):
        pose = Pose(0.0, 0.0, 0.0)
        walls = wall_segments(SCENARIOS[name].outlines)
        beams = (Beam(a, d, q, 0, pose) for a, d, q in ray_scan(walls, x, y, math.radians(yaw)))
        return project_sweep(beams, pose)

    def test_straight_center_and_lateral_recovery(self):
        g = self.scan("loop-narrow", y=0.10)
        for geometry in (g, None):
            route = observed_route(g.points, self.config, geometry=geometry)
            self.assertGreater(route.length, 1.0)
            self.assertAlmostEqual(route.point_at(0.3)[1], -0.10, delta=0.01)
            self.assertLess(route.steer_at(0.30), -5)

    def test_narrowing_bend_preserves_approach_and_follows_visible_exit(self):
        g = self.scan("loop-narrow", 4.65, 0.0)
        route = observed_route(g.points, self.config, geometry=g)
        self.assertGreater(route.length, 0.95)
        self.assertLess(abs(route.steer_at(0.30)), 1)
        self.assertGreater(route.point_at(0.90)[1], 0.15)
        self.assertGreater(route.bilateral_length, 0.35)
        self.assertLess(route.bilateral_length, route.length - 0.1)
        self.assertTrue(all(g.visible(*p) for p in route.points[1:]))

    def test_occupied_cell_and_unknown_do_not_connect_the_route(self):
        g = self.scan("loop-narrow")
        g.occupied.add(cell(0.44, 0.0))
        route = observed_route(g.points, self.config, geometry=g)
        self.assertLess(route.length, 0.44)
        g.free.clear()
        self.assertIsNone(observed_route(g.points, self.config, geometry=g))

    def test_missing_wall_and_branch_do_not_create_a_turn(self):
        g = self.scan("loop-narrow")
        self.assertIsNone(observed_route([p for p in g.points if p[1] < 0], self.config, geometry=g))
        junction = self.scan("junction", 0.15)
        self.assertIsNone(observed_route(junction.points, self.config, geometry=junction))

    def test_route_interpolation_and_curvature_use_rear_axle(self):
        route = ObservedRoute.from_points([(0, 0), (0.30, 0), (0.30, 0.40)], self.config)
        self.assertAlmostEqual(route.length, 0.7)
        self.assertEqual(route.bilateral_length, 0)
        persisted = ObservedRoute.from_points(route.points, self.config, bilateral_length=0.20)
        self.assertEqual(persisted.bilateral_length, 0.20)
        x, y = route.point_at(0.50)
        self.assertAlmostEqual(x, 0.3)
        self.assertAlmostEqual(y, 0.2)
        expected = math.atan2(2 * self.config.wheelbase * y, (x + self.config.sensor_ahead) ** 2 + y ** 2)
        self.assertAlmostEqual(route.steer_at(0.5), math.degrees(expected))
        self.assertEqual(route.point_at(-0.1), route.points[0])
        self.assertEqual(route.point_at(2), route.points[-1])


class RouteVisibilityTests(unittest.TestCase):
    target = (0.923, 0.010)

    def sweep(self, *, omitted=(), near=()):
        pose = Pose(0, 0, 0)
        beams = (Beam(i, 0 if i in omitted else 500 if i in near else 2500,
                      0 if i in omitted else 12, 0, pose) for i in range(360))
        return project_sweep(beams, pose)

    def view(self, old, current):
        history = RouteVisibilityHistory()
        history.update(old, Pose(0.02, 0, 0), 3)
        return history.update(current, Pose(0.07, 0, 0), 3.16)

    def test_neighbouring_current_beams_and_registered_old_free_are_both_required(self):
        x, y = self.target
        current, old = self.sweep(), self.sweep()
        current.free.discard(cell(x, y))
        old.free = {cell(x + 0.05, y)}
        self.assertFalse(current.visible(x, y))
        self.assertTrue(self.view(old, current).visible(x, y))
        old.free = {cell(x - 0.05, y)}
        self.assertFalse(self.view(old, current).visible(x, y))

    def test_occluder_invalid_ray_broad_hole_and_occupied_cell_each_veto(self):
        x, y = self.target
        old = self.sweep()
        for scenario in ("occluder", "invalid slot", "broad hole", "occupied", "no far echo"):
            with self.subTest(scenario=scenario):
                current = self.sweep(omitted=(1,) if scenario == "invalid slot" else (),
                                     near=(0, 1) if scenario == "no far echo" else ())
                cx, cy = cell(x, y)
                current.free.discard((cx, cy))
                if scenario == "broad hole":
                    current.free.difference_update(((cx, cy - 1), (cx, cy + 1)))
                if scenario == "occupied":
                    current.occupied.add((cx, cy))
                if scenario == "occluder":
                    current.occupied.add(cell(0.48, 0.0))
                self.assertFalse(self.view(old, current).visible(x, y))

    def test_unmeasured_stale_jump_and_backwards_time_clear_history(self):
        history = RouteVisibilityHistory()
        current = self.sweep()
        origin = Pose(0, 0, 0)
        for pose, stamp in ((None, 1.16), (origin, 1.6), (Pose(0.65, 0, 0), 1.16),
                            (Pose(0, 0, 1.2), 1.16), (origin, 0.99)):
            with self.subTest(pose=pose, stamp=stamp):
                history.update(self.sweep(), origin, 1)
                self.assertIs(history.update(current, pose, stamp), current)

    def test_expired_raw_evidence_cannot_propagate_from_repaired_views(self):
        history = RouteVisibilityHistory()
        origin = Pose(0, 0, 0)
        x, y = self.target
        history.update(self.sweep(), origin, 1)
        current = self.sweep()
        current.free.discard(cell(x, y))
        for stamp in (1.16, 1.32):
            self.assertTrue(history.update(current, origin, stamp).visible(x, y))
        self.assertFalse(history.update(current, origin, 1.48).visible(x, y))
