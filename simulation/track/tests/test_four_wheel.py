import math
import unittest

from simulation.track.indoor import indoor_environment
from simulation.track.simulator import SCENARIOS, SensorSettings, Simulation, ray_hit


class FourWheelTest(unittest.TestCase):
    def test_steering_envelope_covers_each_tire_and_bumper(self):
        sim = Simulation(SCENARIOS["loop"])
        plant = sim.plant
        for command in (-55, -12, 0, 12, 55):
            curvature = plant.steer(command, sim.motion, 2)
            left, right = plant.front_angles
            if command:
                self.assertEqual(math.copysign(1, curvature), math.copysign(1, command))
                self.assertGreater(abs(left) if command > 0 else abs(right), abs(right) if command > 0 else abs(left))
            for angle, (lo, hi) in zip(plant.front_angles, plant.front_limits):
                self.assertLessEqual(lo, angle)
                self.assertLessEqual(angle, hi)
            current = plant.envelope()
            self.assertGreaterEqual(sim.config.width, current.width)
            self.assertGreaterEqual(sim.config.front, current.front)
            self.assertGreaterEqual(sim.config.rear, current.rear)
            self.assertGreaterEqual(current.front + current.rear, plant.plate_length + 2 * plant.bumper)
            for polygon in plant.wheel_footprints():
                for x, y in polygon:
                    self.assertLessEqual(abs(y), current.width / 2 + 1e-9)
                    self.assertTrue(-current.rear - 1e-9 <= x - sim.config.sensor_ahead <= current.front + 1e-9)

    def test_quantized_dual_encoders_report_motion_without_steering_truth(self):
        sim = Simulation(SCENARIOS["loop"], feedback=True, sensor=SensorSettings(pose_sample_ms=20))
        sim.applied_pwm, sim.target_steer, sim.command_steer = 24, 18, 18
        sim.integrate(0.005)
        self.assertGreater(sim.speed, 0.0)
        self.assertEqual(sim.motion_feedback(5).speed_mps, 0.0)
        for _ in range(199):
            sim.integrate(0.005)
        left, right = sim.plant.encoder_counts
        self.assertTrue(isinstance(left, int) and 0 < left < right)
        measured = sim.motion_feedback(1000)
        self.assertGreater(measured.speed_mps, 0)
        self.assertLess(measured.yaw_rate, 0)
        self.assertIsNone(measured.steer_deg)
        self.assertIsNone(sim.motion_feedback(1200))

    def test_split_grip_affects_one_wheel_and_separates_pose_from_encoder_path(self):
        sim = Simulation(SCENARIOS["loop"], environment="split-grip", feedback=True,
                         sensor=SensorSettings(odometry_yaw_scale=1, odometry_distance_scale=1))
        sim.rear_x = 0.8
        start = sim.rear_x
        sim.applied_pwm = sim.motion.pwm_limit
        for _ in range(60):
            sim.integrate(0.005)
        self.assertGreater(sim.low_grip_ms, 0)
        self.assertNotAlmostEqual(*sim.plant.ground_speeds)
        self.assertGreater(sim.plant.wheel_speeds[0], sim.plant.ground_speeds[0])
        self.assertGreater(sim.plant.total_measured_distance, sim.rear_x - start)

    def test_braking_stops_both_wheels_and_encoder_counts_settle(self):
        sim = Simulation(SCENARIOS["loop"])
        sim.applied_pwm = sim.motion.pwm_limit
        for _ in range(100):
            sim.integrate(0.005)
        start = sim.distance
        sim.applied_pwm = 0
        for _ in range(200):
            sim.integrate(0.005)
        counts = sim.plant.encoder_counts
        self.assertGreater(sim.distance, start)
        self.assertEqual(sim.plant.ground_speeds, (0.0, 0.0))
        sim.integrate(0.02)
        self.assertEqual(sim.plant.encoder_counts, counts)

    def test_telemetry_loss_keeps_real_ticks_but_expires_feedback(self):
        sim = Simulation(SCENARIOS["loop"], environment="encoder-outage", feedback=True)
        sim.applied_pwm = sim.motion.pwm_limit
        for _ in range(800):
            sim.integrate(0.005)
        counts = sim.plant.encoder_counts
        for _ in range(30):
            sim.integrate(0.005)
        self.assertGreater(sim.plant.encoder_counts[0], counts[0])
        self.assertIsNotNone(sim.motion_feedback(3950))
        self.assertIsNone(sim.motion_feedback(4150))
        self.assertIsNone(sim.plant.drive_fault)

    def test_right_motor_stall_cuts_both_outputs_and_coasts(self):
        sim = Simulation(SCENARIOS["loop"], environment="motor-stall")
        sim.applied_pwm = sim.motion.pwm_limit
        for _ in range(900):
            sim.integrate(0.005)
        self.assertEqual(sim.stop_reason, "rear wheel feedback timeout")
        self.assertEqual(sim.applied_pwm, 0)
        for _ in range(250):
            sim.integrate(0.005)
        self.assertEqual(sim.plant.ground_speeds, (0.0, 0.0))

    def test_seeded_indoor_features_leave_the_obstacle_physical_when_echoes_are_weak(self):
        self.assertEqual(indoor_environment("indoor-mixed", 41), indoor_environment("indoor-mixed", 41))
        self.assertNotEqual(indoor_environment("indoor-mixed", 7), indoor_environment("indoor-mixed", 41))
        weak = Simulation(SCENARIOS["loop"], environment="weak-returns", sensor=SensorSettings())
        plain = Simulation(SCENARIOS["loop"], sensor=SensorSettings())
        self.assertEqual(weak.walls, plain.walls)
        for _ in range(4):
            weak.step()
        self.assertGreater(weak.weak_panel_misses, 0)
        self.assertGreater(weak.weak_panel_returns, 0)
        blocked = Simulation(SCENARIOS["straight"], environment="blocked")
        self.assertAlmostEqual(ray_hit(blocked.walls, 0, 0, 1, 0)[0], 1.6)
        blocked.rear_x = 1.6 - blocked.config.sensor_ahead
        self.assertTrue(blocked.body_contacts_wall())

    def test_floor_lip_changes_acceleration_and_creates_floor_echoes(self):
        rough = Simulation(SCENARIOS["loop"], environment="rough-floor", sensor=SensorSettings())
        smooth = Simulation(SCENARIOS["loop"], sensor=SensorSettings())
        for sim in (rough, smooth):
            sim.rear_x = 1.21
            sim.applied_pwm = sim.motion.pwm_limit
            for _ in range(20):
                sim.integrate(0.005)
        self.assertLess(rough.speed, smooth.speed)
        rough.step()
        self.assertGreater(rough.floor_returns, 0)


if __name__ == "__main__":
    unittest.main()
