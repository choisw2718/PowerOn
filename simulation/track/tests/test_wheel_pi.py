from dataclasses import FrozenInstanceError, is_dataclass
import unittest

from lidar.autopilot.planning.wall_track import TrackConfig
from lidar.geometry.scan_geometry import Pose
from simulation.track.conditions import GripZone, StressSettings
from simulation.track.four_wheel import FourWheelPlant
from simulation.track.indoor import IndoorEnvironment, indoor_environment
from simulation.track.simulator import MotionSettings, SensorSettings
from simulation.track.wheel_pi import SpeedPI, WheelPISettings


def make_plant(environment=None, *, wheel_pi=None):
    return FourWheelPlant(
        TrackConfig(),
        IndoorEnvironment("nominal") if environment is None else environment,
        Pose(0.0, 0.0, 0.0),
        SensorSettings(),
        wheel_pi=wheel_pi,
    )


class WheelPITest(unittest.TestCase):
    def test_settings_contract_and_validation(self):
        self.assertTrue(is_dataclass(WheelPISettings))
        defaults = WheelPISettings()
        self.assertGreater(defaults.kp, 0.0)
        self.assertGreater(defaults.ki, 0.0)
        self.assertGreaterEqual(defaults.feedforward, 0.0)
        self.assertAlmostEqual(defaults.sample_s, 0.010, places=4)
        with self.assertRaises(FrozenInstanceError):
            defaults.kp = 1.0

        explicit = WheelPISettings(4.0, 2.0, 8.0, 0.012)
        self.assertEqual((explicit.kp, explicit.ki, explicit.feedforward, explicit.sample_s),
                         (4.0, 2.0, 8.0, 0.012))

        for bad in (
            dict(sample_s=0.0),
            dict(sample_s=-0.01),
            dict(kp=-1.0),
            dict(ki=-1.0),
            dict(feedforward=-1.0),
            dict(max_voltage=0.0),
            dict(bus_voltage=0.0),
            dict(filter_tau_s=-0.1),
            dict(kp=float("nan")),
        ):
            with self.assertRaises(ValueError):
                WheelPISettings(**bad)

    def test_default_plant_matches_explicit_none_byte_for_byte(self):
        env = indoor_environment("indoor-mixed", 7)
        default_plant = FourWheelPlant(TrackConfig(), env, Pose(0.0, 0.0, 0.0), SensorSettings())
        none_plant = FourWheelPlant(TrackConfig(), env, Pose(0.0, 0.0, 0.0), SensorSettings(), wheel_pi=None)
        motion = MotionSettings()
        stress = StressSettings()

        commands = [(45.0, 0.0, 0.0)] * 80 + [(30.0, 15.0, 15.0)] * 60 + [(0.0, 0.0, 0.0)] * 40
        for pwm, center_cmd, front_target in commands:
            out_a = default_plant.advance(0.005, pwm, center_cmd, front_target, motion, stress, 0.8, 0.0, 0.0)
            out_b = none_plant.advance(0.005, pwm, center_cmd, front_target, motion, stress, 0.8, 0.0, 0.0)
            self.assertEqual(out_a, out_b)
            self.assertEqual(default_plant.ground_speeds, none_plant.ground_speeds)
            self.assertEqual(default_plant.wheel_speeds, none_plant.wheel_speeds)
            self.assertEqual(default_plant.encoder_counts, none_plant.encoder_counts)
            self.assertEqual(default_plant.front_angles, none_plant.front_angles)
            self.assertEqual(default_plant.measured_pose, none_plant.measured_pose)
            self.assertEqual(default_plant.total_measured_distance, none_plant.total_measured_distance)
            self.assertEqual(default_plant.total_measured_yaw, none_plant.total_measured_yaw)

    def test_speed_pi_saturation_and_conditional_anti_windup(self):
        slow = SpeedPI()
        fast = SpeedPI()
        self.assertGreater(
            slow.voltage(0.2, 0.1, 0.01, 4.0, 8.0, 2.0, 6.0),
            fast.voltage(0.2, 0.2, 0.01, 4.0, 8.0, 2.0, 6.0),
        )
        for _ in range(100):
            self.assertEqual(slow.voltage(0.5, 0.0, 0.01, 10.0, 10.0, 20.0, 3.0), 3.0)
        self.assertLess(slow.integral, 0.01)
        self.assertEqual(slow.voltage(0.2, 0.5, 0.01, 4.0, 8.0, 2.0, 6.0), 0.0)
        self.assertGreaterEqual(slow.integral, 0.0)
        slow.reset()
        self.assertEqual(slow.integral, 0.0)

    def test_plant_saturation_prevents_windup_and_recovers_promptly(self):
        settings = WheelPISettings(kp=16.0, ki=12.0, feedforward=12.85, sample_s=0.010, max_voltage=6.0)
        plant = make_plant(IndoorEnvironment("low-supply", supply_scale=0.50), wheel_pi=settings)
        motion = MotionSettings()
        stress = StressSettings()

        for _ in range(200):
            plant.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        self.assertEqual(plant.wheel_pi_loop.held_voltages, [6.0, 6.0])
        self.assertEqual(plant.wheel_pi_loop.integrals, (0.0, 0.0))

        # Command an achievable lower speed (15% -> 0.14 m/s); output drops off ceiling at once.
        for _ in range(4):
            plant.advance(0.005, 15.0, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        self.assertLess(plant.wheel_pi_loop.held_voltages[0], 6.0)
        for _ in range(1600):
            plant.advance(0.005, 15.0, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        for speed in plant.ground_speeds:
            self.assertAlmostEqual(speed, 0.14, delta=0.015)

    def test_hold_between_control_ticks_and_quantized_filtered_feedback(self):
        plant = make_plant(wheel_pi=WheelPISettings(sample_s=0.010))
        motion = MotionSettings()
        stress = StressSettings()

        plant.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        self.assertEqual(plant.wheel_pi_loop.held_voltages, [0.0, 0.0])
        self.assertEqual(plant.ground_speeds, (0.0, 0.0))

        plant.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        first_held = list(plant.wheel_pi_loop.held_voltages)
        self.assertGreater(first_held[0], 0.0)

        plant.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        self.assertEqual(plant.wheel_pi_loop.held_voltages, first_held)
        self.assertGreater(plant.ground_speeds[0], 0.0)

        for _ in range(15):
            plant.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        self.assertGreater(plant.encoder_counts[0], 0)
        self.assertGreater(plant.wheel_pi_loop.filtered_speeds[0], 0.0)
        self.assertLess(plant.wheel_pi_loop.filtered_speeds[0], plant.ground_speeds[0])

    def test_straight_speed_regulation_under_asymmetric_load(self):
        env = IndoorEnvironment(
            "asymmetric",
            left_motor_gain=0.80,
            right_motor_gain=1.05,
            supply_scale=0.85,
            load_scale=1.15,
        )
        open_loop = make_plant(env, wheel_pi=None)
        closed_loop = make_plant(env, wheel_pi=WheelPISettings())
        motion = MotionSettings()
        stress = StressSettings()

        for _ in range(1600):
            open_loop.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
            closed_loop.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)

        open_diff = abs(open_loop.ground_speeds[0] - open_loop.ground_speeds[1])
        closed_diff = abs(closed_loop.ground_speeds[0] - closed_loop.ground_speeds[1])
        self.assertGreater(open_diff, 0.08)
        self.assertLess(closed_diff, 0.02)
        for speed in closed_loop.ground_speeds:
            self.assertAlmostEqual(speed, motion.max_speed, delta=0.02)
        self.assertGreater(closed_loop.wheel_pi_loop.integrals[0], closed_loop.wheel_pi_loop.integrals[1])

    def test_stop_resets_integrators_and_outputs_immediately_and_coasts(self):
        plant = make_plant(IndoorEnvironment("sag", supply_scale=0.85), wheel_pi=WheelPISettings())
        motion = MotionSettings()
        stress = StressSettings()

        for _ in range(400):
            plant.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        speed_before_stop = plant.ground_speeds[0]
        self.assertGreater(speed_before_stop, 0.35)
        self.assertGreater(plant.wheel_pi_loop.integrals[0], 0.1)
        self.assertGreater(plant.wheel_pi_loop.held_voltages[0], 0.1)

        # Single 5 ms step (off the 10 ms sample tick) must still reset immediately.
        plant.advance(0.005, 0.0, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        self.assertEqual(plant.wheel_pi_loop.integrals, (0.0, 0.0))
        self.assertEqual(plant.wheel_pi_loop.held_voltages, [0.0, 0.0])
        self.assertLess(plant.ground_speeds[0], speed_before_stop)

        for _ in range(300):
            plant.advance(0.005, 0.0, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        self.assertEqual(plant.ground_speeds, (0.0, 0.0))
        self.assertIsNone(plant.drive_fault)

    def test_retains_encoder_slip_curvature_targets_and_no_progress_fault(self):
        motion = MotionSettings()
        stress = StressSettings()

        # Ground vs encoder slip on a split-grip patch.
        slip_env = IndoorEnvironment("split", grip_zones=(GripZone(0.5, 2.5, -0.5, -0.01, 0.5),))
        slip_plant = make_plant(slip_env, wheel_pi=WheelPISettings())
        for _ in range(25):
            slip_plant.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 1.0, 0.0, 0.0)
        self.assertGreater(slip_plant.wheel_speeds[0], slip_plant.ground_speeds[0])
        self.assertEqual(slip_plant.wheel_speeds[1], slip_plant.ground_speeds[1])

        # Curvature-adjusted per-wheel targets on a left turn (right wheel faster than left).
        turn_plant = make_plant(wheel_pi=WheelPISettings())
        for _ in range(200):
            turn_plant.advance(0.005, 30.0, 20.0, 20.0, motion, stress, 0.0, 0.0, 0.0)
        self.assertGreater(turn_plant.ground_speeds[1], turn_plant.ground_speeds[0])
        self.assertGreater(turn_plant.encoder_counts[1], turn_plant.encoder_counts[0])

        # Existing no-progress safety fault still trips on motor stall.
        stall_plant = make_plant(IndoorEnvironment("stall", stall_right_s=0.5), wheel_pi=WheelPISettings())
        for _ in range(250):
            stall_plant.advance(0.005, motion.pwm_limit, 0.0, 0.0, motion, stress, 0.0, 0.0, 0.0)
        self.assertEqual(stall_plant.drive_fault, "rear wheel feedback timeout")


if __name__ == "__main__":
    unittest.main()
