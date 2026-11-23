from dataclasses import replace
import math
import random
import unittest
from unittest.mock import patch

from simulation.track.sensor_faults import RANGE_BIAS_TAU_S, SensorFaultModel
from simulation.track.simulator import (
    MotionSettings,
    SCENARIOS,
    SENSOR_PROFILES,
    SensorSettings,
    Simulation,
    ray_hit,
)


def _dropout_run_lengths(flags):
    runs = []
    current = 0
    for dropped in flags:
        if dropped:
            current += 1
        elif current:
            runs.append(current)
            current = 0
    if current:
        runs.append(current)
    return runs


class SensorFaultsTest(unittest.TestCase):
    def test_existing_presets_keep_new_fields_disabled_and_outputs_identical(self):
        for name in ("ideal", "rolling", "a1-cadence", "harsh", "vibration"):
            _, profile = SENSOR_PROFILES[name]
            self.assertEqual(profile.burst_start_probability, 0.0, name)
            self.assertEqual(profile.burst_length_points, 0, name)
            self.assertEqual(profile.range_bias_sigma_mm, 0.0, name)
            self.assertEqual(profile.delivery_jitter_ms, 0, name)

        default_sim = Simulation(SCENARIOS["loop"], sensor=SENSOR_PROFILES["rolling"][1], feedback=True)
        explicit_zero = Simulation(
            SCENARIOS["loop"],
            sensor=SensorSettings(
                burst_start_probability=0.0,
                burst_length_points=0,
                range_bias_sigma_mm=0.0,
                delivery_jitter_ms=0,
            ),
            feedback=True,
        )
        self.assertFalse(default_sim.sensor_faults.enabled)
        for _ in range(12):
            default_sim.step()
            explicit_zero.step()
        self.assertEqual(default_sim.samples, explicit_zero.samples)
        self.assertEqual(default_sim.sensor, explicit_zero.sensor)
        self.assertEqual(default_sim.distance, explicit_zero.distance)
        self.assertEqual(default_sim.scan_age_ms, explicit_zero.scan_age_ms)
        self.assertEqual(default_sim.random.random(), explicit_zero.random.random())
        self.assertEqual(default_sim.vibration_random.random(), explicit_zero.vibration_random.random())

    def test_named_disturbed_rolling_preset_activates_all_phenomena(self):
        self.assertIn("disturbed", SENSOR_PROFILES)
        _, preset = SENSOR_PROFILES["disturbed"]
        self.assertTrue(preset.rolling)
        self.assertGreater(preset.burst_start_probability, 0.0)
        self.assertGreater(preset.burst_length_points, 0)
        self.assertGreater(preset.range_bias_sigma_mm, 0.0)
        self.assertGreater(preset.delivery_jitter_ms, 0)

        sim = Simulation(SCENARIOS["loop"], sensor=preset, feedback=True)
        for _ in range(15):
            sim.step()
        self.assertTrue(sim.samples)
        self.assertGreater(sim.sensor_faults.burst_drops, 0)

    def test_burst_dropouts_are_contiguous_and_leave_physical_walls_intact(self):
        burst_length = 6
        model = SensorFaultModel(
            SensorSettings(burst_start_probability=0.05, burst_length_points=burst_length, seed=7)
        )
        flags = [model.next_beam_dropped() for _ in range(1200)]
        runs = _dropout_run_lengths(flags)
        interior = runs[:-1] if flags[-1] else runs
        self.assertGreater(len(interior), 0)
        for length in interior:
            self.assertEqual(length % burst_length, 0)

        burst_sensor = SensorSettings(
            dropout=0.0,
            noise_mm=0.0,
            guard_spikes=False,
            burst_start_probability=0.05,
            burst_length_points=burst_length,
            seed=7,
        )
        plain_sensor = SensorSettings(dropout=0.0, noise_mm=0.0, guard_spikes=False, seed=7)
        sim = Simulation(SCENARIOS["straight"], sensor=burst_sensor, feedback=True)
        plain = Simulation(SCENARIOS["straight"], sensor=plain_sensor, feedback=True)
        self.assertEqual(sim.walls, plain.walls)
        self.assertEqual(ray_hit(sim.walls, 0.0, 0.0, 1.0, 0.0), ray_hit(plain.walls, 0.0, 0.0, 1.0, 0.0))

        collected = []
        previous_samples = sim.samples
        # Normal-incidence echoes isolate the burst mask from independent
        # grazing-angle dropouts, which can join two otherwise distinct bursts.
        with patch("simulation.track.simulator.ray_hit", return_value=(1.0, 1.0)):
            for _ in range(5):
                sim.step()
                if sim.samples and sim.samples is not previous_samples:
                    collected.extend(quality == 0 for _, _, quality in sim.samples)
                    previous_samples = sim.samples
        sim_runs = _dropout_run_lengths(collected)
        sim_interior = sim_runs[:-1] if collected[-1] else sim_runs
        self.assertGreater(len(sim_interior), 0)
        for length in sim_interior:
            self.assertEqual(length % burst_length, 0)

        all_dropped = Simulation(
            SCENARIOS["straight"],
            sensor=SensorSettings(burst_start_probability=1.0, burst_length_points=4, seed=7),
        )
        all_dropped.step()
        self.assertEqual(all_dropped.sensor_faults.burst_drops, len(all_dropped.sweep) + len(all_dropped.scans[0][3]))
        self.assertTrue(all(sample.quality == 0 for sample in all_dropped.scans[0][3]))
        all_dropped.rear_x = 3.2 - all_dropped.config.sensor_ahead
        self.assertTrue(all_dropped.body_contacts_wall())

    def test_reproducibility_and_independent_rng_streams(self):
        preset = replace(SENSOR_PROFILES["disturbed"][1], seed=19)
        first = Simulation(SCENARIOS["loop"], sensor=preset, feedback=True)
        second = Simulation(SCENARIOS["loop"], sensor=preset, feedback=True)
        for _ in range(10):
            first.step()
            second.step()
        self.assertEqual(first.samples, second.samples)
        self.assertEqual(first.sensor, second.sensor)
        self.assertEqual(first.distance, second.distance)
        self.assertEqual(first.last_scan_delivery, second.last_scan_delivery)
        self.assertEqual(first.sensor_faults.range_bias_mm, second.sensor_faults.range_bias_mm)
        self.assertEqual(first.sensor_faults.burst_drops, second.sensor_faults.burst_drops)

        other_seed = Simulation(SCENARIOS["loop"], sensor=replace(preset, seed=29), feedback=True)
        for _ in range(10):
            other_seed.step()
        self.assertNotEqual(first.sensor_faults.range_bias_mm, other_seed.sensor_faults.range_bias_mm)

        # Each phenomenon keeps its own RNG stream independent from the others.
        combined = SensorFaultModel(
            SensorSettings(
                seed=7,
                burst_start_probability=0.05,
                burst_length_points=4,
                range_bias_sigma_mm=15.0,
                delivery_jitter_ms=20,
            )
        )
        burst_only = SensorFaultModel(SensorSettings(seed=7, burst_start_probability=0.05, burst_length_points=4))
        bias_only = SensorFaultModel(SensorSettings(seed=7, range_bias_sigma_mm=15.0))
        jitter_only = SensorFaultModel(SensorSettings(seed=7, delivery_jitter_ms=20))

        self.assertEqual(
            [combined.next_beam_dropped() for _ in range(200)],
            [burst_only.next_beam_dropped() for _ in range(200)],
        )
        self.assertEqual(
            [combined.advance_sweep() for _ in range(25)],
            [bias_only.advance_sweep() for _ in range(25)],
        )
        self.assertEqual(
            [combined.sample_delivery_jitter_ms() for _ in range(25)],
            [jitter_only.sample_delivery_jitter_ms() for _ in range(25)],
        )

        # Extra phenomena do not consume Simulation.random or Simulation.vibration_random draws.
        plain = Simulation(SCENARIOS["straight"], sensor=SensorSettings(seed=7, spike_rate=0.02), feedback=True)
        biased = Simulation(
            SCENARIOS["straight"],
            sensor=SensorSettings(seed=7, spike_rate=0.02, range_bias_sigma_mm=15.0),
            feedback=True,
        )
        plain.step()
        biased.step()
        self.assertEqual(plain.random.random(), biased.random.random())
        self.assertEqual(plain.vibration_random.random(), biased.vibration_random.random())
        self.assertNotEqual(biased.sensor_faults.range_bias_mm, random.Random(7).gauss(0.0, 15.0))

    def test_range_bias_persists_within_sweep_and_follows_ar1_across_sweeps(self):
        base_sensor = SensorSettings(
            noise_mm=0.0,
            offset_mm=0.0,
            yaw_deg=0.0,
            dropout=0.0,
            angular_speed_ripple=0.0,
            range_bias_sigma_mm=25.0,
            guard_spikes=False,
        )
        seed = next(
            candidate for candidate in range(1, 20)
            if abs(SensorFaultModel(replace(base_sensor, seed=candidate)).range_bias_mm) > 5.0
        )
        sensor = replace(base_sensor, seed=seed)
        sim = Simulation(SCENARIOS["straight"], sensor=sensor, calibrate_lidar=False, feedback=True)
        expected_bias = sim.sensor_faults.range_bias_mm
        self.assertGreater(abs(expected_bias), 5.0)
        sim.step()
        sim.step()  # Delivers sweep 0 while vehicle has not yet started moving.
        pose_x, pose_y = sim.sensor
        errors = []
        for angle, distance_mm, quality in sim.samples:
            if quality <= 0:
                continue
            bearing = sim.yaw + math.radians(angle)
            truth_m, _ = ray_hit(sim.walls, pose_x, pose_y, math.cos(bearing), math.sin(bearing))
            errors.append(distance_mm - 1000.0 * truth_m)
        self.assertGreater(len(errors), 250)
        self.assertLessEqual(max(errors) - min(errors), 0.5)
        self.assertAlmostEqual(sum(errors) / len(errors), expected_bias, delta=0.3)

        model = SensorFaultModel(SensorSettings(range_bias_sigma_mm=20.0, period_ms=166, seed=11))
        initial_bias = model.range_bias_mm
        for _ in range(100):
            model.next_beam_dropped()
        self.assertEqual(model.range_bias_mm, initial_bias)

        series = [initial_bias]
        for _ in range(600):
            series.append(model.advance_sweep())
        expected_phi = math.exp(-0.166 / RANGE_BIAS_TAU_S)
        numerator = sum(a * b for a, b in zip(series, series[1:]))
        denominator = sum(a * a for a in series[:-1])
        empirical_phi = numerator / denominator
        self.assertAlmostEqual(empirical_phi, expected_phi, delta=0.12)

    def test_bounded_delivery_jitter_never_precedes_capture_and_remains_fifo(self):
        # Non-rolling with zero base delay and large jitter exercises capture-time clamping and FIFO ordering.
        non_rolling = Simulation(
            SCENARIOS["straight"],
            motion=MotionSettings(sensor_delay_ms=0),
            sensor=SensorSettings(rolling=False, period_ms=70, delivery_jitter_ms=100, seed=11),
            feedback=True,
        )
        deliveries = [non_rolling.last_scan_delivery]
        for _ in range(25):
            non_rolling.step()
            deliveries.append(non_rolling.last_scan_delivery)
        prev_delivery = -1
        for capture_ms, delivery_ms in deliveries:
            self.assertGreaterEqual(delivery_ms, capture_ms)
            self.assertGreaterEqual(delivery_ms, prev_delivery)
            self.assertLessEqual(delivery_ms, capture_ms + 100)
            prev_delivery = delivery_ms
        self.assertGreater(len({delivery - capture for capture, delivery in deliveries}), 1)

        # Rolling scan with nominal sensor delay respects pose cadence, bounds, and FIFO order.
        rolling = Simulation(
            SCENARIOS["loop"],
            motion=MotionSettings(sensor_delay_ms=35),
            sensor=SensorSettings(rolling=True, period_ms=166, pose_sample_ms=20, delivery_jitter_ms=25, seed=13),
            feedback=True,
        )
        rolling_deliveries = []
        for _ in range(25):
            rolling.step()
            rolling_deliveries.append(rolling.last_scan_delivery)
        prev_delivery = -1
        for capture_ms, delivery_ms in rolling_deliveries:
            self.assertGreaterEqual(delivery_ms, capture_ms)
            self.assertGreaterEqual(delivery_ms, math.ceil(capture_ms / 20) * 20)
            self.assertGreaterEqual(delivery_ms, prev_delivery)
            self.assertLessEqual(delivery_ms, capture_ms + 35 + 25)
            prev_delivery = delivery_ms
        self.assertGreater(len({delivery - capture for capture, delivery in rolling_deliveries}), 1)

    def test_invalid_sensor_fault_settings_raise_value_error(self):
        for bad in (
            SensorSettings(burst_start_probability=-0.01),
            SensorSettings(burst_start_probability=1.1),
            SensorSettings(burst_start_probability=math.nan),
            SensorSettings(burst_length_points=-1),
            SensorSettings(burst_length_points=True),
            SensorSettings(burst_length_points=2.5),
            SensorSettings(range_bias_sigma_mm=-1.0),
            SensorSettings(range_bias_sigma_mm=math.inf),
            SensorSettings(delivery_jitter_ms=-5),
            SensorSettings(delivery_jitter_ms=True),
            SensorSettings(delivery_jitter_ms=5.5),
        ):
            with self.assertRaises(ValueError):
                SensorFaultModel(bad)


if __name__ == "__main__":
    unittest.main()
