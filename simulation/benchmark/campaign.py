"""Compare paired, reproducible closed-course trials under uncertain conditions."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import json
import math

from simulation.track.conditions import GripZone
from simulation.track.indoor import IndoorEnvironment
from simulation.track.simulator import MOTION_PRESETS, SCENARIOS, SENSOR_PROFILES
from simulation.track.wheel_pi import WheelPISettings

from .design import latin_hypercube, summarize_trials
from .stress_benchmark import run_case


# Provisional sensitivity ranges. SI units unless a name specifies otherwise.
FACTOR_RANGES = {
    "start_offset_m": (-0.04, 0.04),
    "start_yaw_deg": (-2.0, 2.0),
    "motor_gain_delta": (-0.07, 0.07),
    "supply_scale": (0.85, 1.00),
    "load_scale": (1.0, 1.2),
    "right_rollout_scale": (0.985, 1.015),
    "right_servo_trim_deg": (-1.5, 1.5),
    "right_servo_rate_scale": (0.65, 1.00),
    "acceleration_scale": (0.75, 1.15),
    "coast_deceleration_scale": (0.70, 1.15),
    "local_grip_scale": (0.55, 1.00),
    "sensor_delay_ms": (25.0, 70.0),
    "command_delay_ms": (25.0, 100.0),
    "delivery_jitter_ms": (0.0, 25.0),
    "range_noise_mm": (4.0, 15.0),
    "residual_lidar_yaw_deg": (-1.2, 1.2),
    "residual_range_offset_mm": (-10.0, 10.0),
    "odometry_yaw_scale": (0.94, 1.04),
    "range_bias_sigma_mm": (0.0, 12.0),
    "return_dropout": (0.005, 0.055),
    "burst_start_probability": (0.0, 0.012),
}


def trial_settings(factors):
    motion = MOTION_PRESETS["baseline"][1]
    sensor = SENSOR_PROFILES["a1-cadence"][1]
    difference = factors["motor_gain_delta"]
    environment = IndoorEnvironment(
        "campaign",
        start_offset=factors["start_offset_m"], start_yaw=math.radians(factors["start_yaw_deg"]),
        left_motor_gain=1 + difference, right_motor_gain=1 - difference,
        supply_scale=factors["supply_scale"], load_scale=factors["load_scale"],
        right_rollout_scale=factors["right_rollout_scale"],
        right_servo_trim=factors["right_servo_trim_deg"], right_servo_rate_scale=factors["right_servo_rate_scale"],
        grip_zones=(GripZone(0.65, 2.15, -0.60, 0.0, factors["local_grip_scale"]),),
    )
    return (
        replace(motion, sensor_delay_ms=round(factors["sensor_delay_ms"]),
                command_delay_ms=round(factors["command_delay_ms"]),
                acceleration=motion.acceleration * factors["acceleration_scale"],
                deceleration=motion.deceleration * factors["coast_deceleration_scale"]),
        replace(sensor, noise_mm=factors["range_noise_mm"], dropout=factors["return_dropout"],
                yaw_deg=sensor.yaw_deg + factors["residual_lidar_yaw_deg"],
                offset_mm=sensor.offset_mm + factors["residual_range_offset_mm"],
                odometry_yaw_scale=factors["odometry_yaw_scale"],
                burst_start_probability=factors["burst_start_probability"], burst_length_points=16,
                range_bias_sigma_mm=factors["range_bias_sigma_mm"],
                delivery_jitter_ms=round(factors["delivery_jitter_ms"])),
        environment,
    )


def run_pair(options):
    scenario, case, seed, max_seconds, factors = options
    motion, sensor, environment = trial_settings(factors)
    calibrated = SENSOR_PROFILES["a1-cadence"][1]
    results = {}
    for label, controller in (("pwm", None), ("wheel-pi", WheelPISettings())):
        result = run_case(case, seed, 1, max_seconds, scenario=scenario, sensor=sensor, policy="scan-coast",
                          motion_config=motion, environment_config=environment, wheel_pi=controller,
                          sensor_calibration=(calibrated.yaw_deg, calibrated.offset_mm))
        results[label] = {
            "completed": result.laps == 1,
            "passed": result.passed,
            "collision": result.collision,
            "min_wall_m": result.min_sampled_wall_m,
            "seconds": result.seconds,
            "stop_reason": result.stop_reason,
            "failed_checks": result.failed_checks,
            "scan_gaps": result.scan_gaps,
            "watchdog_trips": result.watchdog_trips,
            "scan_motion": result.scan_motion,
        }
    return {"seed": seed, "factors": factors, "results": results}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    circuits = tuple(name for name, scenario in SCENARIOS.items() if scenario.checkpoints)
    parser.add_argument("--scenario", choices=circuits, default="loop-narrow")
    parser.add_argument("--case", choices=("clean", "sensor", "transport", "traction", "mixed"), default="clean")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--trials", type=int, default=8)
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--max-seconds", type=float, default=120.0)
    args = parser.parse_args(argv)
    if args.trials < 1 or args.jobs < 1 or not math.isfinite(args.max_seconds) or args.max_seconds <= 0:
        parser.error("trials, jobs and max-seconds must be positive")
    samples = latin_hypercube(FACTOR_RANGES, args.trials, args.seed)
    options = [(args.scenario, args.case, args.seed + index, args.max_seconds, factors)
               for index, factors in enumerate(samples)]
    if args.jobs == 1:
        trials = list(map(run_pair, options))
    else:
        with ProcessPoolExecutor(max_workers=args.jobs) as pool:
            trials = list(pool.map(run_pair, options))
    summary = {
        label: summarize_trials([dict(trial["results"][label], factors=trial["factors"]) for trial in trials])
        for label in ("pwm", "wheel-pi")
    }
    print(json.dumps({"scenario": args.scenario, "case": args.case, "design_seed": args.seed,
                      "max_seconds": args.max_seconds, "factor_ranges": FACTOR_RANGES,
                      "summary": summary, "trials": trials}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
