"""Run reproducible closed-course simulator experiments and audit the Pi lease budget."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from contextlib import ExitStack
from dataclasses import asdict, dataclass, replace
from itertools import combinations, product
import json
import math

from simulation.track.conditions import STRESS_PROFILES
from simulation.track.indoor import ENVIRONMENTS
from hardware import PARAMETERS
from simulation.track.simulator import (MOTION_PRESETS, PLANNERS, RAY_CLOCKS, SCENARIOS, SENSOR_PROFILES,
                                        Simulation, sensor_settings, wall_segments)


DEFAULT_SENSOR = SENSOR_PROFILES["rolling"][1]
POLICIES = ("baseline", "scan-coast")
STOP_CASES = {"watchdog": {"command watchdog"},
              "sensor-outage": {"command watchdog", "scan recovery timed out"}}
WHEEL_STOP_REASONS = {
    "encoder-outage": {"motion unavailable", "scan recovery timed out (motion unavailable)"},
    "motor-stall": {"wheel motion exceeds steering range", "rear wheel feedback timeout"},
}
MAX_STANDSTILL_S = 2.0
MAX_STOP_AFTER_FAULT_S = 2.0
MAX_CLEAN_SLOWDOWN_PCT = 5.0


@dataclass(frozen=True)
class Result:
    scenario: str
    case: str
    seed: int
    policy: str
    planner: str
    passed: bool
    failed_checks: tuple[str, ...]
    laps: int
    checkpoints: int
    next_checkpoint: str | None
    seconds: float
    last_lap_s: float | None
    distance_m: float
    collision: bool
    stop_reason: str | None
    min_sampled_wall_m: float
    min_wall_position_m: tuple[float, float]
    below_wall_threshold_s: float
    sectors: tuple[dict, ...]
    stop_context: dict | None
    scan_motion: dict
    power_cuts: dict
    straight_cruise: dict | None
    stationary_episodes: int
    longest_stationary_s: float
    scan_gaps: int
    recovered_scan_gaps: int
    max_scan_recovery_s: float
    standstill_after_fault_s: float | None
    speed_at_fault_mps: float | None
    final_obstacle_clearance_m: float | None
    straight_speed_mps: float | None
    first_corner_speed_mps: float | None
    bridged_scans: int
    waiting_scans: int
    front_gap_scans: int
    dropped_scans: int
    stale_scans: int
    dropped_commands: int
    delayed_commands: int
    watchdog_trips: int
    traction_limited_s: float
    low_grip_s: float
    peak_contact_demand: float
    short_leases: int
    lease_checks: int
    min_lease_slack_ms: int | None
    worst_lease_position_m: tuple[float, float] | None
    worst_lease_time_s: float | None
    vehicle_model: str
    environment: str
    observation: str
    rear_encoder_counts: tuple[int, int] | None


def run_case(case, seed, laps, max_seconds, *, scenario="loop", profile="baseline",
             sensor=DEFAULT_SENSOR, policy="baseline", min_clearance_m=0.10, max_recovery_s=1.25,
             planner="route", vehicle_model="four-wheel", environment="nominal", feedback=True, max_speed=None,
             motion_config=None, environment_config=None, wheel_pi=None, sensor_calibration=None):
    sensor = replace(sensor, seed=seed)
    stress = STRESS_PROFILES[case]
    controller_motion = MOTION_PRESETS[profile][1]
    if max_speed is not None:
        controller_motion = replace(controller_motion, max_speed=max_speed)
    motion = motion_config if motion_config is not None else controller_motion
    sim = Simulation(SCENARIOS[scenario], motion, sensor=sensor, feedback=feedback,
                     stress=stress, audit_leases=True, coast_on_scan_gap=policy == "scan-coast",
                     measure_clearance=True, wall_threshold_m=min_clearance_m, planner=planner,
                     vehicle_model=vehicle_model, environment=environment,
                     controller_motion=controller_motion, environment_config=environment_config, wheel_pi=wheel_pi,
                     sensor_calibration=sensor_calibration)
    limit_ms = math.floor(max_seconds * 1000)
    fault_start_ms = None
    expected_reasons = set()
    if case in STOP_CASES:
        fault_start_ms = stress.faults[0].start_ms
        expected_reasons = STOP_CASES[case]
    elif vehicle_model == "four-wheel" and environment in WHEEL_STOP_REASONS:
        if environment == "motor-stall" or feedback:
            fault_start_ms = round(1000 * (sim.environment.encoder_outage_s or sim.environment.stall_right_s))
            expected_reasons = WHEEL_STOP_REASONS[environment]
    speed_at_fault = None
    while sim.now_ms < limit_ms:
        next_limit = limit_ms
        if fault_start_ms is not None and sim.now_ms < fault_start_ms:
            next_limit = min(limit_ms, fault_start_ms)
        alive = sim.step(target_laps=laps, end_ms=next_limit)
        if sim.now_ms == fault_start_ms:
            speed_at_fault = sim.speed
        if not alive or sim.completed_laps >= laps:
            break
    failures = []
    if sim.collision:
        failures.append("collision")
    if sim.minimum_clearance < min_clearance_m:
        failures.append("wall clearance")
    standstill_after_fault = None
    obstacle_clearance = (sim.wall_clearance(wall_segments(sim.environment.obstacles))
                          if sim.environment.obstacles else None)
    if fault_start_ms is not None:
        if speed_at_fault is None or speed_at_fault < 0.03:
            failures.append("vehicle moving at fault onset")
        if (sim.stop_reason not in expected_reasons or sim.can_advance or sim.applied_pwm > 0
                or sim.stop_context is None or sim.stop_context["time_s"] * 1000 < fault_start_ms):
            failures.append("expected fault stop")
        if speed_at_fault is not None and sim.stopped_since_ms is not None:
            standstill_after_fault = max(0, sim.stopped_since_ms - fault_start_ms) / 1000
        if standstill_after_fault is None or not 0 <= standstill_after_fault <= MAX_STOP_AFTER_FAULT_S:
            failures.append("fault-to-standstill time")
    elif environment == "blocked":
        obstruction = sim.stop_reason or ""
        if (not sim.started_moving or sim.can_advance or sim.applied_pwm > 0 or sim.speed >= 0.03
                or obstacle_clearance > sim.controller.lookahead()
                or not any(cause in obstruction for cause in ("clearance", "clear distance", "obstacle",
                                                             "held steering is unsafe"))):
            failures.append("obstacle stop")
    else:
        if sim.completed_laps < laps:
            failures.append("lap completion")
        if sim.watchdog_trips:
            failures.append("unexpected watchdog")
        if (sim.scan_gaps != sim.recovered_scan_gaps or sim.longest_recovery_ms / 1000 > max_recovery_s):
            failures.append("scan gap recovery")
        if sim.longest_stop_ms / 1000 > MAX_STANDSTILL_S or (case == "clean" and sim.stop_episodes):
            failures.append("in-run standstill")
    checkpoints = sim.completed_laps * len(sim.scenario.checkpoints) + sim.checkpoint_index
    reason = sim.stop_reason
    if reason is None and not sim.collision and sim.completed_laps < laps:
        reason = "simulation time limit"
    sectors = sim.split_history.copy()
    if sim.scenario.checkpoints and sim.now_ms > sim.split_start_ms:
        sectors.append(sim.current_split())
    return Result(
        vehicle_model=vehicle_model, environment=sim.environment.name,
        observation=("unmeasured" if not feedback else "encoder" if vehicle_model == "four-wheel" else "legacy-state"),
        rear_encoder_counts=sim.plant.encoder_counts if vehicle_model == "four-wheel" else None,
        scenario=scenario, case=case, seed=seed, policy=policy, planner=planner,
        passed=not failures, failed_checks=tuple(failures),
        laps=sim.completed_laps,
        checkpoints=checkpoints,
        next_checkpoint=(sim.scenario.checkpoints[sim.checkpoint_index].label
                         if sim.scenario.checkpoints and sim.completed_laps < laps else None),
        seconds=round(sim.now_ms / 1000, 3), last_lap_s=sim.last_lap_s,
        distance_m=round(sim.distance, 3), collision=sim.collision, stop_reason=reason,
        min_sampled_wall_m=round(sim.minimum_clearance, 4),
        min_wall_position_m=tuple(round(axis, 3) for axis in sim.minimum_clearance_position),
        below_wall_threshold_s=sim.below_wall_ms / 1000,
        sectors=tuple(dict(asdict(split), label=sim.scenario.checkpoints[split.checkpoint - 1].label,
                           mean_speed_mps=split.mean_speed) for split in sectors),
        stop_context=sim.stop_context,
        scan_motion=sim.motion_scan_metrics(),
        power_cuts=sim.power_cut_metrics(),
        straight_cruise=sim.straight_cruise,
        stationary_episodes=sim.stop_episodes,
        longest_stationary_s=round(sim.longest_stop_ms / 1000, 3), scan_gaps=sim.scan_gaps,
        recovered_scan_gaps=sim.recovered_scan_gaps, max_scan_recovery_s=sim.longest_recovery_ms / 1000,
        standstill_after_fault_s=standstill_after_fault,
        speed_at_fault_mps=round(speed_at_fault, 4) if speed_at_fault is not None else None,
        final_obstacle_clearance_m=round(obstacle_clearance, 4) if obstacle_clearance is not None else None,
        straight_speed_mps=round(sim.splits[1].mean_speed, 3) if 1 in sim.splits else None,
        first_corner_speed_mps=round(sim.splits[2].mean_speed, 3) if 2 in sim.splits else None,
        bridged_scans=sim.bridged_scans,
        waiting_scans=sim.waiting_scans, front_gap_scans=sim.front_gap_scans,
        dropped_scans=sim.dropped_scans, stale_scans=sim.stale_scans,
        dropped_commands=sim.dropped_commands, delayed_commands=sim.delayed_commands,
        watchdog_trips=sim.watchdog_trips, traction_limited_s=round(sim.traction_limited_ms / 1000, 3),
        low_grip_s=round(sim.low_grip_ms / 1000, 3),
        peak_contact_demand=round(sim.peak_contact_ratio, 3), short_leases=sim.short_leases,
        lease_checks=sim.lease_checks, min_lease_slack_ms=sim.min_lease_slack_ms,
        worst_lease_position_m=(tuple(round(axis, 3) for axis in sim.worst_lease_position)
                                if sim.worst_lease_position is not None else None),
        worst_lease_time_s=sim.worst_lease_time_s,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", choices=STRESS_PROFILES, default=list(STRESS_PROFILES))
    parser.add_argument("--seeds", nargs="+", type=int, default=[7, 23])
    parser.add_argument("--jobs", type=int, default=1, help="Concurrent independent simulator runs")
    parser.add_argument("--policies", nargs="+", choices=POLICIES, default=["scan-coast"])
    parser.add_argument("--planners", nargs="+", choices=PLANNERS, default=["route"])
    parser.add_argument("--vehicle-model", choices=("four-wheel", "legacy"), default="four-wheel")
    parser.add_argument("--environment", choices=ENVIRONMENTS, default="nominal")
    parser.add_argument("--feedback", action=argparse.BooleanOptionalAction, default=True,
                        help="Provide encoder observations or exercise unmeasured operation")
    circuits = tuple(name for name, scenario in SCENARIOS.items() if scenario.checkpoints)
    parser.add_argument("--scenario", nargs="+", choices=circuits, default=["loop"],
                        help="One or more circuits to evaluate with the same settings")
    parser.add_argument("--profile", choices=MOTION_PRESETS, default="baseline")
    parser.add_argument("--max-speed", type=float, help="Model speed at maximum forward PWM in m/s")
    parser.add_argument("--lidar", choices=tuple(k for k, (_, s) in SENSOR_PROFILES.items() if s.rolling),
                        default="rolling")
    parser.add_argument("--scan-ms", type=int, help="Lidar rotation period in milliseconds")
    parser.add_argument("--point-rate-hz", type=int, help="Lidar measurements per second")
    parser.add_argument("--ray-clock", choices=RAY_CLOCKS, default="sample-clock")
    parser.add_argument("--no-spike-guard", action="store_true", help="Disable isolated-return handling for comparison")
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--max-seconds", type=float, help="Simulation-time limit per run")
    parser.add_argument("--min-clearance-m", type=float, default=0.10)
    parser.add_argument("--max-recovery-s", type=float, default=1.25)
    parser.add_argument("--json", action="store_true", help="Write machine-readable results to standard output")
    parser.add_argument("--sector-details", action="store_true", help="Print a row for each completed or partial sector")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    if args.max_speed is not None and (not math.isfinite(args.max_speed) or not 0.05 <= args.max_speed <= 2.0):
        parser.error("--max-speed must be within 0.05..2 m/s")
    limit = args.max_seconds if args.max_seconds is not None else 180 * args.laps
    if args.laps <= 0 or not math.isfinite(limit) or limit <= 0:
        parser.error("--laps and --max-seconds must be positive")
    if not math.isfinite(args.min_clearance_m) or args.min_clearance_m < 0:
        parser.error("--min-clearance-m must be finite and nonnegative")
    if not math.isfinite(args.max_recovery_s) or args.max_recovery_s <= 0:
        parser.error("--max-recovery-s must be finite and positive")
    base_sensor = SENSOR_PROFILES[args.lidar][1]
    try:
        sensor = sensor_settings(base_sensor,
                                 base_sensor.period_ms if args.scan_ms is None else args.scan_ms,
                                 base_sensor.point_rate_hz if args.point_rate_hz is None else args.point_rate_hz,
                                 base_sensor.max_range_m)
    except ValueError as error:
        parser.error(str(error))
    sensor = replace(sensor, ray_clock=args.ray_clock, guard_spikes=not args.no_spike_guard)
    results = []
    jobs = product(args.scenario, args.cases, args.seeds, args.policies, args.planners)
    options = dict(laps=args.laps, max_seconds=limit, profile=args.profile, sensor=sensor,
                   min_clearance_m=args.min_clearance_m, max_recovery_s=args.max_recovery_s,
                   vehicle_model=args.vehicle_model, environment=args.environment, feedback=args.feedback,
                   max_speed=args.max_speed)
    runs = (dict(options, scenario=scenario, case=case, seed=seed, policy=policy, planner=planner)
            for scenario, case, seed, policy, planner in jobs)
    with ExitStack() as stack:
        if args.jobs > 1:
            pool = stack.enter_context(ProcessPoolExecutor(max_workers=args.jobs))
            output = pool.map(run_with_options, runs)
        else:
            output = map(run_with_options, runs)
        for result in output:
            results.append(result)
            if not args.json:
                print_result(result, sectors=args.sector_details)
    clean = {(r.planner, r.scenario, r.seed, r.policy): r for r in results
             if r.case == "clean" and r.laps == args.laps}
    clean_slowdowns = {planner: {scenario: {} for scenario in args.scenario} for planner in args.planners}
    for planner, scenario, seed in product(args.planners, args.scenario, args.seeds):
        baseline, candidate = (clean.get((planner, scenario, seed, policy)) for policy in POLICIES)
        if baseline and candidate:
            clean_slowdowns[planner][scenario][seed] = 100 * (candidate.seconds / baseline.seconds - 1)
    policy_speed_regression = any(pct > MAX_CLEAN_SLOWDOWN_PCT for planner in clean_slowdowns.values()
                                  for track in planner.values() for pct in track.values())
    comparisons = compare_planners(results, args.laps)
    planner_speed_regression = any(pair["case"] == "clean" and pair["time_delta_pct"] is not None
                                   and pair["time_delta_pct"] > MAX_CLEAN_SLOWDOWN_PCT for pair in comparisons)
    speed_regression = policy_speed_regression or planner_speed_regression
    passes = sum(result.passed for result in results)
    if args.json:
        only_scenario = args.scenario[0] if len(args.scenario) == 1 else None
        policy_slowdowns = clean_slowdowns[args.planners[-1]]
        print(json.dumps({"scenario": only_scenario, "scenarios": args.scenario,
                          "profile": args.profile, "lidar": args.lidar, "cad": PARAMETERS["cad"],
                          "model_max_speed_mps": args.max_speed or MOTION_PRESETS[args.profile][1].max_speed,
                          "vehicle_model": args.vehicle_model, "environment": args.environment,
                          "observation": results[0].observation,
                          "planners": args.planners, "planner_comparisons": comparisons,
                          "scan_ms": sensor.period_ms, "point_rate_hz": sensor.point_rate_hz,
                          "ray_clock": sensor.ray_clock, "spike_guard": sensor.guard_spikes,
                          "target_laps": args.laps, "max_seconds": limit, "passed": passes,
                          "criteria": {"min_clearance_m": args.min_clearance_m,
                                       "max_scan_recovery_s": args.max_recovery_s,
                                       "max_in_run_standstill_s": MAX_STANDSTILL_S,
                                       "max_fault_to_standstill_s": MAX_STOP_AFTER_FAULT_S,
                                       "max_clean_slowdown_pct": MAX_CLEAN_SLOWDOWN_PCT},
                          "clean_slowdowns_pct": policy_slowdowns[only_scenario] if only_scenario else policy_slowdowns,
                          "policy_slowdowns_by_planner_pct": clean_slowdowns,
                          "speed_regression": speed_regression,
                          "runs": [asdict(result) for result in results]}, indent=2))
    else:
        for pair in comparisons:
            pace = (f" time_delta={pair['time_delta_s']:+.2f}s" if pair["time_delta_s"] is not None else "")
            print(f"Planner comparison {pair['scenario']} {pair['case']} seed={pair['seed']} {pair['policy']} "
                  f"{pair['baseline_planner']}->{pair['candidate_planner']} "
                  f"laps={pair['baseline_laps']}->{pair['candidate_laps']} gap_delta={pair['gap_delta_m']:+.3f}m "
                  f"cut_delta={pair['power_cut_delta']:+d} moving_off_delta={pair['moving_off_delta_s']:+.3f}s "
                  f"low_gap_delta={pair['below_threshold_delta_s']:+.2f}s{pace}")
        for planner, tracks in clean_slowdowns.items():
            for scenario, track in tracks.items():
                for seed, pct in track.items():
                    print(f"Clean comparison {scenario} {planner} seed={seed}: lap time change {pct:+.2f}% "
                          f"(limit +{MAX_CLEAN_SLOWDOWN_PCT:g}%)")
        if speed_regression:
            print(f"Clean lap time regression exceeded {MAX_CLEAN_SLOWDOWN_PCT:g}%.")
        print(f"Evaluation passed: {passes}/{len(results)}; collisions: {sum(result.collision for result in results)}. "
              f"Model={args.vehicle_model} environment={args.environment} observation={results[0].observation}")
    return int(passes != len(results) or speed_regression)


def run_with_options(options):
    return run_case(**options)


def compare_planners(results, laps):
    groups = {}
    for run in results:
        key = (run.scenario, run.case, run.seed, run.policy)
        groups.setdefault(key, {})[run.planner] = run
    pairs = []
    for group in groups.values():
        planners = (group[name] for name in PLANNERS if name in group)
        for old, new in combinations(planners, 2):
            both_complete = new.laps == old.laps == laps
            old_sectors = {(split["lap"], split["checkpoint"]): split for split in old.sectors if split["completed"]}
            sector_deltas = []
            for split in new.sectors:
                baseline = old_sectors.get((split["lap"], split["checkpoint"]))
                if split["completed"] and baseline is not None:
                    sector_deltas.append(dict(lap=split["lap"], checkpoint=split["checkpoint"], label=split["label"],
                                              gap_delta_m=round(split["min_wall_m"] - baseline["min_wall_m"], 4),
                                              time_delta_s=round(split["seconds"] - baseline["seconds"], 3)))
            pairs.append(dict(scenario=old.scenario, case=old.case, seed=old.seed, policy=old.policy,
                              baseline_planner=old.planner, candidate_planner=new.planner,
                              baseline_laps=old.laps, candidate_laps=new.laps,
                              **{f"{r.planner}_laps": r.laps for r in (old, new)},
                              **{f"{r.planner}_passed": r.passed for r in (old, new)},
                              power_cut_delta=new.power_cuts["count"] - old.power_cuts["count"],
                              moving_off_delta_s=round(new.power_cuts["moving_off_s"] - old.power_cuts["moving_off_s"], 3),
                              distance_delta_m=round(new.distance_m - old.distance_m, 3),
                              gap_delta_m=round(new.min_sampled_wall_m - old.min_sampled_wall_m, 4),
                              below_threshold_delta_s=round(new.below_wall_threshold_s - old.below_wall_threshold_s, 3),
                              time_delta_s=round(new.seconds - old.seconds, 3) if both_complete else None,
                              time_delta_pct=round(100 * (new.seconds / old.seconds - 1), 3) if both_complete else None,
                              sectors=sector_deltas))
    return pairs


def print_result(result, *, sectors=False):
    outcome = result.stop_reason or ("collision" if result.collision else "lap complete")
    status = "PASS" if result.passed else "FAIL"
    worst = ""
    if result.short_leases:
        x, y = result.worst_lease_position_m
        worst = (f" worst={result.min_lease_slack_ms}ms"
                 f"@{result.worst_lease_time_s:.2f}s({x:.2f},{y:.2f})")
    fault_stop = ""
    if result.standstill_after_fault_s is not None:
        fault_stop = f" fault_to_stop={result.standstill_after_fault_s:.3f}s"
    checks = f" failed=[{', '.join(result.failed_checks)}]" if result.failed_checks else ""
    x, y = result.min_wall_position_m
    print(f"{result.scenario:<12} {result.case:<13} {result.policy:<10} {result.planner:<9} seed={result.seed:<4} {status} "
          f"laps={result.laps} checkpoints={result.checkpoints} "
          f"time={result.seconds:.2f}s distance={result.distance_m:.2f}m "
          f"gap={result.min_sampled_wall_m:.3f}m@({x:.2f},{y:.2f}) "
          f"below_threshold={result.below_wall_threshold_s:.2f}s stops={result.stationary_episodes} "
          f"longest_stop={result.longest_stationary_s:.3f}s "
          f"scan_recovery={result.recovered_scan_gaps}/{result.scan_gaps} "
          f"recovery_max={result.max_scan_recovery_s:.3f}s{fault_stop} "
          f"wait={result.waiting_scans} bridge={result.bridged_scans} "
          f"front_gap={result.front_gap_scans} lost_scans={result.dropped_scans} "
          f"late_scans={result.stale_scans} lost_commands={result.dropped_commands} "
          f"late_commands={result.delayed_commands} "
          f"low_grip={result.low_grip_s:.2f}s grip_limit={result.traction_limited_s:.2f}s "
          f"power_cuts={result.power_cuts['count']} moving_off={result.power_cuts['moving_off_s']:.3f}s "
          f"lease_short={result.short_leases}/{result.lease_checks}{worst} outcome={outcome}{checks}", flush=True)
    if sectors:
        for split in result.sectors:
            progress = "done" if split["completed"] else "partial"
            print(f"  lap={split['lap']} sector={split['checkpoint']} {split['label']} {progress} "
                  f"time={split['seconds']:.2f}s speed={split['mean_speed_mps']:.3f}m/s gap={split['min_wall_m']:.3f}m "
                  f"below_threshold={split['below_wall_threshold_s']:.2f}s")
        if result.stop_context:
            stop = result.stop_context
            x, y = stop["position_m"]
            print(f"  stop time={stop['time_s']:.3f}s speed={stop['speed_mps']:.3f}m/s "
                  f"at=({x:.2f},{y:.2f}) steer={stop['actual_steer_deg']:+.1f}deg "
                  f"confirmed_path={stop['confirmed_path_m']:.3f}m")
