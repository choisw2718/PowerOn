"""Train and evaluate a bounded speed policy on the offline simulator."""

import argparse
from collections import Counter
from dataclasses import replace
import json
import math

from simulation.track.conditions import STRESS_PROFILES
from simulation.track.simulator import SCENARIOS, SENSOR_PROFILES, Simulation
from simulation.track.wheel_pi import WheelPISettings

from .speed_learning import SPEED_SCALES, SpeedLearner, speed_state


def progress(sim):
    checkpoints = sim.completed_laps * len(sim.scenario.checkpoints) + sim.checkpoint_index
    return sim.now_ms / 1000, sim.distance, checkpoints, sim.below_wall_ms / 1000


class EpisodeSpeedPolicy:
    """Learner sees the drive gate's observations; reward can use simulator truth."""

    def __init__(self, learner, epsilon, training):
        self.learner = learner
        self.epsilon = epsilon
        self.training = training
        self.sim = None
        self.previous = None
        self.actions = Counter()

    def __call__(self, speed, target, steer, clearance, age):
        state = speed_state(speed, target, steer, clearance, age)
        self.learn(state)
        action = self.learner.choose(state, epsilon=self.epsilon)
        self.previous = state, action, progress(self.sim)
        self.actions[SPEED_SCALES[action]] += 1
        return SPEED_SCALES[action]

    def learn(self, next_state, *, terminated=False, completed=False):
        if self.previous is None or not self.training:
            return
        state, action, previous = self.previous
        seconds, distance, checkpoints, below_wall_s = (
            after - before for before, after in zip(previous, progress(self.sim))
        )
        reward = (4.0 * distance + 0.75 * checkpoints - 0.08 * seconds - 3.0 * below_wall_s)
        if terminated:
            reward += 8.0 if completed else -12.0 if self.sim.collision else -4.0
        self.learner.update(state, action, reward, next_state, terminated=terminated)

    def finish(self, *, terminated, completed):
        if self.previous is not None:
            # A run can hit its time budget between two usable observations.
            self.learn(self.previous[0], terminated=terminated, completed=completed)


def run_episode(scenario, seed, max_seconds, *, learner=None, epsilon=0.0, training=False,
                environment="indoor-mixed", case="clean", wheel_pi=None):
    policy = EpisodeSpeedPolicy(learner, epsilon, training) if learner is not None else None
    sensor = replace(SENSOR_PROFILES["a1-cadence"][1], seed=seed)
    sim = Simulation(SCENARIOS[scenario], sensor=sensor, feedback=True, measure_clearance=True,
                     coast_on_scan_gap=True, stress=STRESS_PROFILES[case], planner="route",
                     environment=environment, wheel_pi=wheel_pi, speed_policy=policy)
    if policy is not None:
        policy.sim = sim
    limit = math.floor(max_seconds * 1000)
    alive = True
    while sim.now_ms < limit and alive and sim.completed_laps < 1:
        alive = sim.step(target_laps=1, end_ms=limit)
    completed = sim.completed_laps == 1
    terminated = completed or not alive
    if policy is not None:
        policy.finish(terminated=terminated, completed=completed)
    seconds, distance, checkpoints, below_wall_s = progress(sim)
    return {
        "scenario": scenario, "seed": seed, "completed": completed, "terminated": terminated,
        "truncated": not terminated, "seconds": seconds, "distance_m": round(distance, 3),
        "checkpoints": checkpoints, "collision": sim.collision,
        "min_wall_m": round(sim.minimum_clearance, 4), "below_wall_threshold_s": below_wall_s,
        "stop_reason": sim.stop_reason, "actions": dict(policy.actions) if policy is not None else {},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    circuits = tuple(name for name, scenario in SCENARIOS.items() if scenario.checkpoints)
    parser.add_argument("--train-scenario", choices=circuits, default="loop-narrow")
    parser.add_argument("--eval-scenario", choices=circuits, default="loop-narrow")
    parser.add_argument("--train-seeds", type=int, nargs="+", default=[7, 23, 41, 53])
    parser.add_argument("--eval-seeds", type=int, nargs="+", default=[101, 103])
    parser.add_argument("--policy-seed", type=int, default=7)
    parser.add_argument("--max-seconds", type=float, default=85.0)
    parser.add_argument("--wheel-pi", action="store_true", help="Use provisional Nano-style wheel PI in both arms")
    parser.add_argument("--case", choices=STRESS_PROFILES, default="clean")
    args = parser.parse_args(argv)
    if not math.isfinite(args.max_seconds) or args.max_seconds < 0.001:
        parser.error("max-seconds must be finite and at least 0.001")
    if set(args.train_seeds) & set(args.eval_seeds) or len(set(args.eval_seeds)) != len(args.eval_seeds):
        parser.error("evaluation seeds must be unique and disjoint from training seeds")
    wheel_pi = WheelPISettings() if args.wheel_pi else None
    learner = SpeedLearner(args.policy_seed)
    training = [
        run_episode(args.train_scenario, seed, args.max_seconds, learner=learner, epsilon=0.35 / math.sqrt(index + 1),
                    training=True, case=args.case, wheel_pi=wheel_pi)
        for index, seed in enumerate(args.train_seeds)
    ]
    evaluation = [
        {"seed": seed,
         "baseline": run_episode(args.eval_scenario, seed, args.max_seconds, case=args.case, wheel_pi=wheel_pi),
         "learned": run_episode(args.eval_scenario, seed, args.max_seconds, learner=learner,
                                case=args.case, wheel_pi=wheel_pi)}
        for seed in args.eval_seeds
    ]
    print(json.dumps({"policy_seed": args.policy_seed, "case": args.case, "wheel_pi": args.wheel_pi,
                      "max_seconds": args.max_seconds, "training": training, "evaluation": evaluation},
                     indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
