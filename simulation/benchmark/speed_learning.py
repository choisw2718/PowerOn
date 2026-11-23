"""Dependency-free tabular Q-learning baseline for offline speed-cap selection."""

from __future__ import annotations

from bisect import bisect_right
import math
import random


SPEED_SCALES = (0.55, 0.75, 1.0)
DEFAULT_ACTION = SPEED_SCALES.index(1.0)

# Transparent bin edges for observed/estimated quantities.
SPEED_BINS_MPS = (0.05, 0.20, 0.34)
TARGET_BINS_MPS = (0.05, 0.22, 0.35)
STEER_BINS_DEG = (4.0, 12.0, 22.0)
CLEAR_BINS_M = (0.35, 0.70, 1.30)
SCAN_AGE_BINS_S = (0.12, 0.25)

__all__ = ("SPEED_SCALES", "SpeedLearner", "speed_state")


def _bin_index(value: float, edges: tuple[float, ...], *, missing_bin: int = 0) -> int:
    if value is None:
        return missing_bin
    numeric = float(value)
    if math.isnan(numeric):
        return missing_bin
    return bisect_right(edges, numeric)


def speed_state(
    speed_mps: float,
    target_mps: float,
    steer_deg: float,
    clear_m: float,
    scan_age_s: float,
) -> tuple[int, int, int, int, int]:
    """Discretize observed/estimated driving quantities into a hashable bin tuple."""
    steer_mag = None if steer_deg is None else abs(float(steer_deg))
    return (
        _bin_index(speed_mps, SPEED_BINS_MPS, missing_bin=0),
        _bin_index(target_mps, TARGET_BINS_MPS, missing_bin=0),
        _bin_index(steer_mag, STEER_BINS_DEG, missing_bin=len(STEER_BINS_DEG)),
        _bin_index(clear_m, CLEAR_BINS_M, missing_bin=0),
        _bin_index(scan_age_s, SCAN_AGE_BINS_S, missing_bin=len(SCAN_AGE_BINS_S)),
    )


class SpeedLearner:
    """Tabular Q-learner over discrete speed-cap scales with greedy 1.0 default."""

    scales = SPEED_SCALES
    default_action = DEFAULT_ACTION

    def __init__(self, seed, *, alpha: float = 0.2, gamma: float = 0.95) -> None:
        if not math.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
            raise ValueError(f"alpha must be within 0..1, got {alpha!r}")
        if not math.isfinite(gamma) or not 0.0 <= gamma <= 1.0:
            raise ValueError(f"gamma must be within 0..1, got {gamma!r}")
        self.seed = seed
        self.alpha = float(alpha)
        self.gamma = float(gamma)
        self._rng = random.Random(seed)
        self._q: dict[object, list[float]] = {}

    def q_values(self, state) -> tuple[float, ...]:
        """Return the current Q-values for ``state`` without mutating the table."""
        values = self._q.get(state)
        if values is None:
            return (0.0,) * len(self.scales)
        return tuple(values)

    def choose(self, state, epsilon: float = 0.0) -> int:
        """Select an action index in ``0..len(SPEED_SCALES)-1`` using epsilon-greedy."""
        if not math.isfinite(epsilon) or not 0.0 <= epsilon <= 1.0:
            raise ValueError(f"epsilon must be within 0..1, got {epsilon!r}")
        if epsilon > 0.0 and self._rng.random() < epsilon:
            return self._rng.randrange(len(self.scales))
        values = self._q.get(state)
        if values is None:
            return self.default_action
        best = max(values)
        if values[self.default_action] == best:
            return self.default_action
        for action in range(len(self.scales) - 1, -1, -1):
            if values[action] == best:
                return action
        return self.default_action

    def update(self, state, action: int, reward: float, next_state, *, terminated: bool = False) -> None:
        """Apply one tabular Q-learning update; bootstrap unless ``terminated`` is true."""
        if isinstance(action, bool) or not isinstance(action, int) or not 0 <= action < len(self.scales):
            raise ValueError(f"action must be an int in 0..{len(self.scales) - 1}, got {action!r}")
        reward_value = float(reward)
        if not math.isfinite(reward_value):
            raise ValueError(f"reward must be finite, got {reward!r}")
        values = self._q.get(state)
        if values is None:
            values = [0.0] * len(self.scales)
            self._q[state] = values
        if terminated:
            next_best = 0.0
        else:
            next_values = self._q.get(next_state)
            next_best = max(next_values) if next_values is not None else 0.0
        target = reward_value + (0.0 if terminated else self.gamma * next_best)
        values[action] += self.alpha * (target - values[action])
