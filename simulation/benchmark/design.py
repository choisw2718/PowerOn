"""Deterministic multivariate experiment design and exploratory campaign summaries."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
import hashlib
import math
import random
import statistics


_REQUIRED_TRIAL_KEYS = frozenset({"factors", "collision", "min_wall_m", "seconds", "completed"})


def _is_real_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _factor_rng(seed: int, factor: str) -> random.Random:
    payload = f"{seed}\x00{factor}".encode("utf-8")
    derived = int.from_bytes(hashlib.sha256(payload).digest(), "big")
    return random.Random(derived)


def _validate_ranges(ranges: Mapping[str, tuple[float, float]]) -> dict[str, tuple[float, float]]:
    if not isinstance(ranges, Mapping):
        raise TypeError("ranges must be a mapping of factor names to (low, high) bounds")
    if not ranges:
        raise ValueError("ranges must not be empty")

    validated: dict[str, tuple[float, float]] = {}
    for name in sorted(ranges):
        if not isinstance(name, str) or not name.strip():
            raise ValueError("factor names must be non-empty strings")
        bounds = ranges[name]
        if isinstance(bounds, (str, bytes)) or not isinstance(bounds, Sequence) or len(bounds) != 2:
            raise ValueError(f"bounds for factor {name!r} must be a 2-element (low, high) sequence")
        raw_low, raw_high = bounds
        if not _is_real_number(raw_low) or not _is_real_number(raw_high):
            raise TypeError(f"bounds for factor {name!r} must be numeric")
        low = float(raw_low)
        high = float(raw_high)
        if not (math.isfinite(low) and math.isfinite(high)):
            raise ValueError(f"bounds for factor {name!r} must be finite")
        if low > high:
            raise ValueError(f"bounds for factor {name!r} must satisfy low <= high")
        validated[name] = (0.0 if low == 0.0 else low, 0.0 if high == 0.0 else high)
    return validated


def _sample_factor_column(low: float, high: float, count: int, rng: random.Random) -> list[float]:
    if low == high:
        return [low] * count

    strata = list(range(count))
    rng.shuffle(strata)
    span = high - low
    finite_span = math.isfinite(span)
    mid = low * 0.5 + high * 0.5
    half_span = high * 0.5 - low * 0.5

    values: list[float] = []
    for stratum in strata:
        u = rng.random()
        unit = (stratum + u) / count
        upper_unit = (stratum + 1) / count
        if unit >= upper_unit:
            unit = math.nextafter(upper_unit, 0.0)

        if finite_span:
            lower_bound = low + (stratum / count) * span
            upper_bound = low + upper_unit * span
            value = low + unit * span
        else:
            lower_bound = mid + (2.0 * (stratum / count) - 1.0) * half_span
            upper_bound = mid + (2.0 * upper_unit - 1.0) * half_span
            value = mid + (2.0 * unit - 1.0) * half_span

        if value < lower_bound:
            value = lower_bound
        if upper_bound > lower_bound and value >= upper_bound:
            value = math.nextafter(upper_bound, lower_bound)
        value = max(low, min(high, value))
        values.append(float(value))
    return values


def latin_hypercube(ranges: dict[str, tuple[float, float]], count: int, seed: int) -> list[dict[str, float]]:
    """Generate a deterministic Latin Hypercube design with independent per-factor streams.

    Each dimension is partitioned into ``count`` equal-width strata, sampled once per
    stratum, and independently shuffled using a deterministic stream derived from
    ``(seed, factor_name)`` so results are invariant to factor insertion order.
    Constant bounds (``low == high``) emit ``float(low)`` for every trial.
    """
    validated = _validate_ranges(ranges)
    if isinstance(count, bool) or not isinstance(count, int):
        raise TypeError("count must be an integer")
    if count <= 0:
        raise ValueError("count must be positive")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise TypeError("seed must be an integer")

    columns = {
        name: _sample_factor_column(low, high, count, _factor_rng(seed, name))
        for name, (low, high) in validated.items()
    }
    return [{name: columns[name][idx] for name in validated} for idx in range(count)]


def _percentile(sorted_values: list[float], quantile: float) -> float | None:
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    pos = quantile * (len(sorted_values) - 1)
    lower = math.floor(pos)
    upper = math.ceil(pos)
    if lower == upper:
        return float(sorted_values[lower])
    weight = pos - lower
    return float(sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight)


def _average_ranks(values: list[float]) -> list[float]:
    n = len(values)
    order = sorted(range(n), key=values.__getitem__)
    ranks = [0.0] * n
    i = 0
    while i < n:
        j = i + 1
        val = values[order[i]]
        while j < n and values[order[j]] == val:
            j += 1
        avg_rank = 0.5 * ((i + 1) + j)
        for k in range(i, j):
            ranks[order[k]] = avg_rank
        i = j
    return ranks


def _spearman_rank(pairs: list[tuple[float, float]]) -> float | None:
    if len(pairs) < 2:
        return None
    xs = [pair[0] for pair in pairs]
    ys = [pair[1] for pair in pairs]
    rx = _average_ranks(xs)
    ry = _average_ranks(ys)
    mean_rank = 0.5 * (len(pairs) + 1)
    dx = [rank - mean_rank for rank in rx]
    dy = [rank - mean_rank for rank in ry]
    ssx = math.fsum(val * val for val in dx)
    ssy = math.fsum(val * val for val in dy)
    if ssx == 0.0 or ssy == 0.0:
        return None
    sxy = math.fsum(a * b for a, b in zip(dx, dy))
    rho = sxy / math.sqrt(ssx * ssy)
    return max(-1.0, min(1.0, float(rho)))


def summarize_trials(trials: Iterable[Mapping[str, object]]) -> dict[str, object]:
    """Summarize campaign trial outcomes and exploratory Spearman associations.

    Expects records with keys ``factors`` (mapping of numeric values),
    ``collision`` (bool), ``min_wall_m`` (float or None), ``seconds`` (float),
    and ``completed`` (bool). Correlations returned under
    ``exploratory_association`` are explicitly labeled as exploratory campaign
    associations rather than causal effects.
    """
    runs = 0
    completions = 0
    collisions = 0
    valid_clearances: list[float] = []
    completion_seconds: list[float] = []
    factor_pairs: dict[str, list[tuple[float, float]]] = {}

    for idx, record in enumerate(trials):
        if not isinstance(record, Mapping):
            raise TypeError(f"trial {idx} must be a mapping")
        missing = _REQUIRED_TRIAL_KEYS.difference(record)
        if missing:
            raise ValueError(f"trial {idx} is missing required keys: {sorted(missing)}")

        factors = record["factors"]
        if not isinstance(factors, Mapping):
            raise TypeError(f"trial {idx} 'factors' must be a mapping")

        runs += 1
        completed = bool(record["completed"])
        collision = bool(record["collision"])
        if completed:
            completions += 1
        if collision:
            collisions += 1

        raw_seconds = record["seconds"]
        if raw_seconds is not None:
            if not _is_real_number(raw_seconds):
                raise TypeError(f"trial {idx} 'seconds' must be numeric")
            seconds = float(raw_seconds)
            if completed and math.isfinite(seconds):
                completion_seconds.append(seconds)

        raw_clearance = record["min_wall_m"]
        clearance: float | None = None
        if raw_clearance is not None:
            if not _is_real_number(raw_clearance):
                raise TypeError(f"trial {idx} 'min_wall_m' must be numeric or None")
            candidate_clearance = float(raw_clearance)
            if math.isfinite(candidate_clearance):
                clearance = candidate_clearance
                valid_clearances.append(clearance)

        for name, raw_val in factors.items():
            if not isinstance(name, str) or not name.strip():
                raise ValueError(f"trial {idx} contains an invalid factor name: {name!r}")
            bucket = factor_pairs.setdefault(name, [])
            if raw_val is None:
                continue
            if not _is_real_number(raw_val):
                raise TypeError(f"trial {idx} factor {name!r} must be numeric or None")
            factor_val = float(raw_val)
            if clearance is not None and math.isfinite(factor_val):
                bucket.append((factor_val, clearance))

    sorted_clearances = sorted(valid_clearances)
    factor_associations = {
        name: _spearman_rank(sorted(factor_pairs[name]))
        for name in sorted(factor_pairs)
    }

    return {
        "runs": runs,
        "completions": completions,
        "collisions": collisions,
        "min_wall_m": {
            "valid_runs": len(sorted_clearances),
            "p05": _percentile(sorted_clearances, 0.05),
            "median": _percentile(sorted_clearances, 0.5),
        },
        "completion_seconds_median": (
            float(statistics.median(completion_seconds)) if completion_seconds else None
        ),
        "exploratory_association": {
            "label": "exploratory_association",
            "scope": "campaign_exploratory_only",
            "exploratory_only": True,
            "causal": False,
            "target": "min_wall_m",
            "method": "spearman_rank",
            "note": (
                "Exploratory Spearman rank association in this simulation campaign; "
                "non-causal and intended only to surface candidate trends."
            ),
            "factors": factor_associations,
        },
    }
