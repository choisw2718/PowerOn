"""Opt-in correlated LiDAR and scan-delivery disturbance models.

All phenomena are inactive when their corresponding ``SensorSettings`` fields
are zero/default, preserving existing RNG streams, scan timing, and outputs.

Range-bias correlation model
----------------------------
When ``range_bias_sigma_mm > 0``, each sweep carries a persistent range offset
``b_k`` (in mm) shared by all valid returns in that sweep and updated once per
sweep as a discrete-time AR(1) sampling of an Ornstein-Uhlenbeck process:

    b_0 ~ N(0, sigma^2)
    b_k = phi * b_{k-1} + sqrt(1 - phi^2) * sigma * eps_k,   eps_k ~ N(0, 1)
    phi = exp(-dt_s / tau_s)

where ``sigma = range_bias_sigma_mm``, ``dt_s = period_ms / 1000``, and the
correlation time constant is ``tau_s = RANGE_BIAS_TAU_S`` (1.5 s). The stationary
marginal distribution of ``b_k`` is ``N(0, sigma^2)`` for every sweep.
"""

import math
import random


RANGE_BIAS_TAU_S = 1.5


def _is_real_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_sensor_faults(settings):
    """Validate correlated LiDAR/transport disturbance parameters."""
    if not _is_real_number(settings.burst_start_probability) or not 0.0 <= settings.burst_start_probability <= 1.0:
        raise ValueError("burst_start_probability must be finite and within 0..1")
    if (isinstance(settings.burst_length_points, bool)
            or not isinstance(settings.burst_length_points, int)
            or settings.burst_length_points < 0):
        raise ValueError("burst_length_points must be a non-negative integer")
    if not _is_real_number(settings.range_bias_sigma_mm) or settings.range_bias_sigma_mm < 0.0:
        raise ValueError("range_bias_sigma_mm must be finite and non-negative")
    if (isinstance(settings.delivery_jitter_ms, bool)
            or not isinstance(settings.delivery_jitter_ms, int)
            or settings.delivery_jitter_ms < 0):
        raise ValueError("delivery_jitter_ms must be a non-negative integer")


class SensorFaultModel:
    """Stateful, separately seeded model for burst dropouts, AR(1) range bias, and delivery jitter."""

    def __init__(self, settings, *, tau_s=RANGE_BIAS_TAU_S):
        validate_sensor_faults(settings)
        if not _is_real_number(tau_s) or tau_s <= 0.0:
            raise ValueError("tau_s must be finite and positive")
        self.burst_start_probability = float(settings.burst_start_probability)
        self.burst_length_points = settings.burst_length_points
        self.range_bias_sigma_mm = float(settings.range_bias_sigma_mm)
        self.delivery_jitter_ms = settings.delivery_jitter_ms
        self.period_ms = settings.period_ms
        self.tau_s = float(tau_s)
        self.enabled = bool(
            (self.burst_start_probability > 0.0 and self.burst_length_points > 0)
            or self.range_bias_sigma_mm > 0.0
            or self.delivery_jitter_ms > 0
        )

        # Keep each extra phenomenon on its own deterministic stream, isolated from
        # Simulator's range/noise/dropout RNG, vibration RNG, and each other.
        self.burst_random = random.Random(f"{settings.seed}:burst")
        self.bias_random = random.Random(f"{settings.seed}:range_bias")
        self.jitter_random = random.Random(f"{settings.seed}:delivery_jitter")

        self._burst_remaining = 0
        self.burst_starts = 0
        self.burst_drops = 0
        self.last_delivery_jitter_ms = 0
        self.range_bias_mm = (
            self.bias_random.gauss(0.0, self.range_bias_sigma_mm)
            if self.range_bias_sigma_mm > 0.0 else 0.0
        )

    def next_beam_dropped(self):
        """Return whether the next beam is lost to a correlated dropout burst."""
        if self.burst_start_probability <= 0.0 or self.burst_length_points <= 0:
            return False
        if self._burst_remaining > 0:
            self._burst_remaining -= 1
            self.burst_drops += 1
            return True
        if self.burst_random.random() < self.burst_start_probability:
            self._burst_remaining = self.burst_length_points - 1
            self.burst_starts += 1
            self.burst_drops += 1
            return True
        return False

    def advance_sweep(self):
        """Step the AR(1) sweep range bias to the next sweep and return the new offset in mm."""
        if self.range_bias_sigma_mm <= 0.0:
            return 0.0
        dt_s = max(0.0, self.period_ms / 1000.0)
        phi = math.exp(-dt_s / self.tau_s) if dt_s > 0.0 else 0.0
        innovation_sigma = self.range_bias_sigma_mm * math.sqrt(max(0.0, 1.0 - phi * phi))
        self.range_bias_mm = phi * self.range_bias_mm + self.bias_random.gauss(0.0, innovation_sigma)
        return self.range_bias_mm

    def sample_delivery_jitter_ms(self):
        """Sample bounded signed integer scan-delivery jitter in [-delivery_jitter_ms, +delivery_jitter_ms]."""
        if self.delivery_jitter_ms <= 0:
            self.last_delivery_jitter_ms = 0
            return 0
        jitter = self.jitter_random.randint(-self.delivery_jitter_ms, self.delivery_jitter_ms)
        self.last_delivery_jitter_ms = jitter
        return jitter

    def apply_to_scan(self, samples):
        """Apply burst dropouts and per-sweep range bias to a non-rolling scan sample list."""
        if not self.enabled:
            return samples
        bias_mm = self.range_bias_mm
        updated = []
        for angle, distance_mm, quality in samples:
            if self.next_beam_dropped() or quality <= 0:
                updated.append((angle, 0.0, 0))
            elif bias_mm:
                updated.append((angle, max(0.0, distance_mm + bias_mm), quality))
            else:
                updated.append((angle, distance_mm, quality))
        return updated
