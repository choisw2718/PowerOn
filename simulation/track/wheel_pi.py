"""Provisional Nano-style rear wheel feedforward + PI loop for offline trials.

Matches the control structure of vehicle/firmware/PowerOnNano/src/motor/
RearSpeedControl.h and DualRear.cpp for FourWheelPlant. Default gains are
explicitly provisional placeholders until hardware calibration values are
measured in hardware/control_profile.json.
"""

from dataclasses import dataclass
import math

from hardware import PARAMETERS


# Provisional offline defaults; dual_rear gains in hardware/control_profile.json are 0.0.
PROVISIONAL_BUS_VOLTAGE_V = float(PARAMETERS["motor"]["nominal_supply_v"])
PROVISIONAL_MAX_VOLTAGE_V = PROVISIONAL_BUS_VOLTAGE_V * PARAMETERS["lidar"]["max_pwm_pct"] / 100
PROVISIONAL_FEEDFORWARD_V_PER_MPS = 12.85
PROVISIONAL_KP_V_PER_MPS = 16.0
PROVISIONAL_KI_V_PER_M = 12.0
PROVISIONAL_SAMPLE_S = 0.010  # 10 ms, matching DualRear.cpp kControlPeriodUs
PROVISIONAL_FILTER_TAU_S = 0.040  # 40 ms, matching DualRear.cpp sample()


@dataclass(frozen=True)
class WheelPISettings:
    """Provisional configuration for simulated Nano rear-wheel speed control.

    No-argument defaults are explicitly provisional and are not calibrated for
    physical hardware.

    Parameter units:
        kp: Proportional gain in V / (m/s).
        ki: Integral gain in V / m (equivalently V / ((m/s) * s)).
        feedforward: Velocity feedforward slope in V / (m/s).
        sample_s: Fixed encoder/PI control sample period in s (default 0.010 s = 10 ms).
        max_voltage: Saturated controller output limit in V (default provisional LiDAR duty ceiling).
        bus_voltage: Nominal motor bus voltage in V corresponding to 100% PWM duty.
        filter_tau_s: First-order low-pass time constant in s for encoder speed (default 0.040 s).
    """

    kp: float = PROVISIONAL_KP_V_PER_MPS
    ki: float = PROVISIONAL_KI_V_PER_M
    feedforward: float = PROVISIONAL_FEEDFORWARD_V_PER_MPS
    sample_s: float = PROVISIONAL_SAMPLE_S
    max_voltage: float = PROVISIONAL_MAX_VOLTAGE_V
    bus_voltage: float = PROVISIONAL_BUS_VOLTAGE_V
    filter_tau_s: float = PROVISIONAL_FILTER_TAU_S

    def __post_init__(self):
        for name in ("kp", "ki", "feedforward", "sample_s", "max_voltage", "bus_voltage", "filter_tau_s"):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")
        if self.kp < 0.0 or self.ki < 0.0 or self.feedforward < 0.0:
            raise ValueError("kp, ki, and feedforward must be non-negative")
        if self.sample_s <= 0.0:
            raise ValueError("sample_s must be positive")
        if not 0.0 < self.max_voltage <= self.bus_voltage:
            raise ValueError("max_voltage must be positive and no greater than bus_voltage")
        if self.filter_tau_s < 0.0:
            raise ValueError("filter_tau_s must be non-negative")


def _clamp(value, minimum, maximum):
    return min(maximum, max(minimum, value))


class SpeedPI:
    """Single-wheel feedforward + PI controller matching rear_speed::SpeedPI."""

    def __init__(self):
        self.integral = 0.0  # Stored integral voltage term in V.

    def reset(self):
        self.integral = 0.0

    def voltage(self, target_mps, measured_mps, dt, feedforward, kp, ki, max_voltage):
        """Compute saturated motor voltage (V) with conditional integration."""
        error = target_mps - measured_mps
        base = feedforward * target_mps + kp * error
        candidate = _clamp(self.integral + ki * error * dt, -max_voltage, max_voltage)
        # Block integration further into saturation. Zero output lets the wheel coast.
        if (base + candidate <= max_voltage or error < 0.0) and (base + candidate >= 0.0 or error > 0.0):
            self.integral = candidate
        return _clamp(base + self.integral, 0.0, max_voltage)


class DualRearWheelPI:
    """Two-wheel sample-and-hold speed loop for FourWheelPlant."""

    def __init__(self, settings, rollout, cpr):
        self.settings = settings
        self.rollout = rollout
        self.cpr = cpr
        self.controllers = (SpeedPI(), SpeedPI())
        self.last_sample_s = 0.0
        self.last_counts = [0, 0]
        self.filtered_speeds = [0.0, 0.0]  # Low-pass filtered wheel speed in m/s.
        self.held_voltages = [0.0, 0.0]  # Held nominal motor command voltage in V.

    @property
    def integrals(self):
        return (self.controllers[0].integral, self.controllers[1].integral)

    def reset_outputs(self):
        """Immediately clear integrators and held voltages on zero PWM."""
        for controller in self.controllers:
            controller.reset()
        self.held_voltages[0] = 0.0
        self.held_voltages[1] = 0.0

    def actuation_speed(self, index, motion):
        """Convert held controller voltage (V) into unscaled motor target speed (m/s)."""
        voltage = self.held_voltages[index]
        if voltage == 0.0 or motion.pwm_limit <= motion.deadband_pwm:
            return 0.0
        duty_pct = 100.0 * abs(voltage) / self.settings.bus_voltage
        if duty_pct <= motion.deadband_pwm:
            return 0.0
        nominal = motion.max_speed * (duty_pct - motion.deadband_pwm) / (motion.pwm_limit - motion.deadband_pwm)
        return math.copysign(nominal, voltage)

    def update(self, encoder_counts, elapsed_s, targets_mps, active):
        """Sample quantized encoder counts and update held PI outputs on sample ticks."""
        dt = elapsed_s - self.last_sample_s
        if dt < self.settings.sample_s - 1e-9:
            return False
        self.last_sample_s = elapsed_s
        tau = self.settings.filter_tau_s
        alpha = dt / (tau + dt) if tau > 0.0 else 1.0
        for idx in (0, 1):
            ticks = encoder_counts[idx] - self.last_counts[idx]
            self.last_counts[idx] = encoder_counts[idx]
            mps = ticks * self.rollout[idx] / (self.cpr[idx] * dt)
            self.filtered_speeds[idx] += alpha * (mps - self.filtered_speeds[idx])
        if not active:
            self.reset_outputs()
            return True
        settings = self.settings
        for idx in (0, 1):
            target = targets_mps[idx]
            direction = 1 if target >= 0.0 else -1
            self.held_voltages[idx] = direction * self.controllers[idx].voltage(
                abs(target),
                direction * self.filtered_speeds[idx],
                dt,
                settings.feedforward,
                settings.kp,
                settings.ki,
                settings.max_voltage,
            )
        return True
