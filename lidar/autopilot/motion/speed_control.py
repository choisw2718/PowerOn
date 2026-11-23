"""Speed targets for the wall planner. Speeds are estimates until measured on the car."""

from dataclasses import dataclass, replace
import math

from hardware import PARAMETERS
from .feedback import PENDING_OVERSPEED


MOTION = PARAMETERS["motion"]


@dataclass(frozen=True)
class SpeedConfig:
    max_speed: float = MOTION["max_speed_mps"]
    acceleration: float = MOTION["acceleration_mps2"]
    deceleration: float = MOTION["deceleration_mps2"]
    lateral_accel: float = MOTION["lateral_acceleration_mps2"]
    delay_ms: int = MOTION["delay_ms"]
    deadband_pwm: float = MOTION["deadband_pwm_pct"]
    steering_delay_ms: float = MOTION["steering_delay_ms"]
    steering_rate_deg_s: float = MOTION["steering_rate_deg_s"]
    propulsion_delay_ms: float = MOTION["propulsion_delay_ms"]
    speed_error_mps: float = 0.02  # Simulated bound. Live driving defaults to the full speed range.


def speed_for_pwm(pwm, pwm_limit, max_speed, deadband):
    if pwm <= deadband or pwm_limit <= deadband:
        return 0.0
    return max_speed * min(1.0, (pwm - deadband) / (pwm_limit - deadband))


def braking_limit(distance, end_speed, deceleration, delay_seconds):
    """Solve v*t + (v² - end_speed²)/(2*a) <= distance for v."""
    delay_term = deceleration * delay_seconds
    return max(end_speed, math.sqrt(delay_term ** 2 + end_speed ** 2
                                    + 2 * deceleration * max(0.0, distance)) - delay_term)


def stopping_distance(speed, deceleration, delay_seconds):
    return speed * delay_seconds + speed ** 2 / (2 * deceleration)


@dataclass(frozen=True)
class SpeedCommand:
    pwm: float
    speed: float
    phase: str


class SpeedController:
    def __init__(self, settings):
        self.settings = settings
        self.target = 0.0
        self.corner_limit = None
        self.straight_scans = 0
        self.wall_limit = 1.0
        self.reliable_scans = 0
        self.feedback_speed = None
        self.pwm_trim = 0.0
        self.slowing_from = None
        self.slowing_elapsed = 0.0

    def lookahead(self, scan_age=0.0, *, feedback=None):
        delay = max(self.settings.delay_ms / 1000, scan_age)
        speed = max(self.target, feedback.speed_mps) if feedback is not None else self.target
        stopping = stopping_distance(speed, self.settings.deceleration, delay)
        return max(0.76, min(2.0, 0.42 + stopping))

    def cruise_horizon(self, scan_age, lease_s):
        speed = self.settings.max_speed
        delay = max(scan_age, self.settings.delay_ms / 1000) + lease_s
        return 0.36 + stopping_distance(speed, self.settings.deceleration, delay)

    def can_stop(self, route, feedback, scan_age):
        delay = max(self.settings.delay_ms / 1000, scan_age)
        required = stopping_distance(feedback.speed_mps, self.settings.deceleration, delay)
        return route.clear_m >= required + 0.055

    def checked_route(self, route, feedback, scan_age, issue, speed_cap):
        if speed_cap == 0 and issue != PENDING_OVERSPEED:
            return replace(route, pwm=0.0, reason=issue)
        if route.pwm <= 0:
            return route
        if feedback is not None and not self.can_stop(route, feedback, scan_age):
            return replace(route, pwm=0.0, reason="measured speed exceeds stopping space")
        if speed_cap == 0:
            return replace(route, pwm=0.0, reason=issue)
        return route

    def update(self, route, track, seconds, *, scan_age=0.0, speed_cap=math.inf, feedback=None):
        settings = self.settings
        previous = self.target
        if route.pwm <= 0:
            self.target = 0.0
            self.corner_limit = None
            self.straight_scans = 0
            self.wall_limit = 1.0
            self.reliable_scans = 0
            self.feedback_speed = None
            self.pwm_trim = 0.0
            self.slowing_from = None
            return SpeedCommand(0.0, 0.0, "stop")

        delay = max(settings.delay_ms / 1000, scan_age)
        limit = min(settings.max_speed, speed_cap,
                    braking_limit(route.clear_m - 0.055, 0.0, settings.deceleration, delay))
        measured_curve = 0.0
        if feedback is not None and feedback.speed_mps >= 0.08:
            measured_curve = abs(feedback.yaw_rate) / feedback.speed_mps
        curve_limit = settings.max_speed
        # Leave room for tyre/steering error during the measured transition.
        lateral = settings.lateral_accel * (0.85 if route.curvature_ahead else 1.0)
        if measured_curve:
            curve_limit = min(curve_limit, math.sqrt(lateral / measured_curve))
        profile = route.curvature_ahead or ((0.0, route.curvature),)
        for distance, curvature in profile:
            if curvature:
                cap = math.sqrt(lateral / curvature)
                curve_limit = min(curve_limit, braking_limit(distance, cap, settings.deceleration, delay))
        if route.turn_in_m is not None:
            corner_speed = math.sqrt(lateral * track.ideal_turn_radius)
            curve_limit = min(curve_limit, braking_limit(route.turn_in_m, corner_speed,
                                                         settings.deceleration, delay))
        strong_turn = route.curvature * track.ideal_turn_radius >= 0.40
        if strong_turn and (not route.curvature_ahead or route.speed_scale < 1.0):
            curve_limit = min(curve_limit, math.sqrt(lateral * track.ideal_turn_radius))
        # A small centring correction on a well-observed straight need not
        # carry a previous hairpin's speed restriction down the whole corridor.
        cruise_clear = (route.left_m is not None and route.right_m is not None
                        and route.speed_scale == 1 and route.turn_in_m is None
                        and min(curve_limit, limit) >= settings.max_speed)
        if not cruise_clear and (route.turn_in_m is not None or max(route.curvature, measured_curve) >= 0.45):
            self.straight_scans = 0
            if self.corner_limit is not None:
                self.corner_limit = min(self.corner_limit, curve_limit)
            elif strong_turn or curve_limit < previous - 1e-6:
                self.corner_limit = curve_limit
        elif self.corner_limit is not None and (cruise_clear or route.clear_m >= 0.65):
            self.straight_scans += 1
            if self.straight_scans >= 2:
                self.corner_limit = None
        else:
            self.straight_scans = 0
        measured_speed_limit = limit = min(limit, curve_limit)
        if route.speed_scale < 1.0:
            self.wall_limit = route.speed_scale
            self.reliable_scans = 0
        elif self.wall_limit < 1.0:
            self.reliable_scans += 1
            if self.reliable_scans >= 2:
                self.wall_limit = 1.0
        if self.wall_limit < 1.0:
            limit = min(limit, settings.max_speed * self.wall_limit, 0.20)
        if self.corner_limit is not None:
            limit = min(limit, self.corner_limit)
        # Raise the target gradually. Lower it immediately so the vehicle can coast down.
        accelerating_target = previous + settings.acceleration * max(0.0, seconds)
        if previous == 0 and feedback is not None:
            # A temporary scan hold resets the command but the chassis can still
            # be rolling when the drive gate next clears a route.
            accelerating_target = max(accelerating_target, feedback.speed_mps)
        self.target = min(limit, accelerating_target)
        if not self.target or settings.deadband_pwm >= route.pwm:
            self.target = 0.0
            return SpeedCommand(0.0, 0.0, "stop")
        pwm = settings.deadband_pwm + (route.pwm - settings.deadband_pwm) * self.target / settings.max_speed
        phase = "decelerate" if self.target < previous - 1e-6 else (
            "accelerate" if self.target > previous + 1e-6 else "hold")
        if feedback is not None:
            measured = feedback.speed_mps
            rise_rate = ((measured - self.feedback_speed) / max(seconds, 0.02)
                         if self.feedback_speed is not None else 0.0)
            self.feedback_speed = measured
            error = self.target - measured
            # Keep forward torque low while following a lower wall-confidence
            # or retained corner target. Coast if the current braking, corner
            # or actuation envelope requires it, or if reduced PWM fails to slow us.
            if error < -0.05:
                self.pwm_trim = min(0.0, self.pwm_trim)
                response_interval = settings.propulsion_delay_ms / 1000 + max(0.0, seconds)
                projected = measured + max(0.0, rise_rate) * response_interval
                if projected > measured_speed_limit or settings.speed_error_mps >= settings.max_speed:
                    self.slowing_from = None
                    return SpeedCommand(0.0, self.target, "coast")
                if self.slowing_from is None or self.slowing_from - measured > settings.speed_error_mps:
                    self.slowing_from, self.slowing_elapsed = measured, 0.0
                else:
                    self.slowing_elapsed += max(0.0, seconds)
                if self.slowing_elapsed >= settings.propulsion_delay_ms / 1000 + 0.20:
                    return SpeedCommand(0.0, self.target, "coast")
            else:
                self.slowing_from = None
            response_time = min(0.35, settings.propulsion_delay_ms / 1000 + seconds / 2)
            rise = max(0.0, min(1.5 * settings.acceleration, rise_rate)) * response_time
            gain = (route.pwm - settings.deadband_pwm) / settings.max_speed
            trim_limit = 0.18 * route.pwm
            correction = 0.85 * gain * (error - rise)
            output = pwm + correction + self.pwm_trim
            can_trim = (0 < output < route.pwm or (output >= route.pwm and error < 0)
                        or (output <= 0 and error > 0))
            if phase == "hold" and can_trim:
                self.pwm_trim = max(-trim_limit, min(trim_limit, self.pwm_trim + 0.65 * gain * error * seconds))
            adjustment = max(-trim_limit, min(trim_limit, correction + self.pwm_trim))
            pwm = max(0.0, min(route.pwm, pwm + adjustment))
            if pwm == 0:
                phase = "coast"
        return SpeedCommand(pwm, self.target, phase)
