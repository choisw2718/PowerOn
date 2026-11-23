"""Measured vehicle response for the Pi controller and simulator."""

from dataclasses import dataclass
import math

from hardware import PARAMETERS
from lidar.geometry.scan_geometry import pose_at


PENDING_OVERSPEED = "checking measured overspeed"


@dataclass(frozen=True)
class MotionFeedback:
    speed_mps: float
    yaw_rate: float  # rad/s, clockwise positive
    steer_deg: float | None = None  # actual front-wheel angle, left positive
    lateral_mps: float = 0.0
    reversing: bool = False

    def observed_steer(self, wheelbase):
        if self.steer_deg is not None:
            return self.steer_deg
        if self.speed_mps >= 0.08:
            angle = math.degrees(math.atan(-wheelbase * self.yaw_rate / self.speed_mps))
            return max(-45.0, min(45.0, angle))
        return None


def measure_motion(times, poses, stamp):
    start = max(times[0], stamp - 0.10)
    before, after = pose_at(times, poses, start), pose_at(times, poses, stamp)
    seconds = stamp - start
    if seconds < 0.04:
        return None
    middle = before.interpolate(after, 0.5)
    forward, lateral = middle.local(after.x, after.y)
    yaw_rate = math.remainder(after.yaw - before.yaw, math.tau) / seconds
    return MotionFeedback(2 * math.hypot(forward, lateral) / seconds, yaw_rate, after.steer_deg,
                          2 * lateral / seconds, 2 * forward / seconds < -0.04)


def approach(value, target, change):
    return value + max(-change, min(change, target - value))


@dataclass(frozen=True)
class SteeringTransition:
    current: float
    previous: float
    speed_mps: float
    delay_s: float
    rate_deg_s: float
    offset_deg: float = 0.0
    right_gain: float = 1.0
    left_gain: float = 1.0

    def wheel_target(self, command):
        gain = self.right_gain if command >= 0 else self.left_gain
        return max(-35.0, min(35.0, command * gain + self.offset_deg))

    def angle_at(self, candidate, distance):
        elapsed = distance / self.speed_mps
        held = approach(self.current, self.wheel_target(self.previous),
                        self.rate_deg_s * min(elapsed, self.delay_s))
        return approach(held, self.wheel_target(candidate),
                        self.rate_deg_s * max(0.0, elapsed - self.delay_s))


class ActuationMonitor:
    def __init__(self, settings):
        self.settings = settings
        self.last_steer = None
        self.steady_steer = 0.0
        self.steer_settles_at = math.inf
        self.coast_started = None
        self.coast_speed = 0.0
        self.powered_since = None
        self.overspeed_sample_at = None
        self.issue = None
        self.streak = 0
        self.steer_offset = 0.0
        self.steer_gains = [1.0, 1.0]  # Left, right in the Nano coordinate convention.

    def transition(self, measured, wheelbase, target_speed):
        actual = measured.observed_steer(wheelbase)
        if actual is None:
            return None
        # Planner angles increase to the right.
        previous = actual if self.last_steer is None else self.last_steer
        return SteeringTransition(-actual, -previous, max(0.08, measured.speed_mps, target_speed),
                                  self.settings.steering_delay_ms / 1000,
                                  self.settings.steering_rate_deg_s, -self.steer_offset,
                                  self.steer_gains[1], self.steer_gains[0])

    def record(self, steer, demand, measured, stamp):
        if self.last_steer is None or abs(steer - self.steady_steer) >= 2.0:
            self.steady_steer = steer
            actual = measured.steer_deg
            travel = abs(steer - actual) if actual is not None else 0.0
            self.steer_settles_at = (stamp + self.settings.steering_delay_ms / 1000
                                     + travel / self.settings.steering_rate_deg_s + 0.10)
        self.last_steer = steer
        if demand.pwm > 0:
            self.coast_started = None
        elif self.coast_started is None:
            self.coast_started, self.coast_speed = stamp, measured.speed_mps
        if demand.pwm <= 0 or demand.speed < 0.11:
            self.powered_since = None
        elif self.powered_since is None:
            self.powered_since = stamp

    def check(self, measured, wheelbase, stamp):
        if measured.reversing:
            return "unexpected reverse motion", 0.0
        overspeed_margin = max(0.04, 0.12 * self.settings.max_speed)
        excess = measured.speed_mps - self.settings.max_speed
        if excess > overspeed_margin:
            if excess > 2 * overspeed_margin or (self.overspeed_sample_at is not None
                                                  and stamp > self.overspeed_sample_at):
                return "measured overspeed", 0.0
            # Wheel slip can produce one high encoder estimate. Coast at once
            # and require a new motion sample and fresh routes before resuming.
            self.overspeed_sample_at = stamp
            return PENDING_OVERSPEED, 0.0
        self.overspeed_sample_at = None
        issue = self.response_issue(measured, wheelbase, stamp)
        if ((issue is None or issue == "steering command mismatch") and measured.steer_deg is not None
                and stamp >= self.steer_settles_at and abs(measured.steer_deg - self.last_steer) <= 10):
            self.observe_steering(measured.steer_deg)
            if issue is not None:
                issue = self.response_issue(measured, wheelbase, stamp)
        self.streak = (self.streak + 1 if issue == self.issue else 1) if issue else 0
        self.issue = issue
        if self.streak >= 4:
            return issue, 0.0
        return issue, 0.15 if issue else math.inf

    def observe_steering(self, actual):
        """Fit only bounded calibration adjustments from settled wheel feedback."""
        command = self.last_steer
        direction = int(command < 0)
        gain = self.steer_gains[direction]
        if abs(command) <= 4:
            offset = actual - gain * command
            if abs(offset) <= 4:
                self.steer_offset += 0.14 * (offset - self.steer_offset)
        elif abs(command) >= 8:
            measured_gain = (actual - self.steer_offset) / command
            if 0.68 <= measured_gain <= 1.30:
                self.steer_gains[direction] += 0.14 * (measured_gain - gain)

    def response_issue(self, measured, wheelbase, stamp):
        # A normal Ackermann turn has unequal rear wheel speeds. Even allowing
        # steering calibration error, a stalled wheel produces much more yaw.
        if measured.speed_mps >= 0.08:
            max_angle = math.radians(PARAMETERS["lidar"]["max_center_steer_deg"] + 6)
            plausible_yaw = measured.speed_mps * math.tan(max_angle) / wheelbase
            if abs(measured.yaw_rate) > plausible_yaw + 0.35:
                return "wheel motion exceeds steering range"
        if self.powered_since is not None and measured.speed_mps < 0.045:
            if stamp - self.powered_since >= self.settings.delay_ms / 1000 + 0.45:
                return "no motion under drive command"
        if self.coast_started is not None:
            seconds = stamp - self.coast_started - self.settings.delay_ms / 1000
            expected_drop = max(0.03, 0.45 * self.settings.deceleration * seconds)
            if seconds >= 0.25 and measured.speed_mps > max(0.06, self.coast_speed - expected_drop):
                return "coasting response too weak"
        steer = measured.steer_deg
        if abs(measured.lateral_mps) > max(0.06, 0.25 * measured.speed_mps):
            return "unexpected lateral motion"
        if steer is not None and measured.speed_mps >= 0.12:
            expected_yaw = -measured.speed_mps * math.tan(math.radians(steer)) / wheelbase
            if abs(measured.yaw_rate - expected_yaw) > max(0.35, 0.50 * abs(expected_yaw)):
                return "yaw response mismatch"
        if steer is not None and stamp >= self.steer_settles_at:
            command = self.last_steer
            expected = self.steer_offset + self.steer_gains[int(command < 0)] * command
            if abs(steer - expected) > 6 or abs(steer - command) > 10:
                return "steering command mismatch"
        return None
