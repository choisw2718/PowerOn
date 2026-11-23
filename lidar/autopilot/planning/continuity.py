"""Keep a short-lived corridor budget across recoverable sensing interruptions."""

from dataclasses import dataclass, replace
from collections import deque
import math

from hardware import PARAMETERS
from lidar.geometry.scan_geometry import MotionUncertainty

from ..motion.feedback import PENDING_OVERSPEED
from ..motion.speed_control import stopping_distance
from .wall_track import ISOLATED_NEAR_RETURN, Decision, footprint_path


MAX_LEASE_MS = PARAMETERS["lidar"]["max_lease_ms"]
MAX_BRIDGE_AGE = 0.50
MAX_WAIT_S = 2.0
MAX_PLAN_AGE = 0.12
UNMEASURED_TARGET_MPS = 0.20
BRIDGEABLE = frozenset({
    "incomplete scan", "insufficient forward returns", "insufficient wall support",
    "insufficient clear distance", "held steering unobserved",
})
RECOVERABLE = BRIDGEABLE | {"ambiguous turn", "stale rotation", "motion unavailable", ISOLATED_NEAR_RETURN,
                            PENDING_OVERSPEED}


def lease_floor_ms(period):
    return max(220, math.ceil(1000 * period - 1e-6) + 40)


def held_clearance(route):
    if route.held_clear_m is not None:
        return route.held_clear_m
    curvature = max(route.curvature, max((value for _, value in route.curvature_ahead), default=0.0))
    return min(route.clear_m, math.pi / (2 * curvature)) if curvature else route.clear_m


def command_clearance(reference_time, track, speed, now, lease_ms, feedback=None, *, measured_at_reference=True):
    """Required observed distance for an uninterrupted command of this duration."""
    bound = max(speed.max_speed, feedback.speed_mps if feedback else 0.0)
    powered = max(0.0, now - reference_time) + lease_ms / 1000
    delay = speed.delay_ms / 1000
    if feedback is None or not measured_at_reference or speed.speed_error_mps >= speed.max_speed:
        reserve = max(track.stop_distance, bound ** 2 / (2 * speed.deceleration))
        return bound * (powered + delay) + reserve + 0.055

    # The empirical stop allowance is measured from the stop decision to rest,
    # including the response delay. The independent physical bound includes
    # continued acceleration through the full delay and then coast-down.
    initial = min(bound, max(0.0, feedback.speed_mps) + speed.speed_error_mps)
    def advancing(duration):
        ramp = min(duration, (bound - initial) / speed.acceleration)
        peak = initial + speed.acceleration * ramp
        distance = (initial + peak) * 0.5 * ramp + peak * (duration - ramp)
        return distance, peak
    until_expiry, _ = advancing(powered)
    until_cut, peak = advancing(powered + delay)
    return max(until_expiry + track.stop_distance,
               until_cut + peak ** 2 / (2 * speed.deceleration)) + 0.055


def lease_budget_ms(route, reference_time, track, speed, now, *, feedback=None, measured_at_reference=True):
    clear = min(route.clear_m, held_clearance(route))
    def fits(duration):
        return command_clearance(reference_time, track, speed, now, duration, feedback,
                                 measured_at_reference=measured_at_reference) <= clear
    if fits(MAX_LEASE_MS):
        return MAX_LEASE_MS
    low, high = 0, MAX_LEASE_MS
    while high - low > 1:
        middle = (low + high) // 2
        if fits(middle):
            low = middle
        else:
            high = middle
    return low


@dataclass(frozen=True)
class ConfirmedPath:
    route: Decision
    reference: float
    end: float


class CommandMotionBound:
    """Bound speed from a stationary start, motor power history and calibrated acceleration limits."""

    def __init__(self, speed):
        self.speed = speed
        self.intervals = deque()

    def bound(self, interval, stamp):
        start, initial, powered = interval
        elapsed = max(0.0, stamp - start)
        delay = self.speed.propulsion_delay_ms / 1000
        powered_time = elapsed if powered else min(delay, elapsed)
        top = min(self.speed.max_speed, initial + self.speed.acceleration * powered_time)
        return max(0.0, top - self.speed.deceleration * (elapsed - powered_time))

    def applied(self, powered, stamp):
        if self.intervals and self.intervals[-1][2] == powered:
            return
        initial = self.bound(self.intervals[-1], stamp) if self.intervals else 0.0
        self.intervals.append((stamp, initial, powered))
        while len(self.intervals) > 1 and self.intervals[1][0] < stamp - 2.0:
            self.intervals.popleft()

    def maximum(self, start, end):
        peak = 0.0
        for index, interval in enumerate(self.intervals):
            left = max(start, interval[0])
            right = min(end, self.intervals[index + 1][0]) if index + 1 < len(self.intervals) else end
            if left <= right:
                coast = interval[0] + self.speed.propulsion_delay_ms / 1000
                peak = max(peak, self.bound(interval, left), self.bound(interval, right),
                           self.bound(interval, min(right, max(left, coast))))
        return peak


def motion_uncertainty(track, speed, age, *, speed_bound=None):
    travel = (speed.max_speed if speed_bound is None else speed_bound) * age
    heading = travel / track.ideal_turn_radius
    translation = travel + 2 * track.sensor_ahead * math.sin(min(math.pi, heading) / 2)
    return MotionUncertainty(translation, heading)


@dataclass(frozen=True)
class DriveGate:
    route: Decision
    action: str
    lease_ms: int = 0
    bridged: bool = False


class TrackContinuity:
    def __init__(self):
        self.confirmed = None
        self.waiting_at = None
        self.waiting_reason = None
        self.good_scans = 0
        self.start_scans = 0

    def gate(self, route, samples, track, speed, start, end, now, *, driving,
             geometry=None, motion=None, steering=None, feedback=None, issue=None,
             deadline=math.inf, period=None):
        """Decide whether the current scan can authorize positive drive on either vehicle."""
        stale = now - end > MAX_PLAN_AGE
        if not driving:
            if stale:
                return DriveGate(Decision(reason="lidar plan expired"), "stale")
            if feedback is not None and feedback.speed_mps > 0.04:
                return DriveGate(Decision(reason="car must be stationary before arming"), "stop")
            self.start_scans = self.start_scans + 1 if route.pwm > 0 and route.speed_scale == 1 else 0
        elif stale and route.pwm > 0:
            route = Decision(reason="stale rotation")
        if route.pwm <= 0 and route.reason not in RECOVERABLE:
            return DriveGate(route, "stop")
        if not driving and self.start_scans < 3:
            return DriveGate(route, "observe")
        floor = lease_floor_ms(end - start if period is None else period)
        remaining_ms = (deadline - now) * 1000
        if remaining_ms < floor:
            return DriveGate(route, "deadline")
        if self.expired(now):
            cause = self.waiting_reason if route.pwm > 0 and self.waiting_reason else route.reason
            return DriveGate(Decision(reason=f"scan recovery timed out ({cause})"), "stop")
        budget = 0
        bridge = False
        if route.pwm > 0:
            reference = end if feedback is not None else start
            budget = lease_budget_ms(route, reference, track, speed, now, feedback=feedback)
        elif route.reason in BRIDGEABLE and not stale:
            retained, budget = self.assess(samples, track, speed, now, geometry=geometry,
                                           motion=motion, steering=steering, feedback=feedback)
            if retained is not None:
                route = retained
                bridge = retained.pwm > 0
            if issue:
                budget = 0
        if route.pwm <= 0 and route.reason not in RECOVERABLE:
            return DriveGate(route, "stop")
        budget = int(min(budget, remaining_ms))
        if route.pwm <= 0 or budget < floor:
            self.wait(now, route.reason if route.pwm <= 0 else "insufficient command clearance")
            return DriveGate(route, "coast", budget)
        if not bridge and not self.ready():
            return DriveGate(route, "coast", budget)
        return DriveGate(route, "drive", budget, bridge)

    def remember(self, route, reference, end):
        if self.confirmed is None or end > self.confirmed.end:
            self.confirmed = ConfirmedPath(route, reference, end)

    def wait(self, now, reason=None):
        if self.waiting_at is None:
            self.waiting_at = now
        if reason is not None:
            self.waiting_reason = reason
        self.good_scans = 0

    def expired(self, now):
        return self.waiting_at is not None and now - self.waiting_at >= MAX_WAIT_S

    def ready(self):
        if self.waiting_at is None:
            return True
        self.good_scans += 1
        if self.good_scans < 2:
            return False
        self.waiting_at = None
        self.waiting_reason = None
        self.good_scans = 0
        return True

    def assess(self, samples, track, speed, now, *, geometry=None, motion=None,
               steering=None, feedback=None):
        """Validate remaining clearance on the last command against new returns."""
        anchor = self.confirmed
        if anchor is None:
            return None, 0
        age = now - anchor.end
        if not 0 <= age < MAX_BRIDGE_AGE:
            return None, 0
        route = anchor.route
        speed_bound = max(speed.max_speed, feedback.speed_mps if feedback else 0.0)
        traveled = speed_bound * max(0.0, now - max(anchor.reference, anchor.end))
        remaining = max(0.0, held_clearance(route) - traveled)
        if remaining <= 0:
            return None, 0

        points = geometry.points if geometry is not None else [
            (distance / 1000 * math.cos(math.radians(angle - track.forward_angle)),
             distance / 1000 * math.sin(math.radians(angle - track.forward_angle)))
            for angle, distance, quality in samples
            if quality > 0 and math.isfinite(angle) and math.isfinite(distance) and distance > 0
        ]
        stop = stopping_distance(speed_bound, speed.deceleration, speed.delay_ms / 1000)
        stopping_clear = min(remaining, max(track.stop_distance, stop) + 0.055)
        held = footprint_path(points, (), track, -route.steer, remaining, geometry=geometry, motion=motion,
                              steering=steering, check_visibility=False, hold_turn=True)
        if held.clear < stopping_clear - 1e-9:
            reason = ISOLATED_NEAR_RETURN if held.isolated_block else "obstacle on last confirmed path"
            return Decision(reason=reason, clear_m=held.clear), 0
        if self.waiting_at is not None:
            return None, 0
        if steering is not None and abs(steering.current - steering.wheel_target(-route.steer)) > 4:
            return None, 0
        lease_route = replace(route, clear_m=held.clear + traveled)
        lease = lease_budget_ms(lease_route, anchor.reference, track, speed, now,
                                feedback=feedback, measured_at_reference=False)
        lease = min(lease, math.floor(1000 * (MAX_BRIDGE_AGE - age)))
        bridged = replace(route, clear_m=held.clear, reason="bridging scan gap", speed_scale=0.4)
        return bridged, lease
