"""Low-speed, planar twin-servo and twin-rear-drive model for offline trials."""

from collections import deque
from dataclasses import dataclass
import math

from hardware import PARAMETERS
from lidar.autopilot.motion.feedback import MotionFeedback, approach
from lidar.autopilot.motion.speed_control import speed_for_pwm
from lidar.geometry.wheel_odometry import integrate_wheel_distances
from .conditions import contact_ratio
from .wheel_pi import DualRearWheelPI


@dataclass(frozen=True)
class BodyEnvelope:
    width: float
    front: float
    rear: float


def wheel_targets(center_deg, wheelbase, pivot_track):
    """Ackermann angles in physical left/right order, positive to the left."""
    curvature = math.tan(math.radians(center_deg)) / wheelbase
    return tuple(math.degrees(math.atan2(wheelbase * curvature, 1 + side * pivot_track * curvature / 2))
                 for side in (-1, 1))


def servo_limits():
    steering = PARAMETERS["steering"]
    crossed = steering["channels_crossed"]
    limits = []
    for channel, side in (("d5", "right" if crossed else "left"), ("d6", "left" if crossed else "right")):
        neutral = steering[f"{side}_aligned_{channel}_us"]
        slope = steering[f"{channel}_us_per_deg"] * steering[f"{channel}_direction"]
        lo, hi = sorted((steering[f"{channel}_{end}_us"] - neutral) / slope for end in ("min", "max"))
        limits.append((max(lo, steering["wheel_min_deg"]), min(hi, steering["wheel_max_deg"])))
    return tuple(reversed(limits)) if crossed else tuple(limits)


class FourWheelPlant:
    def __init__(self, config, environment, start_pose, sensor, *, wheel_pi=None):
        self.config = config
        self.environment = environment
        chassis, defaults = PARAMETERS["chassis"], PARAMETERS["simulation"]
        rear = PARAMETERS["dual_rear"]

        def measurement(key):
            value = rear[key] if rear[key] > 0 else defaults[key]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
            return value

        self.track = measurement("rear_track_m")
        self.rollout = (measurement("left_rollout_m"), measurement("right_rollout_m"))
        self.cpr = (measurement("left_counts_per_rev"), measurement("right_counts_per_rev"))
        self.tire_width = defaults["tire_width_m"]
        self.plate_width = chassis["plate_width_m"]
        self.plate_length = chassis["plate_length_m"]
        self.pivot_track = chassis["steering_pivot_track_m"]
        self.arm = defaults["front_arm_m"]
        self.bumper = defaults["bumper_extension_m"]
        self.front_limits = servo_limits()
        self.max_wheel_speed = rear["max_wheel_speed_mps"]
        max_curve = math.tan(math.radians(PARAMETERS["lidar"]["max_center_steer_deg"])) / config.wheelbase
        if self.track * max_curve >= 2:
            raise ValueError("Rear wheel targets require a positive inside turn radius")
        self.front_angles = (0.0, 0.0)
        self.ground_speeds = (0.0, 0.0)
        self.wheel_speeds = (0.0, 0.0)
        self.encoder_counts = (0, 0)
        self.encoder_turns = [0.0, 0.0]
        self.elapsed_s = 0.0
        self.last_sample_s = 0.0
        self.sample_period = sensor.pose_sample_ms / 1000
        self.measured_pose = start_pose
        self.measured = MotionFeedback(0.0, 0.0)
        self.feedback_history = deque([(0.0, self.measured)], maxlen=max(3, round(2.5 / self.sample_period)))
        self.speed_samples = deque([(0.0, 0.0, 0.0)])
        self.total_measured_distance = 0.0
        self.total_measured_yaw = 0.0
        self.previous_counts = (0, 0)
        self.progress_counts = [0, 0]
        self.progress_at = [0.0, 0.0]
        self.drive_fault = None
        self.rough_floor_s = 0.0
        self._geometry_angles = None
        self.distance_scale = sensor.odometry_distance_scale
        self.yaw_scale = sensor.odometry_yaw_scale
        self.wheel_pi = wheel_pi
        self.wheel_pi_loop = (DualRearWheelPI(wheel_pi, self.rollout, self.cpr)
                              if wheel_pi is not None else None)

    def wheel_footprints(self):
        """FL, FR, RL, RR rectangles, in rear-axle coordinates (forward x, right y)."""
        polygons = []
        for front in (True, False):
            for idx, side in enumerate((-1, 1)):
                angle = math.radians(self.front_angles[idx]) if front else 0.0
                c, s = math.cos(angle), math.sin(angle)
                x = self.config.wheelbase + side * self.arm * s if front else 0.0
                y = side * (self.pivot_track / 2 + self.arm * c if front else self.track / 2)
                radius = self.rollout[idx] / math.tau
                half = self.tire_width / 2
                polygons.append(tuple((x + a * c + b * s, y - a * s + b * c)
                                      for a, b in ((radius, half), (radius, -half), (-radius, -half), (-radius, half))))
        return tuple(polygons)

    def envelope(self):
        if self.front_angles != self._geometry_angles:
            midpoint = self.config.wheelbase / 2  # provisional plate centre ahead of rear axle
            half_length = self.plate_length / 2 + self.bumper
            xmin, xmax, half_width = midpoint - half_length, midpoint + half_length, self.plate_width / 2
            for wheel in self.wheel_footprints():
                for x, y in wheel:
                    xmin, xmax, half_width = min(xmin, x), max(xmax, x), max(half_width, abs(y))
            self._envelope = BodyEnvelope(2 * half_width, xmax - self.config.sensor_ahead, self.config.sensor_ahead - xmin)
            self._geometry_angles = self.front_angles
        return self._envelope

    def swept_envelope(self):
        # A rotating front tire corner stays within this radius of its servo pivot.
        tire_radius = max(self.rollout) / math.tau
        swing = math.hypot(self.arm + self.tire_width / 2, tire_radius)
        half_length = self.plate_length / 2 + self.bumper
        wheelbase = self.config.wheelbase
        x_min = min(-tire_radius, wheelbase / 2 - half_length, wheelbase - swing)
        x_max = max(tire_radius, wheelbase / 2 + half_length, wheelbase + swing)
        width = max(self.plate_width, self.pivot_track + 2 * swing, self.track + self.tire_width)
        return BodyEnvelope(width, x_max - self.config.sensor_ahead, self.config.sensor_ahead - x_min)

    def steer(self, target, motion, dt):
        L, half = self.config.wheelbase, self.pivot_track / 2
        # Limit the common radius before generating either front-wheel target, as on the Nano.
        limits = [hi if target >= 0 else -lo for lo, hi in self.front_limits]
        inner, outer = limits if target >= 0 else limits[::-1]
        radius = max(half + L / math.tan(math.radians(inner)), -half + L / math.tan(math.radians(outer)))
        limited = math.copysign(min(abs(target), math.degrees(math.atan(L / radius))), target)
        angles = wheel_targets(limited, L, self.pivot_track)
        updated, curvature = [], 0.0
        for idx, side in enumerate((-1, 1)):
            lo, hi = self.front_limits[idx]
            desired = min(hi, max(lo, angles[idx] + (self.environment.right_servo_trim if idx else 0)))
            previous = self.front_angles[idx]
            rate = motion.steering_rate * (self.environment.right_servo_rate_scale if idx else 1.0)
            actual = (approach(previous, desired, rate * dt)
                      if abs(desired - previous) > motion.steer_deadband_deg else previous)
            actual = max(lo, min(hi, actual))
            updated.append(actual)
            tangent = math.tan(math.radians(actual))
            curvature += 0.5 * tangent / (L - side * half * tangent)
        self.front_angles = tuple(updated)
        return curvature

    def advance(self, dt, pwm, center_cmd, front_target, motion, stress, x, y, yaw, *, speed_target=None):
        env = self.environment
        curvature = self.steer(front_target, motion, dt)
        requested_curve = math.tan(math.radians(center_cmd)) / self.config.wheelbase
        factors = (1 - self.track * requested_curve / 2, 1 + self.track * requested_curve / 2)
        loop = self.wheel_pi_loop
        if loop is None:
            center = speed_for_pwm(pwm, motion.pwm_limit, motion.max_speed * motion.throttle_gain, motion.deadband_pwm)
            if self.max_wheel_speed and center > 0:
                center = min(center, self.max_wheel_speed / max(factors))
        else:
            if pwm <= 0:
                loop.reset_outputs()
            requested_center = (speed_for_pwm(pwm, motion.pwm_limit, motion.max_speed, motion.deadband_pwm)
                                if speed_target is None else speed_target)
            if self.max_wheel_speed and requested_center > 0:
                requested_center = min(requested_center, self.max_wheel_speed / max(factors))
            wheel_targets_mps = (requested_center * factors[0], requested_center * factors[1])
        speed = sum(self.ground_speeds) / 2
        slope_accel = 9.81 * env.grade_at(x) * math.cos(yaw)
        self.rough_floor_s += dt if slope_accel and speed > 0.03 else 0.0
        ground, rims, distances, ratios, grips = [], [], [], [], []
        for idx, side in enumerate((-1, 1)):
            offset = side * self.track / 2  # right is positive in simulation coordinates
            cx, cy = x - offset * math.sin(yaw), y + offset * math.cos(yaw)
            grip = env.grip_at(cx, cy)
            if stress.combined_grip:
                grip = min(grip, stress.grip_at(cx, cy))
            gain = env.left_motor_gain if idx == 0 else env.right_motor_gain
            if loop is None:
                target = center * factors[idx] * env.supply_scale * gain
            else:
                target = loop.actuation_speed(idx, motion) * env.supply_scale * gain * motion.throttle_gain
            current = self.ground_speeds[idx]
            if idx == 1 and env.stall_right_s is not None and self.elapsed_s >= env.stall_right_s:
                target = current = 0.0
            accelerating = target > current
            rate = (motion.acceleration if accelerating else motion.deceleration) * factors[idx] / env.load_scale
            if slope_accel:
                rate = max(0.02, rate + (-slope_accel if accelerating else slope_accel))
            demand = min(rate, abs(target - current) / dt)
            ratio = (contact_ratio(demand, speed * speed * abs(curvature), rate, motion.lateral_accel, grip)
                     if stress.combined_grip or grip < 1 else 0.0)
            rate /= max(1, ratio)
            ramp = min(dt, abs(target - current) / rate)
            next_speed = approach(current, target, rate * dt)
            travel = (current + next_speed) * ramp / 2 + next_speed * (dt - ramp)
            # Driven-wheel rotation can exceed ground travel while torque is applied to a slick patch.
            spin = 1 + (1 - grip) * min(1.0, demand / motion.acceleration) if accelerating else 1.0
            circumference = self.rollout[idx] * (env.right_rollout_scale if idx else 1.0)
            self.encoder_turns[idx] += travel * spin / circumference
            ground.append(next_speed)
            rims.append(next_speed * spin)
            distances.append(travel)
            ratios.append(ratio)
            grips.append(grip)
        self.ground_speeds, self.wheel_speeds = tuple(ground), tuple(rims)
        distance = sum(distances) / 2
        lateral_ratio = (distance / dt) ** 2 * abs(curvature) / (min(grips) * motion.lateral_accel)
        # Front-wheel constraints dominate; rear speed disagreement produces partial scrub/yaw.
        rear_yaw = (distances[0] - distances[1]) / self.track
        turn = (-0.8 * distance * curvature + 0.2 * rear_yaw) / max(1, lateral_ratio, *ratios)
        self.elapsed_s += dt
        self.encoder_counts = tuple(round(turns * cpr) for turns, cpr in zip(self.encoder_turns, self.cpr))
        if loop is not None:
            loop.update(self.encoder_counts, self.elapsed_s, wheel_targets_mps, active=pwm > 0)
        # Match the Nano dual-rear no-progress trip: both motors stop if a driven wheel
        # fails to accumulate two forward ticks in 350 ms. Host telemetry can fail independently.
        for idx, count in enumerate(self.encoder_counts):
            if not pwm or count - self.progress_counts[idx] >= 2:
                self.progress_counts[idx], self.progress_at[idx] = count, self.elapsed_s
            elif self.elapsed_s - self.progress_at[idx] >= 0.350:
                self.drive_fault = "rear wheel feedback timeout"
        available = env.encoder_outage_s is None or self.elapsed_s < env.encoder_outage_s
        if available and self.elapsed_s - self.last_sample_s >= self.sample_period - 1e-9:
            self.observe()
        return distance, turn, curvature, lateral_ratio, max(ratios), min(grips)

    def observe(self):
        dleft, dright = [(now - old) * rollout / cpr for now, old, rollout, cpr in
                         zip(self.encoder_counts, self.previous_counts, self.rollout, self.cpr)]
        self.previous_counts = self.encoder_counts
        travel = (dleft + dright) / 2 * self.distance_scale
        turn = (dleft - dright) / self.track * self.yaw_scale
        difference = turn * self.track / 2
        self.measured_pose = integrate_wheel_distances(self.measured_pose, travel + difference,
                                                      travel - difference, self.track)
        self.total_measured_distance += travel
        self.total_measured_yaw += turn
        self.speed_samples.append((self.elapsed_s, self.total_measured_distance, self.total_measured_yaw))
        while len(self.speed_samples) > 2 and self.speed_samples[1][0] <= self.elapsed_s - 0.10:
            self.speed_samples.popleft()
        stamp, distance, angle = self.speed_samples[0]
        span = self.elapsed_s - stamp
        self.measured = MotionFeedback((self.total_measured_distance - distance) / span,
                                       (self.total_measured_yaw - angle) / span)
        self.last_sample_s = self.elapsed_s
        self.feedback_history.append((self.elapsed_s, self.measured))

    def feedback_at(self, stamp_ms):
        for ts, sample in reversed(self.feedback_history):
            age = stamp_ms / 1000 - ts
            if age >= -1e-8:
                return sample if age <= max(0.1, 2 * self.sample_period) else None
        return None
