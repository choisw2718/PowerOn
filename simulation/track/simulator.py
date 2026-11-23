"""Run the lidar planner on virtual tracks with simulated vehicle dynamics."""

import argparse
from collections import deque
from dataclasses import dataclass, replace
from itertools import groupby
import math
import random

from hardware import PARAMETERS
from lidar.autopilot.motion.feedback import ActuationMonitor, MotionFeedback, approach
from lidar.autopilot.motion.speed_control import SpeedCommand, SpeedConfig, SpeedController, speed_for_pwm
from lidar.autopilot.planning.continuity import (BRIDGEABLE, MAX_WAIT_S, RECOVERABLE, TrackContinuity,
                                               MAX_LEASE_MS, UNMEASURED_TARGET_MPS, CommandMotionBound, command_clearance,
                                               lease_budget_ms, lease_floor_ms, motion_uncertainty)
from lidar.autopilot.planning.wall_track import ISOLATED_NEAR_RETURN, CorridorMemory, Decision, TrackConfig, plan
from lidar.geometry.scan_geometry import Beam, Pose, estimate_ray_times, pose_at, project_sweep, sample_clock_times

from .conditions import LOW_GRIP_CORNERS, STRESS_PROFILES, StressSettings, contact_ratio
from .courses import CHALLENGE_COURSES, Checkpoint, Scenario, track_outline
from .indoor import ENVIRONMENTS
from .four_wheel import FourWheelPlant, wheel_targets
from .indoor import indoor_environment
from .sensor_faults import SensorFaultModel


SCAN_MS = 150
PHYSICS_MS = 5
WATCHDOG_MS = MAX_LEASE_MS
SCAN_GRACE_MS = 40
LIDAR = PARAMETERS["lidar"]
MOTION = PARAMETERS["motion"]
SIMULATION = PARAMETERS["simulation"]
PLANNERS = ("legacy", "clearance", "route")
PLANNER_LABELS = {"legacy": "기존 경로 점수", "clearance": "차체 여유 우선", "route": "통로 경로 추종"}


@dataclass(frozen=True)
class SensorSettings:
    rolling: bool = True
    period_ms: int = SIMULATION["scan_period_ms"]
    point_rate_hz: int = SIMULATION["point_rate_hz"]
    max_range_m: float = SIMULATION["max_range_m"]
    noise_mm: float = 6.0
    offset_mm: float = 9.0
    yaw_deg: float = 1.8
    dropout: float = 0.01
    angular_speed_ripple: float = 0.06
    odometry_distance_scale: float = 1.015
    odometry_yaw_scale: float = 0.98
    seed: int = 7
    pose_sample_ms: int = 20
    ray_clock: str = "sample-clock"
    vibration_mm: float = 0.0
    vibration_yaw_deg: float = 0.0
    spike_rate: float = 0.0
    guard_spikes: bool = True
    burst_start_probability: float = 0.0
    burst_length_points: int = 0
    range_bias_sigma_mm: float = 0.0
    delivery_jitter_ms: int = 0

SENSOR_PROFILES = {
    "ideal": ("순간 스캔 (비교용)", SensorSettings(rolling=False, period_ms=SCAN_MS)),
    "rolling": ("회전 스캔", SensorSettings()),
    # 8,975 samples across 29 complete A1 rotations in 4.500 s of host timestamps.
    "a1-cadence": ("A1 실측 주기", SensorSettings(period_ms=155, point_rate_hz=1994)),
    "harsh": ("오차·누락 증가", SensorSettings(noise_mm=14, offset_mm=18, yaw_deg=3,
                                            dropout=0.04, angular_speed_ripple=0.12,
                                            odometry_distance_scale=1.06,
                                            odometry_yaw_scale=0.92)),
    "vibration": ("진동·거리 튐", SensorSettings(angular_speed_ripple=0.23, pose_sample_ms=25,
                                                vibration_mm=4, vibration_yaw_deg=0.7, spike_rate=0.015)),
    "disturbed": ("연속 누락·편차·지터", SensorSettings(burst_start_probability=0.015,
                                                        burst_length_points=5,
                                                        range_bias_sigma_mm=15.0,
                                                        delivery_jitter_ms=20)),
}
RAY_CLOCKS = ("sample-clock", "phase", "exact")


@dataclass(frozen=True)
class LidarReturn:
    angle: float
    distance_mm: float
    quality: int
    measured_ms: float
    hit: tuple[float, float] | None = None


@dataclass(frozen=True)
class MotionSettings:
    sensor_delay_ms: int = SIMULATION["sensor_delay_ms"]
    command_delay_ms: int = SIMULATION["command_delay_ms"]
    min_turn_radius: float = PARAMETERS["chassis"]["min_turn_radius_m"]
    acceleration: float = MOTION["acceleration_mps2"]
    deceleration: float = MOTION["deceleration_mps2"]
    steering_rate: float = SIMULATION["steering_rate_deg_s"]
    max_speed: float = MOTION["max_speed_mps"]
    lateral_accel: float = MOTION["lateral_acceleration_mps2"]
    deadband_pwm: float = MOTION["deadband_pwm_pct"]
    pwm_limit: float = LIDAR["preview_pwm_pct"]
    steer_gain: float = 1.0
    steer_bias_deg: float = 0.0
    steer_deadband_deg: float = 0.0
    throttle_gain: float = 1.0


MOTION_FIELDS = {
    "sensor_delay_ms": ("센서·계산 (ms)", int, 0, 1000),
    "command_delay_ms": ("명령 지연 (ms)", int, 0, 1000),
    "min_turn_radius": ("최소 반경 (m)", float, 0.16, 3),
    "steering_rate": ("조향 응답 (°/s)", float, 5, 1000),
    "acceleration": ("가속 (m/s²)", float, 0.05, 5),
    "deceleration": ("감속 (m/s²)", float, 0.05, 5),
    "max_speed": ("상한 PWM 속도 (m/s)", float, 0.05, 2),
    "lateral_accel": ("횡가속 한계 (m/s²)", float, 0.05, 5),
    "deadband_pwm": ("기동 PWM (%)", float, 0, 64),
    "pwm_limit": ("가상 최대 PWM (%)", float, 1, 100),
    "steer_gain": ("조향 배율", float, 0.3, 1.7),
    "steer_bias_deg": ("조향 편차 (°)", float, -12, 12),
    "steer_deadband_deg": ("조향 유격 (°)", float, 0, 6),
    "throttle_gain": ("구동 배율", float, 0.3, 1.7),
}

MOTION_PRESETS = {
    "baseline": ("기본 추정", MotionSettings()),
    "full-power": ("가상 모터 전출력", MotionSettings(
        pwm_limit=100.0, max_speed=MOTION["max_speed_mps"] * 100 / LIDAR["preview_pwm_pct"])),
    "delayed": ("지연·저응답", MotionSettings(sensor_delay_ms=180, command_delay_ms=160,
                                             min_turn_radius=0.45, acceleration=0.48,
                                             deceleration=0.24, steering_rate=65,
                                             max_speed=0.55, lateral_accel=0.24)),
    "mismatch": ("조향·구동 불일치", MotionSettings(command_delay_ms=85, steering_rate=75,
                                                   acceleration=0.45, deceleration=0.32,
                                                   steer_gain=0.82, steer_bias_deg=2.0,
                                                   steer_deadband_deg=0.7, throttle_gain=1.17)),
}


def motion_settings(values):
    for name, (_, _, low, high) in MOTION_FIELDS.items():
        if not low <= getattr(values, name) <= high:
            raise ValueError(f"{MOTION_FIELDS[name][0]} 입력 범위 {low:g}~{high:g}")
    if values.deadband_pwm >= values.pwm_limit:
        raise ValueError("최대 PWM을 기동 PWM보다 높게 설정하세요.")
    return values

STRAIGHT = ((-0.7, -0.50), (3.2, -0.50), (3.2, 0.50), (-0.7, 0.50))
RIGHT_BEND = ((-0.7, -0.45), (1.15, -0.45), (1.15, 2.8), (0.25, 2.8),
              (0.25, 0.45), (-0.7, 0.45))
LONG_BEND = ((-0.7, -0.55), (5.50, -0.55), (5.50, 3.2), (4.30, 3.2),
             (4.30, 0.55), (-0.7, 0.55))
DEAD_END = ((-0.7, -0.5), (1.35, -0.5), (1.35, 0.5), (-0.7, 0.5))
OUTER_LOOP = ((-1.2, -0.45), (2.25, -0.45), (2.25, 2.5), (-1.2, 2.5))
INNER_LOOP = ((-0.3, 0.45), (1.35, 0.45), (1.35, 1.6), (-0.3, 1.6))
# Centerline vertices and distances to the outer and inner walls in meters.
LOOP_RETURN_CORNER = ((6.4, 3.3), 0.44, 0.45)
LARGE_LOOP = (
    ((-2.0, 0.0), 0.52, 0.52),
    ((3.7, 0.0), 0.52, 0.52),
    ((5.0, 0.0), 0.72, 0.56),
    ((6.4, 0.0), 0.72, 0.56),
    ((8.0, 0.0), 0.51, 0.51),
    ((10.0, 0.0), 0.51, 0.51),
    ((10.0, 1.4), 0.51, 0.51),
    ((10.0, 2.5), 0.47, 0.32),
    ((10.0, 3.15), 0.47, 0.32),
    ((10.0, 4.05), 0.50, 0.48),
    ((10.0, 4.9), 0.50, 0.48),
    ((6.4, 4.9), 0.50, 0.45),
    LOOP_RETURN_CORNER,
    ((3.0, 3.3), 0.48, 0.48),
    ((3.0, 6.35), 0.53, 0.46),
    ((-2.0, 6.35), 0.51, 0.50),
)
RETURN_CORNER_INDEX = LARGE_LOOP.index(LOOP_RETURN_CORNER)
BEVELED_LOOP = (LARGE_LOOP[:RETURN_CORNER_INDEX]
                + (((6.4, 3.76), 0.44, 0.45), ((5.94, 3.3), 0.44, 0.45))
                + LARGE_LOOP[RETURN_CORNER_INDEX + 1:])
GAP_EDGES = ((-0.7, -0.5), (3.2, -0.5), (3.2, 0.5), (1.84, 0.5), (1.49, 0.5), (-0.7, 0.5))
SHORT_GAP = ((-0.7, -0.5), (3.2, -0.5), (3.2, 0.5), (1.56, 0.5), (1.5, 0.5), (-0.7, 0.5))
T_JUNCTION = ((-0.7, -0.45), (0.3, -0.45), (0.3, -2.5), (1.15, -2.5),
              (1.15, 2.5), (0.3, 2.5), (0.3, 0.45), (-0.7, 0.45))


def recess(end):
    return ((-0.7, -0.5), (3.2, -0.5), (3.2, 0.5), (end, 0.5), (end, 1.65),
            (1.42, 1.65), (1.42, 0.5), (-0.7, 0.5))


@dataclass(frozen=True)
class Split:
    lap: int
    checkpoint: int
    seconds: float
    distance: float
    peak_speed: float
    completed: bool = True
    min_wall_m: float | None = None
    min_wall_position_m: tuple[float, float] | None = None
    below_wall_threshold_s: float = 0.0

    @property
    def mean_speed(self):
        return self.distance / self.seconds if self.seconds else 0.0


LOOP_CHECKPOINTS = (
    Checkpoint("긴 직선 · 가변 폭", (8.0, 0.0), (1, 0), "직선 12.0 m · 최대 폭 1.28 m"),
    Checkpoint("긴 직선 뒤 90°", (10.0, 1.4), (0, 1), "가속에서 코너 감속으로 전환"),
    Checkpoint("비대칭 협로", (10.0, 4.1), (0, 1), "폭 1.02 → 0.79 → 0.98 m"),
    Checkpoint("짧은 간격 연속 코너", (6.4, 3.9), (0, -1), "코너 기준점 간격 1.6 m"),
    Checkpoint("반대 방향 코너", (3.0, 4.8), (0, 1), "우회전 뒤 좌회전"),
    Checkpoint("복귀 코너", (-2.0, 4.6), (0, -1), "5.0 m 직선 뒤 우회전"),
    Checkpoint("결승선", (0.0, 0.0), (1, 0), "7개 지점을 순서대로 통과"),
)


SCENARIOS = {
    "loop": Scenario("대형 복합 순환 트랙", track_outline(BEVELED_LOOP), checkpoints=LOOP_CHECKPOINTS),
    "loop-sharp": Scenario("대형 복합 트랙 · 연속 직각", track_outline(LARGE_LOOP),
                           checkpoints=LOOP_CHECKPOINTS),
    **CHALLENGE_COURSES,
    "small-loop": Scenario("소형 사각 순환 트랙", (OUTER_LOOP, INNER_LOOP)),
    "long-bend": Scenario("긴 직선 → 우회전", (LONG_BEND,)),
    "straight": Scenario("직선 · 왼쪽 편향", (STRAIGHT,), -0.16),
    "right": Scenario("오른쪽 90° 코너", (RIGHT_BEND,)),
    "left": Scenario("왼쪽 90° 코너", (tuple((x, -y) for x, y in RIGHT_BEND),)),
    "dead-end": Scenario("전방 벽 · 정지", (DEAD_END,)),
    "blocked-straight": Scenario("직선 복도 · 상자로 막힌 통로", (STRAIGHT, ((1.60, -0.47),
                                (1.78, -0.47), (1.78, 0.47), (1.60, 0.47)))),
    "furniture": Scenario("실내 복도 · 엇갈린 가구 돌출", (LONG_BEND,
                          ((1.25, -0.55), (1.85, -0.55), (1.85, -0.30), (1.25, -0.30)),
                          ((2.35, 0.22), (2.95, 0.22), (2.95, 0.55), (2.35, 0.55)))),
    "short-gap": Scenario("짧은 오른쪽 벽 틈", (SHORT_GAP,),
                          missing_walls=(((1.56, 0.5), (1.5, 0.5)),)),
    "wall-gap": Scenario("넓은 틈 · 관측 상실", (GAP_EDGES,),
                         missing_walls=(((1.84, 0.5), (1.49, 0.5)),)),
    "side-opening": Scenario("오른쪽 벽 벌어짐", (recess(1.87),)),
    "wide-opening": Scenario("긴 옆방 · 폭 확인 부족", (recess(2.30),)),
    "junction": Scenario("T자 분기 · 경로 불명", (T_JUNCTION,)),
}


def wall_segments(outlines):
    return tuple((start, end) for outline in outlines
                 for start, end in zip(outline, outline[1:] + outline[:1]))


def ray_hit(walls, x, y, dx, dy):
    distance, incidence = math.inf, 0.0
    for (x1, y1), (x2, y2) in walls:
        sx, sy = x2 - x1, y2 - y1
        determinant = dx * sy - dy * sx
        if abs(determinant) < 1e-9:
            continue
        nx, ny = x1 - x, y1 - y
        travel = (nx * sy - ny * sx) / determinant
        along_wall = (nx * dy - ny * dx) / determinant
        if 0 < travel < distance and 0 <= along_wall <= 1:
            distance = travel
            incidence = min(1.0, abs(determinant) / math.hypot(sx, sy))
    return distance, incidence


def ray_scan(walls, x=0.0, y=0.0, yaw=0.0, forward=0.0, drop_front=False):
    """Return the nearest wall intersection for each clockwise, 1° lidar ray."""
    for angle in range(360):
        relative = (angle - forward + 180) % 360 - 180
        if drop_front and abs(relative) <= 45:
            yield angle, 0.0, 0
            continue
        bearing = math.radians(relative) + yaw
        dx, dy = math.cos(bearing), math.sin(bearing)
        distance, _ = ray_hit(walls, x, y, dx, dy)
        yield angle, distance * 1000, 25 if math.isfinite(distance) else 0


def segment_hits_body(a, b, config):
    """Clip a wall segment in body coordinates to the physical body rectangle."""
    near, far = 0.0, 1.0
    half_width = config.width / 2
    for start, end, low, high in ((a[0], b[0], -config.rear, config.front),
                                  (a[1], b[1], -half_width, half_width)):
        direction = end - start
        if abs(direction) < 1e-9:
            if not low <= start <= high:
                return False
            continue
        entry, exit_ = sorted(((low - start) / direction, (high - start) / direction))
        near, far = max(near, entry), min(far, exit_)
        if near > far:
            return False
    return True


def segment_body_clearance(a, b, config):
    if segment_hits_body(a, b, config):
        return 0.0
    half = config.width / 2
    corners = [(x, y) for x in (-config.rear, config.front) for y in (-half, half)]
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = dx * dx + dy * dy
    distances = [math.hypot(max(-config.rear - x, x - config.front, 0),
                            max(abs(y) - half, 0)) for x, y in (a, b)]
    if length:
        for x, y in corners:
            t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length))
            distances.append(math.hypot(x - a[0] - t * dx, y - a[1] - t * dy))
    return min(distances)


class Simulation:
    def __init__(self, scenario, motion=MotionSettings(), calibrated=True, *,
                 sensor=SENSOR_PROFILES["ideal"][1], deskew=True, calibrate_lidar=True,
                 feedback=False, stress=StressSettings(), audit_leases=False,
                 coast_on_scan_gap=False, measure_clearance=False, planner="legacy",
                 wall_threshold_m=0.10, vehicle_model="four-wheel", environment="nominal",
                 environment_seed=None, environment_config=None, controller_motion=None,
                 wheel_pi=None, speed_policy=None, sensor_calibration=None):
        if vehicle_model not in ("four-wheel", "legacy"):
            raise ValueError(f"Unknown vehicle model: {vehicle_model}")
        if vehicle_model != "four-wheel" and (wheel_pi is not None or speed_policy is not None):
            raise ValueError("Wheel PI and experimental speed policies require the four-wheel model")
        self.vehicle_model = vehicle_model
        seed = sensor.seed if environment_seed is None else environment_seed
        self.environment = environment_config if environment_config is not None else indoor_environment(environment, seed)
        self.scenario = scenario
        self.motion = motion
        controller_motion = motion if controller_motion is None else controller_motion
        self.speed_policy = speed_policy
        self.lidar = sensor
        self.period_ms = sensor.period_ms
        self.deskew = deskew and (feedback or vehicle_model == "legacy")
        self.calibrate_lidar = calibrate_lidar
        self.sensor_calibration = sensor_calibration
        self.feedback_enabled = feedback
        self.stress = stress
        if scenario.grip_zones and stress.grip_zones is LOW_GRIP_CORNERS:
            self.stress = replace(stress, grip_zones=scenario.grip_zones)
        self.audit_leases = audit_leases
        self.coast_on_scan_gap = coast_on_scan_gap
        self.command_armed = False
        self.scan_deadline_ms = None
        self.scan_gap_since_ms = None
        self.scan_gaps = self.recovered_scan_gaps = self.longest_recovery_ms = 0
        self.started_moving = False
        self.power_cut_counts = {cause: 0 for cause in ("speed_control", "observation", "terminal")}
        self.power_cut_moving_ms = dict.fromkeys(self.power_cut_counts, 0)
        self.power_cut_cause = None
        self.stopped_since_ms = None
        self.stop_episodes = self.longest_stop_ms = 0
        # Only these two benchmark courses share the 12 m starting straight.
        self.straight_cruise = None
        if measure_clearance and scenario.checkpoints == LOOP_CHECKPOINTS:
            self.straight_cruise = dict(start_x_m=1.0, end_x_m=7.0, threshold_mps=0.98 * motion.max_speed,
                                        sampled_ms=0, peak_mps=None, peak_target_mps=None, peak_pwm_pct=None,
                                        near_max_ms=0, longest_near_max_ms=0)
        self.near_max_run_ms = 0
        self.lease_checks = self.short_leases = 0
        self.min_lease_slack_ms = None
        self.worst_lease_position = None
        self.worst_lease_time_s = None
        self.dropped_scans = self.stale_scans = self.dropped_commands = self.delayed_commands = 0
        self.watchdog_trips = self.front_gap_scans = 0
        self.traction_limited_ms = 0.0
        self.low_grip_ms = 0.0
        self.peak_contact_ratio = 0.0
        self.random = random.Random(sensor.seed)
        self.vibration_random = random.Random(sensor.seed + 1)
        self.sensor_faults = SensorFaultModel(sensor)
        self.last_scan_delivery = None
        self.range_spikes = self.rejected_far = self.suspect_near = 0
        self.near_return_holds = 0
        self.waiting_for_near_return = False
        self.config = TrackConfig(pwm=controller_motion.pwm_limit,
                                  min_turn_radius=controller_motion.min_turn_radius if calibrated else 0,
                                  prefer_corner_clearance=planner in ("clearance", "route"),
                                  follow_route=planner == "route")
        self.controller = SpeedController(SpeedConfig(
            max_speed=controller_motion.max_speed, acceleration=controller_motion.acceleration,
            deceleration=controller_motion.deceleration, lateral_accel=controller_motion.lateral_accel,
            delay_ms=self.period_ms + controller_motion.sensor_delay_ms + controller_motion.command_delay_ms,
            deadband_pwm=controller_motion.deadband_pwm, steering_delay_ms=controller_motion.command_delay_ms,
            steering_rate_deg_s=controller_motion.steering_rate,
            propulsion_delay_ms=controller_motion.command_delay_ms) if calibrated else SpeedConfig())
        self.actuation = ActuationMonitor(self.controller.settings)
        self.walls = tuple(wall for wall in wall_segments(scenario.outlines + self.environment.obstacles)
                           if wall not in scenario.missing_walls)
        self.yaw = self.environment.start_yaw
        self.rear_x = -self.config.sensor_ahead * math.cos(self.yaw)
        self.rear_y = scenario.start_y + self.environment.start_offset - self.config.sensor_ahead * math.sin(self.yaw)
        self.odometry = Pose(self.rear_x, self.rear_y, self.yaw)
        self.plant = (FourWheelPlant(self.config, self.environment, self.odometry, sensor, wheel_pi=wheel_pi)
                      if vehicle_model == "four-wheel" else None)
        if calibrated and self.plant is not None:
            swept = self.plant.swept_envelope()
            self.config = replace(self.config, width=swept.width, front=swept.front, rear=swept.rear)
        self.collision_envelope = self.plant.envelope() if self.plant else self.config
        self.clearance_envelope = self.collision_envelope
        self.command_steer = 0.0
        self.weak_panel_returns = self.weak_panel_misses = 0
        self.floor_returns = 0
        self.scan_pose_covered = False
        self.observed_sensor_pose = None
        self.now_ms = 0
        self.next_pose_ms = sensor.pose_sample_ms
        self.pose_history = deque([(0, self.sensor_odometry)], maxlen=math.ceil(2500 / sensor.pose_sample_ms))
        self.next_scan_ms = self.period_ms if sensor.rolling else 0
        self.next_point_index = 0
        self.sweep = []
        self.sweep_front_gap = False
        self.scan_geometry = None
        self.scan_errors = None
        self.scan_age_ms = None
        self.moving_error_sums = [0.0, 0.0]
        self.moving_return_count = 0
        self.front_error_sums = [0.0] * 4
        self.front_return_count = 0
        self.scans = deque()
        self.commands = deque()
        self.valid_scans = 0
        self.limited_scans = 0
        self.corner_exit_scans = 0
        self.last_command_ms = 0
        self.distance = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.actual_steer = 0.0
        self.target_steer = 0.0
        self.applied_pwm = 0.0
        self.applied_speed_target = None
        self.coast_commands = 0
        self.response = None
        self.peak_steer = 0.0
        self.peak_speed = 0.0
        self.turn_limited = False
        self.peak_lateral_ratio = 0.0
        self.drop_front = False
        self.collision = False
        self.stop_reason = None
        self.stop_request_distance = None
        self.scan_pose = (*self.sensor, self.yaw)
        self.samples = []
        self.decision = Decision(reason="waiting for scan")
        self.corridor = CorridorMemory()
        self.continuity = TrackContinuity()
        self.control_frame_end = None
        self.motion_bound = CommandMotionBound(self.controller.settings)
        self.bridged_scans = 0
        self.waiting_scans = 0
        self.held_steer = 0.0
        self.command_lease_ms = WATCHDOG_MS
        self.demand = SpeedCommand(0.0, 0.0, "hold")
        self.steer_limit = min(LIDAR["max_center_steer_deg"],
                               math.degrees(math.atan(self.config.wheelbase / motion.min_turn_radius)))
        self.trail = deque([(self.sensor, 0.0)], maxlen=1200)
        self.checkpoint_index = 0
        self.completed_laps = 0
        self.lap_start_ms = 0
        self.last_lap_s = None
        self.split_start_ms = 0
        self.split_start_distance = 0.0
        self.split_peak_speed = 0.0
        self.last_split = None
        self.splits = {}
        self.split_history = []
        self.stop_context = None
        self.clearance_pose = Pose(*self.sensor, self.yaw) if measure_clearance else None
        self.clearance_at_pose = self.wall_clearance() if measure_clearance else None
        self.minimum_clearance = self.clearance_at_pose
        self.minimum_clearance_position = self.sensor if measure_clearance else None
        self.wall_threshold_m = wall_threshold_m
        self.split_minimum_clearance = self.minimum_clearance
        self.split_clearance_position = self.minimum_clearance_position
        self.split_below_wall_ms = self.below_wall_ms = 0
        envelope = self.collision_envelope
        self.body_radius = math.hypot(envelope.width / 2, max(envelope.front, envelope.rear))
        self.process_events()

    @property
    def sensor(self):
        return (self.rear_x + self.config.sensor_ahead * math.cos(self.yaw),
                self.rear_y + self.config.sensor_ahead * math.sin(self.yaw))

    @property
    def sensor_odometry(self):
        return Pose(*self.odometry.world(self.config.sensor_ahead, 0), self.odometry.yaw)

    def acquire_beams(self, before, previous_pose):
        lidar = self.lidar
        current = Pose(*self.sensor, self.yaw)
        sweep_start = self.next_scan_ms - self.period_ms
        point_interval = 1000 / lidar.point_rate_hz
        measured = (self.next_point_index + 0.5) * point_interval
        while measured <= self.now_ms and measured < self.next_scan_ms:
            phase = (measured - sweep_start) / self.period_ms
            fraction = (measured - before) / (self.now_ms - before)
            pose = previous_pose.interpolate(current, fraction)
            vibration = min(1.0, self.speed / 0.15)
            if lidar.vibration_mm or lidar.vibration_yaw_deg:
                seconds = measured / 1000
                radius = lidar.vibration_mm * vibration / 1000
                x, y = pose.world(radius * math.sin(math.tau * 33 * seconds),
                                  radius * math.sin(math.tau * 47 * seconds + 0.7))
                pose = Pose(x, y, pose.yaw + math.radians(lidar.vibration_yaw_deg * vibration)
                            * math.sin(math.tau * 39 * seconds))
            angle = (phase + lidar.angular_speed_ripple * math.sin(math.tau * phase) / math.tau) * 360
            relative = math.remainder(angle + lidar.yaw_deg, 360)
            bearing = pose.yaw + math.radians(relative)
            dx, dy = math.cos(bearing), math.sin(bearing)
            distance, incidence = ray_hit(self.walls, pose.x, pose.y, dx, dy)
            if self.environment.uneven_floor:
                # Simplified vertical displacement: a nose-down scan can hit the floor before a wall.
                pitch = self.environment.pitch_at(pose.x) * math.cos(pose.yaw) * math.cos(math.radians(relative))
                if pitch > 0:
                    floor_range = SIMULATION["lidar_height_m"] / math.tan(pitch)
                    if floor_range < distance:
                        distance, incidence = floor_range, math.sin(pitch)
                        self.floor_returns += 1
            strength = 1.0
            if self.environment.weak_panel and math.isfinite(distance):
                strength = self.environment.reflectivity(pose.x + distance * dx, pose.y + distance * dy)
            dropout = lidar.dropout + 0.06 * (1 - incidence) ** 3 + (1 - strength) * (0.5 + 0.4 * (1 - incidence))
            manual_gap = abs(relative) <= 45 and self.drop_front
            scheduled_gap = abs(relative) <= 45 and self.stress.fault("front-gap", measured)
            if manual_gap or scheduled_gap:
                self.sweep_front_gap = True
            burst_dropped = self.sensor_faults.next_beam_dropped()
            if manual_gap or not 0.15 <= distance <= lidar.max_range_m or self.random.random() < dropout:
                self.sweep.append(LidarReturn(angle, 0.0, 0, measured))
                self.weak_panel_misses += strength < 1.0
            else:
                bias_mm = self.sensor_faults.range_bias_mm
                measured_range = 1000 * distance + lidar.offset_mm + bias_mm + self.random.gauss(
                    0.0, lidar.noise_mm * (2 - incidence) / strength)
                if bias_mm:
                    measured_range = max(0.0, measured_range)
                if lidar.spike_rate and self.vibration_random.random() < lidar.spike_rate * vibration:
                    amplitude = self.vibration_random.uniform(90, 230)
                    measured_range += amplitude if self.vibration_random.random() < 0.65 else -amplitude
                    self.range_spikes += not burst_dropped
                hit = pose.x + distance * dx, pose.y + distance * dy
                quality = round((12 + 13 * incidence) * strength)
                self.weak_panel_returns += strength < 1.0 and not burst_dropped
                self.weak_panel_misses += strength < 1.0 and burst_dropped
                if scheduled_gap or burst_dropped:
                    self.sweep.append(LidarReturn(angle, 0.0, 0, measured))
                else:
                    self.sweep.append(LidarReturn(angle, round(measured_range * 4) / 4, quality, measured, hit))
            self.next_point_index += 1
            measured = (self.next_point_index + 0.5) * point_interval

    def process_sweep(self, beams, capture_end, capture_pose):
        stamps, odometry = zip(*self.pose_history)
        start = capture_end - self.period_ms
        if self.lidar.ray_clock == "phase":
            beam_times = estimate_ray_times([(b.angle, b.distance_mm, b.quality) for b in beams],
                                            start, capture_end)
        elif self.lidar.ray_clock == "exact":
            beam_times = (beam.measured_ms for beam in beams)
        else:
            beam_times = sample_clock_times(len(beams), start, capture_end)
        self.scan_pose_covered = stamps[0] <= start and stamps[-1] >= capture_end
        deskew = self.deskew and self.scan_pose_covered
        reference = pose_at(stamps, odometry, capture_end) if deskew else Pose(0.0, 0.0, 0.0)
        self.observed_sensor_pose = reference if deskew else None
        timed_beams = (Beam(beam.angle, beam.distance_mm, beam.quality, stamp,
                            pose_at(stamps, odometry, stamp) if deskew else reference)
                       for beam, stamp in zip(beams, beam_times))
        sensor_yaw, offset = 0.0, 0.0
        if self.calibrate_lidar:
            nominal = self.sensor_calibration
            sensor_yaw, offset = (self.lidar.yaw_deg, self.lidar.offset_mm) if nominal is None else nominal
        geometry = project_sweep(timed_beams, reference, yaw_deg=sensor_yaw, offset_mm=offset,
                                 deskew=deskew, guard_spikes=self.lidar.guard_spikes)
        self.scan_geometry = geometry
        self.samples = geometry.samples
        self.rejected_far += geometry.rejected_far
        self.suspect_near += len(geometry.wall_exclusions)
        real_pose = Pose(*capture_pose)
        self.scan_pose = capture_pose
        self.scan_age_ms = self.now_ms - start
        raw_error = processed_error = 0.0
        valid = 0
        for beam, sample in zip(beams, self.samples):
            if not sample[2]:
                continue
            estimated_hit = geometry.points[valid]
            valid += 1
            truth = real_pose.local(*beam.hit)
            angle = math.radians(beam.angle)
            raw = (beam.distance_mm / 1000 * math.cos(angle),
                   beam.distance_mm / 1000 * math.sin(angle))
            raw_error += math.dist(raw, truth) ** 2
            processed_error += math.dist(estimated_hit, truth) ** 2
            truth_bearing = math.atan2(truth[1], truth[0])
            if self.speed >= 0.05 and abs(truth_bearing) <= math.pi / 4:
                truth_range = math.hypot(*truth)
                for index, estimate in enumerate((raw, estimated_hit)):
                    delta_angle = math.remainder(math.atan2(estimate[1], estimate[0]) - truth_bearing, math.tau)
                    self.front_error_sums[2 * index] += (math.hypot(*estimate) - truth_range) ** 2
                    self.front_error_sums[2 * index + 1] += delta_angle ** 2
                self.front_return_count += 1
        self.scan_errors = ((1000 * math.sqrt(raw_error / valid),
                             1000 * math.sqrt(processed_error / valid)) if valid else None)
        if self.speed >= 0.05:
            self.moving_error_sums[0] += raw_error
            self.moving_error_sums[1] += processed_error
            self.moving_return_count += valid

    def motion_scan_metrics(self):
        metrics = {"range_spikes": self.range_spikes, "far_rejected": self.rejected_far,
                   "near_suspect": self.suspect_near, "near_return_holds": self.near_return_holds,
                   "front_returns": self.front_return_count}
        if self.sensor_faults.enabled:
            metrics.update(burst_starts=self.sensor_faults.burst_starts, burst_drops=self.sensor_faults.burst_drops)
        if self.moving_return_count:
            for label, errors in zip(("raw", "corrected"), self.moving_error_sums):
                metrics[f"moving_{label}_rms_mm"] = round(1000 * math.sqrt(errors / self.moving_return_count), 2)
        if self.front_return_count:
            scale = (1000, 180 / math.pi, 1000, 180 / math.pi)
            names = ("front_raw_range_mm", "front_raw_bearing_deg",
                     "front_corrected_range_mm", "front_corrected_bearing_deg")
            for name, factor, errors in zip(names, scale, self.front_error_sums):
                metrics[name] = round(factor * math.sqrt(errors / self.front_return_count), 2)
        return metrics

    @property
    def can_advance(self):
        return not self.collision and (self.stop_reason is None or self.speed > 0
                                       or self.applied_pwm > 0)

    def queue_command(self, pwm, steer, lease=WATCHDOG_MS, *, cause="speed_control", speed_target=None):
        if self.stress.fault("command-drop", self.now_ms):
            self.dropped_commands += 1
            return False
        extra = self.stress.fault("command-delay", self.now_ms)
        self.delayed_commands += extra > 0
        delivery = self.now_ms + self.motion.command_delay_ms + extra
        if self.commands:
            delivery = max(delivery, self.commands[-1][0])
        command = (delivery, pwm, steer, lease, cause)
        self.commands.append(command if speed_target is None else (*command, speed_target))
        return True

    def apply_pwm(self, pwm, cause):
        if self.applied_pwm > 0 and pwm <= 0 and self.speed >= 0.03:
            self.power_cut_counts[cause] += 1
            self.power_cut_cause = cause
        elif pwm > 0:
            self.power_cut_cause = None
        self.applied_pwm = pwm
        self.applied_speed_target = None

    def power_cut_metrics(self):
        by_cause = {cause: dict(count=count, moving_off_s=self.power_cut_moving_ms[cause] / 1000)
                    for cause, count in self.power_cut_counts.items()}
        count = sum(self.power_cut_counts.values())
        return dict(count=count, per_10m=round(10 * count / self.distance, 3) if self.distance else None,
                    moving_off_s=sum(self.power_cut_moving_ms.values()) / 1000, by_cause=by_cause)

    def motion_feedback(self, stamp_ms=None):
        if self.feedback_enabled:
            if self.plant is not None:
                return self.plant.feedback_at(self.now_ms if stamp_ms is None else stamp_ms)
            return MotionFeedback(round(self.speed * self.lidar.odometry_distance_scale, 3),
                                  self.yaw_rate * self.lidar.odometry_yaw_scale, round(self.actual_steer, 1))
        return None

    def process_events(self):
        now = self.now_ms
        live_policy = self.vehicle_model == "four-wheel"
        watchdog_active = self.command_armed if live_policy or self.coast_on_scan_gap else self.applied_pwm > 0
        if watchdog_active and now - self.last_command_ms >= self.command_lease_ms:
            self.watchdog_trips += 1
            self.stop_reason = self.stop_reason or "command watchdog"
            if self.stop_request_distance is None:
                self.stop_request_distance = self.distance
            self.apply_pwm(0.0, "terminal")
            self.command_armed = False
            self.commands.clear()
        if now == self.next_scan_ms:
            pose = (*self.sensor, self.yaw)
            if self.lidar.rolling:
                samples, self.sweep = self.sweep, []
                front_missing, self.sweep_front_gap = self.sweep_front_gap, False
            else:
                front_missing = self.drop_front or bool(self.stress.fault("front-gap", now))
                samples = list(ray_scan(self.walls, *pose, self.config.forward_angle, front_missing))
                if self.sensor_faults.enabled:
                    samples = self.sensor_faults.apply_to_scan(samples)
            self.front_gap_scans += bool(front_missing)
            if self.stress.fault("scan-drop", now):
                self.dropped_scans += 1
            else:
                delivery = now + self.motion.sensor_delay_ms + self.stress.fault("scan-delay", now)
                delivery = max(now, delivery + self.sensor_faults.sample_delivery_jitter_ms())
                if self.lidar.rolling:
                    cadence = self.lidar.pose_sample_ms
                    delivery = max(delivery, math.ceil(now / cadence) * cadence)
                if self.scans:
                    delivery = max(delivery, self.scans[-1][0])
                self.scans.append((delivery, now, pose, samples))
                self.last_scan_delivery = (now, delivery)
            self.sensor_faults.advance_sweep()
            self.next_scan_ms += self.period_ms
        while self.scans and self.scans[0][0] <= now:
            _, capture_end, acquired_pose, samples = self.scans.popleft()
            max_delay = self.stress.scan_max_delivery_ms
            if max_delay is not None and now - capture_end > max_delay:
                self.stale_scans += 1
                if not live_policy:
                    continue
            self.scan_deadline_ms = now + self.period_ms + SCAN_GRACE_MS
            if self.lidar.rolling:
                self.process_sweep(samples, capture_end, acquired_pose)
            else:
                self.scan_pose, self.samples = acquired_pose, samples
            if self.stop_reason:
                continue
            age = ((self.scan_age_ms + self.motion.command_delay_ms) / 1000
                   if self.lidar.rolling else 0.0)
            feedback, steering, cap = self.motion_feedback(capture_end), None, math.inf
            uncertainty = None
            observed_end = capture_end / 1000
            observed_start = observed_end - self.period_ms / 1000 if self.lidar.rolling else observed_end
            geometry = self.scan_geometry
            command_distance = 0.0
            if live_policy:
                age = max(0.0, now / 1000 - observed_start)
                reference = observed_end if feedback is not None else observed_start
                command_distance = command_clearance(reference, self.config, self.controller.settings,
                                                     now / 1000, lease_floor_ms(self.period_ms / 1000), feedback)
                if feedback is None:
                    cap = UNMEASURED_TARGET_MPS
                    geometry = None
                    if self.continuity.confirmed and self.lidar.rolling:
                        bound = self.motion_bound.maximum(observed_start, now / 1000)
                        uncertainty = motion_uncertainty(self.config, self.controller.settings, age, speed_bound=bound)
            if feedback is not None:
                self.response, cap = self.actuation.check(feedback, self.config.wheelbase, now / 1000)
                steering = self.actuation.transition(feedback, self.config.wheelbase, self.controller.target)
            self.decision = plan(self.samples, self.config, self.held_steer if live_policy else self.decision.steer,
                                 memory=self.corridor, now=observed_end if live_policy else now / 1000,
                                 geometry=geometry, motion=uncertainty, command_distance=command_distance,
                                 lookahead=self.controller.lookahead(age, feedback=feedback),
                                 sensor_pose=self.observed_sensor_pose if feedback is not None else None,
                                 cruise_horizon=self.controller.cruise_horizon(age, lease_floor_ms(self.period_ms / 1000) / 1000),
                                 steering=steering)
            self.decision = self.controller.checked_route(self.decision, feedback, age, self.response, cap)
            if live_policy:
                if (self.feedback_enabled and self.continuity.confirmed
                        and (feedback is None or self.lidar.rolling and self.deskew and not self.scan_pose_covered)):
                    self.decision = Decision(reason="motion unavailable")
                self.process_drive_gate(observed_start, observed_end, age, feedback, steering, cap, uncertainty,
                                        geometry)
                continue
            live_reference = observed_end if feedback is not None else observed_start
            old_wait = self.continuity.waiting_at is not None
            bridge = False
            lease = WATCHDOG_MS
            if ((self.scan_gap_since_ms is not None or self.waiting_for_near_return)
                    and self.decision.pwm > 0):
                lease = lease_budget_ms(self.decision, live_reference, self.config,
                                        self.controller.settings, now / 1000, feedback=feedback)
                if lease < lease_floor_ms(self.period_ms / 1000):
                    self.decision = replace(self.decision, pwm=0.0, reason="insufficient clear distance")
            if self.continuity.confirmed and self.decision.reason in BRIDGEABLE:
                retained, budget = self.continuity.assess(
                    self.samples, self.config, self.controller.settings, now / 1000,
                    geometry=self.scan_geometry, steering=steering, feedback=feedback)
                if retained is not None and retained.pwm <= 0:
                    self.decision = retained
                elif (retained is not None and not self.response
                      and budget >= lease_floor_ms(self.period_ms / 1000)):
                    self.decision = retained
                    bridge, lease = True, budget
                    cap = min(cap, 0.15)
                    self.bridged_scans += 1
            self.limited_scans += self.decision.speed_scale < 1.0
            self.corner_exit_scans += self.decision.reason == "confirmed corner exit"
            if self.decision.reason == ISOLATED_NEAR_RETURN:
                self.near_return_holds += 1
                self.waiting_for_near_return = True
                if not self.continuity.confirmed:
                    self.valid_scans = 0
                    continue
            sensing_gap = self.decision.pwm <= 0 and self.decision.reason in RECOVERABLE
            if (self.continuity.confirmed and not self.continuity.expired(now / 1000)
                    and (sensing_gap or self.decision.pwm > 0)):
                if sensing_gap:
                    self.continuity.wait(now / 1000)
                waiting = self.continuity.waiting_at is not None
                if waiting and (self.decision.pwm <= 0 or not self.continuity.ready()):
                    self.waiting_scans += 1
                    if not old_wait:
                        self.stop_request_distance = self.distance
                    self.demand = self.controller.update(Decision(), self.config, 0)
                    queued = self.queue_command(0.0, self.held_steer, cause="observation")
                    if queued and feedback is not None:
                        self.actuation.record(self.held_steer, SpeedCommand(0, 0, "coast"), feedback,
                                              now / 1000)
                    continue
            elif self.continuity.expired(now / 1000) and self.decision.pwm > 0:
                self.decision = Decision(reason="scan recovery timed out")
            if self.decision.pwm <= 0:
                self.stop_reason = self.decision.reason
                if self.stop_request_distance is None:
                    self.stop_request_distance = self.distance
                self.demand = self.controller.update(self.decision, self.config, self.period_ms / 1000,
                                                     scan_age=age)
                self.queue_command(0.0, self.held_steer, cause="terminal")
            else:
                self.valid_scans += 1
                if self.valid_scans >= 3:
                    self.demand = self.controller.update(self.decision, self.config, self.period_ms / 1000,
                                                         scan_age=age, speed_cap=cap, feedback=feedback)
                    queued = self.queue_command(self.demand.pwm, self.decision.steer, lease)
                    if not queued:
                        continue
                    if self.demand.pwm > 0:
                        self.waiting_for_near_return = False
                    self.coast_commands += self.demand.phase == "coast"
                    if feedback is not None:
                        self.actuation.record(self.decision.steer, self.demand, feedback, now / 1000)
                    self.held_steer = self.decision.steer
                    if not bridge:
                        reference = observed_end if self.scan_geometry is not None and self.deskew else observed_start
                        self.continuity.remember(self.decision, reference, observed_end)
                        if self.audit_leases and self.demand.pwm > 0:
                            budget = lease_budget_ms(self.decision, live_reference, self.config,
                                                     self.controller.settings, now / 1000, feedback=feedback)
                            slack = budget - lease_floor_ms(self.period_ms / 1000)
                            self.lease_checks += 1
                            self.short_leases += slack < 0
                            if self.min_lease_slack_ms is None or slack < self.min_lease_slack_ms:
                                self.min_lease_slack_ms = slack
                                self.worst_lease_position = self.sensor
                                self.worst_lease_time_s = now / 1000
                        self.stop_request_distance = None
        if (self.coast_on_scan_gap and self.command_armed and self.stop_reason is None
                and self.scan_deadline_ms is not None and now >= self.scan_deadline_ms):
            self.coast_for_missing_scan()
        while self.commands and self.commands[0][0] <= now:
            self.last_command_ms, pwm, steer, *metadata = self.commands.popleft()
            self.apply_pwm(pwm, metadata[1] if len(metadata) > 1 else "speed_control")
            if pwm > 0 and len(metadata) > 2:
                self.applied_speed_target = metadata[2]
            self.command_armed = self.stop_reason is None
            self.motion_bound.applied(self.applied_pwm > 0, now / 1000)
            self.command_lease_ms = metadata[0] if metadata else WATCHDOG_MS
            actual_target = steer * self.motion.steer_gain + self.motion.steer_bias_deg
            self.command_steer = steer
            self.target_steer = max(-self.steer_limit, min(self.steer_limit, actual_target))
            if self.applied_pwm > 0 and self.scan_gap_since_ms is not None and self.continuity.waiting_at is None:
                self.recovered_scan_gaps += 1
                self.longest_recovery_ms = max(self.longest_recovery_ms, now - self.scan_gap_since_ms)
                self.scan_gap_since_ms = None

    def process_drive_gate(self, start, end, age, feedback, steering, cap, uncertainty, geometry):
        now = self.now_ms / 1000
        driving = self.continuity.confirmed is not None
        settings = self.controller.settings
        gate = self.continuity.gate(self.decision, self.samples, self.config, settings, start, end,
                                    now, driving=driving, geometry=geometry, motion=uncertainty,
                                    steering=steering, feedback=feedback, issue=self.response,
                                    period=self.period_ms / 1000)
        self.decision = gate.route
        self.valid_scans = self.continuity.start_scans
        self.bridged_scans += gate.bridged
        self.limited_scans += self.decision.speed_scale < 1
        self.corner_exit_scans += self.decision.reason == "confirmed corner exit"
        self.near_return_holds += self.decision.reason == ISOLATED_NEAR_RETURN
        if self.audit_leases and self.decision.pwm > 0 and gate.action in {"drive", "coast"}:
            slack = gate.lease_ms - lease_floor_ms(self.period_ms / 1000)
            self.lease_checks += 1
            self.short_leases += slack < 0
            if self.min_lease_slack_ms is None or slack < self.min_lease_slack_ms:
                self.min_lease_slack_ms = slack
                self.worst_lease_position, self.worst_lease_time_s = self.sensor, now
        if gate.action == "observe":
            return
        if gate.action != "drive":
            if gate.action == "coast":
                self.waiting_scans += 1
            else:
                self.stop_reason = self.decision.reason
            if self.stop_request_distance is None:
                self.stop_request_distance = self.distance
            self.control_frame_end = None
            self.demand = self.controller.update(Decision(), self.config, 0)
            cause = "observation" if gate.action == "coast" else "terminal"
            if driving and self.queue_command(0, self.held_steer, cause=cause) and feedback is not None:
                self.actuation.record(self.held_steer, self.demand, feedback, now)
            return
        interval = 0.15 if self.control_frame_end is None else min(0.38, end - self.control_frame_end)
        self.control_frame_end = end
        if self.speed_policy is not None and feedback is not None and not gate.bridged:
            scale = self.speed_policy(feedback.speed_mps, self.controller.target, self.decision.steer,
                                      self.decision.clear_m, age)
            if not math.isfinite(scale) or not 0 < scale <= 1:
                raise ValueError("experimental speed policy must return a finite scale in (0, 1]")
            cap = min(cap, settings.max_speed * scale)
        self.demand = self.controller.update(self.decision, self.config, interval, scan_age=age,
                                             speed_cap=min(cap, 0.15) if gate.bridged else cap,
                                             feedback=feedback)
        speed_target = self.demand.speed if self.plant.wheel_pi_loop is not None else None
        if not self.queue_command(self.demand.pwm, self.decision.steer, gate.lease_ms, speed_target=speed_target):
            return
        self.coast_commands += self.demand.phase == "coast"
        self.held_steer = self.decision.steer
        if feedback is not None:
            self.actuation.record(self.held_steer, self.demand, feedback, now)
        if not gate.bridged:
            self.continuity.remember(self.decision, end if feedback is not None else start, end)
            self.stop_request_distance = None

    def coast_for_missing_scan(self):
        now = self.now_ms
        if self.scan_gap_since_ms is None:
            self.scan_gaps += 1
            self.scan_gap_since_ms = now
            self.stop_request_distance = self.distance
        if now - self.scan_gap_since_ms >= MAX_WAIT_S * 1000:
            self.stop_reason = "scan recovery timed out"
        self.continuity.wait(now / 1000)
        self.control_frame_end = None
        self.demand = self.controller.update(Decision(), self.config, 0)
        if self.queue_command(0.0, self.held_steer, cause="terminal" if self.stop_reason else "observation"):
            feedback = self.motion_feedback()
            if feedback is not None:
                self.actuation.record(self.held_steer, self.demand, feedback, now / 1000)
        self.scan_deadline_ms = min(now + WATCHDOG_MS // 2, self.scan_gap_since_ms + MAX_WAIT_S * 1000)

    def walls_in_body_frame(self, walls=None):
        x, y = self.sensor
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)

        def local(point):
            dx, dy = point[0] - x, point[1] - y
            return dx * cosine + dy * sine, -dx * sine + dy * cosine

        return ((local(a), local(b)) for a, b in (self.walls if walls is None else walls))

    def body_contacts_wall(self):
        return any(segment_hits_body(a, b, self.collision_envelope) for a, b in self.walls_in_body_frame())

    def wall_clearance(self, walls=None):
        return min((segment_body_clearance(a, b, self.collision_envelope) for a, b in self.walls_in_body_frame(walls)),
                   default=math.inf)

    def record_motion_metrics(self, step_ms):
        cruise = self.straight_cruise
        if (cruise is not None and self.checkpoint_index == 0
                and cruise["start_x_m"] <= self.sensor[0] <= cruise["end_x_m"]):
            cruise["sampled_ms"] += step_ms
            cruise["peak_mps"] = max(cruise["peak_mps"] or 0, self.speed)
            cruise["peak_target_mps"] = max(cruise["peak_target_mps"] or 0, self.demand.speed)
            cruise["peak_pwm_pct"] = max(cruise["peak_pwm_pct"] or 0, self.applied_pwm)
            if self.speed >= cruise["threshold_mps"]:
                cruise["near_max_ms"] += step_ms
                self.near_max_run_ms += step_ms
                cruise["longest_near_max_ms"] = max(cruise["longest_near_max_ms"], self.near_max_run_ms)
            else:
                self.near_max_run_ms = 0
        else:
            self.near_max_run_ms = 0
        if self.speed >= 0.03:
            self.started_moving = True
            self.stopped_since_ms = None
        elif self.started_moving:
            if self.stopped_since_ms is None:
                self.stopped_since_ms = self.now_ms
            duration = self.now_ms - self.stopped_since_ms
            self.stop_episodes += duration >= 200 and duration - step_ms < 200
            self.longest_stop_ms = max(self.longest_stop_ms, duration)
        if self.clearance_pose is not None:
            pose = Pose(*self.sensor, self.yaw)
            distance = math.hypot(pose.x - self.clearance_pose.x, pose.y - self.clearance_pose.y)
            rotation = abs(math.remainder(pose.yaw - self.clearance_pose.yaw, math.tau))
            # A wall cannot get closer by more than any point of the body moved.
            movement = distance + self.body_radius * rotation
            if self.plant:
                current, prior = self.collision_envelope, self.clearance_envelope
                movement += math.hypot((current.width - prior.width) / 2,
                                       max(abs(current.front - prior.front), abs(current.rear - prior.rear)))
            lower = self.clearance_at_pose - movement
            upper = self.clearance_at_pose + movement
            if (lower < self.split_minimum_clearance or lower < self.wall_threshold_m <= upper):
                self.clearance_at_pose = self.wall_clearance()
                self.clearance_pose = pose
                self.clearance_envelope = self.collision_envelope
                upper = self.clearance_at_pose
                if self.clearance_at_pose < self.split_minimum_clearance:
                    self.split_minimum_clearance = self.clearance_at_pose
                    self.split_clearance_position = pose.x, pose.y
                if self.clearance_at_pose < self.minimum_clearance:
                    self.minimum_clearance = self.clearance_at_pose
                    self.minimum_clearance_position = pose.x, pose.y
            if upper < self.wall_threshold_m:
                self.split_below_wall_ms += step_ms
                self.below_wall_ms += step_ms

    def current_split(self, *, completed=False):
        return Split(self.completed_laps + 1, self.checkpoint_index + 1,
                     (self.now_ms - self.split_start_ms) / 1000,
                     self.distance - self.split_start_distance, self.split_peak_speed,
                     completed, self.split_minimum_clearance, self.split_clearance_position,
                     self.split_below_wall_ms / 1000)

    def update_lap(self, previous_sensor):
        self.split_peak_speed = max(self.split_peak_speed, self.speed)
        checkpoints = self.scenario.checkpoints
        checkpoint = checkpoints[self.checkpoint_index]
        if not checkpoint.crossed(previous_sensor, self.sensor):
            return
        self.last_split = self.current_split(completed=True)
        self.splits[self.checkpoint_index] = self.last_split
        if self.clearance_pose is not None:
            self.split_history.append(self.last_split)
            self.clearance_pose = Pose(*self.sensor, self.yaw)
            self.clearance_at_pose = self.wall_clearance()
            self.clearance_envelope = self.collision_envelope
            self.split_minimum_clearance = self.clearance_at_pose
            self.split_clearance_position = self.sensor
            self.split_below_wall_ms = 0
        self.split_start_ms = self.now_ms
        self.split_start_distance = self.distance
        self.split_peak_speed = self.speed
        self.checkpoint_index = (self.checkpoint_index + 1) % len(checkpoints)
        if self.checkpoint_index == 0:
            self.completed_laps += 1
            self.last_lap_s = (self.now_ms - self.lap_start_ms) / 1000
            self.lap_start_ms = self.now_ms

    def integrate(self, dt):
        if self.plant is not None:
            self.integrate_four_wheels(dt)
            return
        motion = self.motion
        next_steer = self.actual_steer
        if abs(self.target_steer - self.actual_steer) > motion.steer_deadband_deg:
            next_steer = approach(self.actual_steer, self.target_steer, motion.steering_rate * dt)
        target_speed = speed_for_pwm(self.applied_pwm, motion.pwm_limit,
                                     motion.max_speed * motion.throttle_gain, motion.deadband_pwm)
        rate = motion.acceleration if target_speed > self.speed else motion.deceleration
        delta = target_speed - self.speed
        curvature = math.tan(math.radians(-(self.actual_steer + next_steer) / 2)) / self.config.wheelbase
        grip, combined = 1.0, 1.0
        if self.stress.combined_grip:
            grip = self.stress.grip_at(self.rear_x, self.rear_y)
            if grip < 1.0 and self.speed >= 0.03:
                self.low_grip_ms += 1000 * dt
            requested_rate = min(rate, abs(delta) / dt)
            combined = contact_ratio(requested_rate, self.speed ** 2 * abs(curvature),
                                     rate, motion.lateral_accel, grip)
            self.peak_contact_ratio = max(self.peak_contact_ratio, combined)
            if combined > 1.05:
                self.traction_limited_ms += 1000 * dt
            rate /= max(1.0, combined)
        change_time = min(dt, abs(delta) / rate)
        next_speed = target_speed if change_time < dt else self.speed + math.copysign(rate * dt, delta)
        travel = (self.speed + next_speed) * change_time / 2 + next_speed * (dt - change_time)
        mean_speed = travel / dt
        lateral_ratio = mean_speed ** 2 * abs(curvature) / (grip * motion.lateral_accel)
        self.peak_lateral_ratio = max(self.peak_lateral_ratio, lateral_ratio)
        if lateral_ratio > 1.0:
            self.turn_limited = True
        curvature /= max(1.0, combined, lateral_ratio)
        turn = travel * curvature
        self.yaw_rate = turn / dt
        odometry_turn = turn * self.lidar.odometry_yaw_scale
        odometry_travel = travel * self.lidar.odometry_distance_scale
        heading = self.odometry.yaw + odometry_turn / 2
        self.odometry = Pose(self.odometry.x + odometry_travel * math.cos(heading),
                             self.odometry.y + odometry_travel * math.sin(heading),
                             self.odometry.yaw + odometry_turn)
        self.rear_x += travel * math.cos(self.yaw + turn / 2)
        self.rear_y += travel * math.sin(self.yaw + turn / 2)
        self.yaw = (self.yaw + turn) % math.tau
        self.distance += travel
        self.speed = max(0.0, next_speed)
        self.actual_steer = next_steer
        self.peak_speed = max(self.peak_speed, self.speed)
        self.peak_steer = max(self.peak_steer, abs(self.actual_steer))
        self.collision = self.body_contacts_wall()

    def integrate_four_wheels(self, dt):
        plant = self.plant
        if plant.elapsed_s == 0.0 and self.speed:
            # Permit simulations to start from a specified moving state.
            plant.ground_speeds = (self.speed, self.speed)
            plant.front_angles = wheel_targets(self.actual_steer, self.config.wheelbase, plant.pivot_track)
        travel, turn, curvature, lateral, demand, grip = plant.advance(
            dt, self.applied_pwm, self.command_steer, self.target_steer, self.motion, self.stress,
            self.rear_x, self.rear_y, self.yaw, speed_target=self.applied_speed_target)
        self.speed = sum(plant.ground_speeds) / 2
        self.actual_steer = math.degrees(math.atan(self.config.wheelbase * curvature))
        self.yaw_rate = turn / dt
        heading = self.yaw + turn / 2
        self.rear_x += travel * math.cos(heading)
        self.rear_y += travel * math.sin(heading)
        self.yaw = (self.yaw + turn) % math.tau
        self.distance += travel
        self.odometry = plant.measured_pose
        self.peak_lateral_ratio = max(self.peak_lateral_ratio, lateral)
        self.peak_contact_ratio = max(self.peak_contact_ratio, demand)
        self.turn_limited |= lateral > 1
        self.traction_limited_ms += 1000 * dt if demand > 1.05 else 0
        self.low_grip_ms += 1000 * dt if self.speed >= 0.03 and grip < 1.0 else 0
        self.peak_speed = max(self.peak_speed, self.speed)
        self.peak_steer = max(self.peak_steer, abs(self.actual_steer))
        self.collision_envelope = plant.envelope()
        self.body_radius = max(self.body_radius, math.hypot(self.collision_envelope.width / 2,
                               max(self.collision_envelope.front, self.collision_envelope.rear)))
        self.collision = self.body_contacts_wall()

        if plant.drive_fault is not None and self.stop_reason is None:
            self.stop_reason = plant.drive_fault
            self.stop_request_distance = self.distance
            self.apply_pwm(0.0, "terminal")
            self.command_armed = False
            self.commands.clear()

    def step(self, target_laps=None, end_ms=None):
        end = self.now_ms + self.period_ms
        if end_ms is not None:
            end = min(end, end_ms)
        while self.now_ms < end and self.can_advance:
            following = min(end, self.now_ms + PHYSICS_MS, self.next_scan_ms)
            if self.lidar.rolling:
                following = min(following, self.next_pose_ms)
            if self.scans:
                following = min(following, self.scans[0][0])
            if self.commands:
                following = min(following, self.commands[0][0])
            watchdog_active = (self.command_armed if self.vehicle_model == "four-wheel" or self.coast_on_scan_gap
                               else self.applied_pwm > 0)
            if watchdog_active:
                following = min(following, self.last_command_ms + self.command_lease_ms)
            before = self.now_ms
            # Integrate the actual off interval, excluding startup and settled standstill.
            if self.power_cut_cause is not None and self.speed >= 0.03:
                self.power_cut_moving_ms[self.power_cut_cause] += following - before
            previous_pose = Pose(*self.sensor, self.yaw)
            self.integrate((following - before) / 1000)
            self.now_ms = following
            if self.lidar.rolling and following == self.next_pose_ms:
                if self.plant is None or self.feedback_enabled and abs(self.plant.last_sample_s * 1000 - following) < 1e-6:
                    self.pose_history.append((following, self.sensor_odometry))
                self.next_pose_ms += self.lidar.pose_sample_ms
            self.record_motion_metrics(following - before)
            if self.scenario.checkpoints and not self.collision:
                self.update_lap((previous_pose.x, previous_pose.y))
                if self.completed_laps == target_laps:
                    break
            if self.lidar.rolling:
                self.acquire_beams(before, previous_pose)
            if not self.collision:
                self.process_events()
            if self.clearance_pose is not None and self.stop_context is None and (self.stop_reason or self.collision):
                self.stop_context = dict(time_s=self.now_ms / 1000, position_m=self.sensor, speed_mps=self.speed,
                                         actual_steer_deg=self.actual_steer, confirmed_path_m=self.decision.clear_m,
                                         reason=self.stop_reason or "collision")
        if self.sensor != self.trail[-1][0]:
            self.trail.append((self.sensor, self.speed))
        return self.can_advance


STOP_LABELS = {
    "ambiguous turn": "좌우 경로 구분 불가",
    "incomplete scan": "스캔 불완전",
    "insufficient forward returns": "전방 스캔 부족",
    "insufficient clear distance": "전진 여유 부족",
    "insufficient wall support": "양쪽 벽 확인 부족",
    "command watchdog": "명령 만료",
    "measured overspeed": "실측 속도 상한 초과",
    "no motion under drive command": "구동 명령 후 이동 없음",
    "measured speed exceeds stopping space": "실측 속도에서 정지 여유 부족",
    "steering command mismatch": "조향 오차 지속",
    "coasting response too weak": "관성 감속 부족",
    "unexpected lateral motion": "측면 이동 이상",
    "yaw response mismatch": "회전 반응 불일치",
    "held steering is unsafe": "조향이 늦어질 때 전진 여유 부족",
    ISOLATED_NEAR_RETURN: "가까운 고립 반사점 재확인",
    "obstacle on last confirmed path": "직전 경로에 장애물 확인",
    "scan recovery timed out": "스캔 복구 대기 만료",
}
PHASE_LABELS = {"accelerate": "가속 목표", "decelerate": "감속 목표", "hold": "속도 유지",
                "coast": "모터 출력 차단 · 관성 감속", "stop": "출력 정지"}


class SimulatorWindow:
    def __init__(self, root, scenario, motion, calibrated, sensor, deskew, calibrate_lidar,
                 feedback=False, stress=StressSettings(), audit_leases=False, coast_on_scan_gap=False,
                 planner="route", vehicle_model="four-wheel", environment="nominal"):
        import tkinter as tk
        from tkinter import ttk

        self.root = root
        self.timer = None
        self.motion = motion
        self.calibrated = calibrated
        self.vehicle_model = vehicle_model
        self.stress = stress
        self.audit_leases = audit_leases
        self.coast_on_scan_gap = coast_on_scan_gap
        self.sensor_profile, self.sensor = sensor
        root.title("라이다 자율주행 시뮬레이터")
        root.geometry("1060x740")
        root.minsize(800, 650)
        toolbar = ttk.Frame(root, padding=10)
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="시나리오").pack(side="left")
        self.selection = ttk.Combobox(toolbar, state="readonly", width=20,
                                      values=[item.label for item in SCENARIOS.values()])
        self.selection.set(SCENARIOS[scenario].label)
        self.selection.bind("<<ComboboxSelected>>", self.reset)
        self.selection.pack(side="left", padx=(6, 12))
        self.start_button = ttk.Button(toolbar, text="시작", command=self.toggle)
        self.start_button.pack(side="left")
        self.step_button = ttk.Button(toolbar, text="한 스캔", command=self.step)
        self.step_button.pack(side="left", padx=5)
        ttk.Button(toolbar, text="초기화", command=self.reset).pack(side="left")
        self.drop_front = tk.BooleanVar()
        ttk.Checkbutton(toolbar, text="전방 스캔 끊기", variable=self.drop_front,
                        command=self.change_scan).pack(side="left", padx=10)
        ttk.Label(toolbar, text="재생").pack(side="left", padx=(0, 4))
        self.playback = ttk.Combobox(toolbar, state="readonly", width=4, values=("1×", "2×", "4×", "최대"))
        self.playback.set("1×")
        self.playback.pack(side="left")
        ttk.Label(toolbar, text="환경").pack(side="left", padx=(10, 3))
        self.environment_select = ttk.Combobox(toolbar, state="readonly", width=13, values=ENVIRONMENTS)
        self.environment_select.set(environment)
        self.environment_select.bind("<<ComboboxSelected>>", self.reset)
        self.environment_select.pack(side="left")
        settings = ttk.LabelFrame(root, text="차량 동역학과 지연", padding=(10, 3, 10, 8))
        settings.pack(fill="x", padx=10, pady=(0, 8))
        self.entries = {}
        for index, (name, (label, _, _, _)) in enumerate(MOTION_FIELDS.items()):
            field = ttk.Frame(settings)
            field.grid(row=index // 5, column=index % 5, sticky="w", padx=(0, 11), pady=2)
            ttk.Label(field, text=label).pack(side="left", padx=(0, 3))
            value = tk.StringVar(value=f"{getattr(motion, name):g}")
            self.entries[name] = value
            ttk.Entry(field, textvariable=value, width=5).pack(side="left")
        settings_actions = ttk.Frame(settings)
        settings_actions.grid(row=math.ceil(len(MOTION_FIELDS) / 5), column=0, columnspan=5,
                              sticky="ew", pady=(5, 0))
        self.preset = ttk.Combobox(settings_actions, state="readonly", width=12,
                                   values=[item[0] for item in MOTION_PRESETS.values()])
        self.preset.set("프리셋 선택")
        self.preset.bind("<<ComboboxSelected>>", self.load_preset)
        self.preset.pack(side="left")
        self.calibrated_box = tk.BooleanVar(value=calibrated)
        ttk.Checkbutton(settings_actions, text="입력값을 주행 계획에도 반영", variable=self.calibrated_box
                        ).pack(side="left", padx=8)
        self.feedback_box = tk.BooleanVar(value=feedback)
        ttk.Checkbutton(settings_actions, text="주행 피드백 사용", variable=self.feedback_box
                        ).pack(side="left", padx=(0, 8))
        ttk.Button(settings_actions, text="적용 후 초기화", command=self.apply_settings).pack(side="left")
        self.planner_select = ttk.Combobox(settings_actions, state="readonly", width=13,
                                           values=tuple(PLANNER_LABELS.values()))
        self.planner_select.set(PLANNER_LABELS[planner])
        self.planner_select.bind("<<ComboboxSelected>>", self.reset)
        self.planner_select.pack(side="left", padx=8)
        self.validation = tk.StringVar()
        ttk.Label(settings_actions, textvariable=self.validation, foreground="#a83232").pack(side="left", padx=8)
        sensor_row = ttk.LabelFrame(root, text="라이다 측정과 보정", padding=(10, 3, 10, 8))
        sensor_row.pack(fill="x", padx=10, pady=(0, 8))
        self.sensor_select = ttk.Combobox(sensor_row, state="readonly", width=17,
                                         values=[label for label, _ in SENSOR_PROFILES.values()])
        self.sensor_select.set(SENSOR_PROFILES[self.sensor_profile][0])
        self.sensor_select.bind("<<ComboboxSelected>>", self.load_sensor)
        self.sensor_select.pack(side="left")
        self.period_entry = tk.StringVar(value=str(self.sensor.period_ms))
        self.rate_entry = tk.StringVar(value=str(self.sensor.point_rate_hz))
        self.range_entry = tk.StringVar(value=f"{self.sensor.max_range_m:g}")
        for label, value in (("회전 (ms)", self.period_entry), ("측정점/초", self.rate_entry),
                             ("최대 (m)", self.range_entry)):
            ttk.Label(sensor_row, text=label).pack(side="left", padx=(8, 3))
            ttk.Entry(sensor_row, textvariable=value, width=5).pack(side="left")
        self.deskew_box = tk.BooleanVar(value=deskew)
        self.static_box = tk.BooleanVar(value=calibrate_lidar)
        ttk.Checkbutton(sensor_row, text="움직임 보정", variable=self.deskew_box).pack(side="left", padx=6)
        ttk.Checkbutton(sensor_row, text="거리·각도 보정", variable=self.static_box).pack(side="left")
        ttk.Button(sensor_row, text="적용 후 초기화", command=self.apply_sensor).pack(side="left", padx=6)
        map_area = ttk.Frame(root, padding=(10, 0, 10, 0))
        map_area.pack(fill="both", expand=True)
        self.split_panel = ttk.LabelFrame(map_area, text="구간별 기록", padding=8)
        self.split_rows = []
        for _ in LOOP_CHECKPOINTS:
            name = ttk.Label(self.split_panel, font=("맑은 고딕", 10, "bold"))
            name.pack(anchor="w")
            stats = ttk.Label(self.split_panel)
            stats.pack(anchor="w", pady=(0, 5))
            self.split_rows.append((name, stats))
        self.canvas = tk.Canvas(map_area, background="#112126", highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _: self.draw())
        self.readout = tk.StringVar()
        ttk.Label(root, textvariable=self.readout, padding=(10, 8)).pack(fill="x")
        ttk.Label(root, text="회색 · 벽     파란색 · 라이다     이동 경로   주황 · 저속   연두 · 중속   민트 · 고속",
                  padding=(10, 0, 10, 8)).pack(fill="x")
        self.reset()

    def stop(self):
        if self.timer is not None:
            self.root.after_cancel(self.timer)
            self.timer = None
        self.start_button.configure(text="시작")
        self.step_button.configure(state="normal")

    def reset(self, event=None):
        self.stop()
        self.drop_front.set(False)
        scenario = next(item for item in SCENARIOS.values() if item.label == self.selection.get())
        planner = next(key for key, label in PLANNER_LABELS.items() if label == self.planner_select.get())
        self.sim = Simulation(scenario, self.motion, self.calibrated, sensor=self.sensor,
                              deskew=self.deskew_box.get(), calibrate_lidar=self.static_box.get(),
                              feedback=self.feedback_box.get(), stress=self.stress, audit_leases=self.audit_leases,
                              coast_on_scan_gap=self.coast_on_scan_gap, planner=planner,
                              vehicle_model=self.vehicle_model, environment=self.environment_select.get())
        if scenario.checkpoints:
            self.split_panel.pack(side="right", fill="y", padx=(8, 0))
        else:
            self.split_panel.pack_forget()
        self.draw()

    def load_sensor(self, event=None):
        self.sensor_profile = next(key for key, (label, _) in SENSOR_PROFILES.items()
                                   if label == self.sensor_select.get())
        sensor = SENSOR_PROFILES[self.sensor_profile][1]
        self.period_entry.set(str(sensor.period_ms))
        self.rate_entry.set(str(sensor.point_rate_hz))
        self.range_entry.set(f"{sensor.max_range_m:g}")
        self.apply_sensor()

    def apply_sensor(self):
        try:
            period, rate = int(self.period_entry.get()), int(self.rate_entry.get())
            max_range = float(self.range_entry.get())
        except ValueError:
            self.validation.set("회전, 측정점/초와 최대 거리에 숫자를 입력하세요.")
            return
        try:
            base = replace(SENSOR_PROFILES[self.sensor_profile][1], ray_clock=self.sensor.ray_clock,
                           guard_spikes=self.sensor.guard_spikes, seed=self.sensor.seed)
            self.sensor = sensor_settings(base, period, rate, max_range)
        except ValueError as error:
            self.validation.set(str(error))
            return
        self.validation.set("")
        self.reset()

    def load_preset(self, event=None):
        motion = next(value for label, value in MOTION_PRESETS.values() if label == self.preset.get())
        for name, value in self.entries.items():
            value.set(f"{getattr(motion, name):g}")
        self.apply_settings()

    def apply_settings(self):
        values = {}
        for name, (label, number, _, _) in MOTION_FIELDS.items():
            try:
                values[name] = number(self.entries[name].get())
            except ValueError:
                self.validation.set(f"{label}에 숫자를 입력하세요.")
                return
        try:
            self.motion = motion_settings(MotionSettings(**values))
        except ValueError as error:
            self.validation.set(str(error))
            return
        self.calibrated = self.calibrated_box.get()
        self.validation.set("")
        self.reset()

    def change_scan(self):
        self.sim.drop_front = self.drop_front.get()
        self.draw()

    def step(self):
        self.sim.step()
        self.draw()

    def toggle(self):
        if self.timer is not None:
            self.stop()
            self.draw()
        elif self.sim.can_advance:
            self.start_button.configure(text="일시정지")
            self.step_button.configure(state="disabled")
            self.tick()

    def tick(self):
        self.timer = None
        if self.sim.step():
            playback = self.playback.get()
            delay = 1 if playback == "최대" else max(1, self.sim.period_ms // int(playback[0]))
            self.timer = self.root.after(delay, self.tick)
        else:
            self.stop()
        self.draw()

    def draw(self):
        sim, canvas = self.sim, self.canvas
        canvas.delete("all")
        xs, ys = zip(*(point for outline in sim.scenario.outlines for point in outline))
        canvas_width, canvas_height = canvas.winfo_width(), canvas.winfo_height()
        scale = max(1, min((canvas_width - 56) / (max(xs) - min(xs)),
                           (canvas_height - 56) / (max(ys) - min(ys))))
        center_x, center_y = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2

        def screen(point):
            return (canvas_width / 2 + (point[0] - center_x) * scale,
                    canvas_height / 2 + (point[1] - center_y) * scale)

        def coordinates(points):
            return [value for point in points for value in screen(point)]

        for index, outline in enumerate(sim.scenario.outlines):
            canvas.create_polygon(coordinates(outline), fill="#1c343a" if index == 0 else "#112126")
        for zone in (*sim.stress.grip_zones, *sim.environment.grip_zones):
            canvas.create_rectangle(*screen((zone.x_min, zone.y_min)), *screen((zone.x_max, zone.y_max)),
                                    fill="#e19b52", stipple="gray25", outline="#9d733e", dash=(4, 3))
        x, y = sim.sensor
        scan_x, scan_y, scan_yaw = sim.scan_pose
        if sim.scan_geometry is not None:
            pose = Pose(scan_x, scan_y, scan_yaw)
            for ax, ay, bx, by in sim.scan_geometry.ray_segments[::12]:
                canvas.create_line(*screen(pose.world(ax, ay)), *screen(pose.world(bx, by)),
                                   fill="#35636b")
        else:
            for angle, distance_mm, quality in sim.samples[::12]:
                if quality:
                    bearing = math.radians(angle - sim.config.forward_angle) + scan_yaw
                    distance = distance_mm / 1000
                    hit = scan_x + distance * math.cos(bearing), scan_y + distance * math.sin(bearing)
                    canvas.create_line(*screen((scan_x, scan_y)), *screen(hit), fill="#35636b")
        for start, end in sim.walls:
            canvas.create_line(*screen(start), *screen(end), fill="#9fb8bf", width=3)
        if len(sim.corridor.route_points) >= 2:
            scan_origin = Pose(scan_x, scan_y, scan_yaw)
            line = [scan_origin.world(*point) for point in sim.corridor.route_points]
            canvas.create_line(coordinates(line), fill="#6eb8ff", width=3, dash=(6, 2))
        for index, checkpoint in enumerate(sim.scenario.checkpoints):
            gx, gy = checkpoint.center
            dx, dy = checkpoint.direction
            gate_half_width = checkpoint.display_width
            highlighted = index == sim.checkpoint_index
            color = "#ffdb8b" if highlighted else "#759499"
            canvas.create_line(*screen((gx + gate_half_width * dy, gy - gate_half_width * dx)),
                               *screen((gx - gate_half_width * dy, gy + gate_half_width * dx)),
                               fill=color, dash=(4, 3), width=2)
            label = "결승선" if index == len(sim.scenario.checkpoints) - 1 else str(index + 1)
            canvas.create_text(*screen((gx - 0.52 * dy, gy + 0.52 * dx)),
                               text=label, fill=color, font=("맑은 고딕", 10, "bold"))
        if len(sim.trail) > 1:
            def color_for_speed(item):
                speed_ratio = item[1] / sim.motion.max_speed
                if speed_ratio < 0.55:
                    return "#efaa63"
                if speed_ratio < 0.85:
                    return "#d6d883"
                return "#66dec2"

            previous = sim.trail[0][0]
            for color, group in groupby(sim.trail, key=color_for_speed):
                path = [previous, *(point for point, _ in group)]
                canvas.create_line(coordinates(path), fill=color, width=2)
                previous = path[-1]
        cosine, sine = math.cos(sim.yaw), math.sin(sim.yaw)
        if sim.plant:
            half_width = sim.plant.plate_width / 2
            midpoint = sim.config.wheelbase / 2 - sim.config.sensor_ahead
            front = midpoint + sim.plant.plate_length / 2
            rear = sim.plant.plate_length / 2 - midpoint
        else:
            half_width = sim.config.width / 2
            front, rear = sim.config.front, sim.config.rear
        body = [(x + along * cosine - side * sine, y + along * sine + side * cosine)
                for along, side in ((front, -half_width), (front, half_width),
                                    (-rear, half_width), (-rear, -half_width))]
        canvas.create_polygon(coordinates(body), fill="#7ee8cd" if not sim.collision else "#f18a81")
        if sim.plant:
            rear_pose = Pose(sim.rear_x, sim.rear_y, sim.yaw)
            for tire in sim.plant.wheel_footprints():
                corners = [rear_pose.world(along, side) for along, side in tire]
                canvas.create_polygon(coordinates(corners), fill="#273339", outline="#c2cfd3")
        canvas.create_line(*screen((x, y)), *screen((x + (front + 0.08) * cosine,
                                                     y + (front + 0.08) * sine)),
                           arrow="last", fill="#f2fbfa", width=2)
        cx, cy = screen((x, y))
        canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#112126", outline="")
        result = sim.decision
        if sim.collision:
            state = "충돌 · 시뮬레이션 종료"
        elif sim.stop_reason:
            label = STOP_LABELS.get(sim.stop_reason, sim.stop_reason)
            state = f"{'정지 중' if sim.can_advance else '정지'} · {label}"
        elif sim.valid_scans < 3:
            state = "초기 스캔 대기"
        elif sim.continuity.waiting_at is not None:
            state = "출력 차단 · 새 스캔 대기"
        elif result.reason == "bridging scan gap":
            state = "스캔 누락 · 확인된 통로에서 감속"
        elif result.speed_scale < 1.0:
            state = "벽 확인 부족 · 제한 속도"
        else:
            state = PHASE_LABELS[sim.demand.phase] if self.timer is not None else "대기"
        overrun = (f"   |   정지 판정 후 {sim.distance - sim.stop_request_distance:.2f} m"
                   if sim.stop_request_distance is not None else "")
        scan_info = ""
        if sim.scan_errors is not None:
            raw, processed = sim.scan_errors
            scan_info = (f"\n회전 {sim.period_ms} ms   |   유효점 {len(sim.scan_geometry.points)}/{len(sim.samples)}"
                         f"   |   가장 오래된 측정 {sim.scan_age_ms:.0f} ms"
                         f"   |   위치 오차 RMS {raw:.0f} → {processed:.0f} mm")
            if sim.lidar.spike_rate:
                scan_info += (f"\n멀게 튄 점 제외 {sim.rejected_far}개"
                              f"   |   가까운 고립 반사점 재확인 {sim.near_return_holds}회")
        lap_info = ""
        if sim.scenario.checkpoints:
            checkpoint = sim.scenario.checkpoints[sim.checkpoint_index]
            lap_info = (f"\n완주 {sim.completed_laps}바퀴   |   구간 {sim.checkpoint_index + 1}/"
                        f"{len(sim.scenario.checkpoints)} {checkpoint.label}   |   누적 {sim.distance:.1f} m")
            if sim.last_lap_s is not None:
                lap_info += f"   |   직전 랩 {sim.last_lap_s:.1f} s"
            for index, (checkpoint, (name, stats)) in enumerate(zip(sim.scenario.checkpoints,
                                                                   self.split_rows)):
                current = index == sim.checkpoint_index
                name.configure(text=f"{index + 1}. {checkpoint.label}",
                               foreground="#187267" if current else "")
                split = sim.splits.get(index)
                stats.configure(text=(f"{split.seconds:.1f} s · 평균 {split.mean_speed:.2f} m/s"
                                      if split else checkpoint.detail))
        if sim.stress.faults or sim.stress.combined_grip or sim.audit_leases:
            lap_info += (f"\n누락 스캔 {sim.dropped_scans}  ·  지연 스캔 {sim.stale_scans}"
                         f"  ·  누락 명령 {sim.dropped_commands}  ·  명령 여유 부족 {sim.short_leases}")
        if sim.coast_on_scan_gap:
            lap_info += f"\n스캔 도착 중단 {sim.scan_gaps}회  ·  주행 복귀 {sim.recovered_scan_gaps}회"
        if sim.plant:
            left, right = sim.plant.front_angles
            lc, rc = sim.plant.encoder_counts
            lap_info += f"\n앞바퀴 좌/우 {left:+.1f}° / {right:+.1f}°   ·   후륜 엔코더 {lc} / {rc} 틱"
        self.readout.set(f"{state}   |   모델 속도 {sim.speed:.2f} m/s   |   목표 {sim.demand.speed:.2f} m/s"
                         f"   |   경과 {sim.now_ms / 1000:.1f} s{overrun}\n"
                         f"모델 조향 {sim.actual_steer:+.1f}°   |   명령 {result.steer:+.1f}°"
                         f"   |   적용 PWM {sim.applied_pwm:.1f}%   |   전진 여유 {result.clear_m:.2f} m"
                         + ("   |   횡가속 한계 초과 이력" if sim.turn_limited else "") + scan_info + lap_info)


def sensor_settings(profile, period_ms, point_rate_hz, max_range_m):
    if not 70 <= period_ms <= 380 or not 800 <= point_rate_hz <= 5000:
        raise ValueError("회전은 70~380 ms, 측정점/초는 800~5000 범위에서 입력하세요.")
    if not 1 <= max_range_m <= 12:
        raise ValueError("최대 측정 거리는 1~12 m 범위에서 입력하세요.")
    return replace(profile, period_ms=period_ms, point_rate_hz=point_rate_hz, max_range_m=max_range_m)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, default="loop")
    parser.add_argument("--headless", action="store_true", help="Run without a window")
    parser.add_argument("--steps", type=int, help="Maximum scan steps in headless mode")
    parser.add_argument("--laps", type=int, help="Required laps for a circuit in headless mode")
    parser.add_argument("--drop-front-at", type=int, help="Remove front returns at this headless step")
    parser.add_argument("--drop-front-steps", type=int, help="Restore front returns after this many steps")
    parser.add_argument("--profile", choices=MOTION_PRESETS, default="baseline")
    parser.add_argument("--lidar", choices=SENSOR_PROFILES, default="rolling")
    parser.add_argument("--scan-ms", type=int, help="Lidar rotation period in milliseconds")
    parser.add_argument("--point-rate-hz", type=int, help="Lidar measurements per second")
    parser.add_argument("--max-range-m", type=float, help="Simulated usable range in meters")
    parser.add_argument("--seed", type=int, help="Repeatable seed for range noise and dropouts")
    parser.add_argument("--no-deskew", action="store_true", help="Treat every return as simultaneous")
    parser.add_argument("--ray-clock", choices=RAY_CLOCKS, help="Select the estimated per-ray clock or simulation truth")
    parser.add_argument("--no-spike-guard", action="store_true", help="Disable isolated-return handling for comparison")
    parser.add_argument("--no-lidar-calibration", action="store_true", help="Keep range and yaw bias")
    parser.add_argument("--uncalibrated", action="store_true", help="Use nominal assumptions in the planner")
    parser.add_argument("--feedback", action=argparse.BooleanOptionalAction, default=True,
                        help="Use rear encoder feedback in the four-wheel model")
    parser.add_argument("--vehicle-model", choices=("four-wheel", "legacy"), default="four-wheel")
    parser.add_argument("--environment", choices=ENVIRONMENTS, default="nominal")
    parser.add_argument("--stress", choices=STRESS_PROFILES, default="clean",
                        help="Inject a reproducible sensor, transport or grip condition")
    parser.add_argument("--audit-leases", action="store_true",
                        help="Report commands that would have too little lease time on the Pi")
    parser.add_argument("--coast-on-scan-gap", action=argparse.BooleanOptionalAction, default=True,
                        help="Cut power on overdue scans and recover after two usable plans")
    parser.add_argument("--planner", choices=PLANNERS, default="route")
    for name, (label, number, _, _) in MOTION_FIELDS.items():
        parser.add_argument("--" + name.replace("_", "-"), type=number, help=label.replace("%", "%%"))
    args = parser.parse_args()
    scenario = SCENARIOS[args.scenario]
    if args.laps is not None and (not args.headless or not scenario.checkpoints or args.laps < 1):
        parser.error("--laps requires --headless, a circuit with checkpoints and a positive count")
    overrides = {name: getattr(args, name) for name in MOTION_FIELDS if getattr(args, name) is not None}
    try:
        motion = motion_settings(replace(MOTION_PRESETS[args.profile][1], **overrides))
        sensor = SENSOR_PROFILES[args.lidar][1]
        sensor = sensor_settings(sensor, sensor.period_ms if args.scan_ms is None else args.scan_ms,
                                 sensor.point_rate_hz if args.point_rate_hz is None else args.point_rate_hz,
                                 sensor.max_range_m if args.max_range_m is None else args.max_range_m)
        if args.seed is not None:
            sensor = replace(sensor, seed=args.seed)
        if args.ray_clock is not None:
            sensor = replace(sensor, ray_clock=args.ray_clock)
        if args.no_spike_guard:
            sensor = replace(sensor, guard_spikes=False)
    except ValueError as error:
        parser.error(str(error))
    lap_goal = args.laps or int(bool(scenario.checkpoints) and args.steps is None)
    default_steps = math.ceil(300_000 * lap_goal / sensor.period_ms) if lap_goal else 220
    steps = default_steps if args.steps is None else args.steps
    if steps < 1 or (args.drop_front_at is not None
                     and (not args.headless or not 0 <= args.drop_front_at < steps)):
        parser.error("--steps must be positive and --drop-front-at must be within a headless run")
    if args.drop_front_steps is not None and (args.drop_front_at is None or args.drop_front_steps < 1):
        parser.error("--drop-front-steps needs --drop-front-at and a positive duration")
    if args.headless:
        sim = Simulation(scenario, motion, not args.uncalibrated, sensor=sensor,
                         deskew=not args.no_deskew, calibrate_lidar=not args.no_lidar_calibration,
                         feedback=args.feedback, stress=STRESS_PROFILES[args.stress],
                         audit_leases=args.audit_leases, coast_on_scan_gap=args.coast_on_scan_gap,
                         planner=args.planner, vehicle_model=args.vehicle_model, environment=args.environment)
        for index in range(steps):
            if index == args.drop_front_at:
                sim.drop_front = True
            if args.drop_front_steps is not None and index == args.drop_front_at + args.drop_front_steps:
                sim.drop_front = False
            previous_split = sim.last_split
            alive = sim.step()
            if sim.last_split is not previous_split:
                split = sim.last_split
                print(f"lap={split.lap} checkpoint={split.checkpoint}/{len(scenario.checkpoints)} "
                      f"time={split.seconds:.2f}s distance={split.distance:.2f}m "
                      f"mean_speed={split.mean_speed:.2f}m/s peak_speed={split.peak_speed:.2f}m/s",
                      flush=True)
            if not alive or (lap_goal and sim.completed_laps >= lap_goal):
                break
        x, y = sim.sensor
        reason = "collision" if sim.collision else sim.stop_reason or sim.decision.reason
        if sim.can_advance and sim.completed_laps < lap_goal:
            reason = "step limit before lap goal"
        overrun = (sim.distance - sim.stop_request_distance if sim.stop_request_distance is not None
                   else None)
        scan_info = ""
        if sim.scan_errors is not None:
            raw, processed = sim.scan_errors
            scan_info = (f" scan_period={sim.period_ms}ms scan_age={sim.scan_age_ms:.0f}ms "
                         f"valid_points={len(sim.scan_geometry.points)}/{len(sim.samples)} "
                         f"raw_rms={raw:.1f}mm processed_rms={processed:.1f}mm")
        if sim.moving_return_count:
            raw, processed = (1000 * math.sqrt(error / sim.moving_return_count)
                              for error in sim.moving_error_sums)
            scan_info += f" moving_raw_rms={raw:.1f}mm moving_processed_rms={processed:.1f}mm"
        if sim.lidar.rolling:
            scan_info += f" ray_clock={sim.lidar.ray_clock} pose_period={sim.lidar.pose_sample_ms}ms"
            for name, measurement in sim.motion_scan_metrics().items():
                if not name.startswith("moving_"):
                    scan_info += f" {name}={measurement}"
        lap_info = ""
        if scenario.checkpoints:
            count = len(scenario.checkpoints)
            completed = sim.completed_laps * count + sim.checkpoint_index
            lap_info = f" laps={sim.completed_laps} checkpoints={completed}"
            if lap_goal:
                lap_info += f"/{count * lap_goal} lap_goal={lap_goal}"
        if sim.last_lap_s is not None:
            lap_info += f" last_lap={sim.last_lap_s:.2f}s"
        if args.stress != "clean" or args.audit_leases:
            scan_info += (f" dropped_scans={sim.dropped_scans} stale_scans={sim.stale_scans}"
                          f" dropped_commands={sim.dropped_commands} delayed_commands={sim.delayed_commands}"
                          f" watchdog={sim.watchdog_trips}"
                          f" traction_limited_ms={sim.traction_limited_ms:.0f} low_grip_ms={sim.low_grip_ms:.0f}")
            if sim.audit_leases:
                scan_info += f" short_leases={sim.short_leases}/{sim.lease_checks}"
        if sim.coast_on_scan_gap:
            scan_info += f" scan_gaps={sim.scan_gaps} recovered_scan_gaps={sim.recovered_scan_gaps}"
        print(f"{args.scenario} {args.vehicle_model} {args.environment}: time={sim.now_ms / 1000:.2f}s distance={sim.distance:.2f}m "
              f"x={x:.2f}m y={y:.2f}m max_steer={sim.peak_steer:.0f}deg "
              f"peak_speed={sim.peak_speed:.2f}m/s turn_limited={sim.turn_limited} "
              f"peak_lateral_ratio={sim.peak_lateral_ratio:.2f} "
              f"collision={sim.collision} limited_scans={sim.limited_scans} "
              f"corner_exit_scans={sim.corner_exit_scans} "
              f"feedback={sim.feedback_enabled} coast_commands={sim.coast_commands} "
              f"bridged_scans={sim.bridged_scans} waiting_scans={sim.waiting_scans} reason={reason}"
              + (f" stop_travel={overrun:.2f}m" if overrun is not None else "") + lap_info + scan_info)
        return int(sim.collision or sim.completed_laps < lap_goal)

    import tkinter as tk

    root = tk.Tk()
    SimulatorWindow(root, args.scenario, motion, not args.uncalibrated, (args.lidar, sensor),
                    not args.no_deskew, not args.no_lidar_calibration, args.feedback,
                    STRESS_PROFILES[args.stress], args.audit_leases, args.coast_on_scan_gap, args.planner,
                    args.vehicle_model, args.environment)
    root.mainloop()
    return 0
