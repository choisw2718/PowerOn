"""Preview wall following or explicitly drive a Nano from a directly attached RPLIDAR."""

import argparse
from contextlib import AbstractContextManager, ExitStack, closing, nullcontext
import csv
from dataclasses import asdict, dataclass, field, replace
from getpass import getpass
import math
import json
from pathlib import Path
import sys
import threading
import time
import tempfile

from hardware import PARAMETERS
from vehicle import drive_data
from lidar.acquisition.latest_scan import OVERDUE_SCAN, latest_scans
from lidar.acquisition.serial_scan import (DEFAULT_PI_PORT, UART_BYTE_S, LidarPort, ScanDecoder,
                                           ScanInterrupted, reconnecting_scans, scan_chunks)
from lidar.geometry.scan_geometry import (Pose, PoseLog, PoseUnavailable, pose_at,
                                          corrected_range, deskew_rotation, estimate_ray_times,
                                          guard_raw_returns, sample_clock_times)
from vehicle.wifi.client import DEFAULT_PORT, WifiController
from lidar.geometry.wheel_odometry import EncoderLog

from .motion.feedback import ActuationMonitor, measure_motion
from .motion.speed_control import SpeedCommand, SpeedConfig, SpeedController
from .planning.continuity import (MAX_LEASE_MS, MAX_PLAN_AGE, UNMEASURED_TARGET_MPS, CommandMotionBound, TrackContinuity,
                                  command_clearance, lease_budget_ms, lease_floor_ms, motion_uncertainty)
from .planning.wall_track import CorridorMemory, Decision, TrackConfig, plan


MAX_SCAN_SPAN = 0.38
MIN_SCAN_SPAN = 0.07
NANO_RESPONSE_S = 0.24
FIRST_RETRY_S = 0.25
POSE_WAIT_S = 0.045


@dataclass
class Rotation:
    start: float
    end: float
    samples: list[tuple[float, float, int]]
    sample_times: tuple[float, ...] | None = None
    timing_source: str = "angle-estimate"

    def calibrated_samples(self, offset_mm):
        if not offset_mm:
            return self.samples
        return [(angle, corrected_range(distance, quality, offset_mm), quality)
                for angle, distance, quality in self.samples]


@dataclass
class DriveProgress:
    lease_ends_at: float | None = None
    guard: AbstractContextManager = field(default_factory=nullcontext, repr=False, compare=False)


class AutonomyStopped(RuntimeError):
    """The planner must stop and needs a fresh start before moving again."""


def prepare_scan(frame, track, speed, age, offset, *, poses=None, stationary=False,
                 deadline=math.inf, speed_bound=None):
    if poses is not None:
        times, measured = poses.window(frame.start, frame.end,
                                       wait_until=min(frame.end + POSE_WAIT_S, deadline))
        geometry = deskew_rotation(frame.samples, frame.start, frame.end, times, measured,
                                   sensor_ahead=track.sensor_ahead, yaw_deg=-track.forward_angle,
                                   offset_mm=offset, sample_times=frame.sample_times)
        return geometry.samples, geometry, None, "deskew"

    samples = guard_raw_returns(frame.calibrated_samples(offset))
    if stationary:
        return samples, None, None, "stationary"
    if not math.isfinite(age) or not math.isfinite(frame.start) or not frame.end > frame.start:
        raise PoseUnavailable("invalid lidar scan time")
    return samples, None, motion_uncertainty(track, speed, age, speed_bound=speed_bound), "bounded"


def drive_lease_ms(route, frame, track, speed_config, now, *, feedback=None):
    # Deskewed geometry is expressed at the end-of-rotation pose.
    reference = frame.end if feedback is not None else frame.start
    return lease_budget_ms(route, reference, track, speed_config, now, feedback=feedback)


def ordered_sweep(samples):
    # The A1 emits placeholder bearings on no-return packets. Keep those packets
    # for the sample clock and visibility checks, but do not use them to infer
    # motor direction or reject an otherwise correctly ordered revolution.
    first = None
    peak = 0.0
    for angle, distance, quality in samples:
        if quality <= 0 or not math.isfinite(distance) or distance <= 0:
            continue
        if not math.isfinite(angle) or not 0 <= angle < 360:
            return False
        if first is None:
            first = angle
        phase = (angle - first) % 360
        if peak < 5 and phase > 355:
            phase = 0.0
        if phase < peak - 25:
            return False
        peak = max(phase, peak)
    return first is not None


def live_rotations(chunks, require_fresh, *, deadline=math.inf, stop=None, reader_stop=None,
                   serial_timing=True):
    def read():
        with closing(chunks) if hasattr(chunks, "close") else nullcontext(chunks):
            yield from decode_rotations(chunks, require_fresh, deadline=deadline, serial_timing=serial_timing)

    if reader_stop is not None:
        yield from latest_scans(read(), stop, reader_stop, deadline, gap_grace=0.04 if require_fresh else None)
    else:
        yield from read()


def decode_rotations(chunks, require_fresh, *, deadline, serial_timing):
    decoder = ScanDecoder()
    started = None
    skipped_at_start = 0
    next_rotation_deadline = time.monotonic() + 5.0
    samples = []
    accepted = False
    discarded = False
    for chunk in chunks:
        now = time.monotonic()
        if now >= deadline:
            return
        if now > next_rotation_deadline:
            raise ScanInterrupted("No complete, recent lidar rotation")
        for first, angle, distance, quality, trailing in decoder.feed(chunk, with_offsets=True):
            if first:
                marker_time = now - trailing * UART_BYTE_S if serial_timing else now
                completed = None
                synchronized = skipped_at_start == decoder.skipped_bytes
                if started is not None:
                    stamps = (sample_clock_times(len(samples), started, marker_time)
                              if synchronized and marker_time > started else None)
                    completed = Rotation(started, marker_time, samples, stamps,
                                         "sample-clock" if stamps is not None else "angle-estimate")
                started, samples = marker_time, []
                skipped_at_start = decoder.skipped_bytes
                next_rotation_deadline = now + MAX_SCAN_SPAN
                if completed is not None:
                    usable = (not require_fresh
                              or synchronized and MIN_SCAN_SPAN <= completed.end - completed.start <= MAX_SCAN_SPAN
                              and ordered_sweep(completed.samples))
                    drive_data.record_rotation(completed, usable)
                    if usable:
                        accepted, discarded = True, False
                        yield completed
                    elif not accepted or discarded:
                        raise ScanInterrupted("Delayed or buffered lidar rotation")
                    else:
                        # A single unusable rotation cannot renew the last command.
                        discarded = True
            if started is not None:
                samples.append((angle, distance, quality))
                if len(samples) > 3000:
                    raise ScanInterrupted("Too many points without a rotation marker")
    if time.monotonic() < deadline:
        raise ScanInterrupted("Lidar stream ended")


def recorded_rotations(file):
    started = None
    samples = []
    sample_times = []
    previous_sample = -math.inf
    rows = csv.DictReader(file)
    timed = "sample_timestamp_s" in (rows.fieldnames or ())
    for row in rows:
        if timed:
            stamp = float(row["sample_timestamp_s"])
            if not math.isfinite(stamp) or stamp <= previous_sample:
                raise ValueError("Scan sample_timestamp_s must increase strictly")
            previous_sample = stamp
        if row["start_of_rotation"] == "1":
            end = stamp if timed else float(row["host_receive_elapsed_s"])
            if started is not None:
                times = tuple(sample_times) if timed else sample_clock_times(len(samples), started, end)
                yield Rotation(started, end, samples, times, "recorded-time" if timed else "sample-clock")
            started, samples, sample_times = end, [], []
        if started is not None:
            samples.append((float(row["angle_deg"]), float(row["distance_mm"]), int(row["quality"])))
            if timed:
                sample_times.append(stamp)


class NanoDrive:
    def __init__(self, host, port):
        try:
            self.controller = WifiController(host, port, connect_timeout=2.0)
        except OSError as error:
            raise ScanInterrupted(f"Nano connection unavailable: {error}") from error
        self.pending = bytearray(self.controller.initial_data)

    def request(self, command, expected, *, prefix=False):
        try:
            self.controller.send(command)
        except OSError as error:
            raise ScanInterrupted(f"Nano command transport interrupted: {error}") from error
        deadline = time.monotonic() + NANO_RESPONSE_S
        while time.monotonic() < deadline:
            while b"\n" in self.pending:
                line, _, tail = self.pending.partition(b"\n")
                self.pending = tail
                reply = line.decode("ascii", errors="replace").strip()
                if reply == expected or (prefix and reply.startswith(expected)):
                    return reply
                if reply == "ERR LIDAR lease expired" and expected == "OK LIDAR DRIVE LEASE":
                    raise ScanInterrupted("Lidar command window expired before the fresh scan arrived")
                if reply.startswith("ERR"):
                    raise ConnectionError(reply)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self.controller.socket.settimeout(remaining)
            try:
                data = self.controller.socket.recv(512)
            except OSError as error:
                raise ScanInterrupted(f"Nano response interrupted: {error}") from error
            if not data:
                raise ScanInterrupted("Nano closed the connection")
            if len(self.pending) + len(data) > 4096:
                raise ConnectionError("Invalid Nano response")
            self.pending.extend(data)
        raise ScanInterrupted(f"No Nano acknowledgement for {expected}")

    def close(self):
        self.controller.socket.settimeout(0.2)
        self.controller.close_safely()


def renew_drive(nano, pwm, steer, lease_ms, progress):
    with progress.guard if progress is not None else nullcontext():
        if progress is not None and pwm > 0:
            # A lost ACK does not mean the Nano missed the command.
            possible_end = time.monotonic() + NANO_RESPONSE_S + lease_ms / 1000
            progress.lease_ends_at = max(progress.lease_ends_at or 0.0, possible_end)
        nano.request(f"LIDAR DRIVE {pwm:.2f} {steer:.2f} {lease_ms}", "OK LIDAR DRIVE LEASE")
        stamp = time.monotonic()
        if progress is not None:
            if pwm > 0:
                progress.lease_ends_at = stamp + lease_ms / 1000
            elif progress.lease_ends_at is not None:
                progress.lease_ends_at = min(progress.lease_ends_at, stamp)
        return stamp


def run(rotations, config, nano, seconds=0.0, speed_config=SpeedConfig(), *,
        progress=None, deadline=math.inf, range_offset_mm=0.0, scan_timing=False,
        poses=None, stationary_preview=False):
    if seconds:
        deadline = min(deadline, time.monotonic() + seconds)
    previous_steer = 0.0
    saw_scan = False
    saw_plan = False
    previous_frame = None
    driving = False
    armed = False
    held_steer = 0.0
    speed = SpeedController(speed_config)
    actuation = ActuationMonitor(speed_config)
    corridor = CorridorMemory()
    continuity = TrackContinuity()
    motion_bound = CommandMotionBound(speed_config)
    for frame in rotations:
        if time.monotonic() >= deadline:
            return
        if frame is OVERDUE_SCAN:
            continuity.start_scans = 0
            if nano is not None and armed:
                now = time.monotonic()
                continuity.wait(now, "scan delivery overdue")
                speed.update(Decision(), config, 0)
                previous_frame = None
                lease = max(1, int(min(MAX_LEASE_MS, (deadline - now) * 1000)))
                stamp = renew_drive(nano, 0.0, held_steer, lease, progress)
                motion_bound.applied(False, stamp)
                drive_data.event("scan_gap", action="coast", steer_deg=held_steer)
            continue
        saw_scan = True
        period = max(0.0, frame.end - frame.start)
        plan_age = period + (max(0.0, time.monotonic() - frame.end) if nano else 0.0)
        stationary = (nano is None and stationary_preview) or (nano is not None and not driving)
        pose_missing = False
        try:
            samples, geometry, uncertainty, motion_mode = prepare_scan(
                frame, config, speed_config, plan_age, range_offset_mm, poses=poses,
                stationary=stationary, deadline=deadline,
                speed_bound=motion_bound.maximum(frame.start, time.monotonic()) if nano is not None else None)
        except PoseUnavailable as error:
            if nano is not None and driving and poses is not None and frame.end > frame.start:
                samples, geometry, uncertainty, _ = prepare_scan(
                    frame, config, speed_config, plan_age, range_offset_mm)
                motion_mode, pose_missing = "unavailable", True
            elif nano is not None:
                raise AutonomyStopped(f"Stopped: motion unavailable: {error}") from error
            else:
                corridor = CorridorMemory()
                speed = SpeedController(speed_config)
                actuation = ActuationMonitor(speed_config)
                previous_steer, previous_frame = 0.0, None
                print(f"motion unavailable: {error} PWM=  0.0% motion=unavailable", flush=True)
                continue
        if nano is not None:
            plan_age = period + max(0.0, time.monotonic() - frame.end)
        feedback = (measure_motion(poses.times, poses.poses, frame.end)
                    if poses is not None and not pose_missing else None)
        speed_cap = UNMEASURED_TARGET_MPS if feedback is None and not stationary_preview else math.inf
        issue, steering = None, None
        if feedback is not None:
            issue, speed_cap = actuation.check(feedback, config.wheelbase, frame.end)
            steering = actuation.transition(feedback, config.wheelbase, speed.target)
        if pose_missing:
            result = Decision(reason="motion unavailable")
        else:
            reference = frame.end if feedback is not None else frame.start
            command_distance = (command_clearance(reference, config, speed_config,
                                                  time.monotonic() if nano is not None else frame.end,
                                                  lease_floor_ms(period), feedback)
                                if nano is not None or config.prefer_corner_clearance else 0.0)
            sensor_pose = None
            if config.follow_route and geometry is not None and poses is not None:
                rear_pose = pose_at(poses.times, poses.poses, frame.end)
                sensor_pose = Pose(*rear_pose.world(config.sensor_ahead, 0.0), rear_pose.yaw)
            result = plan(samples, config, previous_steer, memory=corridor, now=frame.end,
                          lookahead=speed.lookahead(plan_age, feedback=feedback), geometry=geometry,
                          motion=uncertainty, steering=steering, command_distance=command_distance,
                          sensor_pose=sensor_pose,
                          cruise_horizon=speed.cruise_horizon(plan_age, lease_floor_ms(period) / 1000))
        result = speed.checked_route(result, feedback, plan_age, issue, speed_cap)
        saw_plan = True
        lease_ms = 0
        bridge = False
        checked_at = time.monotonic()
        lease_floor = lease_floor_ms(period)
        if nano is not None:
            gate = continuity.gate(result, samples, config, speed_config, frame.start, frame.end,
                                   checked_at, driving=driving, geometry=geometry, motion=uncertainty,
                                   steering=steering, feedback=feedback, issue=issue, deadline=deadline)
            result, lease_ms, bridge = gate.route, gate.lease_ms, gate.bridged
            drive_data.event("drive_gate", scan_end=frame.end, action=gate.action, reason=result.reason,
                             lease_ms=lease_ms, bridged=bridge)
            if gate.action == "stale":
                raise TimeoutError(f"Stopped: {result.reason}")
            if gate.action == "stop":
                raise AutonomyStopped(f"Stopped: {result.reason}")
            if gate.action == "observe":
                continue
            if gate.action == "deadline":
                return
            if bridge:
                speed_cap = min(speed_cap, 0.15)
            if gate.action == "coast":
                until_deadline = (deadline - checked_at) * 1000
                speed.update(Decision(), config, 0)
                previous_frame = None
                if armed:
                    print(f"Waiting for fresh clearance: {result.reason} PWM=0%", flush=True)
                    stamp = renew_drive(nano, 0.0, held_steer, int(min(MAX_LEASE_MS, until_deadline)), progress)
                    motion_bound.applied(False, stamp)
                    if feedback is not None:
                        actuation.record(held_steer, SpeedCommand(0.0, 0.0, "coast"), feedback, stamp)
                continue

        if nano is None:
            previous_steer = result.steer
        interval = 0.15 if previous_frame is None else min(MAX_SCAN_SPAN, frame.end - previous_frame)
        previous_frame = frame.end
        demand = speed.update(result, config, interval, scan_age=plan_age, speed_cap=speed_cap,
                              feedback=feedback)
        if drive_data.recording():
            drive_data.event("plan", scan_end=frame.end, age_s=plan_age, motion_mode=motion_mode,
                             command=asdict(demand), decision=asdict(result), lease_ms=lease_ms,
                             feedback=asdict(feedback) if feedback else None)
        walls = (f" left={result.left_m:.2f}m right={result.right_m:.2f}m"
                 if result.left_m is not None and result.speed_scale == 1.0 else "")
        timing = ""
        if scan_timing and period and samples:
            times = frame.sample_times or estimate_ray_times(frame.samples, frame.start, frame.end)
            timing = (f" rx_rotation={period * 1000:.0f}ms points={len(samples)}"
                      f" rx_rate={len(samples) / period:.0f}/s"
                      f" ray_clock={frame.timing_source} ray_span={(times[-1] - times[0]) * 1000:.0f}ms")
            if geometry is not None:
                timing += f" far_rejected={geometry.rejected_far} near_suspect={len(geometry.wall_exclusions)}"
        if uncertainty is not None:
            timing += (f" origin_bound={uncertainty.translation_m * 1000:.0f}mm"
                       f" yaw_bound={math.degrees(uncertainty.heading_rad):.1f}deg")
        if feedback is not None:
            timing += f" measured={feedback.speed_mps:.2f}m/s yaw_rate={math.degrees(feedback.yaw_rate):+.1f}deg/s"
            if feedback.steer_deg is not None:
                timing += f" actual_steer={feedback.steer_deg:+.1f}deg"
        if issue:
            timing += f" response={issue}"
        print(f"{result.reason:28s} PWM={demand.pwm:5.1f}% steer={result.steer:+5.1f}deg"
              f" target={demand.speed:.2f}m/s {demand.phase}"
              f" clear={result.clear_m:.2f}m motion={motion_mode}{walls}{timing}", flush=True)
        if nano is None:
            continue
        if demand.speed <= 0:
            raise AutonomyStopped("Stopped: no forward speed target")
        if not armed:
            if time.monotonic() - frame.end > MAX_PLAN_AGE:
                raise TimeoutError("Stopped: lidar plan expired")
            nano.request("LIDAR START", "OK LIDAR START")
            armed = True
            if time.monotonic() - frame.end > MAX_PLAN_AGE:
                raise TimeoutError("Stopped: lidar plan expired during arming")
        now = time.monotonic()
        lease_ms = math.floor(lease_ms - max(0.0, now - checked_at) * 1000)
        if lease_ms < lease_floor or (not bridge and now - frame.end > MAX_PLAN_AGE):
            continuity.wait(now)
            speed.update(Decision(), config, 0)
            previous_frame = None
            if deadline > now:
                stamp = renew_drive(nano, 0.0, held_steer,
                                    int(min(MAX_LEASE_MS, max(1, (deadline - now) * 1000))), progress)
                motion_bound.applied(False, stamp)
                if feedback is not None:
                    actuation.record(held_steer, SpeedCommand(0.0, 0.0, "coast"), feedback, stamp)
            continue
        stamp = renew_drive(nano, demand.pwm, result.steer, lease_ms, progress)
        motion_bound.applied(demand.pwm > 0, now if demand.pwm > 0 else stamp)
        driving = True
        previous_steer = held_steer = result.steer
        if not bridge:
            reference = frame.end if feedback is not None else frame.start
            continuity.remember(result, reference, frame.end)
        if feedback is not None:
            actuation.record(result.steer, demand, feedback, stamp)
    if time.monotonic() < deadline:
        if nano is not None and not saw_plan:
            raise AutonomyStopped("Stopped: no usable rotations")
        if not saw_scan:
            raise ValueError("No complete rotations in the input")
        if not saw_plan:
            raise ValueError("No rotations had matching motion data")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    drive_data.add_arguments(parser)
    lidar = PARAMETERS["lidar"]
    motion_defaults = SpeedConfig()
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--csv", help="Replay a scan.csv capture without connecting to Nano")
    source.add_argument("--session", type=Path, help="Replay a captured run directory without connecting to Nano")
    source.add_argument("--ssh", help="Preview-only lidar source, e.g. user@andrea.local")
    parser.add_argument("--port", help="Lidar USB port on the host with the sensor (default /dev/ttyUSB0)")
    parser.add_argument("--drive", action="store_true", help="Enable live commands to Nano")
    parser.add_argument("--interactive", action="store_true",
                        help="Use keyboard manual/autonomous switching with --drive")
    parser.add_argument("--host", help="Nano IPv4 address for --drive")
    parser.add_argument("--nano-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--seconds", type=float, default=0.0, help="Limit runtime; 0 runs until stopped")
    parser.add_argument("--range-offset-mm", type=float, default=lidar["range_offset_mm"],
                        help="Measured range minus actual range (mm), subtracted from valid returns")
    parser.add_argument("--scan-timing", action="store_true", help="Show host rotation rate and estimated ray times")
    parser.add_argument("--planner", choices=("legacy", "clearance", "route"), default="route",
                        help="Select the same path score as the simulator")
    parser.add_argument("--wheelbase", type=float, default=PARAMETERS["chassis"]["wheelbase_m"],
                        help="Rear axle to front steering axis in metres")
    parser.add_argument("--steering-delay-ms", type=float, default=motion_defaults.steering_delay_ms,
                        help="Measured upper command-to-steering delay (ms)")
    parser.add_argument("--steering-rate-deg-s", type=float, default=motion_defaults.steering_rate_deg_s,
                        help="Measured lower loaded steering rate (degrees/s)")
    parser.add_argument("--propulsion-delay-ms", type=float, default=motion_defaults.propulsion_delay_ms,
                        help="Measured command-to-speed-response delay (ms)")
    parser.add_argument("--speed-error-mps", type=float,
                        help="Measured upper error of speed at scan end, including wheel slip and sample lag")
    motion = parser.add_mutually_exclusive_group()
    motion.add_argument("--pose-csv", help="Timestamped rear-axle poses for scan motion compensation")
    motion.add_argument("--encoder-csv", help="Synchronized timestamp_s,left_count,right_count using measured dual_rear calibration")
    motion.add_argument("--stationary-preview", action="store_true",
                        help="Disable motion allowance for a fixed lidar in preview only")
    limits = {
        "forward_angle": ((0, 359.999999), "Raw lidar angle for vehicle forward (degrees)"),
        "sensor_ahead": ((0, 0.5), "Distance from rear axle to sensor (m)"),
        "width": ((0.08, 1.0), "Full vehicle width including wheels (m)"),
        "front": ((0.03, 1.0), "Sensor to front edge (m)"),
        "rear": ((0.03, 1.0), "Sensor to rear edge (m)"),
        "stop_distance": ((0.20, 1.5), "Measured stopping allowance (m)"),
        "pwm": ((1, lidar["max_pwm_pct"]), "Maximum forward PWM percentage"),
        "min_turn_radius": ((0.16, 3.0), "Measured rear-axle-center turning radius (m)"),
    }
    speed_limits = {
        "max_speed": ((0.05, 2.0), "Measured speed at maximum PWM (m/s)"),
        "acceleration": ((0.05, 5.0), "Longitudinal acceleration limit (m/s²)"),
        "deceleration": ((0.05, 5.0), "Measured coasting deceleration (m/s²)"),
        "lateral_accel": ((0.05, 5.0), "Allowed lateral acceleration for corners (m/s²)"),
        "delay_ms": ((0, 1500), "Conservative sensing and actuation delay (ms)"),
        "deadband_pwm": ((0, 64), "PWM threshold below which the vehicle cannot move"),
    }
    all_limits = limits | speed_limits
    for name, (_, help_text) in all_limits.items():
        parser.add_argument("--" + name.replace("_", "-"), type=float, help=help_text)
    args = parser.parse_args()
    if not 0 <= args.steering_delay_ms <= 1000 or not 5 <= args.steering_rate_deg_s <= 1000:
        parser.error("invalid steering delay or steering rate")
    if not 0 <= args.propulsion_delay_ms <= 1000:
        parser.error("invalid propulsion response delay")
    if args.speed_error_mps is not None and (not math.isfinite(args.speed_error_mps) or args.speed_error_mps < 0):
        parser.error("speed-error-mps must be finite and nonnegative")
    if not -100 <= args.range_offset_mm <= 100:
        parser.error("range-offset-mm must be between -100 and 100")
    if (args.csv or args.session) and args.port:
        parser.error("--port cannot be used with replay")
    if (args.pose_csv or args.encoder_csv) and args.ssh:
        parser.error("Motion feedback needs local USB or CSV replay with a shared time base")
    if args.stationary_preview and args.drive:
        parser.error("--stationary-preview cannot be used with --drive")
    if args.interactive and not args.drive:
        parser.error("--interactive requires --drive and its measured vehicle settings")
    if args.drive:
        if args.csv or args.session or args.ssh or not args.host:
            parser.error("--drive needs a local USB lidar and --host; CSV and SSH are preview-only")
        missing = ["--" + name.replace("_", "-") for name in all_limits if getattr(args, name) is None]
        if missing:
            parser.error("--drive needs measured settings: " + ", ".join(missing))
    settings = {}
    for name, ((low, high), _) in all_limits.items():
        value = getattr(args, name)
        if value is not None:
            if not low <= value <= high:
                parser.error(f"{name.replace('_', '-')} must be between {low} and {high}")
            settings[name] = value
    if not 1 <= args.nano_port <= 65535 or not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("invalid Nano port or duration")
    if not math.isfinite(args.wheelbase) or args.wheelbase <= 0:
        parser.error("--wheelbase must be a positive finite length")
    config = TrackConfig(wheelbase=args.wheelbase, prefer_corner_clearance=args.planner in ("clearance", "route"),
                         follow_route=args.planner == "route",
                         **{name: settings[name] for name in limits if name in settings})
    speed_config = SpeedConfig(steering_delay_ms=args.steering_delay_ms,
                               steering_rate_deg_s=args.steering_rate_deg_s,
                               propulsion_delay_ms=args.propulsion_delay_ms,
                               speed_error_mps=(args.speed_error_mps if args.speed_error_mps is not None
                                                else args.max_speed if args.drive else motion_defaults.speed_error_mps),
                               **{name: settings[name] for name in speed_limits if name in settings})
    if speed_config.deadband_pwm >= config.pwm:
        parser.error("deadband-pwm must be less than pwm")
    if args.drive and (config.sensor_ahead > config.wheelbase
                       or config.rear < config.sensor_ahead
                       or config.front < config.wheelbase - config.sensor_ahead):
        parser.error("measured body extents must contain both axles and the sensor must be between them")
    return args, config, speed_config


def run_driving(args, config, speed_config, open_rotations, deadline, poses=None):
    recovering = False
    retry_delay = FIRST_RETRY_S
    offset = getattr(args, "range_offset_mm", 0.0)
    scan_timing = getattr(args, "scan_timing", False)
    while time.monotonic() < deadline:
        progress = DriveProgress()
        try:
            with closing(open_rotations(True)) as rotations:
                if recovering:
                    first = next(rotations, None)
                    if first is None:
                        return
                    samples, geometry, motion, _ = prepare_scan(
                        first, config, speed_config, 0.0, offset, poses=poses, stationary=True,
                        deadline=deadline)
                    route = plan(samples, config, geometry=geometry, motion=motion)
                    if route.pwm <= 0:
                        raise RuntimeError(f"Stopped after lidar recovery: {route.reason}")
                with closing(NanoDrive(args.host, args.nano_port)) as nano:
                    try:
                        run(rotations, config, nano, speed_config=speed_config,
                            progress=progress, deadline=deadline,
                            range_offset_mm=offset, scan_timing=scan_timing, poses=poses)
                    except ScanInterrupted:
                        if progress.lease_ends_at is not None:
                            now = time.monotonic()
                            coast_time = speed_config.max_speed / speed_config.deceleration
                            hold = max(0.0, progress.lease_ends_at - now)
                            print(f"Lidar interrupted. Last command has at most {hold:.2f}s left. "
                                  "Waiting for motor coast before recovery.", flush=True)
                            wait = max(0.0, min(deadline, progress.lease_ends_at + coast_time) - now)
                            time.sleep(wait)
                        raise
            return
        except ScanInterrupted as error:
            if progress.lease_ends_at is not None:
                recovering = True
                retry_delay = FIRST_RETRY_S
            if not recovering:
                raise
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            print(f"Lidar recovery: {error}. Retrying in {retry_delay:g}s. "
                  "Driving resumes only after three fresh, reliable scans.", flush=True)
            time.sleep(min(retry_delay, remaining))
            retry_delay = min(8, retry_delay * 2)


def run_live(args, config, speed_config, poses=None):
    if getattr(args, "interactive", False) and not sys.stdin.isatty():
        raise RuntimeError("--interactive requires a terminal (use ssh -t when connecting from Windows)")
    stop = threading.Event()
    port = args.port or DEFAULT_PI_PORT
    if args.ssh:
        from lidar.acquisition.ssh import ssh_scan_chunks
        user, _, host = args.ssh.partition("@")
        if not user or not host or "@" in host:
            raise ValueError("SSH address must be user@host")
        password = getpass("Pi account password: ")

        def open_sensor(stop_event):
            return ssh_scan_chunks(args.ssh, port, password, stop_event)
    else:
        usb = LidarPort(port)

        def open_sensor(stop_event):
            return scan_chunks(usb.resolve(), stop_event)

    deadline = time.monotonic() + args.seconds if args.seconds else math.inf

    def open_rotations(fresh, stop_event=stop):
        reader_stop = None if args.ssh else threading.Event()
        yield from live_rotations(open_sensor(reader_stop if reader_stop is not None else stop_event), fresh,
                                  deadline=deadline, stop=stop_event, reader_stop=reader_stop,
                                  serial_timing=not args.ssh)

    def report_retry(delay, error):
        print(f"Lidar interrupted: {error}. Retrying in {delay}s.", flush=True)

    if args.drive:
        if getattr(args, "interactive", False):
            from .interactive import run_console

            return run_console(args, config, speed_config, open_rotations, deadline, poses)
        try:
            run_driving(args, config, speed_config, open_rotations, deadline, poses)
            return 0
        except ScanInterrupted as error:
            print(f"Initial drive did not start: {error}. Reconnecting lidar in preview only.", flush=True)
    with closing(reconnecting_scans(lambda: open_rotations(False), stop, report_retry,
                                    deadline=deadline)) as rotations:
        run(rotations, config, None, speed_config=speed_config, deadline=deadline,
            range_offset_mm=getattr(args, "range_offset_mm", 0.0),
            scan_timing=getattr(args, "scan_timing", False), poses=poses,
            stationary_preview=getattr(args, "stationary_preview", False))
    return int(args.drive)


def main():
    args, config, speed_config = parse_args()
    if not args.drive:
        print("PREVIEW: only calculates commands; no connection to Nano")
    try:
        with ExitStack() as stack:
            replay = args.csv or args.session
            def record_motion(stamp, pose, raw):
                drive_data.event("motion_sample", source="encoder" if args.encoder_csv else "pose",
                                 timestamp_s=stamp, x_m=pose.x, y_m=pose.y, yaw_deg=math.degrees(pose.yaw),
                                 steer_deg=pose.steer_deg, raw_csv=raw)

            poses = (PoseLog(stack.enter_context(open(args.pose_csv, encoding="utf-8-sig")),
                             live=not replay, on_sample=None if replay else record_motion) if args.pose_csv else None)
            if args.encoder_csv:
                poses = EncoderLog(stack.enter_context(open(args.encoder_csv, encoding="utf-8-sig")),
                                   PARAMETERS["dual_rear"], live=not replay, on_sample=None if replay else record_motion)
            if args.session and poses is None and not args.stationary_preview:
                poses = recorded_session_motion(args.session, stack)
            if not replay:
                mode = "lidar-interactive" if args.interactive else "lidar-auto" if args.drive else "lidar-preview"
                settings = dict(track=asdict(config), speed=asdict(speed_config),
                                range_offset_mm=args.range_offset_mm, stationary_preview=args.stationary_preview,
                                motion_source="encoder" if args.encoder_csv else "pose" if args.pose_csv else "none",
                                scan_source="ssh" if args.ssh else "local")
                stack.enter_context(drive_data.session(args, mode, settings))
                return drive_data.finish(run_live(args, config, speed_config, poses))
            if args.session:
                capture = stack.enter_context((args.session / "scans.jsonl").open(encoding="utf-8"))
                rotations = session_rotations(capture)
            else:
                capture = stack.enter_context(open(args.csv, newline="", encoding="utf-8-sig"))
                rotations = recorded_rotations(capture)
            run(rotations, config, None, args.seconds, speed_config,
                range_offset_mm=args.range_offset_mm, scan_timing=args.scan_timing, poses=poses,
                stationary_preview=args.stationary_preview)
    except KeyboardInterrupt:
        print("Stopped by operator")
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        print(f"Stopped: {error}")
        return 1
    return 0


def session_rows(stream):
    for line in stream:
        try:
            yield json.loads(line)
        except json.JSONDecodeError:
            if line.endswith("\n"):
                raise
            # A process or power loss can interrupt the final JSONL write.
            return


def session_rotations(capture):
    for row in session_rows(capture):
        if row.get("type") == "scan" and row.get("usable", True):
            yield Rotation(row["start"], row["end"], row["samples"], row.get("sample_times"),
                           row.get("timing_source", "angle-estimate"))


def recorded_session_motion(folder, stack):
    """Recover the poses used during this run, even after wheel calibration changes."""
    path = folder / "events.jsonl"
    if not path.is_file():
        return None
    output = None
    with path.open(encoding="utf-8") as events:
        for event in session_rows(events):
            if event.get("type") != "motion_sample":
                continue
            steering = event.get("steer_deg")
            if output is None:
                output = stack.enter_context(tempfile.SpooledTemporaryFile(
                    max_size=1024 * 1024, mode="w+", encoding="utf-8", newline=""))
                writer = csv.writer(output)
                has_steering = steering is not None
                header = ["timestamp_s", "x_m", "y_m", "yaw_deg"]
                writer.writerow(header + (["steer_deg"] if has_steering else []))
            if has_steering != (steering is not None):
                raise ValueError("Recorded steering sensor availability changed within a run")
            values = [event[key] for key in ("timestamp_s", "x_m", "y_m", "yaw_deg")]
            writer.writerow(values + ([steering] if has_steering else []))
    if output is None:
        return None
    output.seek(0)
    return PoseLog(output)
