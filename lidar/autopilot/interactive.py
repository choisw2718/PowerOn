"""Share one Nano connection between the Pi's planner and the operator."""

from contextlib import closing, suppress
import threading
import time

from vehicle.manual_control.keyboard import KeyboardCommands, KeyboardInput
from vehicle.drive_data import event
from lidar.acquisition.serial_scan import ScanInterrupted

from .planning.continuity import MAX_LEASE_MS
from .runner import AutonomyStopped, DriveProgress, NanoDrive, run


POLL_S = 0.05
RETRY_S = 0.5
MANUAL_SETTLE_S = 2.0
MANUAL_REPLIES = {
    "DRIVE": "OK DRIVE applied_nominal_V=",
    "IDLE": "OK IDLE",
    "STOP": "OK STOP",
    "STATUS": "STATUS ",
    "NETWORK": "NETWORK ",
    "SENSORS": "SENSORS ",
    "PING": "PONG LED=",
}


class AttemptCancelled(Exception):
    """The operator or another part of the console revoked this attempt."""


class AutoAttempt:
    def __init__(self, console):
        self.console = console
        self.stop = threading.Event()
        self.progress = DriveProgress(guard=console.lock)

    def request(self, command, expected):
        console = self.console
        # Keep the ownership check and the actual Nano write under the same lock.
        with console.lock:
            if self is not console.attempt or console.stop.is_set():
                raise AttemptCancelled
            reply = console.nano.request(command, expected)
            if (not console.autonomous and command.startswith("LIDAR DRIVE ")
                    and float(command.split()[2]) > 0):
                console.autonomous = True
                console.report("fresh scans confirmed")
            return reply


class DriveConsole:
    def __init__(self, host, port, speed_config):
        self.host = host
        self.port = port
        self.coast_s = speed_config.max_speed / speed_config.deceleration
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.keys = KeyboardCommands()
        self.nano = None
        self.attempt = None
        self.auto_requested = False
        self.autonomous = False
        self.settled_at = 0.0
        self.manual_power_possible = False
        self.worker_failed = False

    def report(self, reason):
        if self.autonomous:
            mode = "AUTONOMOUS"
        elif self.auto_requested:
            mode = "MANUAL (automatic recovery pending)"
        else:
            mode = "MANUAL"
        event("mode", mode=mode, reason=reason)
        print(f"MODE {mode}: {reason}", flush=True)

    def _disconnect(self):
        nano, self.nano = self.nano, None
        self.keys.reset()
        if nano is not None:
            with suppress(OSError):
                nano.close()

    def _request(self, command):
        if self.nano is None:
            return None
        expected = MANUAL_REPLIES[command.split()[0]]
        try:
            return self.nano.request(command, expected, prefix=expected.endswith((" ", "=")))
        except OSError as error:
            self._disconnect()
            self.report(f"Nano connection lost: {error}")
            return None

    def _end_attempt(self):
        attempt, self.attempt = self.attempt, None
        self.autonomous = False
        if attempt is None:
            return False
        attempt.stop.set()
        stopped = self._request("IDLE") is not None
        if attempt.progress.lease_ends_at is not None:
            now = time.monotonic()
            last_power = now if stopped else max(now + MAX_LEASE_MS / 1000, attempt.progress.lease_ends_at)
            self.settled_at = max(self.settled_at, last_power + self.coast_s)
        return True

    def _settle_after_manual(self):
        if self.manual_power_possible:
            # Manual commands may exceed the planner's normal speed setting.
            self.settled_at = max(self.settled_at, time.monotonic() + max(MANUAL_SETTLE_S, self.coast_s))
            self.manual_power_possible = False

    def handle_key(self, key):
        with self.lock:
            if self.stop.is_set():
                return
            command = self.keys.command_for(key)
            if command is None:
                return
            if command == "AUTO STATUS":
                self.report("Nano connected" if self.nano is not None else "Nano reconnecting")
                return
            if command == "AUTO START":
                if not self.auto_requested:
                    if self._request("IDLE") is not None:
                        self._settle_after_manual()
                    self.auto_requested = True
                    self.report("waiting for three fresh scans")
                return

            verb = command.split()[0]
            if verb in {"DRIVE", "STOP", "IDLE"}:
                if self.auto_requested:
                    self.auto_requested = False
                    stopped_attempt = self._end_attempt()
                    self.report("operator selected manual control")
                    if stopped_attempt and verb in {"STOP", "IDLE"}:
                        return
                if self.nano is None or (verb == "DRIVE" and time.monotonic() < self.settled_at):
                    self.keys.reset()
                    self.report("input discarded while reconnecting or stopping; press the key again")
                    return
                if verb == "DRIVE" and self.keys.motor_voltage:
                    self.manual_power_possible = True
            reply = self._request(command)
            if reply is None:
                if self.attempt is not None:
                    self._end_attempt()
                self.report("waiting for Nano; command was not confirmed")
            else:
                print(f"< {reply}", flush=True)

    def _connect(self):
        try:
            nano = NanoDrive(self.host, self.port)
        except OSError as error:
            with self.lock:
                if not self.stop.is_set():
                    self.report(str(error))
            return False
        with self.lock:
            if self.stop.is_set():
                nano.close()
                return False
            self.nano = nano
            # READY on the new socket means the previous session has stopped.
            self._settle_after_manual()
            self.keys.reset()
            self.report("Nano connected")
            return True

    def work(self, args, config, speed_config, open_rotations, deadline, poses):
        try:
            while not self.stop.is_set() and time.monotonic() < deadline:
                with self.lock:
                    connected = self.nano is not None
                    attempt = None
                    if connected and self.auto_requested and time.monotonic() >= self.settled_at:
                        attempt = self.attempt = AutoAttempt(self)
                if not connected:
                    if not self._connect():
                        self.stop.wait(RETRY_S)
                    continue
                if attempt is None:
                    self.stop.wait(POLL_S)
                    continue
                try:
                    with closing(open_rotations(True, attempt.stop)) as rotations:
                        run(rotations, config, attempt, speed_config=speed_config,
                            progress=attempt.progress, deadline=deadline,
                            range_offset_mm=args.range_offset_mm, scan_timing=args.scan_timing,
                            poses=poses)
                    reason = "autonomous scan session ended"
                except AttemptCancelled:
                    continue
                except (AutonomyStopped, OSError) as error:
                    reason = str(error)
                with self.lock:
                    if self.attempt is attempt:
                        self._end_attempt()
                        self.report(reason)
                self.stop.wait(RETRY_S)
        except Exception as error:
            self.worker_failed = True
            print(f"Drive console failed: {error}", flush=True)
        finally:
            self.close()

    def close(self):
        with self.lock:
            self.stop.set()
            self.auto_requested = False
            if not self._end_attempt():
                self._request("IDLE")
            self._disconnect()


def print_help():
    print("1/G: request autonomous driving | 0/-/I: stop and select manual mode")
    print("W/S/A/D/C: manual driving | Space/X: stop | B: current mode | Q/Esc: exit")
    print("R: status | N: network | L: IR sensors | P: ping | H/?: help", flush=True)


class ConsoleScans:
    """One continuously recorded USB stream, shared across manual and autonomous intervals."""
    def __init__(self, open_rotations, stop, deadline):
        self.open_rotations = open_rotations
        self.stop = stop
        self.deadline = deadline
        self.ready = threading.Condition()
        self.sequence = 0
        self.latest = None
        self.failure = None
        self.generation = 0

    def read(self):
        while not self.stop.is_set() and time.monotonic() < self.deadline:
            try:
                with closing(self.open_rotations(True, self.stop)) as rotations:
                    for frame in rotations:
                        with self.ready:
                            self.sequence += 1
                            self.failure = None
                            self.latest = frame
                            self.ready.notify_all()
                if self.stop.is_set() or time.monotonic() >= self.deadline:
                    return
                raise ScanInterrupted("Lidar stream ended")
            except Exception as error:
                event("sensor_state", sensor="lidar", state="unavailable", reason=str(error))
                with self.ready:
                    self.failure = str(error)
                    self.latest = None
                    self.generation += 1
                    self.ready.notify_all()
                self.stop.wait(RETRY_S)

    def frames(self, fresh, attempt_stop):
        with self.ready:
            after = self.sequence
            generation = self.generation
        while not attempt_stop.is_set() and not self.stop.is_set() and time.monotonic() < self.deadline:
            with self.ready:
                if self.failure is not None or self.generation != generation:
                    raise ScanInterrupted(self.failure or "Lidar was reconnected")
                if self.sequence == after:
                    self.ready.wait(POLL_S)
                    continue
                after, frame = self.sequence, self.latest
            if frame is not None:
                yield frame


def run_console(args, config, speed_config, open_rotations, deadline, poses):
    console = DriveConsole(args.host, args.nano_port, speed_config)
    scans = ConsoleScans(open_rotations, console.stop, deadline)
    reader = threading.Thread(target=scans.read, name="console-lidar", daemon=True)
    worker = threading.Thread(target=console.work, name="poweron-autonomy", daemon=True,
                              args=(args, config, speed_config, scans.frames, deadline, poses))
    print_help()
    console.report("starting in manual mode")
    try:
        with KeyboardInput() as keyboard:
            reader.start()
            worker.start()
            while not console.stop.is_set() and time.monotonic() < deadline:
                key = keyboard.read(POLL_S)
                if time.monotonic() >= deadline:
                    break
                if key is None:
                    continue
                if key.lower() in {"", "q", "\x1b"}:
                    break
                if key.lower() in {"h", "?"}:
                    print_help()
                else:
                    console.handle_key(key)
    except KeyboardInterrupt:
        print("Stopped by operator", flush=True)
    finally:
        console.close()
        if worker.ident is not None:
            worker.join(6.0)
        if reader.ident is not None:
            reader.join(2.0)
    return int(console.worker_failed)
