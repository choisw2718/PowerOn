"""A run directory belongs to the machine that observed the data."""

import argparse
from contextlib import contextmanager, nullcontext
from datetime import datetime, timezone, timedelta
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import subprocess
import sys
import threading
import time
import uuid
import zipfile

from hardware import PARAMETERS


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIR = ROOT / "주행 데이터"
_current = None
KST = timezone(timedelta(hours=9), "KST")
RUN_ID = re.compile(r"[0-9]{8}T[0-9]{6}(?:Z|\+0900)-[0-9a-f]{12}")
PAYLOADS = ("events.jsonl", "scans.jsonl", "sources.zip")


def timestamp():
    return datetime.now(timezone.utc).isoformat()


def json_text(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(content):
    return hashlib.sha256(content).hexdigest()


def file_digest(path):
    with path.open("rb") as stream:
        return _hash_stream(stream)


def _hash_stream(stream):
    result = hashlib.sha256()
    for block in iter(lambda: stream.read(262144), b""):
        result.update(block)
    return result.hexdigest()


def atomic_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json_text(value) + "\n", encoding="utf-8")
    temporary.replace(path)


def source_versions(root, archive, mode):
    """Include active public Python sources even in untracked and scp deployments."""
    sources = [root / "hardware/__init__.py", root / "vehicle/__init__.py",
               root / "vehicle/wifi/client.py", root / "vehicle/manual_control/keyboard.py"]
    sources.extend(root / n for n in ("vehicle/drive_data/__init__.py", "vehicle/drive_data/transfer.py",
                                      "vehicle/drive_data/__main__.py", "lidar/acquisition/ssh.py",
                                      "lidar/acquisition/serial_scan.py", "lidar/acquisition/latest_scan.py",
                                      "lidar/autopilot/runner.py", "lidar/geometry/scan_geometry.py"))
    if mode.startswith("lidar"):
        for pattern in ("lidar/autopilot/interactive.py",
                        "lidar/autopilot/planning/wall_track.py", "lidar/autopilot/planning/continuity.py",
                        "lidar/autopilot/motion/feedback.py", "lidar/autopilot/motion/speed_control.py",
                        "lidar/geometry/wheel_odometry.py"):
            sources.append(root / pattern)
        # Include new, actually imported planner/motion modules on the executing host.
        for name, module in tuple(sys.modules.items()):
            if name.startswith(("lidar.autopilot.motion.", "lidar.autopilot.planning.", "lidar.geometry.")):
                file = getattr(module, "__file__", None)
                if file:
                    path = Path(file).resolve()
                    if path.suffix == ".py" and path.is_relative_to(root):
                        sources.append(path)
    else:
        sources.extend(root / name for name in ("vehicle/__main__.py", "vehicle/manual_control/rear_drive.py",
                                                "vehicle/sensor_monitor/monitor.py",
                                                "vehicle/ir_wall_following/preview.py", "vehicle/wifi/discovery.py"))
    hashes = {}
    with (zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) if archive else nullcontext()) as output:
        for path in sorted(set(sources)):
            if not path.is_file():
                continue
            relative = path.relative_to(root).as_posix()
            content = path.read_bytes().replace(b"\r\n", b"\n")
            hashes[relative] = digest(content)
            if output:
                output.writestr(relative, content)
    def git(*args):
        try:
            result = subprocess.run(["git", "-c", f"safe.directory={root.as_posix()}", "-C", str(root), *args],
                                    capture_output=True, timeout=2)
            return result.stdout.decode(errors="replace").strip() if result.returncode == 0 else None
        except (OSError, subprocess.TimeoutExpired):
            return None
    status = git("status", "--porcelain", "--untracked-files=all", "--", *hashes)
    return dict(sha256=digest(json_text(hashes).encode()), files=hashes,
                git_commit=git("rev-parse", "HEAD"), dirty=None if status is None else bool(status))


def acquire_lock(path):
    """OS releases the session lock on a crash; other running sessions keep theirs."""
    stream = path.open("a+b")
    try:
        if os.name == "nt":
            import msvcrt
            stream.seek(0)
            msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        stream.close()
        raise
    return stream


def recover_interrupted(data_dir):
    if not data_dir.is_dir():
        return
    for folder in data_dir.iterdir():
        marker = folder / "manifest.json"
        if not RUN_ID.fullmatch(folder.name) or not marker.is_file() or not (folder / ".owner.lock").is_file():
            continue
        try:
            with acquire_lock(folder / ".owner.lock"):
                manifest = json.loads(marker.read_text(encoding="utf-8"))
                if manifest.get("state") != "recording" or manifest.get("run_id") != folder.name:
                    continue
                manifest.update(state="incomplete", outcome="process_interrupted", ended_utc=None,
                                recovered_utc=timestamp(), dropped=None,
                                files={n: file_digest(folder / n) for n in PAYLOADS if (folder / n).is_file()})
                atomic_json(marker, manifest)
        except (OSError, ValueError):
            continue


def add_arguments(parser: argparse.ArgumentParser):
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DIR, help="Directory containing drive sessions")
    parser.add_argument("--hardware-rev", help="Physical hardware revision as identified by the operator")
    parser.add_argument("--change-note", default="", help="Physical or calibration changes for this run")


class Recorder:
    def __init__(self, data_dir, mode, settings, revision=None, note="", root=ROOT):
        started = datetime.now(KST)
        self.id = started.strftime("%Y%m%dT%H%M%S%z") + "-" + uuid.uuid4().hex[:12]
        recover_interrupted(Path(data_dir))
        self.folder = Path(data_dir) / self.id
        self.folder.mkdir(parents=True, exist_ok=False)
        self.owner = acquire_lock(self.folder / ".owner.lock")
        self.queue = queue.Queue(maxsize=256)
        self.dropped = {}
        self.error = None
        self.exit_code = None
        self.lock = threading.Lock()
        self.accepting = True
        self.mono = time.monotonic()
        cad = PARAMETERS.get("cad", {})
        profile_path = root / "hardware/control_profile.json"
        profile = json.loads(profile_path.read_text(encoding="utf-8")) if profile_path.is_file() else {}
        self.manifest = dict(schema_version=1, run_id=self.id, mode=mode, started_utc=started.astimezone(timezone.utc).isoformat(),
                             started_kst=started.isoformat(),
                             start_monotonic_s=self.mono, state="recording", settings=settings,
                             settings_sha256=digest(json_text(settings).encode()), hardware_profile=profile,
                             hardware_profile_sha256=digest(json_text(profile).encode()),
                             cad_file=cad.get("source_name"), cad_sha256=cad.get("source_sha256"),
                             hardware_revision=revision or cad.get("hardware_revision") or "unknown",
                             hardware_revision_source="operator" if revision else "profile",
                             change_note=note, firmware_reported=[], firmware_by_connection={},
                             event_counts={}, software=source_versions(
                                 root, self.folder / "sources.zip", mode))
        atomic_json(self.folder / "manifest.json", self.manifest)
        self.worker = threading.Thread(target=self._write, name="drive-data-writer", daemon=True)
        self.worker.start()
        print(f"주행 데이터 · {self.folder}", flush=True)

    def record(self, kind, payload):
        stamp = time.monotonic()
        with self.lock:
            if not self.accepting:
                return
            try:
                self.queue.put_nowait((kind, stamp, payload))
            except queue.Full:
                self.dropped[kind] = self.dropped.get(kind, 0) + 1

    def _write(self):
        try:
            with (self.folder / "events.jsonl").open("w", encoding="utf-8", buffering=1) as events, \
                 (self.folder / "scans.jsonl").open("w", encoding="utf-8", buffering=1) as scans:
                while True:
                    item = self.queue.get()
                    if item is None:
                        return
                    kind, stamp, payload = item
                    if kind == "scan":
                        stream = scans
                    else:
                        stream = events
                    counts = self.manifest["event_counts"]
                    counts[kind] = counts.get(kind, 0) + 1
                    connections = self.manifest["firmware_by_connection"]
                    if kind == "nano_connect":
                        connections[payload["connection"]] = "unknown"
                    if kind == "nano_reply" and payload["line"].startswith(("STATUS ", "FIRMWARE ")):
                        fields = dict(token.split("=", 1) for token in payload["line"].split() if "=" in token)
                        reported = fields.get("firmware_id") or fields.get("build_id")
                        if reported:
                            connections[payload.get("connection", "unspecified")] = reported
                            if reported not in self.manifest["firmware_reported"]:
                                self.manifest["firmware_reported"].append(reported)
                    stream.write(json_text(dict(t_s=stamp - self.mono, type=kind, **payload)) + "\n")
        except (OSError, ValueError, TypeError) as error:
            self.error = str(error)
            print(f"주행 기록 쓰기 실패 · {error}", file=sys.stderr, flush=True)

    def close(self, outcome):
        with self.lock:
            self.accepting = False
        while self.worker.is_alive():
            try:
                self.queue.put(None, timeout=0.15)
                break
            except queue.Full:
                pass
        self.worker.join()
        if self.manifest["firmware_by_connection"]:
            self.manifest["firmware_reported"] = sorted(set(self.manifest["firmware_by_connection"].values()))
        if not self.manifest["firmware_reported"]:
            self.manifest["firmware_reported"] = ["unknown"]
        self.manifest.update(state="incomplete" if self.error or self.dropped else "closed",
                             outcome=outcome, exit_code=self.exit_code, ended_utc=timestamp(),
                             ended_kst=datetime.now(KST).isoformat(),
                             dropped=self.dropped, writer_error=self.error)
        try:
            self.manifest["files"] = {name: file_digest(self.folder / name)
                                      for name in PAYLOADS if (self.folder / name).is_file()}
            atomic_json(self.folder / "manifest.json", self.manifest)
        finally:
            self.owner.close()


@contextmanager
def session(args, mode, settings):
    global _current
    recorder = Recorder(getattr(args, "data_dir", DEFAULT_DIR), mode, settings,
                        getattr(args, "hardware_rev", None), getattr(args, "change_note", ""))
    _current = recorder
    outcome = "finished"
    try:
        yield recorder
    except BaseException as error:
        outcome = type(error).__name__
        raise
    finally:
        _current = None
        recorder.close(outcome)


def recording():
    return _current is not None


def finish(code):
    if _current is not None:
        _current.exit_code = code or 0
    return code


def event(kind, **payload):
    recorder = _current
    if recorder is not None:
        recorder.record(kind, payload)


def record_rotation(frame, usable=True):
    event("scan", start=frame.start, end=frame.end, samples=frame.samples,
          sample_times=frame.sample_times, timing_source=frame.timing_source, usable=usable)


def differences(first, second):
    changed = {}
    for key in ("mode", "hardware_profile_sha256", "hardware_revision",
                "change_note", "cad_sha256", "firmware_reported"):
        if first.get(key) != second.get(key):
            changed[key] = [first.get(key), second.get(key)]
    a, b = (m.get("software", {}).get("files", {}) for m in (first, second))
    files = sorted(p for p in a.keys() | b.keys() if a.get(p) != b.get(p))
    if files:
        changed["software_files"] = files
    for name in ("settings", "hardware_profile"):
        delta = {}
        compare_values([], first.get(name, {}), second.get(name, {}), delta)
        if delta:
            changed[name] = delta
    return changed


def compare_values(path, a, b, out):
    if isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(a.keys() | b.keys()):
            compare_values(path + [key], a.get(key), b.get(key), out)
    elif a != b:
        out[".".join(path)] = [a, b]


def current_differences(run):
    profile = json.loads((ROOT / "hardware/control_profile.json").read_text(encoding="utf-8"))
    cad = PARAMETERS.get("cad", {})
    current = dict(run, software=source_versions(ROOT, None, run["mode"]), hardware_profile=profile,
                   hardware_profile_sha256=digest(json_text(profile).encode()),
                   hardware_revision=cad.get("hardware_revision") or "unknown",
                   cad_sha256=cad.get("source_sha256"))
    return differences(run, current)
