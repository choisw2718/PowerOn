"""Collect sealed sessions over SFTP into a local project without changing the Pi."""

from contextlib import contextmanager
import json
import os
from pathlib import Path
import posixpath
import re
import stat
import tempfile
import threading
import time

from . import event, file_digest, RUN_ID, PAYLOADS


DEFAULT_PI = "user@andrea.local"
DEFAULT_REMOTE = "poweron-lidar/주행 데이터"


def collect(sftp, remote_root, local_root):
    local_root = Path(local_root)
    local_root.mkdir(parents=True, exist_ok=True)
    count = 0
    for item in sftp.listdir_attr(remote_root):
        name = item.filename
        if not RUN_ID.fullmatch(name) or not stat.S_ISDIR(item.st_mode):
            continue
        remote = posixpath.join(remote_root, name)
        try:
            with sftp.open(posixpath.join(remote, "manifest.json"), "rb") as stream:
                raw = stream.read(4 * 1024 * 1024 + 1)
        except FileNotFoundError:
            continue
        if len(raw) > 4 * 1024 * 1024:
            raise ValueError("Oversized drive manifest")
        manifest = json.loads(raw)
        if manifest.get("state") not in ("closed", "incomplete"):
            continue
        files = manifest.get("files", {})
        if (manifest.get("run_id") != name or not files or not set(files) <= set(PAYLOADS)
                or not all(isinstance(v, str) and re.fullmatch(r"[0-9a-f]{64}", v) for v in files.values())
                or (manifest["state"] == "closed" and set(files) != set(PAYLOADS))):
            raise ValueError(f"Invalid drive manifest for {name}")
        dest = local_root / name
        if dest.exists():
            if ((dest / "manifest.json").is_file() and (dest / "manifest.json").read_bytes() == raw
                    and all((dest / f).is_file() and file_digest(dest / f) == h for f, h in files.items())):
                continue
            raise FileExistsError(f"Different local data already exists for {name}")
        with tempfile.TemporaryDirectory(prefix=".collect-", dir=local_root) as temp:
            stage = Path(temp) / name
            stage.mkdir()
            for filename, sha in files.items():
                source = posixpath.join(remote, filename)
                if not stat.S_ISREG(sftp.lstat(source).st_mode):
                    raise ValueError(f"Unexpected type for {filename}")
                target = stage / filename
                sftp.get(source, str(target))
                if file_digest(target) != sha:
                    raise ValueError(f"Transfer checksum failed for {name}/{filename}")
            (stage / "manifest.json").write_bytes(raw)
            stage.rename(dest)
        count += 1
    return count


def collect_pi(target, remote_root, local_root, password=None):
    import paramiko
    username, sep, host = target.partition("@")
    if not sep or not username or not host or "@" in host:
        raise ValueError("Use user@host for the Pi")
    with paramiko.SSHClient() as client:
        client.load_system_host_keys()
        client.connect(host, username=username, password=password, timeout=5,
                       auth_timeout=5, banner_timeout=5)
        with client.open_sftp() as sftp:
            sftp.get_channel().settimeout(15)
            return collect(sftp, remote_root, local_root)


@contextmanager
def notebook_capture(args):
    """USB acquisition and remote data collection run outside the Nano command path."""
    target = getattr(args, "lidar_ssh", None)
    if not target:
        event("sensor_state", sensor="lidar", state="no_source")
        yield
        return
    stop = threading.Event()

    def sensor():
        try:
            from lidar.acquisition.ssh import ssh_scan_chunks
            from lidar.autopilot.runner import live_rotations
        except ImportError as error:
            event("sensor_state", sensor="lidar", state="unavailable", reason=str(error))
            return
        while not stop.is_set():
            try:
                chunks = ssh_scan_chunks(target, args.lidar_port, None, stop)
                for _ in live_rotations(chunks, False, stop=stop, serial_timing=False):
                    if stop.is_set():
                        return
            except (RuntimeError, ValueError, OSError, EOFError) as error:
                event("sensor_state", sensor="lidar", state="unavailable", reason=str(error))
            stop.wait(5)

    def sync():
        while not stop.is_set():
            try:
                count = collect_pi(target, args.pi_data_root, args.data_dir)
                if count:
                    event("sync", received=count)
            except Exception as error:
                event("sync_unavailable", reason=str(error))
            stop.wait(60)

    workers = [threading.Thread(target=target_fn, daemon=True, name=name)
               for target_fn, name in ((sensor, "pi-lidar"), (sync, "pi-drive-data"))]
    for worker in workers:
        worker.start()
    try:
        yield
    finally:
        stop.set()
        end = time.monotonic() + 6
        for worker in workers:
            worker.join(max(0, end - time.monotonic()))


def add_notebook_arguments(parser):
    parser.add_argument("--lidar-ssh", default=os.environ.get("POWERON_LIDAR_SSH", DEFAULT_PI if os.name == "nt" else ""),
                        help="Pi for parallel lidar capture and drive data collection, e.g. user@andrea.local")
    parser.add_argument("--lidar-port", default="/dev/ttyUSB0")
    parser.add_argument("--pi-data-root", default=DEFAULT_REMOTE)
