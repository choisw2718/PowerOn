import json
import shutil
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile

from lidar.autopilot.runner import Rotation, session_rotations
from lidar.autopilot.interactive import ConsoleScans
from vehicle import drive_data
from vehicle.drive_data import transfer
from vehicle.wifi.client import _RecordingSocket


class FileSFTP:
    def listdir_attr(self, target):
        return [SimpleNamespace(filename=p.name, st_mode=p.stat().st_mode) for p in Path(target).iterdir()]

    def lstat(self, target):
        return Path(target).lstat()

    def open(self, target, mode):
        return open(target, mode)

    def get(self, source, dest):
        shutil.copyfile(source, dest)


class DriveDataTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.args = SimpleNamespace(data_dir=self.root / "pi", hardware_rev="revision-A", change_note="tire change")

    def test_timed_replay_and_observed_firmware(self):
        chunks = iter((b"STATUS firm", b"ware_id=test-1\r\nREAR left ticks=41\n"))
        with drive_data.session(self.args, "lidar-interactive", {"pwm": 12}) as rec:
            frame = Rotation(1.0, 1.1, [(0, 1000, 9), (10, 999.5, 7)], (1.02, 1.06), "sample-clock")
            drive_data.record_rotation(frame)
            drive_data.record_rotation(frame, False)
            sock = _RecordingSocket(SimpleNamespace(recv=lambda n: next(chunks)), "connected")
            sock.recv(512)
            sock.recv(512)
            drive_data.finish(0)
        manifest = json.loads((rec.folder / "manifest.json").read_text(encoding="utf-8"))
        self.assertIn("+0900", rec.id)
        self.assertTrue(manifest["started_kst"].endswith("+09:00"))
        self.assertEqual(manifest["firmware_reported"], ["test-1"])
        self.assertEqual(manifest["exit_code"], 0)
        self.assertEqual(manifest["files"]["events.jsonl"], drive_data.file_digest(rec.folder / "events.jsonl"))
        with (rec.folder / "scans.jsonl").open(encoding="utf-8") as capture:
            frames = list(session_rotations(capture))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sample_times, [1.02, 1.06])
        self.assertEqual(frames[0].end, frame.end)
        self.assertEqual(frames[0].samples[1], [10, 999.5, 7])
        with zipfile.ZipFile(rec.folder / "sources.zip") as sources:
            self.assertIn("lidar/geometry/wheel_odometry.py", sources.namelist())
            self.assertFalse(any("secret" in p for p in sources.namelist()))

    def test_unknown_firmware_and_hash_verified_idempotent_transfer(self):
        with drive_data.session(self.args, "manual", {}) as run:
            drive_data.event("nano_reply", line="STATUS driver=L298N")
        local = self.root / "local"
        self.assertEqual(transfer.collect(FileSFTP(), str(self.args.data_dir), local), 1)
        self.assertEqual(transfer.collect(FileSFTP(), str(self.args.data_dir), local), 0)
        manifest = json.loads((local / run.id / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["firmware_reported"], ["unknown"])
        (run.folder / "events.jsonl").write_bytes(b"wrong stream")
        empty = self.root / "fresh"
        with self.assertRaisesRegex(ValueError, "checksum"):
            transfer.collect(FileSFTP(), str(self.args.data_dir), empty)
        self.assertFalse((empty / run.id).exists())

    def test_recovery_respects_active_recorder(self):
        with drive_data.session(self.args, "manual", {}) as run:
            manifest_path = run.folder / "manifest.json"
            drive_data.recover_interrupted(self.args.data_dir)
            self.assertEqual(json.loads(manifest_path.read_bytes())["state"], "recording")
        manifest = json.loads(manifest_path.read_bytes())
        manifest["state"] = "recording"
        drive_data.atomic_json(manifest_path, manifest)
        drive_data.recover_interrupted(self.args.data_dir)
        sealed = json.loads(manifest_path.read_bytes())
        self.assertEqual(sealed["state"], "incomplete")
        self.assertEqual(sealed["outcome"], "process_interrupted")

    def test_changed_untracked_algorithm_and_manual_physical_revision(self):
        src = self.root / "lidar/autopilot/planning/wall_track.py"
        src.parent.mkdir(parents=True)
        src.write_bytes(b"gain = 4\r\n")
        a = drive_data.source_versions(self.root, None, "lidar-auto")
        src.write_bytes(b"gain = 4\n")
        self.assertEqual(a["sha256"], drive_data.source_versions(self.root, None, "lidar-auto")["sha256"])
        src.write_bytes(b"gain = 5\n")
        b = drive_data.source_versions(self.root, None, "lidar-auto")
        diff = drive_data.differences({"software": a, "hardware_revision": "A"},
                                      {"software": b, "hardware_revision": "B"})
        self.assertEqual(diff["software_files"], ["lidar/autopilot/planning/wall_track.py"])
        self.assertEqual(diff["hardware_revision"], ["A", "B"])

    def test_only_known_run_files_can_be_downloaded(self):
        with drive_data.session(self.args, "manual", {}) as run:
            pass
        marker = run.folder / "manifest.json"
        spec = json.loads(marker.read_bytes())
        spec["files"]["../other"] = "0" * 64
        drive_data.atomic_json(marker, spec)
        with self.assertRaisesRegex(ValueError, "Invalid"):
            transfer.collect(FileSFTP(), str(self.args.data_dir), self.root / "local")

    def test_manual_and_auto_share_one_stream_and_never_replay_manual_frame(self):
        ready = threading.Event()
        go = threading.Event()
        stop = threading.Event()
        opened = []
        def stream(fresh, cancelled):
            opened.append(fresh)
            yield "manual-frame"
            ready.set()
            go.wait(2)
            yield "autonomous-frame"
            cancelled.wait(2)
        scans = ConsoleScans(stream, stop, time.monotonic() + 3)
        reader = threading.Thread(target=scans.read)
        reader.start()
        try:
            self.assertTrue(ready.wait(1))
            original_wait = scans.ready.wait
            def subscribed(timeout):
                go.set()
                return original_wait(timeout)
            with patch.object(scans.ready, "wait", side_effect=subscribed):
                frame = next(scans.frames(True, stop))
            self.assertEqual(frame, "autonomous-frame")
            self.assertEqual(opened, [True])
        finally:
            go.set()
            stop.set()
            reader.join(3)

    def test_offline_replay_cannot_arm_a_nano(self):
        from contextlib import redirect_stderr
        from io import StringIO
        from lidar.autopilot.runner import parse_args
        with patch("sys.argv", ["autopilot", "--drive", "--session", "anything", "--host", "test"]), \
                redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            parse_args()


if __name__ == "__main__":
    unittest.main()
