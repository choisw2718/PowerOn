from contextlib import redirect_stdout
from io import StringIO
import threading
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from vehicle import __main__ as control
from vehicle.manual_control import keyboard


class KeyboardModeTests(unittest.TestCase):
    def run_keys(self, keys):
        class Controller:
            def __init__(self):
                self.commands = []
                self.stop_event = threading.Event()
                self.closed = False
            def send(self, command): self.commands.append(command)
            def reader_loop(self): pass
            def close_safely(self): self.closed = True; return True
        class Keyboard:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def read(self, timeout): return next(inputs)
        inputs = iter(keys)
        controller = Controller()
        args = SimpleNamespace(preview_csv=None, port=5000, host="fake-nano", sensors=False,
                               connect_timeout=5.0, ready_timeout=3.0,
                               sensor_interval=0.5, sensor_duration=0.0, rear_status=False,
                               rear_speed=None, rear_test=None, rear_duration=1, rear_steer=0)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        args.data_dir = Path(directory.name)
        with patch.object(control, "parse_args", return_value=args), \
                patch.object(control.sys.stdin, "isatty", return_value=True), \
                patch.object(control, "WifiController", return_value=controller), \
                patch.object(keyboard, "KeyboardInput", Keyboard), redirect_stdout(StringIO()):
            self.assertEqual(control.main(), 0)
        self.assertTrue(controller.closed)
        return controller.commands

    def test_one_starts_and_zero_stops_then_manual_driving_works(self):
        self.assertEqual(self.run_keys(["1", "0", "w", "a", "q"]),
                         ["AUTO START", "IDLE", "DRIVE 7.0 0.0", "DRIVE 7.0 2.0"])

    def test_transition_clears_old_manual_speed_and_steering(self):
        self.assertEqual(self.run_keys(["w", "a", "1", "0", "d", "w", "q"]),
                         ["DRIVE 7.0 0.0", "DRIVE 7.0 2.0", "AUTO START", "IDLE",
                          "DRIVE 0.0 -2.0", "DRIVE 7.0 -2.0"])

    def test_zero_also_resets_manual_mode_without_starting_autonomy(self):
        self.assertEqual(self.run_keys(["w", "a", "0", "a", "q"]),
                         ["DRIVE 7.0 0.0", "DRIVE 7.0 2.0", "IDLE", "DRIVE 0.0 2.0"])

    def test_existing_start_and_stop_shortcuts_still_work(self):
        self.assertEqual(self.run_keys(["g", "i", "1", " ", "0", "q"]),
                         ["AUTO START", "IDLE", "AUTO START", "STOP", "IDLE"])

    def test_minus_stops_and_clears_manual_input(self):
        self.assertEqual(self.run_keys(["w", "a", "1", "-", "d", "q"]),
                         ["DRIVE 7.0 0.0", "DRIVE 7.0 2.0", "AUTO START", "IDLE", "DRIVE 0.0 -2.0"])


if __name__ == "__main__":
    unittest.main()
