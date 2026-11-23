from pathlib import Path
import tempfile
import threading
from contextlib import nullcontext
import unittest
from unittest.mock import patch

from vehicle import __main__ as control
from vehicle.ir_wall_following import preview


class FakeController:
    def __init__(self):
        self.commands = []
        self.stop_event = threading.Event()

    def send(self, command):
        self.commands.append(command)


class PreviewTests(unittest.TestCase):
    def read_csv(self, content):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.csv"
            path.write_text(content, encoding="utf-8")
            return preview.load_preview_csv(str(path))

    def test_valid_csv_and_explicit_invalid_marker(self):
        self.assertEqual(
            self.read_csv("duration_s,left_mm,front_mm,right_mm\n0.2,305,750,-1\n"),
            [(0.2, 305, 750, -1)],
        )

    def test_bad_inputs_rejected_before_connection(self):
        for row in ["nan,305,750,305", "0,305,750,305", "1,0,750,305",
                    "1,305,inf,305", "1,305,7000,305", "1,99,750,305", "1,305,801,305", "1,305", "1,x,750,305"]:
            with self.subTest(row=row), self.assertRaises(ValueError):
                self.read_csv("duration_s,left_mm,front_mm,right_mm\n" + row + "\n")
        with self.assertRaises(ValueError):
            self.read_csv("duration_s,left_mm,front_mm,right_mm\n")
        with self.assertRaises(ValueError):
            self.read_csv("left,front,right\n305,750,305\n")

    def test_preview_never_sends_motion_commands(self):
        controller = FakeController()
        clock = [0.0]
        def sleep(seconds):
            clock[0] += seconds
        with patch.object(preview.time, "monotonic", lambda: clock[0]), \
                patch.object(preview.time, "sleep", sleep):
            preview.replay_preview(controller, [(0.2, 305, 750, 305), (0.2, 750, 750, 305)])
        self.assertEqual(controller.commands[0], "STOP")
        self.assertIn("AUTO TEST", controller.commands)
        self.assertNotIn("AUTO START", controller.commands)
        self.assertTrue(all(command.split()[0] in {"STOP", "RANGES", "AUTO"}
                            for command in controller.commands))
        self.assertGreaterEqual(sum(command.startswith("RANGES ") for command in controller.commands), 9)

    def test_disconnect_aborts_replay(self):
        controller = FakeController()
        controller.stop_event.set()
        with self.assertRaises(ConnectionError):
            preview.replay_preview(controller, [(0.1, 305, 750, 305)])

    def test_preview_accepts_noninteractive_input_and_always_closes(self):
        controller = FakeController()
        controller.initial_data = b""
        controller.reader_loop = lambda: None
        controller.close_safely = unittest.mock.Mock(return_value=True)
        with patch.object(control.sys, "argv", ["vehicle", "--preview-csv", "example.csv",
                                                "--host", "fake-nano"]), \
                patch.object(control, "session", return_value=nullcontext()), \
                patch.object(control, "load_preview_csv", return_value=[(0.1, 305, 750, 305)]), \
                patch.object(control.sys.stdin, "isatty", return_value=False), \
                patch.object(control, "WifiController", return_value=controller), \
                patch.object(control, "replay_preview") as replay:
            self.assertEqual(control.main(), 0)
        replay.assert_called_once()
        controller.close_safely.assert_called_once()


if __name__ == "__main__":
    unittest.main()
