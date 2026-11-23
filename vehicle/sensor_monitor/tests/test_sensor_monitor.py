from contextlib import redirect_stdout
from io import StringIO
import socket
import unittest
from unittest.mock import patch

from vehicle.sensor_monitor import monitor


def sensor_line(status="in_range", mm="205.0"):
    fields = "SENSORS model=GP2Y0A21YK0F pins=A0,A1,A2 available=1 age_ms=12"
    for name in ("left", "front", "right"):
        fields += (f" {name}_adc=807 {name}_sensor_V=1.300 {name}_mm={mm}"
                   f" {name}_status={status} {name}_usable={int(status == 'in_range')}")
    return fields


class MonitorTests(unittest.TestCase):
    def test_format_three_channels_and_centimetres(self):
        formatted = monitor.format_sensor_reading(sensor_line())
        self.assertIn("20.5 cm", formatted)
        self.assertIn("1.300 V", formatted)
        for channel in ("LEFT / A0", "FRONT / A1", "RIGHT / A2"):
            self.assertIn(channel, formatted)

    def test_unknown_range_is_not_shown_as_measured_80_cm(self):
        formatted = monitor.format_sensor_reading(sensor_line("far_or_unknown", "800.0"))
        self.assertIn("far_or_unknown", formatted)
        self.assertNotIn("80.0 cm", formatted)
        self.assertIn("--", formatted)

    def test_pin_voltage_and_actual_divider_are_visible(self):
        message = sensor_line()
        for name in ("left", "front", "right"):
            message += f" {name}_adc_V=1.300 {name}_divider=1.000"
        formatted = monitor.format_sensor_reading(message)
        self.assertIn("ADC PIN", formatted)
        self.assertIn("left=1.000", formatted)
        self.assertIn("front=1.000", formatted)
        self.assertIn("right=1.000", formatted)

    def test_older_firmware_does_not_invent_a_divider_setting(self):
        self.assertIn("left=unknown", monitor.format_sensor_reading(sensor_line()))

    def test_disconnected_channels_show_no_measurements(self):
        formatted = monitor.format_sensor_reading(sensor_line("not_connected", "0"))
        self.assertIn("not_connected", formatted)
        self.assertNotIn("1.300 V", formatted)
        self.assertNotIn("807", formatted)
        self.assertNotIn("0.0 cm", formatted)

    def test_malformed_fields_are_rejected(self):
        for message in ("SENSORS model=other", sensor_line().replace("left_adc=807", "left_adc=9000"),
                        sensor_line().replace("left_mm=205.0", "left_mm=nan")):
            with self.subTest(message=message), self.assertRaises(ValueError):
                monitor.format_sensor_reading(message)

    def run_monitor(self, chunks):
        clock = [0.0]
        class Event:
            def wait(self, seconds): clock[0] += seconds
            def set(self): pass
        class Socket:
            def recv(self, size): return chunks.pop(0) if chunks else b""
        class Controller:
            initial_data = b"READY PowerOn Nano 33 IoT\nSTATUS enabled=0\n"
            stop_event = Event()
            socket = Socket()
            commands = []
            def send(self, command): self.commands.append(command)
        controller = Controller()
        with patch.object(monitor.time, "monotonic", lambda: clock[0]), redirect_stdout(StringIO()) as output:
            monitor.monitor_sensors(controller, interval=0.5, duration=0.5)
        return controller.commands, output.getvalue()

    def test_fragmented_response_and_only_sensor_queries(self):
        response = sensor_line().encode() + b"\r\n"
        commands, output = self.run_monitor([b"OK STOP\n" + response[:70], response[70:]])
        self.assertEqual(commands, ["STOP", "SENSORS"])
        self.assertIn("20.5 cm", output)

    def test_old_firmware_gives_upload_instruction(self):
        with self.assertRaisesRegex(RuntimeError, "Upload the current PowerOnNano.ino"):
            self.run_monitor([b"OK STOP\nERR bad command; send HELP\n"])

    def test_disconnect_is_reported(self):
        with self.assertRaises(ConnectionError):
            self.run_monitor([b""])


if __name__ == "__main__":
    unittest.main()
