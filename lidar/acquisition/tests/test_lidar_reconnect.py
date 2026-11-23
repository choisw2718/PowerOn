from contextlib import closing
from pathlib import Path
import queue
import shlex
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from lidar.acquisition.serial_scan import DEFAULT_PI_PORT, LidarPort, ScanInterrupted, reconnecting_scans
from lidar.acquisition.ssh import ssh_scan_chunks
from lidar.radar.window import read_scan


def adapter(device, serial="0001", location="1-2"):
    return SimpleNamespace(device=device, vid=0x10C4, pid=0xEA60,
                           serial_number=serial, location=location)


class ImmediateStop(threading.Event):
    def wait(self, timeout=None):
        return self.is_set()


class LidarReconnectTest(unittest.TestCase):
    def test_usb_number_can_change_for_the_same_adapter(self):
        port = LidarPort("COM19")
        with patch("lidar.acquisition.serial_scan.list_ports.comports", side_effect=[
            [adapter("COM19")],
            [adapter("COM19", serial="other")],
            [adapter("COM19", serial="other"), adapter("COM27")],
        ]):
            self.assertEqual(port.resolve(), "COM19")
            with self.assertRaises(ScanInterrupted):
                port.resolve()
            self.assertEqual(port.resolve(), "COM27")

    def test_pi_default_selects_unique_adapter_and_rejects_ambiguity(self):
        with patch("lidar.acquisition.serial_scan.list_ports.comports", return_value=[adapter("/dev/ttyUSB2")]):
            self.assertEqual(LidarPort(DEFAULT_PI_PORT).resolve(), "/dev/ttyUSB2")
        with patch("lidar.acquisition.serial_scan.list_ports.comports", return_value=[
            adapter("/dev/ttyUSB0"), adapter("/dev/ttyUSB1", location="1-3")
        ]):
            with self.assertRaises(ScanInterrupted):
                LidarPort(DEFAULT_PI_PORT).resolve()

    def test_retry_backoff_resets_only_after_data_and_stop_cancels_retries(self):
        stop = ImmediateStop()
        delays = []
        attempts = 0

        def open_session():
            nonlocal attempts
            attempts += 1
            if attempts == 5:
                yield "scan"
            raise ScanInterrupted("disconnect")

        def retry(delay, _error):
            delays.append(delay)
            if len(delays) == 5:
                stop.set()

        self.assertEqual(list(reconnecting_scans(open_session, stop, retry)), ["scan"])
        self.assertEqual(delays, [2, 4, 8, 15, 2])
        self.assertEqual(attempts, 5)

    def test_configuration_errors_do_not_retry(self):
        def bad_ssh():
            raise RuntimeError("authentication failed")
            yield

        with self.assertRaisesRegex(RuntimeError, "authentication"):
            list(reconnecting_scans(bad_ssh, ImmediateStop(), lambda *_: self.fail("Retried")))

    def test_ssh_stream_uses_serial_module_and_sends_heartbeat(self):
        client = MagicMock()
        client.__enter__.return_value = client
        transport = client.get_transport.return_value
        channel = transport.open_session.return_value
        channel.__enter__.return_value = channel
        channel.exit_status_ready.side_effect = [False, True]
        channel.recv.return_value = b"scan data"
        backend = SimpleNamespace(SSHClient=lambda: client, SSHException=OSError)

        with patch.dict("sys.modules", {"paramiko": backend}), closing(
            ssh_scan_chunks("user@pi", DEFAULT_PI_PORT, None, threading.Event())
        ) as chunks:
            self.assertEqual(next(chunks), b"scan data")

        command = shlex.split(channel.exec_command.call_args.args[0])
        self.assertEqual(command[:3], ["python3", "-u", "-c"])
        self.assertEqual(command[4:], [DEFAULT_PI_PORT])
        source_path = Path(__file__).resolve().parents[1] / "serial_scan.py"
        self.assertEqual(command[3], source_path.read_text(encoding="utf-8"))
        channel.sendall.assert_called_once_with(b".")
        channel.shutdown_write.assert_called_once()

    def test_radar_resumes_on_new_usb_number_without_old_partial_packet(self):
        stop = ImmediateStop()
        messages = queue.Queue()
        marker = bytes((0x65, 1, 0, 0xA0, 0x0F))
        ports = []

        def open_sensor(port, _stop):
            ports.append(port)
            if port == "COM19":
                yield marker + marker[:3]
                raise ScanInterrupted("USB unplugged")
            yield marker
            stop.set()

        with patch("lidar.acquisition.serial_scan.list_ports.comports", side_effect=[
            [adapter("COM19")], [adapter("COM27")]
        ]), patch("lidar.radar.window.scan_chunks", open_sensor):
            read_scan("COM19", stop, messages)

        received = list(messages.queue)
        self.assertEqual(ports, ["COM19", "COM27"])
        self.assertEqual([kind for kind, _ in received],
                         ["status", "points", "reset", "status", "points", "done"])
        self.assertEqual(received[4][1][1], [(0.0, 1000.0, 25)])
