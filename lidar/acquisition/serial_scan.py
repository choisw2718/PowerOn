"""RPLIDAR A1 USB reader, also runnable on the Pi over an SSH channel."""

from contextlib import closing, suppress
import errno
import os
import select
import sys
import threading
import time

import serial
from serial.tools import list_ports


DEFAULT_PI_PORT = '/dev/ttyUSB0'
CP210X_USB_ID = (0x10C4, 0xEA60)
BAUD_RATE = 115200
UART_BYTE_S = 10 / BAUD_RATE


class ScanInterrupted(OSError):
    """A sensor or transport interruption that a new scan session can recover from."""


class LidarPort:
    def __init__(self, requested):
        self.requested = requested
        self.serial_number = None
        self.location = None

    def resolve(self):
        if self.requested.startswith(('/dev/serial/by-id/', '/dev/serial/by-path/')):
            return self.requested
        ports = [port for port in list_ports.comports() if (port.vid, port.pid) == CP210X_USB_ID]
        if self.serial_number or self.location:
            matches = [port for port in ports
                       if (port.serial_number == self.serial_number if self.serial_number
                           else port.location == self.location)]
            if len(matches) > 1 and self.location:
                matches = [port for port in matches if port.location == self.location]
            if len(matches) != 1:
                raise ScanInterrupted('기존 라이다 USB 어댑터를 하나로 식별할 수 없습니다.')
            self.location = matches[0].location or self.location
            return matches[0].device

        selected = next((port for port in ports if port.device.casefold() == self.requested.casefold()), None)
        if self.requested == DEFAULT_PI_PORT:
            if len(ports) > 1:
                raise ScanInterrupted('CP210x 어댑터가 여러 개입니다. /dev/serial/by-id/ 포트를 지정하세요.')
            if ports:
                selected = ports[0]
        if selected is not None:
            self.serial_number, self.location = selected.serial_number, selected.location
            return selected.device
        return self.requested


def reconnecting_scans(open_session, stop, on_retry, *, deadline=float('inf')):
    """Use a fresh decoder via open_session and retry only classified interruptions."""
    delay = 2
    while not stop.is_set() and time.monotonic() < deadline:
        try:
            with closing(open_session()) as session:
                for item in session:
                    if stop.is_set() or time.monotonic() >= deadline:
                        return
                    delay = 2
                    yield item
            return
        except ScanInterrupted as error:
            remaining = deadline - time.monotonic()
            if stop.is_set() or remaining <= 0:
                return
            on_retry(delay, error)
            if stop.wait(min(delay, remaining)):
                return
            delay = min(delay * 2, 15)


class ScanDecoder:
    def __init__(self):
        self.pending = bytearray()
        self.skipped_bytes = 0

    def feed(self, chunk, *, with_offsets=False):
        self.pending.extend(chunk)
        offset = 0
        while len(self.pending) - offset >= 5:
            a, b, c, d, e = self.pending[offset:offset + 5]
            angle = ((b | (c << 8)) >> 1) / 64.0
            if (a & 1) == ((a >> 1) & 1) or not (b & 1) or angle >= 360:
                offset += 1
                self.skipped_bytes += 1
                continue
            offset += 5
            packet = bool(a & 1), angle, (d | (e << 8)) / 4.0, a >> 2
            yield (*packet, len(self.pending) - offset) if with_offsets else packet
        del self.pending[:offset]


def scan_chunks(port, stop):
    device = serial.Serial(port=None, baudrate=BAUD_RATE, timeout=0.15,
                           write_timeout=1, exclusive=True)
    device.port = port
    device.dtr = False
    try:
        device.open()
        device.write(b'\xa5\x25')
        if stop.wait(0.2):
            return
        device.reset_input_buffer()
        device.write(b'\xa5\x20')
        header = bytearray()
        deadline = time.monotonic() + 3
        while len(header) < 7 and time.monotonic() < deadline and not stop.is_set():
            header.extend(device.read(7 - len(header)))
        if stop.is_set():
            return
        if header != bytes.fromhex('A5 5A 05 00 00 40 81'):
            raise ScanInterrupted('스캔 응답 없음 · USB 연결과 포트 사용 여부를 확인하세요.')
        while not stop.is_set():
            yield device.read(min(max(device.in_waiting, 1), 4096))
    except OSError as error:
        if isinstance(error, ScanInterrupted):
            raise
        if os.name != 'nt' and error.errno in (errno.EACCES, errno.EPERM):
            raise PermissionError(f'{port} 접근 권한을 확인하세요.') from error
        raise ScanInterrupted(str(error)) from error
    finally:
        if device.is_open:
            try:
                device.write(b'\xa5\x25')
                if stop.is_set():
                    device.flush()
            except (serial.SerialException, OSError):
                pass
            finally:
                with suppress(OSError):
                    device.close()


def stream_to_ssh(port):
    stop = threading.Event()

    def wait_for_disconnect():
        try:
            fd = sys.stdin.fileno()
            while select.select([fd], [], [], 8)[0] and os.read(fd, 4096):
                pass
        finally:
            stop.set()

    threading.Thread(target=wait_for_disconnect, daemon=True).start()
    try:
        output = sys.stdout.fileno()
        os.set_blocking(output, False)
        with closing(scan_chunks(LidarPort(port).resolve(), stop)) as chunks:
            for chunk in chunks:
                remaining = memoryview(chunk)
                while remaining and not stop.is_set():
                    if select.select([], [output], [], 0.15)[1]:
                        try:
                            remaining = remaining[os.write(output, remaining):]
                        except BlockingIOError:
                            pass
    except BrokenPipeError:
        return 0
    except PermissionError as error:
        print(error, file=sys.stderr)
        return 1
    except (OSError, RuntimeError) as error:
        print(error, file=sys.stderr)
        return 75
    return 0


if __name__ == '__main__':
    sys.exit(stream_to_ssh(sys.argv[1]))
