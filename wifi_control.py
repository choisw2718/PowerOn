"""Keyboard controller for the PowerOn Nano 33 IoT on the local Wi-Fi."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import socket
import sys
import threading
import time


DEFAULT_PORT = 5000
DISCOVERY_PORT = 5001
DISCOVERY_REQUEST = b"POWERON_DISCOVER"
READY_BANNER = b"READY PowerOn Nano 33 IoT"
MOTOR_VOLTAGE_STEP = 0.5
STEERING_STEP_DEGREES = 2.0
MAX_MOTOR_VOLTAGE = 12.0
MIN_RUNNING_VOLTAGE = 7.0
MAX_KEYBOARD_STEERING_DEGREES = 30.0


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def step_motor_voltage(current: float, increase: bool) -> float:
    """Step voltage without commanding the measured motor dead zone."""
    if increase:
        if current < -MIN_RUNNING_VOLTAGE:
            return min(current + MOTOR_VOLTAGE_STEP, -MIN_RUNNING_VOLTAGE)
        if current < 0.0:
            return 0.0
        if current == 0.0:
            return MIN_RUNNING_VOLTAGE
        return min(current + MOTOR_VOLTAGE_STEP, MAX_MOTOR_VOLTAGE)

    if current > MIN_RUNNING_VOLTAGE:
        return max(current - MOTOR_VOLTAGE_STEP, MIN_RUNNING_VOLTAGE)
    if current > 0.0:
        return 0.0
    if current == 0.0:
        return -MIN_RUNNING_VOLTAGE
    return max(current - MOTOR_VOLTAGE_STEP, -MAX_MOTOR_VOLTAGE)


class KeyboardInput:
    """Read one key without Enter on Windows, macOS, or Linux terminals."""

    def __init__(self) -> None:
        self.is_windows = os.name == "nt"
        self.file_descriptor: int | None = None
        self.saved_terminal_settings: list[object] | None = None

    def __enter__(self) -> KeyboardInput:
        if self.is_windows:
            return self

        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard control requires an interactive terminal.")

        import termios
        import tty

        self.file_descriptor = sys.stdin.fileno()
        self.saved_terminal_settings = termios.tcgetattr(self.file_descriptor)
        tty.setcbreak(self.file_descriptor)
        return self

    def __exit__(self, *_: object) -> None:
        if self.is_windows or self.file_descriptor is None:
            return

        import termios

        if self.saved_terminal_settings is not None:
            termios.tcsetattr(
                self.file_descriptor,
                termios.TCSADRAIN,
                self.saved_terminal_settings,
            )

    def read(self, timeout_seconds: float) -> str | None:
        if self.is_windows:
            import msvcrt

            deadline = time.monotonic() + timeout_seconds
            while time.monotonic() < deadline:
                if msvcrt.kbhit():
                    key = msvcrt.getwch()
                    if key in {"\x00", "\xe0"}:
                        msvcrt.getwch()
                        return None
                    return key
                time.sleep(0.01)
            return None

        import select

        readable, _, _ = select.select([sys.stdin], [], [], timeout_seconds)
        return sys.stdin.read(1) if readable else None


def print_keyboard_help() -> None:
    print(
        f"W/S: 0 V <-> +/-{MIN_RUNNING_VOLTAGE:.1f} V, then "
        f"{MOTOR_VOLTAGE_STEP:.1f} V steps | "
        f"A/D: steering left/right {STEERING_STEP_DEGREES:.0f} deg"
    )
    print("Space or X: motor stop | C: steering center | I: stop + center")
    print("P: toggle Nano LED (Wi-Fi ping) | R: status | N: network | H or ?: help")
    print("Q or Esc: safe exit")


class WifiController:
    def __init__(
        self, host: str, port: int, connect_timeout: float, ready_timeout: float = 3.0
    ) -> None:
        self.socket = socket.create_connection((host, port), timeout=connect_timeout)
        try:
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # WiFiNINA's server.accept() can wait for the first client payload.
            # STATUS is read-only and makes the Nano accept and send READY.
            self.socket.sendall(b"STATUS\n")
            self.initial_data = self._receive_ready_banner(ready_timeout)
            self.socket.settimeout(0.2)
        except (OSError, ValueError):
            self.socket.close()
            raise
        self.write_lock = threading.Lock()
        self.stop_event = threading.Event()

    def _receive_ready_banner(self, timeout_seconds: float) -> bytes:
        """A TCP handshake alone does not mean the Nano accepted this controller."""
        deadline = time.monotonic() + timeout_seconds
        pending = bytearray()
        while b"\n" not in pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    "Nano did not send READY; another controller may already be connected."
                )
            self.socket.settimeout(remaining)
            try:
                chunk = self.socket.recv(512)
            except socket.timeout as error:
                raise TimeoutError(
                    "Nano did not send READY; another controller may already be connected."
                ) from error
            if not chunk:
                raise ConnectionError("Nano closed the connection before sending READY.")
            pending.extend(chunk)
            if len(pending) > 1024:
                raise ConnectionError("The TCP server did not send the Nano READY banner.")
        banner = bytes(pending).split(b"\n", 1)[0].rstrip(b"\r")
        if banner != READY_BANNER:
            raise ConnectionError(
                f"Unexpected TCP greeting: {banner.decode('utf-8', errors='replace')!r}"
            )
        return bytes(pending)

    def send(self, command: str) -> None:
        payload = (command.rstrip("\r\n") + "\n").encode("ascii")
        with self.write_lock:
            self.socket.sendall(payload)

    def reader_loop(self) -> None:
        pending = bytearray(self.initial_data)
        while not self.stop_event.is_set():
            while b"\n" in pending:
                raw_line, _, remaining = pending.partition(b"\n")
                pending = bytearray(remaining)
                print(f"\n< {raw_line.rstrip().decode('utf-8', errors='replace')}")
            try:
                chunk = self.socket.recv(512)
            except socket.timeout:
                continue
            except OSError as error:
                if not self.stop_event.is_set():
                    print(f"\nWi-Fi read error: {error}")
                self.stop_event.set()
                return

            if not chunk:
                print("\nNano 33 IoT closed the connection.")
                self.stop_event.set()
                return

            pending.extend(chunk)

    def close_safely(self) -> bool:
        if self.socket.fileno() < 0:
            return False

        idle_sent = False
        try:
            if not self.stop_event.is_set():
                self.send("IDLE")
                idle_sent = True
                time.sleep(0.05)
        except OSError as error:
            print(f"Could not send final IDLE: {error}")
        finally:
            self.stop_event.set()
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.socket.close()
        return idle_sent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control the PowerOn car through Nano 33 IoT Wi-Fi."
    )
    parser.add_argument("--host", help="Nano IP address; discovered automatically if omitted")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--ready-timeout", type=float, default=3.0)
    return parser.parse_args()


def configured_ssid() -> str | None:
    try:
        header = Path(__file__).with_name("wifi_secrets.h").read_text(encoding="utf-8")
    except (OSError, UnicodeError):
        return None
    match = re.search(r'^\s*#define\s+WIFI_SSID\s+"([^"]+)"', header, re.MULTILINE)
    return match.group(1) if match else None


def discover_host(port: int, timeout_seconds: float = 3.0) -> str | None:
    expected = f"POWERON_NANO {port}".encode("ascii")
    deadline = time.monotonic() + timeout_seconds
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            probe.bind(("", 0))
            while time.monotonic() < deadline:
                probe.sendto(DISCOVERY_REQUEST, ("255.255.255.255", DISCOVERY_PORT))
                probe.settimeout(min(0.5, max(0.01, deadline - time.monotonic())))
                try:
                    message, address = probe.recvfrom(64)
                except socket.timeout:
                    continue
                if message.strip() == expected:
                    return address[0]
    except OSError as error:
        raise OSError(f"UDP discovery could not run: {error}") from error
    return None


def main() -> int:
    args = parse_args()
    if not 0 < args.port < 65536 or args.connect_timeout <= 0 or args.ready_timeout <= 0:
        print("Port must be 1-65535 and connection timeouts must be positive.")
        return 2
    if not sys.stdin.isatty():
        print("Keyboard control requires an interactive terminal.")
        return 2
    if args.host:
        host = args.host
    else:
        print(f"Searching for Nano by UDP on port {DISCOVERY_PORT}...")
        try:
            host = discover_host(args.port)
        except OSError as error:
            print(error)
            host = None
    if host is None:
        ssid = configured_ssid()
        network = f"'{ssid}'" if ssid else "the configured Wi-Fi"
        print(f"Nano not found on {network}.")
        print("No UDP reply does not prove the Nano's TCP connection is down.")
        print("Check the Nano's READY WIFI=... IP=... line, or pass --host <Nano IP>.")
        return 1
    try:
        controller = WifiController(host, args.port, args.connect_timeout, args.ready_timeout)
    except OSError as error:
        print(f"Could not establish a Nano control session at {host}:{args.port}: {error}")
        print("Use the Nano's IP from the READY line, not this computer's IP.")
        return 1

    print(f"Connected to Nano 33 IoT at {host}:{args.port}.")
    reader = threading.Thread(target=controller.reader_loop, daemon=True)
    reader.start()

    motor_voltage = 0.0
    steering_degrees = 0.0
    print_keyboard_help()

    result = 0
    try:
        with KeyboardInput() as keyboard:
            while not controller.stop_event.is_set():
                key = keyboard.read(0.05)
                if key is None:
                    continue

                lower_key = key.lower()
                command: str | None = None

                if lower_key == "w":
                    motor_voltage = step_motor_voltage(motor_voltage, True)
                    command = f"DRIVE {motor_voltage:.1f} {steering_degrees:.1f}"
                elif lower_key == "s":
                    motor_voltage = step_motor_voltage(motor_voltage, False)
                    command = f"DRIVE {motor_voltage:.1f} {steering_degrees:.1f}"
                elif lower_key == "a":
                    steering_degrees = clamp(
                        steering_degrees + STEERING_STEP_DEGREES,
                        -MAX_KEYBOARD_STEERING_DEGREES,
                        MAX_KEYBOARD_STEERING_DEGREES,
                    )
                    command = f"DRIVE {motor_voltage:.1f} {steering_degrees:.1f}"
                elif lower_key == "d":
                    steering_degrees = clamp(
                        steering_degrees - STEERING_STEP_DEGREES,
                        -MAX_KEYBOARD_STEERING_DEGREES,
                        MAX_KEYBOARD_STEERING_DEGREES,
                    )
                    command = f"DRIVE {motor_voltage:.1f} {steering_degrees:.1f}"
                elif lower_key == "x" or key == " ":
                    motor_voltage = 0.0
                    command = "STOP"
                elif lower_key == "c":
                    steering_degrees = 0.0
                    command = f"DRIVE {motor_voltage:.1f} 0.0"
                elif lower_key == "i":
                    motor_voltage = 0.0
                    steering_degrees = 0.0
                    command = "IDLE"
                elif lower_key == "r":
                    command = "STATUS"
                elif lower_key == "n":
                    command = "NETWORK"
                elif lower_key == "p":
                    command = "PING"
                elif lower_key in {"h", "?"}:
                    print_keyboard_help()
                elif lower_key == "q" or key == "\x1b":
                    break

                if command is not None:
                    controller.send(command)
                    print(
                        f"> {command}  "
                        f"[voltage={motor_voltage:+.1f} V, "
                        f"steer={steering_degrees:+.1f} deg]"
                    )
    except (EOFError, KeyboardInterrupt, RuntimeError) as error:
        if str(error):
            print(f"\n{error}")
        else:
            print()
    except OSError as error:
        print(f"Wi-Fi error: {error}")
        result = 1
    finally:
        disconnected = controller.stop_event.is_set()
        idle_sent = controller.close_safely()

    if disconnected:
        print("Nano control connection was lost.")
        return 1
    if idle_sent:
        print("Disconnected after requesting IDLE (motor stop + steering center).")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
