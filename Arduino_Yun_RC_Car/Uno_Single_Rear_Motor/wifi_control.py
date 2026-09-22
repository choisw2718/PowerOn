"""Keyboard controller for the PowerOn ESP-01 TCP bridge."""

from __future__ import annotations

import argparse
import os
import socket
import sys
import threading
import time


DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 5000
KEEPALIVE_PERIOD_SECONDS = 0.5
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
    print("R: status | H or ?: help | Q or Esc: safe exit")


class WifiController:
    def __init__(self, host: str, port: int, connect_timeout: float) -> None:
        self.socket = socket.create_connection((host, port), timeout=connect_timeout)
        self.socket.settimeout(0.2)
        self.write_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.motor_running = threading.Event()

    def send(self, command: str) -> None:
        payload = (command.rstrip("\r\n") + "\n").encode("ascii")
        with self.write_lock:
            self.socket.sendall(payload)

    def reader_loop(self) -> None:
        pending = bytearray()
        while not self.stop_event.is_set():
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
                print("\nESP-01 closed the connection.")
                self.stop_event.set()
                return

            pending.extend(chunk)
            while b"\n" in pending:
                raw_line, _, remaining = pending.partition(b"\n")
                pending = bytearray(remaining)
                print(f"\n< {raw_line.rstrip().decode('utf-8', errors='replace')}")

    def keepalive_loop(self) -> None:
        while not self.stop_event.wait(KEEPALIVE_PERIOD_SECONDS):
            if self.motor_running.is_set():
                try:
                    self.send("KEEPALIVE")
                except OSError as error:
                    if not self.stop_event.is_set():
                        print(f"\nWi-Fi write error: {error}")
                    self.stop_event.set()
                    return

    def set_motor_running(self, voltage: float) -> None:
        if abs(voltage) > 1e-9:
            self.motor_running.set()
        else:
            self.motor_running.clear()

    def close_safely(self) -> None:
        self.motor_running.clear()
        if self.socket.fileno() < 0:
            return

        try:
            self.send("IDLE")
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control the PowerOn car through the ESP-01 Wi-Fi bridge."
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        controller = WifiController(args.host, args.port, args.connect_timeout)
    except OSError as error:
        print(f"Could not connect to {args.host}:{args.port}: {error}")
        print("Connect the laptop to the PowerOn-Car Wi-Fi network first.")
        return 1

    print(f"Connected to ESP-01 at {args.host}:{args.port}.")
    reader = threading.Thread(target=controller.reader_loop, daemon=True)
    keepalive = threading.Thread(target=controller.keepalive_loop, daemon=True)
    reader.start()
    keepalive.start()

    motor_voltage = 0.0
    steering_degrees = 0.0
    print_keyboard_help()

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
                elif lower_key in {"h", "?"}:
                    print_keyboard_help()
                elif lower_key == "q" or key == "\x1b":
                    break

                if command is not None:
                    controller.send(command)
                    controller.set_motor_running(motor_voltage)
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
        return 1
    finally:
        controller.close_safely()

    print("Disconnected after requesting IDLE (motor stop + steering center).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
