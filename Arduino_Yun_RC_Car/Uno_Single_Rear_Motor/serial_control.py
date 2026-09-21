"""Laptop-side USB Serial controller for Uno_Single_Rear_Motor.ino."""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import serial
from serial.tools import list_ports


BAUD_RATE = 115_200
KEEPALIVE_PERIOD_SECONDS = 0.5
MOTOR_VOLTAGE_STEP = 0.5
STEERING_STEP_DEGREES = 2.0
MAX_MOTOR_VOLTAGE = 12.0
MIN_RUNNING_VOLTAGE = 7.0
MAX_KEYBOARD_STEERING_DEGREES = 30.0


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def step_motor_voltage(current: float, increase: bool) -> float:
    """Step voltage while never commanding the measured motor dead zone."""
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
                        # Discard the second byte of function/arrow keys.
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


class SerialController:
    def __init__(self, port: str, baud_rate: int) -> None:
        self.serial = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=0.1,
            write_timeout=0.5,
        )
        self.write_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.motor_running = threading.Event()

    def send(self, command: str) -> None:
        payload = (command.rstrip("\r\n") + "\n").encode("ascii")
        with self.write_lock:
            self.serial.write(payload)
            self.serial.flush()

    def reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                raw = self.serial.readline()
            except serial.SerialException as error:
                print(f"\nSerial read error: {error}")
                self.stop_event.set()
                return
            if raw:
                print(f"\n< {raw.decode('utf-8', errors='replace').rstrip()}")

    def keepalive_loop(self) -> None:
        while not self.stop_event.wait(KEEPALIVE_PERIOD_SECONDS):
            if self.motor_running.is_set():
                try:
                    self.send("KEEPALIVE")
                except serial.SerialException as error:
                    print(f"\nSerial write error: {error}")
                    self.stop_event.set()
                    return

    def update_motor_state(self, command: str) -> None:
        words = command.upper().split()
        if not words:
            return
        if words[0] in {"STOP", "IDLE"}:
            self.motor_running.clear()
            return
        if len(words) != 2 or words[0] not in {"VOLTAGE", "MOTOR"}:
            return
        try:
            nonzero = abs(float(words[1])) > 1e-9
        except ValueError:
            return
        if nonzero:
            self.motor_running.set()
        else:
            self.motor_running.clear()

    def close_safely(self) -> None:
        self.motor_running.clear()
        if self.serial.is_open:
            try:
                self.send("IDLE")
                time.sleep(0.05)
            except serial.SerialException as error:
                print(f"Could not send final IDLE: {error}")
            finally:
                self.stop_event.set()
                self.serial.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control one rear DC motor and two steering servos over USB Serial."
    )
    parser.add_argument("port", nargs="?", help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument(
        "--list",
        action="store_true",
        help="list detected serial ports and exit",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list:
        ports = available_ports()
        print("\n".join(ports) if ports else "No serial ports detected.")
        return 0
    if not args.port:
        print("A serial port is required. Example: py serial_control.py COM5")
        print("Detected:", ", ".join(available_ports()) or "none")
        return 2

    try:
        controller = SerialController(args.port, args.baud)
    except serial.SerialException as error:
        print(f"Could not open {args.port}: {error}")
        return 1
    print(f"Connected to {args.port} at {args.baud} baud.")
    print("Waiting for the Uno reset after opening the port...")
    time.sleep(2.0)

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
                    command = f"VOLTAGE {motor_voltage:.1f}"
                elif lower_key == "s":
                    motor_voltage = step_motor_voltage(motor_voltage, False)
                    command = f"VOLTAGE {motor_voltage:.1f}"
                elif lower_key == "a":
                    steering_degrees = clamp(
                        steering_degrees + STEERING_STEP_DEGREES,
                        -MAX_KEYBOARD_STEERING_DEGREES,
                        MAX_KEYBOARD_STEERING_DEGREES,
                    )
                    command = f"STEER {steering_degrees:.1f}"
                elif lower_key == "d":
                    steering_degrees = clamp(
                        steering_degrees - STEERING_STEP_DEGREES,
                        -MAX_KEYBOARD_STEERING_DEGREES,
                        MAX_KEYBOARD_STEERING_DEGREES,
                    )
                    command = f"STEER {steering_degrees:.1f}"
                elif lower_key == "x" or key == " ":
                    motor_voltage = 0.0
                    command = "STOP"
                elif lower_key == "c":
                    steering_degrees = 0.0
                    command = "CENTER"
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
                    controller.update_motor_state(command)
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
        print()
    except serial.SerialException as error:
        print(f"Serial error: {error}")
        return 1
    finally:
        controller.close_safely()

    print("Disconnected after sending IDLE (motor stop + steering center).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
