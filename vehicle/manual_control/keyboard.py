from __future__ import annotations

import os
import sys
import time

from hardware import PARAMETERS
from vehicle.drive_data import recording
from vehicle.wifi.client import WifiController

MOTOR_VOLTAGE_STEP = 0.5
STEERING_STEP_DEGREES = 2.0
MAX_MOTOR_VOLTAGE = PARAMETERS["motor"]["nominal_supply_v"]
MIN_RUNNING_VOLTAGE = PARAMETERS["motor"]["manual_start_v"]
MAX_KEYBOARD_STEERING_DEGREES = PARAMETERS["steering"]["manual_max_center_deg"]


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
    print("Steering degrees are the equivalent center angle; Nano computes Ackermann L/R.")
    print("R shows requested/applied center angle, wheel targets, and steering limits.")
    print("P: toggle Nano LED (Wi-Fi ping) | R: status | N: network | H or ?: help")
    print("1: autonomous mode ON (AUTO START) | 0 or -: autonomous OFF + stop/centre, manual control")
    print("G: same as 1 | B: AUTO STATUS (AUTO START requires live sensors + firmware enable)")
    print("L: live Sharp sensor ADC/voltage/distance/status (read-only)")
    print("Auto steering uses fractional degrees and 1 us pulses; keyboard step stays 2 deg.")
    print("Any manual movement key takes over autonomy; Space/X/I also cancel it.")
    print("Q or Esc: safe exit")


class KeyboardCommands:
    def __init__(self):
        self.reset()

    def reset(self):
        self.motor_voltage = 0.0
        self.steering_degrees = 0.0

    def command_for(self, key: str) -> str | None:
        lower_key = key.lower()
        command: str | None = None

        if lower_key == "w":
            self.motor_voltage = step_motor_voltage(self.motor_voltage, True)
            command = f"DRIVE {self.motor_voltage:.1f} {self.steering_degrees:.1f}"
        elif lower_key == "s":
            self.motor_voltage = step_motor_voltage(self.motor_voltage, False)
            command = f"DRIVE {self.motor_voltage:.1f} {self.steering_degrees:.1f}"
        elif lower_key == "a":
            self.steering_degrees = clamp(
                self.steering_degrees + STEERING_STEP_DEGREES,
                -MAX_KEYBOARD_STEERING_DEGREES,
                MAX_KEYBOARD_STEERING_DEGREES,
            )
            command = f"DRIVE {self.motor_voltage:.1f} {self.steering_degrees:.1f}"
        elif lower_key == "d":
            self.steering_degrees = clamp(
                self.steering_degrees - STEERING_STEP_DEGREES,
                -MAX_KEYBOARD_STEERING_DEGREES,
                MAX_KEYBOARD_STEERING_DEGREES,
            )
            command = f"DRIVE {self.motor_voltage:.1f} {self.steering_degrees:.1f}"
        elif lower_key == "x" or key == " ":
            self.motor_voltage = 0.0
            command = "STOP"
        elif lower_key == "c":
            self.steering_degrees = 0.0
            command = f"DRIVE {self.motor_voltage:.1f} 0.0"
        elif lower_key in {"0", "-", "i"}:
            self.motor_voltage = 0.0
            self.steering_degrees = 0.0
            command = "IDLE"
        elif lower_key == "r":
            command = "STATUS"
        elif lower_key in {"1", "g"}:
            self.motor_voltage = 0.0
            self.steering_degrees = 0.0
            command = "AUTO START"
        elif lower_key == "b":
            command = "AUTO STATUS"
        elif lower_key == "l":
            command = "SENSORS"
        elif lower_key == "n":
            command = "NETWORK"
        elif lower_key == "p":
            command = "PING"
        return command


def run_keyboard(controller: WifiController) -> None:
    commands = KeyboardCommands()
    print_keyboard_help()
    with KeyboardInput() as keyboard:
        next_telemetry = time.monotonic() + 1.0
        while not controller.stop_event.is_set():
            key = keyboard.read(0.05)
            if key is None:
                if recording() and time.monotonic() >= next_telemetry:
                    controller.send("STATUS")
                    controller.send("SENSORS")
                    next_telemetry = time.monotonic() + 1.0
                continue
            if key.lower() == "q" or key == "\x1b":
                break
            if key.lower() in {"h", "?"}:
                print_keyboard_help()
                continue
            command = commands.command_for(key)
            if command is not None:
                controller.send(command)
                print(
                    f"> {command}  "
                    f"[voltage={commands.motor_voltage:+.1f} V, "
                    f"steer={commands.steering_degrees:+.1f} deg]"
                )
