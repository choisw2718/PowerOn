"""Keyboard serial controller for the STM32 vehicle.

Example:
    python tools/keyboard_drive.py              (auto-detects the ST-LINK COM port)
    python tools/keyboard_drive.py --port COM5 --baud 115200

Keys:
    W speed up
    S speed down / reverse
    A steer left
    D steer right
    C center steering
    Z speed zero
    X or Space stop rear motors and hold steering
    I idle: stop rear motors and center steering
    H help
    Q quit this PC program

Programmatic command example:
    python tools/keyboard_drive.py --port COM5 --command "@DRIVE 0.30 10"
"""

from __future__ import annotations

import argparse
import re
import sys
import time
import threading
from pathlib import Path


def read_config_define(name: str, fallback: float) -> float:
    root = Path(__file__).resolve().parents[1]
    header = root / "Config" / "steering_servo_hardware.h"
    try:
        text = header.read_text(encoding="utf-8")
    except OSError:
        return fallback

    match = re.search(rf"#define\s+{name}\s+([0-9.+\-eE]+)f?", text)
    if not match:
        return fallback
    return float(match.group(1))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_line_command(command: str, max_speed: float, min_steer: float, max_steer: float) -> str:
    prefix = "@" if not command.startswith(("@", ":")) else command[0]
    text = command[1:] if command.startswith(("@", ":")) else command
    parts = text.strip().split()
    if not parts:
        return prefix

    verb = parts[0].upper()
    try:
        if verb in ("DRIVE", "D") and len(parts) >= 3:
            speed = clamp(float(parts[1]), -max_speed, max_speed)
            steer = clamp(float(parts[2]), min_steer, max_steer)
            return f"{prefix}DRIVE {speed:.3f} {steer:.2f}"
        if verb in ("SPEED", "V") and len(parts) >= 2:
            speed = clamp(float(parts[1]), -max_speed, max_speed)
            return f"{prefix}SPEED {speed:.3f}"
        if verb in ("STEER", "S") and len(parts) >= 2:
            steer = clamp(float(parts[1]), min_steer, max_steer)
            return f"{prefix}STEER {steer:.2f}"
    except ValueError:
        return command

    return command


def read_key_windows() -> str:
    import msvcrt

    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0"):
        msvcrt.getch()
        return ""
    return ch.decode(errors="ignore")


def read_key_posix() -> str:
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def read_key() -> str:
    if sys.platform.startswith("win"):
        return read_key_windows()
    return read_key_posix()


STLINK_VID_PID = (0x0483, 0x374B)


def autodetect_port() -> str:
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    matches = [
        p for p in ports
        if (p.vid, p.pid) == STLINK_VID_PID
        or "stlink" in (p.description or "").lower()
        or "st-link" in (p.description or "").lower()
    ]
    if len(matches) == 1:
        print(f"Auto-detected ST-LINK port: {matches[0].device} ({matches[0].description})")
        return matches[0].device
    if not ports:
        raise SystemExit("No serial ports found. Is the NUCLEO board plugged in over USB?")
    listing = "\n".join(f"  {p.device}: {p.description}" for p in ports)
    if not matches:
        raise SystemExit(
            f"No ST-LINK port found. Pass --port explicitly. Available ports:\n{listing}"
        )
    raise SystemExit(
        f"Multiple ST-LINK ports found; pass --port explicitly:\n{listing}"
    )


def main() -> int:
    default_min_steer = read_config_define("STEERING_SERVO_SAFE_MIN_STEER_DEG", -45.0)
    default_max_steer = read_config_define("STEERING_SERVO_SAFE_MAX_STEER_DEG", 55.0)

    parser = argparse.ArgumentParser(description="Drive STM32 vehicle over UART.")
    parser.add_argument("--port", help="Serial port, for example COM5 (default: auto-detect ST-LINK)")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--command", help="Send one line command and exit")
    parser.add_argument("--speed-step", type=float, default=0.05, help="Speed step in m/s")
    parser.add_argument("--steer-step", type=float, default=5.0, help="Steering step in degrees")
    parser.add_argument("--max-speed", type=float, default=0.60, help="Maximum absolute speed in m/s")
    parser.add_argument("--min-steer", type=float, default=default_min_steer, help="Minimum steering angle in degrees")
    parser.add_argument("--max-steer", type=float, default=default_max_steer, help="Maximum steering angle in degrees")
    parser.add_argument("--heartbeat", type=float, default=0.20,
                        help="Command resend period in seconds; 0 disables resend")
    parser.add_argument("--log-tx", action="store_true",
                        help="Print every serial command sent by this program")
    parser.add_argument("--line-mode", action="store_true",
                        help="Use Enter-based commands instead of single-key input")
    args = parser.parse_args()
    if args.min_steer > args.max_steer:
        parser.error("--min-steer must not exceed --max-steer")

    try:
        import serial
    except ImportError:
        print("pyserial is not installed. Install it with: python -m pip install pyserial")
        return 2

    port = args.port or autodetect_port()

    with serial.Serial(port, args.baud, timeout=0.05) as dev:
        time.sleep(0.2)

        if args.command:
            command = normalize_line_command(args.command, args.max_speed, args.min_steer, args.max_steer)
            if not command.startswith(("@", ":")):
                command = "@" + command
            dev.write((command.rstrip() + "\n").encode("ascii"))
            print(f"sent: {command}")
            # A one-shot command used to close the port before the firmware's
            # acknowledgement or STATUS text arrived, hiding parser and stale
            # firmware failures. Collect the response before exiting.
            deadline = time.monotonic() + 0.5
            response = bytearray()
            while time.monotonic() < deadline:
                waiting = dev.in_waiting
                if waiting:
                    response.extend(dev.read(waiting))
                    deadline = time.monotonic() + 0.1
                else:
                    time.sleep(0.01)
            if response:
                print(response.decode(errors="replace"), end="")
            else:
                print("No STM32 response. Check firmware version, COM port, reset, and 115200 baud.")
            return 0

        # Firmware defaults to echoing input and acking every line, which is
        # right for a human terminal but floods this tool's 200 ms heartbeat.
        dev.write(b"@ECHO 0\n@ACK 0\n")
        time.sleep(0.1)
        dev.read(dev.in_waiting or 0)

        dev.write(b"@STATUS\n")
        time.sleep(0.2)
        initial = dev.read(dev.in_waiting or 1).decode(errors="replace")

        # The replacement RB35GM gear ratio, actual counts/rev, A/B signs, and
        # PID gains must be verified before speed-based motion is safe. Refuse
        # W/S driving unless the firmware explicitly reports readiness; an old
        # firmware with no marker is treated as not ready.
        rb35gm_firmware = "motor=RB35GM_09TYPE_26P" in initial
        drive_ready = rb35gm_firmware and "closed_loop_ready=1" in initial

        state = {
            "speed": 0.0,
            "steer": 0.0,
            "active": False,
            "running": True,
        }
        lock = threading.Lock()
        write_lock = threading.Lock()

        def send_line(line: str) -> None:
            line = line.rstrip()
            with write_lock:
                dev.write((line + "\n").encode("ascii"))
            if args.log_tx:
                print(f"\nTX {time.monotonic():.3f} {line}")

        def heartbeat_loop() -> None:
            if args.heartbeat <= 0:
                return

            while True:
                with lock:
                    if not state["running"]:
                        return
                    active = state["active"]
                    speed = state["speed"]
                    steer = state["steer"]
                    if active:
                        send_line(f"@DRIVE {speed:.3f} {steer:.2f}")
                time.sleep(args.heartbeat)

        def reader_loop() -> None:
            while True:
                with lock:
                    if not state["running"]:
                        return
                waiting = dev.in_waiting
                if waiting:
                    data = dev.read(waiting).decode(errors="replace")
                    print(data, end="")
                time.sleep(0.05)

        heartbeat_thread = threading.Thread(target=heartbeat_loop, daemon=True)
        reader_thread = threading.Thread(target=reader_loop, daemon=True)
        heartbeat_thread.start()
        reader_thread.start()

        print("Keyboard drive connected.")
        if initial:
            print(initial, end="" if initial.endswith(("\n", "\r")) else "\n")
        else:
            print("No STM32 response yet. Check board reset, COM port, and baud rate.")
        if not drive_ready:
            print("RB35GM calibration is incomplete: W/S speed control is locked.")
            print("Use firmware @MOTOR commands only for capped bench tests after verifying the six-wire mapping.")
        print("W/S speed, A/D steering, C center, Z speed0, X/Space stop motors, I idle, H help, Q quit")
        if args.line_mode:
            print("Line mode: type w/s/a/d/c/z/x/i/h/q then press Enter.")

        def apply_key(key: str) -> bool:
            if key in ("q", "Q", "\x03"):
                with lock:
                    state["speed"] = 0.0
                    state["running"] = False
                    send_line("@STOP")
                    # Hand the board back to raw-terminal users with feedback on.
                    send_line("@ECHO 1")
                    send_line("@ACK 1")
                print("\nquit")
                return False

            if key == "\r":
                key = "\n"

            with lock:
                if key in ("w", "W"):
                    if not drive_ready:
                        print("\nBlocked: RB35GM gear ratio/counts/PID are not verified.")
                        return True
                    state["speed"] = clamp(state["speed"] + args.speed_step,
                                           -args.max_speed,
                                           args.max_speed)
                elif key in ("s", "S"):
                    if not drive_ready:
                        print("\nBlocked: RB35GM gear ratio/counts/PID are not verified.")
                        return True
                    state["speed"] = clamp(state["speed"] - args.speed_step,
                                           -args.max_speed,
                                           args.max_speed)
                elif key in ("a", "A"):
                    state["steer"] = clamp(state["steer"] + args.steer_step,
                                           args.min_steer,
                                           args.max_steer)
                elif key in ("d", "D"):
                    state["steer"] = clamp(state["steer"] - args.steer_step,
                                           args.min_steer,
                                           args.max_steer)
                elif key in ("c", "C"):
                    state["steer"] = 0.0
                elif key in ("z", "Z"):
                    state["speed"] = 0.0
                elif key in ("x", "X", " "):
                    state["speed"] = 0.0
                    state["active"] = False
                    send_line("@STOP")
                    print("\rsent @STOP                         ", end="", flush=True)
                    return True
                elif key in ("i", "I"):
                    state["speed"] = 0.0
                    state["steer"] = 0.0
                    state["active"] = False
                    send_line("@IDLE")
                    print("\rsent @IDLE                        ", end="", flush=True)
                    return True
                elif key in ("h", "H", "?"):
                    send_line("@HELP")
                    return True
                else:
                    return True

                speed = state["speed"]
                steer = state["steer"]
                state["active"] = abs(speed) > 1.0e-6
                send_line(f"@DRIVE {speed:.3f} {steer:.2f}")

            print(f"\rsent @DRIVE {speed:+.3f} {steer:+.1f}   ", end="", flush=True)
            return True

        def sync_state_from_line(command: str) -> None:
            text = command[1:] if command.startswith(("@", ":")) else command
            parts = text.strip().split()
            if not parts:
                return

            verb = parts[0].upper()
            try:
                with lock:
                    if verb == "DRIVE" and len(parts) >= 3:
                        state["speed"] = clamp(float(parts[1]), -args.max_speed, args.max_speed)
                        state["steer"] = clamp(float(parts[2]), args.min_steer, args.max_steer)
                    elif verb == "SPEED" and len(parts) >= 2:
                        state["speed"] = clamp(float(parts[1]), -args.max_speed, args.max_speed)
                    elif verb == "STEER" and len(parts) >= 2:
                        state["steer"] = clamp(float(parts[1]), args.min_steer, args.max_steer)
                    elif verb == "CENTER":
                        state["steer"] = 0.0
                    elif verb in ("STOP", "IDLE"):
                        state["speed"] = 0.0
                        state["active"] = False
                        if verb == "IDLE":
                            state["steer"] = 0.0
                        return
                    else:
                        return
                    state["active"] = abs(state["speed"]) > 1.0e-6
            except ValueError:
                return

        if args.line_mode:
            while True:
                try:
                    text = input("\ncmd> ").strip()
                except (EOFError, KeyboardInterrupt):
                    text = "q"
                if not text:
                    continue
                if text.startswith(("@", ":")):
                    command = normalize_line_command(text, args.max_speed, args.min_steer, args.max_steer)
                    command_parts = command[1:].strip().split()
                    if (not drive_ready and command_parts and
                            command_parts[0].upper() in ("DRIVE", "SPEED") and
                            len(command_parts) >= 2):
                        try:
                            requested_speed = float(command_parts[1])
                        except ValueError:
                            requested_speed = 0.0
                        if abs(requested_speed) > 1.0e-6:
                            print("Blocked: RB35GM gear ratio/counts/PID are not verified.")
                            continue
                    sync_state_from_line(command)
                    send_line(command)
                    print(f"sent {command}")
                    continue
                if not apply_key(text[0]):
                    return 0

        while True:
            key = read_key()
            if not key:
                continue
            if not apply_key(key):
                return 0


if __name__ == "__main__":
    raise SystemExit(main())
