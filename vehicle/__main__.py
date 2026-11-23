"""Control the PowerOn car through Nano 33 IoT Wi-Fi."""

import argparse
import math
import sys
import threading
from contextlib import ExitStack

from .drive_data import add_arguments, session, finish
from .drive_data.transfer import add_notebook_arguments, notebook_capture

from .ir_wall_following.preview import load_preview_csv, replay_preview
from .manual_control.keyboard import run_keyboard
from .manual_control.rear_drive import run_rear_drive
from .sensor_monitor.monitor import monitor_sensors
from .wifi.client import DEFAULT_PORT, WifiController
from .wifi.discovery import DISCOVERY_PORT, configured_ssid, discover_host


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control the PowerOn car through Nano 33 IoT Wi-Fi."
    )
    parser.add_argument("--host", help="Nano IP address; discovered automatically if omitted")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--ready-timeout", type=float, default=3.0)
    add_arguments(parser)
    add_notebook_arguments(parser)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument(
        "--preview-csv",
        help="Replay duration_s,left_mm,front_mm,right_mm in AUTO TEST; no keyboard or actuation",
    )
    modes.add_argument("--sensors", action="store_true", help="Continuously display three live sensors; no driving")
    modes.add_argument("--rear-status", action="store_true", help="Read dual rear encoder telemetry")
    modes.add_argument("--rear-test", nargs=2, metavar=("L_or_R", "SIGNED_PWM"), help="Run one configured rear bench pulse")
    modes.add_argument("--rear-speed", type=float, metavar="MPS", help="Run a bounded dual rear speed test")
    parser.add_argument("--rear-steer", type=float, default=0.0, help="Center steering angle for rear speed test, positive left")
    parser.add_argument("--rear-duration", type=float, default=1.0, help="Rear speed test duration from 0.2 to 10 seconds")
    parser.add_argument("--sensor-interval", type=float, default=0.5, help="Sensor display interval in seconds (default: 0.5)")
    parser.add_argument("--sensor-duration", type=float, default=0.0, help="Stop sensor monitoring after N seconds; 0=until Ctrl+C")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        preview_frames = load_preview_csv(args.preview_csv) if args.preview_csv else None
    except (OSError, ValueError) as error:
        print(f"Preview input error: {error}")
        return 2
    if not 0 < args.port < 65536 or args.connect_timeout <= 0 or args.ready_timeout <= 0:
        print("Port must be 1-65535 and connection timeouts must be positive.")
        return 2
    if not math.isfinite(args.sensor_interval) or args.sensor_interval < 0.1 or \
            not math.isfinite(args.sensor_duration) or args.sensor_duration < 0:
        print("Sensor interval must be finite and >=0.1 s; duration must be finite and >=0.")
        return 2
    rear_mode = args.rear_status or args.rear_test or args.rear_speed is not None
    if args.rear_test:
        side, duty = args.rear_test
        try:
            valid_pulse = side in ("L", "R") and 1 <= abs(float(duty)) <= 10
        except ValueError:
            valid_pulse = False
        if not valid_pulse:
            print("Rear test requires L or R and signed PWM with magnitude 1..10%.")
            return 2
    if args.rear_speed is not None and (
        not math.isfinite(args.rear_speed) or abs(args.rear_speed) < 0.0001
        or not -28 <= args.rear_steer <= 28 or not 0.2 <= args.rear_duration <= 10
    ):
        print("Rear speed must be finite and nonzero, steer within +/-28 deg, duration 0.2..10 s.")
        return 2
    if preview_frames is None and not args.sensors and not rear_mode and not sys.stdin.isatty():
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
    mode = "rear-test" if rear_mode else "ir-sensors" if args.sensors else "ir-preview" if preview_frames is not None else "manual"
    settings = {name: getattr(args, name) for name in ("rear_status", "rear_test", "rear_speed",
                "rear_steer", "rear_duration", "sensor_interval", "sensor_duration")}
    captures_lidar = mode in ("manual", "rear-test") and not args.rear_status
    settings["scan_source"] = "ssh" if captures_lidar and getattr(args, "lidar_ssh", None) else "none"
    settings["time_base"] = "controller_monotonic_receive"
    try:
        with ExitStack() as stack:
            stack.enter_context(session(args, mode, settings))
            if captures_lidar:
                stack.enter_context(notebook_capture(args))
            return finish(connected_run(args, host, rear_mode, preview_frames))
    except OSError as error:
        print(f"Session error: {error}")
        return 1


def connected_run(args, host, rear_mode, preview_frames):
    try:
        controller = WifiController(host, args.port, args.connect_timeout, args.ready_timeout)
    except OSError as error:
        print(f"Could not establish a Nano control session at {host}:{args.port}: {error}")
        print("Use the Nano's IP from the READY line, not this computer's IP.")
        return 1

    print(f"Connected to Nano 33 IoT at {host}:{args.port}.")
    if rear_mode:
        try:
            run_rear_drive(controller, args)
            return 0
        except KeyboardInterrupt:
            return 130
        except (OSError, RuntimeError) as error:
            print(f"Rear drive error: {error}")
            return 1
        finally:
            controller.close_safely()
    if args.sensors:
        try:
            monitor_sensors(controller, args.sensor_interval, args.sensor_duration)
            return 0
        except KeyboardInterrupt:
            print("\nSensor monitor stopped.")
            return 0
        except (OSError, RuntimeError, ValueError) as error:
            print(f"Sensor monitor error: {error}")
            return 1
        finally:
            controller.close_safely()
    reader = threading.Thread(target=controller.reader_loop, daemon=True)
    reader.start()

    result = 0
    try:
        if preview_frames is not None:
            replay_preview(controller, preview_frames)
            return 0 if not controller.stop_event.is_set() else 1
        run_keyboard(controller)
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
