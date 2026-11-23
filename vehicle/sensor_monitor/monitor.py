from datetime import datetime
import math
import socket
import time

from vehicle.wifi.client import WifiController


def format_sensor_reading(message: str) -> str:
    """Display only measured in-range distances, not saturation markers as 80 cm."""
    fields = dict(token.split("=", 1) for token in message.split()[1:] if "=" in token)
    if fields.get("model") != "GP2Y0A21YK0F":
        raise ValueError("Unexpected SENSORS model")
    rows = []
    ratios = []
    try:
        age = int(fields["age_ms"])
        available = fields["available"] == "1"
        for name, label in (("left", "LEFT / A0"), ("front", "FRONT / A1"), ("right", "RIGHT / A2")):
            adc = int(fields[f"{name}_adc"])
            voltage = float(fields[f"{name}_sensor_V"])
            mm = float(fields[f"{name}_mm"])
            status = fields[f"{name}_status"]
            usable = fields[f"{name}_usable"] == "1"
            adc_voltage = float(fields[f"{name}_adc_V"]) if f"{name}_adc_V" in fields else None
            divider = float(fields[f"{name}_divider"]) if f"{name}_divider" in fields else None
            if not 0 <= adc <= 4095 or not math.isfinite(voltage) or not math.isfinite(mm):
                raise ValueError("Invalid sensor values")
            if adc_voltage is not None and (not math.isfinite(adc_voltage) or not 0 <= adc_voltage <= 3.3):
                raise ValueError("Invalid ADC pin voltage")
            if divider is not None and (not math.isfinite(divider) or not 0 < divider <= 1):
                raise ValueError("Invalid divider ratio")
            ratios.append(f"{name}={divider:.3f}" if divider is not None else f"{name}=unknown")
            connected = status != "not_connected"
            adc_text = f"{adc_voltage:6.3f} V" if connected and adc_voltage is not None else "      --"
            adc_count_text = f"{adc:4d}" if connected else "  --"
            sensor_text = f"{voltage:6.3f} V" if connected else "      --"
            distance = f"{mm / 10.0:7.1f} cm" if available and status == "in_range" else "       -- "
            rows.append(f"{label:<12} {adc_count_text}  {adc_text}  {sensor_text}  {distance}  {status:<16} {'yes' if usable else 'no'}")
    except (KeyError, ValueError) as error:
        raise ValueError(f"Incomplete or invalid SENSORS response: {error}") from error
    stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    return (f"\n{stamp}  sample age={age} ms  available={int(available)}\n"
            f"Divider ratios (ADC/Vo): {', '.join(ratios)}\n"
            "POSITION      ADC     ADC PIN    SENSOR Vo   DISTANCE  STATUS           USABLE\n" + "\n".join(rows))


def monitor_sensors(controller: WifiController, interval: float, duration: float = 0.0) -> None:
    """Sequential requests share one receive buffer; no background reader races."""
    pending = bytearray(controller.initial_data)
    started = time.monotonic()
    print("GP2Y0A21YK0F live monitor: LEFT=A0, FRONT=A1, RIGHT=A2. Ctrl+C to stop.")
    print("ADC PIN is the input voltage; SENSOR Vo uses the configured divider ratio (direct=1, 10k/10k=0.5).")
    print("'--' means no in-range distance. Voltages use the configured 3.3 V ADC reference.")
    controller.send("STOP")
    while duration == 0.0 or time.monotonic() - started < duration:
        tick = time.monotonic()
        controller.send("SENSORS")
        deadline = tick + 3.0
        received = False
        while not received:
            while b"\n" in pending:
                raw, _, remaining = pending.partition(b"\n")
                pending = bytearray(remaining)
                message = raw.rstrip(b"\r").decode("utf-8", errors="replace")
                if message.startswith("SENSORS "):
                    print(format_sensor_reading(message), flush=True)
                    received = True
                    break
                if message.startswith("ERR"):
                    raise RuntimeError(
                        f"Nano rejected sensor query: {message}\n"
                        "Upload the current PowerOnNano.ino (with SENSORS support), then retry."
                    )
            if received:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("No SENSORS reply from Nano within 3 seconds")
            try:
                chunk = controller.socket.recv(2048)
            except socket.timeout:
                continue
            if not chunk:
                controller.stop_event.set()
                raise ConnectionError("Nano disconnected during sensor monitoring")
            pending.extend(chunk)
            if len(pending) > 16384:
                raise RuntimeError("Nano returned an oversized response")
        wait = max(0.0, interval - (time.monotonic() - tick))
        if duration:
            wait = min(wait, max(0.0, duration - (time.monotonic() - started)))
        controller.stop_event.wait(wait)
