import csv
import math
from pathlib import Path
import time

from vehicle.wifi.client import WifiController

PREVIEW_PERIOD_SECONDS = 0.05


def load_preview_csv(path: str) -> list[tuple[float, float, float, float]]:
    """Validate the complete input before opening a vehicle connection."""
    frames = []
    with open(path, newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames != ["duration_s", "left_mm", "front_mm", "right_mm"]:
            raise ValueError("CSV columns must be duration_s,left_mm,front_mm,right_mm")
        for row_number, row in enumerate(reader, start=2):
            try:
                duration, left, front, right = (
                    float(row[key]) for key in reader.fieldnames
                )
            except (TypeError, ValueError) as error:
                raise ValueError(f"Invalid number on CSV row {row_number}") from error
            if not all(math.isfinite(v) for v in (duration, left, front, right)):
                raise ValueError(f"Non-finite number on CSV row {row_number}")
            if not 0.05 <= duration <= 60.0:
                raise ValueError(f"duration_s must be 0.05..60 on CSV row {row_number}")
            if not all(v == -1.0 or 100.0 <= v <= 800.0 for v in (left, front, right)):
                raise ValueError(f"Ranges must be 100..800 mm or -1 on row {row_number}")
            frames.append((duration, left, front, right))
    if not frames:
        raise ValueError("Preview CSV contains no frames")
    return frames


def replay_preview(controller: WifiController, frames: list[tuple[float, float, float, float]]) -> None:
    """Inject TEST ranges at 20 Hz. Never sends AUTO START or a drive command."""
    def send_ranges(left: float, front: float, right: float) -> None:
        controller.send(f"RANGES {left:.1f} {front:.1f} {right:.1f}")

    print("AUTO TEST replay: computing targets only. Normal connection/IDLE steering centring still applies.")
    controller.send("STOP")
    _, left, front, right = frames[0]
    send_ranges(left, front, right)
    controller.send("AUTO TEST")
    for duration, left, front, right in frames:
        deadline = time.monotonic() + duration
        controller.send("AUTO STATUS")
        while time.monotonic() < deadline:
            if controller.stop_event.is_set():
                raise ConnectionError("Nano connection lost during preview")
            send_ranges(left, front, right)
            time.sleep(min(PREVIEW_PERIOD_SECONDS, max(0.0, deadline - time.monotonic())))
    controller.send("AUTO STATUS")
    # Allow final status to be read before close_safely requests IDLE.
    time.sleep(0.1)
