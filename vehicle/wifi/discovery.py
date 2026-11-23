from pathlib import Path
import re
import socket
import time

DISCOVERY_PORT = 5001
DISCOVERY_REQUEST = b"POWERON_DISCOVER"


def configured_ssid() -> str | None:
    sketch = Path(__file__).resolve().parents[1] / "firmware" / "PowerOnNano"
    try:
        header = (sketch / "wifi_secrets.h").read_text(encoding="utf-8")
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
