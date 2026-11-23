import socket
import threading
import time
import uuid

from vehicle.drive_data import event, recording

DEFAULT_PORT = 5000
READY_BANNER = b"READY PowerOn Nano 33 IoT"


class _RecordingSocket:
    """Observe incoming lines once, regardless of which session owns the reader."""
    def __init__(self, sock, connection):
        self.sock = sock
        self.connection = connection
        self.pending = bytearray()

    def __getattr__(self, name):
        return getattr(self.sock, name)

    def recv(self, size):
        chunk = self.sock.recv(size)
        self.pending.extend(chunk)
        while b"\n" in self.pending:
            line, _, self.pending = self.pending.partition(b"\n")
            text = line.rstrip(b"\r").decode("ascii", errors="replace")
            if text.startswith(("READY", "OK", "ERR", "STATUS", "REAR ", "STEERING", "SENSORS",
                                "FIRMWARE ", "PONG", "AUTO ")):
                event("nano_reply", connection=self.connection, line=text)
        if len(self.pending) > 16384:
            self.pending.clear()
        if not chunk:
            event("nano_disconnect", connection=self.connection)
        return chunk


class WifiController:
    def __init__(
        self, host: str, port: int, connect_timeout: float, ready_timeout: float = 3.0
    ) -> None:
        self.socket = socket.create_connection((host, port), timeout=connect_timeout)
        self.connection_id = uuid.uuid4().hex[:12]
        if recording():
            self.socket = _RecordingSocket(self.socket, self.connection_id)
            event("nano_connect", connection=self.connection_id)
        try:
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # WiFiNINA's server.accept() can wait for the first client payload.
            # STATUS is read-only and makes the Nano accept and send READY.
            self.socket.sendall(b"STATUS\n")
            event("nano_command", connection=self.connection_id, command="STATUS")
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
            try:
                self.socket.sendall(payload)
            except OSError:
                event("nano_send_unconfirmed", connection=self.connection_id, command=command)
                raise
            event("nano_command", connection=self.connection_id, command=command)

    def reader_loop(self) -> None:
        pending = bytearray(self.initial_data)
        while not self.stop_event.is_set():
            while b"\n" in pending:
                raw_line, _, remaining = pending.partition(b"\n")
                pending = bytearray(remaining)
                message = raw_line.rstrip().decode('utf-8', errors='replace')
                if message != "OK RANGES TEST ONLY":
                    print(f"\n< {message}")
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
