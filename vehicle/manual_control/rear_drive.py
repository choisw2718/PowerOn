"""Bounded Wi-Fi bench commands for the dual rear drivetrain."""

import time


class RearSession:
    def __init__(self, controller):
        self.controller = controller
        self.pending = bytearray(controller.initial_data)

    def receive(self, expected, timeout=0.2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            while b"\n" in self.pending:
                line, _, rest = self.pending.partition(b"\n")
                self.pending = rest
                line = line.decode("utf-8", errors="replace").strip()
                if line.startswith("ERR "):
                    raise RuntimeError(line)
                if line.startswith(expected):
                    return line
            try:
                chunk = self.controller.socket.recv(512)
            except TimeoutError:
                continue
            if not chunk:
                raise ConnectionError("Nano disconnected during rear drive command.")
            self.pending.extend(chunk)
        raise TimeoutError(f"No reply ({expected}) from Nano.")

    def send(self, command, expected):
        self.controller.send(command)
        return self.receive(expected)

    def status(self):
        self.controller.send("STATUS")
        summary = self.receive("STATUS ", timeout=2.0)
        print(summary)
        for prefix in ("REAR left ", "REAR right "):
            print(self.receive(prefix, timeout=2.0))
        return summary


def run_rear_drive(controller, args):
    session = RearSession(controller)
    # Drain greeting telemetry before requesting the first status/acknowledgment.
    session.receive("STEERING ", timeout=2.0)
    if args.rear_status:
        session.status()
    elif args.rear_test:
        side, duty = args.rear_test
        session.send(f"DUAL TEST {side} {float(duty):.3f}", "OK DUAL TEST")
        time.sleep(0.35)
        session.status()
    else:
        session.send(f"STEER {args.rear_steer:.3f}", "OK")
        time.sleep(0.5)
        session.send("DUAL ARM", "OK DUAL ARM")
        command = f"SPEED {args.rear_speed:.4f} {args.rear_steer:.3f}"
        start = time.monotonic()
        deadline = start + args.rear_duration
        samples = []
        while time.monotonic() < deadline:
            started = time.monotonic()
            samples.append((started - start, session.send(command, "OK SPEED")))
            time.sleep(max(0.0, min(deadline, started + 0.1) - time.monotonic()))
        session.send("STOP", "OK STOP")
        for elapsed, sample in samples:
            print(f"{elapsed:.3f}s {sample}")
        if "fault=none" not in session.status():
            raise RuntimeError("Nano latched a dual rear drive fault.")
