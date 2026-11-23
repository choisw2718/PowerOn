from contextlib import suppress
from pathlib import Path
import shlex
import socket
import threading
import time

from .serial_scan import ScanInterrupted


def ssh_scan_chunks(target, port, password, stop):
    try:
        import paramiko
    except ImportError as error:
        raise RuntimeError('SSH 연결에 Paramiko가 필요합니다. Windows에서 python -m pip install paramiko 실행') from error

    username, _, host = target.partition('@')
    known_hosts = Path.home() / '.ssh' / 'known_hosts'
    with paramiko.SSHClient() as client:
        if known_hosts.is_file():
            client.load_host_keys(str(known_hosts))
        if not client.get_host_keys().lookup(host):
            raise RuntimeError(f'PowerShell에서 ssh {target}로 먼저 접속해 Pi의 SSH 키를 확인하세요.')
        try:
            client.connect(host, username=username, password=password or None,
                           allow_agent=not password, look_for_keys=not password,
                           timeout=5, banner_timeout=5, auth_timeout=5)
        except paramiko.AuthenticationException as error:
            raise RuntimeError('Pi SSH 로그인 실패 · 사용자 이름과 Pi 계정 비밀번호를 확인하세요.') from error
        except paramiko.BadHostKeyException as error:
            raise RuntimeError(f'{host}의 SSH 키가 바뀌었습니다. PowerShell에서 Pi 주소와 키를 확인하세요.') from error
        except (OSError, EOFError, paramiko.SSHException) as error:
            raise ScanInterrupted(f'Pi SSH 연결 실패 · {error}') from error
        if stop.is_set():
            return

        # Run the same serial reader on the Pi without a separate service or install step.
        source = Path(__file__).with_name('serial_scan.py').read_text(encoding='utf-8')
        command = shlex.join(['python3', '-u', '-c', source, port])
        transport = client.get_transport()
        transport.set_keepalive(10)
        try:
            channel = transport.open_session(timeout=5)
        except (OSError, EOFError, paramiko.SSHException) as error:
            raise ScanInterrupted(f'Pi SSH 세션 연결 실패 · {error}') from error
        with channel:
            channel.settimeout(0.15)
            try:
                # Paramiko's channel read timeout does not bound the exec acknowledgement.
                timer = threading.Timer(5, transport.close)
                timer.daemon = True
                timer.start()
                try:
                    channel.exec_command(command)
                finally:
                    timer.cancel()
                next_heartbeat = 0
                while not stop.is_set():
                    now = time.monotonic()
                    if now >= next_heartbeat and not channel.exit_status_ready():
                        channel.sendall(b'.')
                        next_heartbeat = now + 2
                    try:
                        chunk = channel.recv(4096)
                    except socket.timeout:
                        yield b''
                        continue
                    if not chunk:
                        errors = bytearray()
                        while channel.recv_stderr_ready():
                            errors.extend(channel.recv_stderr(4096))
                        message = errors.decode('utf-8', errors='replace').strip().splitlines()
                        detail = f'Pi 스캔 종료 · {message[-1]}' if message else 'Pi 스캔 연결이 끊겼습니다.'
                        if channel.exit_status_ready() and channel.recv_exit_status() not in (-1, 0, 75):
                            raise RuntimeError(detail)
                        raise ScanInterrupted(detail)
                    yield chunk
            except (OSError, EOFError, paramiko.SSHException) as error:
                raise ScanInterrupted(str(error)) from error
            finally:
                with suppress(OSError, EOFError, paramiko.SSHException):
                    channel.shutdown_write()
                    # Give the Pi time to send STOP and close the serial port before we disconnect.
                    deadline = time.monotonic() + 2
                    while not channel.exit_status_ready() and time.monotonic() < deadline:
                        if channel.recv_ready():
                            channel.recv(4096)
                        else:
                            time.sleep(0.02)
