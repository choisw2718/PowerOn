"""Live RPLIDAR A1 radar from PC USB or a Raspberry Pi over SSH."""
import argparse
from contextlib import closing
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import simpledialog, ttk

from serial.tools import list_ports

from lidar.acquisition.serial_scan import (CP210X_USB_ID, DEFAULT_PI_PORT, LidarPort,
                                           ScanDecoder, ScanInterrupted, reconnecting_scans,
                                           scan_chunks)
from lidar.acquisition.ssh import ssh_scan_chunks


def read_scan(port, stop, messages, ssh=None, password=None):
    usb = LidarPort(port)

    def post(kind, data):
        # Bounded queue: a slow window never causes unlimited sample storage.
        try:
            messages.put_nowait((kind, data))
        except queue.Full:
            try:
                messages.get_nowait()
            except queue.Empty:
                pass
            messages.put_nowait((kind, data))

    def receive():
        if ssh:
            source = ssh_scan_chunks(ssh, port, password, stop)
            label = ssh
        else:
            label = usb.resolve()
            source = scan_chunks(label, stop)
        decoder = ScanDecoder()
        last_packet = None
        receiving = False
        with closing(source) as chunks:
            for chunk in chunks:
                if stop.is_set():
                    break
                now = time.monotonic()
                if last_packet is None:
                    last_packet = now
                # Retain invalid returns as updates so old points are cleared.
                points = [(angle, distance, quality)
                          for _, angle, distance, quality in decoder.feed(chunk)]
                if points:
                    if not receiving:
                        post('status', f'{label} · 115200 bps · 수신 중')
                        receiving = True
                    last_packet = now
                    yield now, points
                elif now - last_packet > (3 if receiving or not ssh else 5):
                    raise ScanInterrupted('데이터 수신 시간 초과')
        if not stop.is_set():
            raise ScanInterrupted('라이다 데이터 스트림 종료')

    def on_retry(delay, error):
        post('reset', f'수신 중단 · {delay}초 후 재연결 · {error}')

    try:
        with closing(reconnecting_scans(receive, stop, on_retry)) as scans:
            for item in scans:
                post('points', item)
    except Exception as error:
        if not stop.is_set():
            post('error', str(error))
    finally:
        post('done', None)


class Radar:
    def __init__(self, root, usb_port, ssh=None, pi_port=DEFAULT_PI_PORT):
        self.root = root
        self.worker = None
        self.stop_event = threading.Event()
        self.messages = queue.Queue(maxsize=128)
        self.points = {}
        self.total = 0
        self.valid_total = 0
        self.last_angle = 0
        self.last_data = 0
        self.closing = False
        root.title('RPLIDAR A1 · 실시간 레이더')
        root.geometry('760x760')
        root.minsize(600, 600)
        root.configure(bg='#07151c')
        settings = ttk.Frame(root, padding=(12, 10, 12, 0))
        settings.pack(fill='x')
        ttk.Label(settings, text='연결').pack(side='left')
        self.mode = tk.StringVar(value='Raspberry Pi (SSH)' if ssh else 'PC USB')
        self.mode_box = ttk.Combobox(settings, textvariable=self.mode,
                                     values=('PC USB', 'Raspberry Pi (SSH)'),
                                     state='readonly', width=19)
        self.mode_box.pack(side='left', padx=(6, 12))
        self.mode_box.bind('<<ComboboxSelected>>', self.update_connection_fields)
        self.ssh_settings = ttk.Frame(settings)
        ttk.Label(self.ssh_settings, text='SSH 주소').pack(side='left')
        self.ssh_target = tk.StringVar(value=ssh or '')
        self.ssh_entry = ttk.Entry(self.ssh_settings, textvariable=self.ssh_target, width=28)
        self.ssh_entry.pack(side='left', padx=6)
        top = ttk.Frame(root, padding=12)
        top.pack(fill='x')
        ttk.Label(top, text='USB 포트').pack(side='left')
        self.usb_port = tk.StringVar(value=usb_port)
        self.pi_port = tk.StringVar(value=pi_port)
        self.port_entry = ttk.Entry(top, width=16)
        self.port_entry.pack(side='left', padx=(6, 16))
        self.start_button = ttk.Button(top, text='시작', command=self.start)
        self.start_button.pack(side='left')
        self.stop_button = ttk.Button(top, text='정지', command=self.stop, state='disabled')
        self.stop_button.pack(side='left', padx=6)
        ttk.Label(top, text='표시 반경').pack(side='left', padx=(14, 6))
        self.radius = tk.StringVar(value='4 m')
        ttk.Combobox(top, textvariable=self.radius, values=['1 m', '2 m', '4 m', '8 m', '12 m'],
                     state='readonly', width=6).pack(side='left')
        self.canvas = tk.Canvas(root, bg='#07151c', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.stats = tk.StringVar(value='연결 대기')
        tk.Label(root, textvariable=self.stats, bg='#07151c', fg='#66f2b5',
                 font=('Malgun Gothic', 12), pady=8).pack(fill='x')
        self.status = tk.StringVar(value='연결 대기')
        tk.Label(root, textvariable=self.status, bg='#102630', fg='#d2e6eb',
                 font=('Malgun Gothic', 10), pady=8, wraplength=760).pack(fill='x')
        tk.Label(root, text='센서 기준 각도 · 0° 위 / 시계 방향 증가 · 최근 1초 측정값 표시',
                 bg='#07151c', fg='#86a8b4', pady=8).pack(fill='x')
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(50, self.tick)
        self.update_connection_fields()
        root.after_idle(self.start)

    def update_connection_fields(self, event=None):
        if self.mode.get() == 'PC USB':
            self.ssh_settings.pack_forget()
            self.port_entry.configure(textvariable=self.usb_port)
        else:
            self.ssh_settings.pack(side='left')
            self.port_entry.configure(textvariable=self.pi_port)

    def start(self):
        if self.worker and self.worker.is_alive():
            return
        is_ssh = self.mode.get() == 'Raspberry Pi (SSH)'
        port = (self.pi_port if is_ssh else self.usb_port).get().strip()
        if not port:
            self.status.set('Pi USB 포트를 입력하세요. 예: /dev/ttyUSB0' if is_ssh
                            else 'USB 포트를 입력하세요. 예: COM19')
            return
        target = password = None
        if is_ssh:
            target = self.ssh_target.get().strip()
            username, _, host = target.partition('@')
            if not username or not host or '@' in host or any(c.isspace() for c in target):
                self.status.set('SSH 주소를 사용자@호스트 형식으로 입력하세요.')
                return
            password = simpledialog.askstring('Pi SSH 로그인',
                                              f'{target}\nPi 계정 비밀번호',
                                              show='*', parent=self.root)
            if password is None:
                self.status.set('연결 취소 · 시작 버튼으로 다시 연결')
                return
        self.messages = queue.Queue(maxsize=128)
        self.stop_event = threading.Event()
        self.points.clear()
        self.total = self.valid_total = 0
        self.last_data = 0
        self.status.set(f'{target or port} 연결 중…')
        self.start_button.configure(state='disabled')
        self.port_entry.configure(state='disabled')
        self.mode_box.configure(state='disabled')
        self.ssh_entry.configure(state='disabled')
        self.stop_button.configure(state='normal')
        self.worker = threading.Thread(target=read_scan,
                                       args=(port, self.stop_event, self.messages, target, password),
                                       daemon=True)
        self.worker.start()

    def stop(self):
        self.stop_event.set()
        self.stop_button.configure(state='disabled')
        self.status.set('수집 정지 중…')

    def close(self):
        self.closing = True
        self.stop_event.set()
        self.status.set('연결 종료 중…')

    def tick(self):
        for _ in range(256):
            try:
                kind, value = self.messages.get_nowait()
            except queue.Empty:
                break
            if kind in ('status', 'error'):
                self.status.set(value)
            elif kind == 'reset':
                self.points.clear()
                self.last_data = 0
                self.status.set(value)
            elif kind == 'points':
                received, points = value
                self.last_data = received
                for angle, distance, quality in points:
                    self.total += 1
                    self.last_angle = angle
                    key = int(angle * 2)  # 0.5 degree display bins
                    if distance > 0 and quality > 0:
                        self.valid_total += 1
                        self.points[key] = (angle, distance, received)
                    else:
                        self.points.pop(key, None)
            elif kind == 'done':
                self.start_button.configure(state='normal')
                self.port_entry.configure(state='normal')
                self.mode_box.configure(state='readonly')
                self.ssh_entry.configure(state='normal')
                self.stop_button.configure(state='disabled')
                if self.stop_event.is_set():
                    self.status.set('정지됨 · 시작 버튼으로 다시 연결')
        if self.closing and (not self.worker or not self.worker.is_alive()):
            print(f'packets={self.total} valid_points={self.valid_total}', flush=True)
            self.root.destroy()
            return
        self.draw()
        self.root.after(50, self.tick)

    def draw(self):
        canvas = self.canvas
        canvas.delete('all')
        width, height = canvas.winfo_width(), canvas.winfo_height()
        cx, cy = width / 2, height / 2
        radius = max(10, min(width, height) / 2 - 38)
        max_mm = float(self.radius.get().split()[0]) * 1000
        for i in range(1, 5):
            r = radius * i / 4
            canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline='#214733')
            canvas.create_text(cx+5, cy-r+12, text=f'{max_mm*i/4000:g} m',
                               fill='#6e9d88', anchor='w')
        for degrees in range(0, 360, 30):
            angle = math.radians(degrees)
            x, y = math.sin(angle), -math.cos(angle)
            canvas.create_line(cx, cy, cx+x*radius, cy+y*radius, fill='#16352c')
            canvas.create_text(cx+x*(radius+20), cy+y*(radius+20), text=f'{degrees}°',
                               fill='#86a8b4')
        now = time.monotonic()
        self.points = {k: p for k, p in self.points.items() if now-p[2] < 1}
        visible = 0
        for angle_deg, distance, received in self.points.values():
            if distance > max_mm:
                continue
            angle = math.radians(angle_deg)
            r = distance / max_mm * radius
            x, y = cx+math.sin(angle)*r, cy-math.cos(angle)*r
            color = '#65ffb0' if now-received < 0.25 else '#279d68'
            canvas.create_oval(x-2, y-2, x+2, y+2, fill=color, outline='')
            visible += 1
        if now-self.last_data < 0.5:
            angle = math.radians(self.last_angle)
            canvas.create_line(cx, cy, cx+math.sin(angle)*radius,
                               cy-math.cos(angle)*radius, fill='#39d492', width=2)
        canvas.create_oval(cx-4, cy-4, cx+4, cy+4, fill='#e0fff0', outline='')
        self.stats.set(f'화면 {visible:,}점   |   누적 유효값 {self.valid_total:,}   |   수신 {self.total:,}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='USB serial port, e.g. COM19 or /dev/ttyUSB0')
    parser.add_argument('--ssh', help='Read the lidar from a Pi over SSH, e.g. user@andrea.local')
    parser.add_argument('--fallback-ssh', help='Use this SSH address when no PC USB sensor is detected')
    parser.add_argument('--duration', type=float, help='Close after this many seconds (verification)')
    args = parser.parse_args()
    ports = [p.device for p in list_ports.comports() if (p.vid, p.pid) == CP210X_USB_ID]
    usb_port = ports[0] if len(ports) == 1 else ''
    ssh = args.ssh or (args.fallback_ssh if not ports and not args.port else None)
    root = tk.Tk()
    if ssh:
        app = Radar(root, usb_port, ssh=ssh, pi_port=args.port or DEFAULT_PI_PORT)
    else:
        app = Radar(root, args.port or usb_port)
    if args.duration is not None:
        if args.duration <= 0:
            parser.error('--duration must be positive')
        root.after(int(args.duration * 1000), app.close)
    root.mainloop()
