#!/usr/bin/env python3
"""Read-only joydev monitor. No vehicle control interfaces."""
import argparse
import json
import os
from pathlib import Path
import select
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from teleop import command_from_axes

CENTERS = [-16380, 16379, -16380, 16379, 0, 0]
LOW = [-32767, 0, -32767, 0, -32767, -32767]
HIGH = [0, 32767, 0, 32767, 32767, 32767]
DEVICE = '/dev/input/by-id/usb-BEITONG_BTP-KP20_NS-joystick'
ROOT = Path(__file__).resolve().parent


def normalize(index, value):
    center = CENTERS[index]
    span = HIGH[index] - center if value >= center else center - LOW[index]
    return max(-1.0, min(1.0, (value - center) / span))


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.axes = CENTERS.copy()
        self.buttons = [0] * 14
        self.connected = False
        self.error = '正在连接手柄'
        self.last_event = None
        self.seen_axes = set()
        self.seen_lt = False

    def event(self, packet):
        _, value, kind, index = struct.unpack('IhBB', packet)
        with self.lock:
            if kind & 2 and index < len(self.axes):
                self.axes[index] = value
                self.seen_axes.add(index)
            elif kind & 1 and index < len(self.buttons):
                self.buttons[index] = int(bool(value))
                if index == 7: self.seen_lt = True
            else:
                return
            if not kind & 128:
                self.last_event = {'kind': 'axis' if kind & 2 else 'button', 'index': index, 'value': value, 'at': time.time()}

    def disconnect(self, error='手柄已断开，正在重连'):
        with self.lock:
            self.connected = False
            self.axes = CENTERS.copy()
            self.buttons = [0] * 14
            self.last_event = None
            self.seen_axes.clear()
            self.seen_lt = False
            self.error = error

    def snapshot(self):
        with self.lock:
            return {'connected': self.connected, 'error': self.error,
                    'initialized': len(self.seen_axes) == 6 and self.seen_lt,
                    'axes': self.axes.copy(), 'buttons': self.buttons.copy(),
                    'normalized': [normalize(i, v) for i, v in enumerate(self.axes)],
                    'last_event': self.last_event, 'server_time': time.time()}


def read_device(state, path):
    while True:
        fd = None
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            with state.lock:
                state.connected = True
                state.error = ''
            while True:
                ready, _, _ = select.select([fd], [], [], 0.5)
                if not ready:
                    continue
                packet = os.read(fd, 8)
                if len(packet) != 8:
                    raise OSError('设备停止输出')
                state.event(packet)
        except OSError as exc:
            state.disconnect('无读取权限' if isinstance(exc, PermissionError) else '手柄已断开，正在重连')
        finally:
            if fd is not None:
                os.close(fd)
        time.sleep(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=9984)
    parser.add_argument('--device', default=DEVICE)
    args = parser.parse_args()
    state = State()
    threading.Thread(target=read_device, args=(state, args.device), daemon=True).start()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/state':
                snapshot = state.snapshot()
                snapshot['control'] = {'execute': False, 'enabled': False,
                    'sdk_target': list(command_from_axes(snapshot['normalized'])) if snapshot['connected'] else [0,0,0],
                    'sdk_command': [0,0,0], 'reason': '预览模式 · 尚未启用底盘输出'}
                try:
                    report = json.loads(Path('/tmp/arx-gamepad-teleop-state.json').read_text())
                    if 0 <= time.time() - report['at'] < .5:
                        snapshot['control'] = report
                except (OSError, ValueError, KeyError, TypeError):
                    pass
                body = json.dumps(snapshot, ensure_ascii=False).encode()
                mime = 'application/json; charset=utf-8'
            elif path in ('/', '/index.html', '/app.js', '/style.css'):
                name = 'index.html' if path == '/' else path[1:]
                body = (ROOT / name).read_bytes()
                mime = {'html': 'text/html', 'js': 'text/javascript', 'css': 'text/css'}[name.rsplit('.', 1)[1]] + '; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    print(f'Gamepad viewer http://{args.host}:{args.port}', flush=True)
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
