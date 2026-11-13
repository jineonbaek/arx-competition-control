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
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from teleop import command_from_axes
from profiles import NS, PROFILES, UnknownMode, inspect_device, raw_normalized, logical_axes, lt_pressed

ROOT = Path(__file__).resolve().parent


def normalize(index, value):
    return raw_normalized(NS, index, value)


class State:
    def __init__(self, profile=NS):
        self.lock = threading.Lock()
        self.profile = profile
        self.boot = uuid.uuid4().hex
        self.generation = 0
        self.connected = False
        self.error = '正在识别手柄模式'
        self.name = ''
        self._reset()

    def _reset(self):
        self.axes = self.profile['centers'].copy()
        self.buttons = [0] * len(self.profile['button_codes'])
        self.last_event = None
        self.seen_axes = set()
        self.seen_buttons = set()

    def connect(self, profile, name):
        with self.lock:
            self.profile = profile
            self.name = name
            self._reset()
            self.generation += 1
            self.connected = True
            self.error = ''

    def event(self, packet):
        _, value, kind, index = struct.unpack('IhBB', packet)
        with self.lock:
            if kind & 2 and index < len(self.axes):
                self.axes[index] = value
                self.seen_axes.add(index)
            elif kind & 1 and index < len(self.buttons):
                self.buttons[index] = int(bool(value))
                self.seen_buttons.add(index)
            else:
                return
            if not kind & 128:
                self.last_event = {'kind': 'axis' if kind & 2 else 'button', 'index': index, 'value': value, 'at': time.time()}

    def disconnect(self, error='手柄已断开，正在重连'):
        with self.lock:
            self.connected = False
            self._reset()
            self.error = error
            self.generation += 1

    def snapshot(self):
        with self.lock:
            p = self.profile
            initialized = len(self.seen_axes) == len(self.axes)
            if 'LT' in p['buttons']:
                initialized = initialized and p['buttons']['LT'] in self.seen_buttons
            return {'connected': self.connected, 'error': self.error,
                    'initialized': initialized,
                    'mode': p['mode'] if self.connected else None,
                    'device_name': self.name,
                    'device_session': f'{self.boot}:{self.generation}',
                    'mapping': {'buttons': p['buttons'], 'triggers': p['triggers'], 'logical': p['logical']},
                    'axes': self.axes.copy(), 'buttons': self.buttons.copy(),
                    'normalized': [raw_normalized(p,i,v) for i,v in enumerate(self.axes)],
                    'logical_axes': logical_axes(p,self.axes),
                    'lt_pressed': lt_pressed(p,self.axes,self.buttons) if self.connected else False,
                    'last_event': self.last_event, 'server_time': time.time()}


def read_device(state, path=None):
    while True:
        fd = None
        try:
            candidates = [Path(path)] if path else sorted(Path('/dev/input').glob('js[0-9]*'))
            if not candidates:
                raise FileNotFoundError('未检测到手柄')
            if len(candidates) != 1:
                raise UnknownMode('检测到多个手柄，无法唯一选择；禁止底盘输出')
            fd, profile, name = inspect_device(candidates[0])
            state.connect(profile,name)
            while True:
                ready, _, _ = select.select([fd], [], [], 0.5)
                if not ready:
                    continue
                packet = os.read(fd, 8)
                if len(packet) != 8:
                    raise OSError('设备停止输出')
                state.event(packet)
        except UnknownMode as exc:
            state.disconnect(str(exc))
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
    parser.add_argument('--device', default=None, help='默认自动识别唯一连接手柄及其模式')
    args = parser.parse_args()
    state = State()
    threading.Thread(target=read_device, args=(state, args.device), daemon=True).start()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/state':
                snapshot = state.snapshot()
                snapshot['control'] = {'execute': False, 'enabled': False,
                    'sdk_target': list(command_from_axes(snapshot['logical_axes'])) if snapshot['connected'] else [0,0,0],
                    'sdk_command': [0,0,0], 'reason': snapshot['error'] or '预览模式 · 尚未启用底盘输出'}
                try:
                    report = json.loads(Path('/tmp/arx-gamepad-teleop-state.json').read_text())
                    if (0 <= time.time() - report['at'] < .5 and
                        report.get('device_session') == snapshot['device_session']):
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
