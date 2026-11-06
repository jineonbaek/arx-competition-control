import http.client
import json
from pathlib import Path
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from server import make_handler
from control import Controller
from test_control import FakeAdapter, configured
from arxgent import ChatError


class FakeChat:
    def __init__(self):
        self.calls = []

    def snapshot(self, client):
        return {'ok': True, 'name': 'arxgent', 'messages': [], 'read_only': True}

    def handle(self, client, data):
        self.calls.append((client, data))
        if data.get('action') != 'send':
            raise ChatError(400, '请求无效')
        return {'ok': True, 'busy': True}


class ArxgentHTTPTests(unittest.TestCase):
    def setUp(self):
        cfg = configured()
        cfg['execution_enabled'] = False
        self.adapter = FakeAdapter()
        self.chat = FakeChat()
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(Controller(cfg, self.adapter), self.chat))
        self.server.daemon_threads = True
        self.server.stopping = threading.Event()
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.port = self.server.server_port
        self.host = f'127.0.0.1:{self.port}'
        _, headers, _ = self.request('GET', '/')
        self.headers = {'Cookie': headers['Set-Cookie'].split(';')[0], 'Origin': 'http://' + self.host, 'Content-Type': 'application/json'}

    def tearDown(self):
        self.server.stopping.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        connection.request(method, path, body, headers or {})
        reply = connection.getresponse()
        result = reply.status, dict(reply.getheaders()), reply.read()
        connection.close()
        return result

    def test_chat_get_and_send_require_cookie_and_same_origin_without_motion(self):
        self.assertEqual(self.request('GET', '/api/arxgent')[0], 403)
        self.assertEqual(self.request('GET', '/api/arxgent', headers=self.headers)[0], 200)
        data = json.dumps({'action': 'send'})
        self.assertEqual(self.request('POST', '/api/arxgent', data, {**self.headers, 'Origin': 'http://evil.invalid'})[0], 403)
        self.assertEqual(self.request('POST', '/api/arxgent', data, {k: v for k, v in self.headers.items() if k != 'Cookie'})[0], 403)
        self.assertEqual(self.chat.calls, [])
        self.assertEqual(self.request('POST', '/api/arxgent', data, self.headers)[0], 202)
        self.assertEqual(len(self.chat.calls), 1)
        self.assertEqual(self.adapter.calls, [])
        self.assertEqual(self.request('GET', '/api/state')[0], 200)

    def test_malformed_duplicate_or_oversized_chat_json_is_rejected_before_dispatch(self):
        for body in ['{', '{"action":"send","action":"new"}', '{"action":NaN}', ' ' * 8193]:
            try:
                self.assertEqual(self.request('POST', '/api/arxgent', body, self.headers)[0], 400)
            except (ConnectionAbortedError, ConnectionResetError):
                # Windows may reset a rejected request whose oversized body remains unread.
                self.assertGreater(len(body), 8192)
        self.assertEqual(self.chat.calls, [])
        self.assertEqual(self.adapter.calls, [])
        self.assertEqual(self.request('GET', '/agent/.secrets/qwen.env')[0], 404)


if __name__ == '__main__':
    unittest.main()
