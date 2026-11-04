import http.client
import json
from pathlib import Path
import sys
import threading
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from server import make_handler
from control import Controller
from http.server import ThreadingHTTPServer
from test_control import FakeAdapter, configured

class HTTPTests(unittest.TestCase):
    def setUp(self):
        config = configured()
        config["execution_enabled"] = False
        self.adapter = FakeAdapter()
        self.controller = Controller(config, self.adapter)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(self.controller))
        self.server.daemon_threads = True
        self.server.stopping = threading.Event()
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.port = self.server.server_port
    def tearDown(self):
        self.server.stopping.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
    def request(self, method, path, body=None, headers=None):
        c = http.client.HTTPConnection("127.0.0.1", self.port)
        c.request(method, path, body, headers or {})
        r = c.getresponse()
        result = (r.status, dict(r.getheaders()), r.read())
        c.close()
        return result
    def test_static_state_and_traversal(self):
        code, headers, body = self.request("GET", "/")
        self.assertEqual(code, 200)
        self.assertIn("SameSite=Strict", headers["Set-Cookie"])
        self.assertIn(b"ARX", body)
        code, _, body = self.request("GET", "/api/state")
        self.assertFalse(json.loads(body)["execution_enabled"])
        self.assertEqual(self.request("GET", "/%2e%2e/config/control.json")[0], 404)
        self.assertEqual(self.adapter.calls, [])
    def test_origin_and_observation_guard(self):
        _, headers, _ = self.request("GET", "/")
        cookie = headers["Set-Cookie"].split(";")[0]
        body = json.dumps({"type": "arm.stop", "arm": "left"})
        headers = {"Content-Type": "application/json", "Cookie": cookie, "Origin": "http://evil.invalid"}
        self.assertEqual(self.request("POST", "/api/command", body, headers)[0], 403)
        headers["Origin"] = "http://127.0.0.1:%d" % self.port
        self.assertEqual(self.request("POST", "/api/command", body, headers)[0], 400)
        self.assertEqual(self.adapter.calls, [])

    def test_mode_endpoint_blocks_observation_then_dispatches_full_message_to_fake_adapter(self):
        _, response_headers, _ = self.request("GET", "/")
        headers = {"Content-Type": "application/json",
                   "Cookie": response_headers["Set-Cookie"].split(";")[0],
                   "Origin": "http://127.0.0.1:%d" % self.port}
        for mode in range(4):
            body = json.dumps({"type": "arm.mode", "arm": "right", "mode": mode})
            self.assertEqual(self.request("POST", "/api/command", body, headers)[0], 400)
        self.assertEqual(self.adapter.calls, [])
        self.controller.config["execution_enabled"] = True
        self.controller.config["arms"]["right"]["verified_modes"] = [0, 1, 2, 3]
        self.controller.update_arm("right", [.2] * 6, -1.2, [.3] * 6)
        for mode in range(4):
            body = json.dumps({"type": "arm.mode", "arm": "right", "mode": mode})
            status, _, payload = self.request("POST", "/api/command", body, headers)
            self.assertEqual(status, 200)
            self.assertEqual(json.loads(payload)["mode_status"], "requested_unconfirmed")
            self.assertEqual(self.adapter.calls[-1],
                             ("arm", ("right", [.2] * 6, -1.2, [.3] * 6, mode)))

if __name__ == "__main__": unittest.main()
