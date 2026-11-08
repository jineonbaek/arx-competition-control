"""Offline Chrome checks for the preparation-only D405 page; no robot SDK."""
import json
from html.parser import HTMLParser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
import unittest
from urllib.parse import urlsplit

PUBLIC = Path(__file__).resolve().parents[1] / "public"


class Elements(HTMLParser):
    def __init__(self, source):
        super().__init__()
        self.by_id = {}
        self.links = []
        self.feed(source)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if "id" in attrs:
            self.by_id[attrs["id"]] = attrs
        if tag == "a":
            self.links.append(attrs)


class HandeyePreparationTests(unittest.TestCase):
    def test_main_page_has_d405_entry(self):
        elements = Elements((PUBLIC / "index.html").read_text())
        self.assertTrue(any(a.get("href") == "handeye.html" for a in elements.links))
        self.assertTrue((PUBLIC / "handeye.html").is_file())

    def browser(self, *, side="", stale=False, offline=False):
        chrome = shutil.which("google-chrome") or shutil.which("chromium")
        self.assertIsNotNone(chrome, "Headless Chrome is required")
        requests = []

        class Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(PUBLIC), **kwargs)

            def log_message(self, *args):
                pass

            def send_body(self, content, kind):
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def do_GET(self):
                path = urlsplit(self.path).path
                requests.append(("GET", path))
                if path == "/api/state":
                    now = time.time()
                    data = {"time": now, "ros": {"connected": True}, "arms": {
                        arm: {"joints": [.1, .2, .3, .4, .5, .6], "fresh": not stale,
                              "age_s": 30 if stale else .1, "timestamp": now - (30 if stale else .1)}
                        for arm in ("left", "right")}}
                    return self.send_body(json.dumps(data).encode(), "application/json")
                if path == "/camera-live/d405.json":
                    now = time.time()
                    data = {"state": "offline" if offline else "online", "sequence": 1,
                            "captured_at": now - (30 if stale else .1),
                            "capture_session": "fixture", "image_file": "d405-v-abc123-1.jpg",
                            "depth": {"state": "disabled", "requested": False}}
                    return self.send_body(json.dumps(data).encode(), "application/json")
                if path == "/camera-live/d405-v-abc123-1.jpg":
                    svg = b'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480"><rect width="640" height="480" fill="#123"/></svg>'
                    return self.send_body(svg, "image/svg+xml")
                if path == "/handeye.html":
                    source = (PUBLIC / "handeye.html").read_text()
                    source = source.replace("</body>", '<script src="/exercise.js"></script></body>')
                    return self.send_body(source.encode(), "text/html; charset=utf-8")
                if path == "/exercise.js":
                    source = """
window.addEventListener('load', () => {
  const select = document.getElementById('handeye-arm');
  select.value = SIDE; select.dispatchEvent(new Event('change'));
  document.getElementById('handeye-start').click();
  for (const code of ['KeyW','KeyA','KeyQ','Space']) {
    document.dispatchEvent(new KeyboardEvent('keydown', {code,bubbles:true}));
    document.dispatchEvent(new KeyboardEvent('keyup', {code,bubbles:true}));
  }
});
""".replace("SIDE", json.dumps(side))
                    return self.send_body(source.encode(), "text/javascript")
                return super().do_GET()

            def do_POST(self):
                requests.append(("POST", urlsplit(self.path).path))
                self.send_body(b"{}", "application/json")

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory(prefix="handeye-browser-") as profile:
                result = subprocess.run([
                    chrome, "--headless=new", "--no-sandbox", "--disable-gpu",
                    "--disable-dev-shm-usage", "--no-first-run", "--no-default-browser-check",
                    "--disable-background-networking", "--disable-extensions",
                    "--user-data-dir=" + profile, "--virtual-time-budget=6000", "--dump-dom",
                    "http://127.0.0.1:%s/handeye.html" % server.server_port,
                ], capture_output=True, text=True, timeout=25)
                self.assertEqual(result.returncode, 0, result.stderr[-2000:])
                return result.stdout, requests
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def check_inert(self, dom, requests):
        elements = Elements(dom)
        self.assertIn("disabled", elements.by_id["handeye-start"])
        self.assertFalse(any(method != "GET" for method, _ in requests), requests)
        self.assertFalse(any(path.startswith(("/api/command", "/api/camera/depth", "/events"))
                             for _, path in requests), requests)
        self.assertIn("等待限位确认", dom)
        self.assertIn("尚未采集", dom)

    def test_initial_page_does_not_assume_installation_side(self):
        dom, requests = self.browser()
        self.check_inert(dom, requests)
        self.assertIn("请选择 D405 安装侧", dom)
        self.assertFalse(any(path == "/api/state" for _, path in requests), requests)

    def test_healthy_feedback_and_keyboard_never_unlock_capture(self):
        dom, requests = self.browser(side="right")
        self.check_inert(dom, requests)
        self.assertIn("640 × 480", dom)
        self.assertIn("右臂反馈实时", dom)
        self.assertIn("5.7°", dom)
        self.assertNotIn("hidden", Elements(dom).by_id["d405-preview"])

    def test_stale_frames_and_arm_feedback_are_not_shown_as_current(self):
        dom, requests = self.browser(side="left", stale=True)
        self.check_inert(dom, requests)
        self.assertIn("hidden", Elements(dom).by_id["d405-preview"])
        self.assertIn("反馈缺失或已过期", dom)
        self.assertNotIn("5.7°", dom)

    def test_offline_camera_cannot_show_a_live_image(self):
        dom, requests = self.browser(offline=True)
        self.check_inert(dom, requests)
        self.assertIn("hidden", Elements(dom).by_id["d405-preview"])


if __name__ == "__main__":
    unittest.main()
