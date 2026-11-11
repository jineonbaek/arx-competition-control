#!/usr/bin/env python3
"""LAN web control server: static UI + JSON commands + server-sent feedback."""
import argparse
import http.cookies
import json
import mimetypes
import os
from pathlib import Path
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit
import uuid

from control import Controller, Rejected
from handeye_steps import Steps
from grasp_preview import GraspPreview
from vision_preview import VisionPreview
from foundationpose_preview import FoundationPosePreview
from d435_cad import preview as d435_cad_preview
from camera_depth_control import DepthControl
from ros_adapter import RosAdapter
from arxgent import ArxgentChat, ChatError, strict_json

WEB_ROOT = Path(__file__).resolve().parents[1]
PUBLIC = WEB_ROOT / "public"


def make_handler(controller, arxgent=None, depth_control=None, vision=None, foundation=None):
    depth_control=depth_control or DepthControl()
    steps = Steps(controller, depth_control)
    grasps = GraspPreview(controller, depth_control)
    vision = vision or VisionPreview()
    foundation = foundation or FoundationPosePreview(controller,depth_control)
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            # Do not log cookie values or request bodies.
            print("%s %s" % (self.address_string(), fmt % args), flush=True)

        def response_headers(self, status, content_type, length=None):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'; base-uri 'none'")
            if length is not None:
                self.send_header("Content-Length", str(length))

        def client(self):
            c = http.cookies.SimpleCookie()
            try:
                c.load(self.headers.get("Cookie", ""))
                value = c["arx_web_session"].value
                return str(uuid.UUID(value))
            except (KeyError, ValueError, http.cookies.CookieError):
                return None

        def json(self, status, obj):
            payload = json.dumps(obj, ensure_ascii=False, allow_nan=False).encode()
            self.response_headers(status, "application/json; charset=utf-8", len(payload))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/api/d435-cad":
                try:
                    return self.json(200, d435_cad_preview(controller.snapshot().get("body") or {}))
                except (ValueError, OSError) as error:
                    return self.json(503, {"message":str(error)})
            if path == "/api/vision":
                if not self.client(): return self.json(403, {"message":"请先打开视觉识别页"})
                return self.json(200, vision.snapshot())
            if path in ("/api/vision/stream", "/api/vision/review"):
                if not self.client(): return self.json(403, {"message":"请先打开视觉识别页"})
                from urllib.parse import parse_qs
                ident=parse_qs(urlsplit(self.path).query).get("run", [""])[0]
                if path.endswith("/review"):
                    frame=vision.frame(ident, review=True)
                    if frame is None: return self.json(404, {"message":"复核画面尚未生成"})
                    self.response_headers(200,"image/jpeg",len(frame[1]))
                    self.end_headers()
                    self.wfile.write(frame[1])
                    return
                self.response_headers(200, "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Connection","close")
                self.end_headers()
                previous=None
                try:
                    while not self.server.stopping.is_set():
                        state=vision.snapshot()
                        if not state["running"] or state.get("run_id")!=ident: break
                        frame=vision.frame(ident)
                        if frame is not None and frame[0]!=previous:
                            previous=frame[0]
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "+str(len(frame[1])).encode()+b"\r\n\r\n"+frame[1]+b"\r\n")
                            self.wfile.flush()
                        self.server.stopping.wait(.1)
                except (BrokenPipeError,ConnectionResetError,OSError):
                    pass
                finally:
                    self.close_connection=True
                return
            if path == "/api/foundationpose":
                if not self.client():return self.json(403, {"message":"请先打开位姿验证页"})
                return self.json(200,foundation.snapshot())
            if path == "/api/handle-preview":
                if not self.client():return self.json(403, {"message":"请先打开提手预览页"})
                return self.json(200, grasps.handle_view())
            if path == "/api/camera/depth":
                return self.json(200, depth_control.snapshot())
            if path == "/api/handeye":
                return self.json(200, steps.state(self.client()))
            if path == "/api/state":
                return self.json(200, controller.snapshot())
            if path == '/api/arxgent':
                if not self.client():
                    return self.json(403, {'ok': False, 'message': '请先打开网页'})
                if arxgent is None:
                    return self.json(503, {'ok': False, 'message': 'arxgent 尚未启用'})
                try:
                    return self.json(200, arxgent.snapshot(self.client()))
                except ChatError as error:
                    return self.json(error.status, {'ok': False, 'message': error.message})
            if path == "/events":
                client = self.client()
                if not client:
                    return self.json(403, {"ok": False, "message": "请先打开网页"})
                self.response_headers(200, "text/event-stream; charset=utf-8")
                self.send_header("Connection", "close")
                self.end_headers()
                try:
                    while not self.server.stopping.is_set():
                        payload = json.dumps(controller.snapshot(), ensure_ascii=False, allow_nan=False)
                        self.wfile.write(("data: " + payload + "\n\n").encode())
                        self.wfile.flush()
                        self.server.stopping.wait(0.1)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    controller.disconnected(client)
                    self.close_connection = True
                return
            target = (PUBLIC / ("index.html" if path == "/" else unquote(path).lstrip("/"))).resolve()
            if not target.is_relative_to(PUBLIC.resolve()) or not target.is_file():
                return self.json(404, {"ok": False, "message": "页面不存在"})
            body = target.read_bytes()
            kind = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
            self.response_headers(200, kind + ("; charset=utf-8" if kind.startswith(("text/", "application/javascript")) else ""), len(body))
            if path in ("/", "/handeye.html", "/handle.html", "/vision.html", "/foundationpose.html") and not self.client():
                self.send_header("Set-Cookie", "arx_web_session=%s; Path=/; HttpOnly; SameSite=Strict" % uuid.uuid4())
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            path = urlsplit(self.path).path
            if path not in ('/api/command', '/api/arxgent', '/api/camera/depth', '/api/handeye', '/api/handle-preview', '/api/vision', '/api/foundationpose'):
                return self.json(404, {"ok": False, "message": "接口不存在"})
            expected = "http://" + self.headers.get("Host", "")
            if self.headers.get("Origin") != expected or not self.client():
                self.close_connection = True
                return self.json(403, {"ok": False, "message": "请求来源无效"})
            try:
                if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    raise Rejected("仅接受 JSON")
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 8192:
                    raise Rejected("请求大小无效")
                raw = self.rfile.read(length)
                if path == '/api/vision':
                    return self.json(200, vision.action(self.client(), strict_json(raw)))
                if path == '/api/foundationpose':
                    return self.json(200,foundation.action(self.client(),strict_json(raw)))
                if path == '/api/handle-preview':
                    return self.json(200, grasps.handle_action(strict_json(raw)))
                if path == '/api/handeye':
                    return self.json(200, steps.action(self.client(), strict_json(raw)))
                if path == '/api/camera/depth':
                    data=strict_json(raw)
                    try:result=depth_control.manual_request(data)
                    except RuntimeError as error:
                        return self.json(503,{'ok':False,'message':str(error)})
                    controller.log(data['camera']+' depth '+('enable' if data['enabled'] else 'disable')+' requested')
                    return self.json(202,result)
                if path == '/api/arxgent':
                    if arxgent is None:
                        return self.json(503, {'ok': False, 'message': 'arxgent 尚未启用'})
                    data = strict_json(raw)
                    return self.json(202, arxgent.handle(self.client(), data))
                data = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(ValueError("non-finite JSON")))
                if isinstance(data,dict) and data.get("type")=="chassis.nav_sdk" and self.client_address[0] not in ("127.0.0.1","::1"):
                    raise Rejected("导航 SDK 入口仅接受本机执行器")
                return self.json(200, steps.manual_submit(data, self.client()))
            except ChatError as error:
                return self.json(error.status, {'ok': False, 'message': error.message})
            except (ValueError, TypeError, Rejected) as e:
                self.close_connection = True
                return self.json(400, {"ok": False, "message": str(e)})
            except Exception:
                controller.log("控制接口异常，未确认执行结果", "warning")
                return self.json(503, {"ok": False, "message": "控制接口异常，请查看反馈"})
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9983)
    args = parser.parse_args()
    config = json.loads((WEB_ROOT / "config/control.json").read_text())
    adapter = RosAdapter(config)
    controller = Controller(config, adapter)
    arxgent = ArxgentChat.from_repository(WEB_ROOT.parent, args.port)
    vision = VisionPreview()
    foundation = FoundationPosePreview(controller,DepthControl())
    server = ThreadingHTTPServer((args.host, args.port), make_handler(controller, arxgent, vision=vision, foundation=foundation))
    server.daemon_threads = True
    server.stopping = threading.Event()
    adapter.start(controller)
    def stop(signum, frame):
        server.stopping.set()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print("ARX Web http://%s:%d | execution_enabled=%s | ROS_DOMAIN_ID=%s" %
          (args.host, args.port, config["execution_enabled"], os.environ.get("ROS_DOMAIN_ID", "0")), flush=True)
    try:
        server.serve_forever(poll_interval=0.1)
    finally:
        server.stopping.set()
        foundation.close()
        vision.close()
        arxgent.close()
        adapter.close()
        server.server_close()


if __name__ == "__main__":
    main()
