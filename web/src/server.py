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
from urllib.parse import unquote, urlsplit, parse_qs
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
from tcp_calibration import TcpCalibration
from grasp_integration import GraspIntegration
from navigation_view import read_snapshot, STATE_FILE
from navigation_route import RouteMailbox
from navigation_session import NavigationSession
from battery_task import BatteryTask
from placement_preview import PlacementPreview
from agent.cabinet_readiness import readiness as cabinet_readiness, admission_probe as cabinet_admission_probe
from cabinet_preview import CabinetPreview
from cabinet_rehearsal import CabinetRehearsal
from cabinet_execution import CabinetExecutionTask
from fast_visual_level import FastVisualLevel
from placement_task import make_placement_task
from competition_task import CompetitionTask
from competition_modules import GraspModule, CarryReceiptModule, PlacementModule, QwenRecovery
from agent.battery_production import workflow_factory

WEB_ROOT = Path(__file__).resolve().parents[1]
PUBLIC = WEB_ROOT / "public"


def make_battery_task(controller,port,policy_path,depth_control=None):
    if policy_path is None:return None
    limits=controller.config['arms']['right']['gripper_limits_raw']
    factory=workflow_factory(policy_path,console_url=f'http://127.0.0.1:{port}',
                             gripper_limits=limits)
    return BatteryTask(WEB_ROOT.parent/'agent/runs',controller,factory,depth_control=depth_control)


def make_handler(controller, arxgent=None, depth_control=None, vision=None, foundation=None, grasp_integration=None, battery_task=None, competition_task=None, placement_task=None, placement_preview=None, cabinet_preview=None, cabinet_rehearsal=None, cabinet_execution=None, fast_level=None):
    depth_control=depth_control or DepthControl()
    placement_preview=placement_preview or PlacementPreview(controller,depth_control)
    cabinet_preview=cabinet_preview or CabinetPreview(controller,depth_control)
    cabinet_rehearsal=cabinet_rehearsal or CabinetRehearsal(controller)
    cabinet_execution=cabinet_execution or CabinetExecutionTask(controller,cabinet_preview)
    fast_level=fast_level or FastVisualLevel(controller,depth_control)
    steps = Steps(controller, depth_control)
    grasps = GraspPreview(controller, depth_control)
    head_scene = GraspPreview(controller, depth_control, camera="d435")
    vision = vision or VisionPreview()
    foundation = foundation or FoundationPosePreview(controller,depth_control)
    tcp_calibration = TcpCalibration(controller)
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
            if path == '/api/fast-level':
                if not self.client():return self.json(403,{'message':'请先打开控制页面'})
                return self.json(200,fast_level.snapshot())
            if path == '/api/cabinet-execution':
                if not self.client():return self.json(403,{'message':'请先打开控制页面'})
                return self.json(200,cabinet_execution.snapshot())
            if path == '/api/cabinet-controller':
                if not self.client():return self.json(403,{'message':'请先打开控制页面'})
                return self.json(200,dict(cabinet_readiness(controller), execution_enabled=cabinet_execution.snapshot()['execution_enabled'], execution_backend='existing_controller', reason='使用现有控制器执行完整水平放置；原独立桥接探针不发送运动'))
            if path == '/api/cabinet-preview':
                if not self.client():return self.json(403,{'message':'请先打开入仓预览页'})
                return self.json(200,cabinet_preview.snapshot())
            if path == '/api/placement-preview':
                if not self.client():return self.json(403,{'message':'请先打开自动放置页面'})
                return self.json(200,placement_preview.latest)
            if path == '/api/placement-task':
                if not self.client():return self.json(403, {'message':'请先打开自动放置页面'})
                if placement_task is None:return self.json(200, {'status':'idle','execution_enabled':False,'available':False,'reason':'仅预览，未配置放置执行后端'})
                return self.json(200, placement_task.snapshot(self.client()))
            if path == '/api/competition-task':
                if not self.client():return self.json(403, {'message':'请先打开地图与任务页'})
                if competition_task is None:return self.json(503, {'message':'比赛总控未启用'})
                return self.json(200, competition_task.snapshot(self.client()))
            if path == "/api/battery-task/result":
                if not self.client():return self.json(403,{"message":"请先打开控制台"})
                if battery_task is None:return self.json(503,{"message":"自动抓取执行后端未启用"})
                try:
                    after=parse_qs(urlsplit(self.path).query).get('after',[''])[0]
                    return self.json(200,battery_task.result(after))
                except (OSError,ValueError):return self.json(503,{"message":"任务输出正在更新，请稍后重试"})
            if path == "/api/battery-task":
                client=self.client()
                if not client:return self.json(403,{"message":"请先打开控制台"})
                if battery_task is None:return self.json(503,{"message":"自动抓取执行后端未启用"})
                return self.json(200,battery_task.snapshot(client))
            if path in ("/api/grasp-integration", "/api/grasp-integration/result"):
                if not self.client():return self.json(403, {"message":"请先打开抓取链路预览页"})
                if grasp_integration is None:return self.json(503, {"message":"抓取链路预览未启用"})
                try:
                    result=grasp_integration.result(self.client()) if path.endswith("/result") else grasp_integration.snapshot(self.client())
                    return self.json(200,result)
                except ChatError as error:return self.json(error.status,{"message":error.message})
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
            if path == "/api/tcp-calibration":
                client = self.client()
                if not client:
                    return self.json(403, {"message":"请先打开 TCP 标定页面"})
                return self.json(200, tcp_calibration.state(client))
            if path == "/api/navigation-view":
                if not self.client():
                    return self.json(403, {"message": "请先打开地图页面"})
                data = read_snapshot()
                task = battery_task.snapshot(self.client()) if battery_task else {"status": "unavailable", "phase": "idle"}
                data["task"] = {key: task.get(key) for key in ("status", "phase", "mission_stage", "reason", "run_id", "recovery_required")}
                data['competition'] = competition_task.snapshot(self.client()) if competition_task else None
                return self.json(200, data)
            if path == '/api/cabinet-rehearsal':
                if not self.client():return self.json(403,{'message':'请先打开控制页'})
                return self.json(200,cabinet_rehearsal.snapshot())
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
            if path in ("/", "/map.html", "/handeye.html", "/handle.html", "/vision.html", "/foundationpose.html", "/tcp-calibration.html", "/grasp-integration.html", "/placement.html", "/visual-docking.html") and not self.client():
                self.send_header("Set-Cookie", "arx_web_session=%s; Path=/; HttpOnly; SameSite=Strict" % uuid.uuid4())
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            path = urlsplit(self.path).path
            if path not in ('/api/fast-level','/api/cabinet-execution', '/api/cabinet-controller/probe', '/api/cabinet-rehearsal', '/api/cabinet-preview', '/api/placement-preview', '/api/placement-task', '/api/command', '/api/arxgent', '/api/grasp-integration', '/api/battery-task', '/api/competition-task', '/api/camera/depth', '/api/handeye', '/api/handle-preview', '/api/grasp-scene', '/api/vision', '/api/foundationpose', '/api/tcp-calibration'):
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
                if path == '/api/fast-level':
                    return self.json(200,fast_level.action(strict_json(raw)))
                if path == '/api/cabinet-execution':
                    return self.json(200,cabinet_execution.action(strict_json(raw)))
                if path == '/api/cabinet-controller/probe':
                    return self.json(200,cabinet_admission_probe(controller,strict_json(raw)))
                if path == '/api/cabinet-rehearsal':
                    return self.json(202,cabinet_rehearsal.action(strict_json(raw)))
                if path == '/api/cabinet-preview':
                    return self.json(200,cabinet_preview.action(strict_json(raw)))
                if path == '/api/placement-preview':
                    if placement_task is not None and placement_task.snapshot(self.client()).get('status')=='running':raise ValueError('执行期间不能另开红圈采样')
                    return self.json(200,placement_preview.action(strict_json(raw)))
                if path == '/api/placement-task':
                    if placement_task is None:return self.json(503,{'message':'放置执行后端未启用'})
                    return self.json(202,placement_task.action(self.client(),strict_json(raw)))
                if path == '/api/competition-task':
                    if competition_task is None:return self.json(503, {'message':'比赛总控未启用'})
                    return self.json(202, competition_task.action(self.client(), strict_json(raw)))
                if path == '/api/battery-task':
                    if battery_task is None:return self.json(503,{"message":"自动抓取执行后端未启用"})
                    return self.json(202,battery_task.action(self.client(),strict_json(raw)))
                if path == '/api/grasp-integration':
                    if grasp_integration is None:return self.json(503,{"message":"抓取链路预览未启用"})
                    return self.json(202,grasp_integration.handle(self.client(),strict_json(raw)))
                if path == '/api/tcp-calibration':
                    return self.json(200, tcp_calibration.action(self.client(), strict_json(raw)))
                if path == '/api/vision':
                    return self.json(200, vision.action(self.client(), strict_json(raw)))
                if path == '/api/foundationpose':
                    return self.json(200,foundation.action(self.client(),strict_json(raw)))
                if path == '/api/grasp-scene':
                    data=strict_json(raw)
                    if not isinstance(data,dict) or set(data)!={'camera'} or data['camera'] not in ('d435','d405'):
                        raise ValueError('仅接受 camera=d435 或 d405')
                    capture=head_scene if data['camera']=='d435' else grasps
                    return self.json(200,capture.handle_action({'op':'capture'}))
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
                if isinstance(data,dict) and (data.get('type')=='chassis.nav_sdk' or
                        data.get('type')=='chassis.stop' and self.client_address[0] in ('127.0.0.1','::1')
                        and controller.is_navigation_client(self.client())):
                    return self.json(200,steps.manual_submit(data,controller.navigation_client(data,self.client())))
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
    parser.add_argument("--battery-policy", help="explicit fixed-flow execution policy")
    parser.add_argument("--placement-policy",help="explicit local placement policy; default preview only")
    args = parser.parse_args()
    config = json.loads((WEB_ROOT / "config/control.json").read_text())
    adapter = RosAdapter(config)
    controller = Controller(config, adapter)
    depth_control=DepthControl()
    arxgent = ArxgentChat.from_repository(WEB_ROOT.parent, args.port)
    grasp_integration = GraspIntegration.from_repository(WEB_ROOT.parent, args.port)
    vision = VisionPreview()
    foundation = FoundationPosePreview(controller,depth_control)
    cabinet_rehearsal=CabinetRehearsal(controller)
    cabinet_preview=CabinetPreview(controller,depth_control)
    cabinet_execution=CabinetExecutionTask(controller,cabinet_preview)
    fast_level=FastVisualLevel(controller,depth_control)
    placement_preview=PlacementPreview(controller,depth_control)
    placement_task=make_placement_task(controller,placement_preview,args.placement_policy,depth_control,WEB_ROOT.parent)
    battery_task=make_battery_task(controller,args.port,args.battery_policy,depth_control)
    navigation=NavigationSession(controller,RouteMailbox(STATE_FILE.parent,read_snapshot))
    modules={'navigate_pick':navigation.stage('point_1'),'navigate_place':navigation.stage('point_2'),
             'place':PlacementModule(cabinet_execution)}
    if battery_task:
        grasp=GraspModule(battery_task)
        modules.update(pick=grasp,carry=CarryReceiptModule(grasp))
    competition_task=CompetitionTask(controller, WEB_ROOT.parent/'agent/competition-runs',
        modules=modules,navigation=navigation,recovery=QwenRecovery(arxgent.config))
    server = ThreadingHTTPServer((args.host, args.port), make_handler(controller, arxgent,
        depth_control=depth_control,vision=vision,foundation=foundation,grasp_integration=grasp_integration,
        battery_task=battery_task,competition_task=competition_task,placement_task=placement_task,placement_preview=placement_preview,cabinet_rehearsal=cabinet_rehearsal,cabinet_preview=cabinet_preview,cabinet_execution=cabinet_execution,fast_level=fast_level))
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
        fast_level.close()
        cabinet_execution.close()
        cabinet_rehearsal.close()
        if placement_task is not None:placement_task.close()
        competition_task.close()
        foundation.close()
        vision.close()
        arxgent.close()
        grasp_integration.close()
        if battery_task is not None:battery_task.close()
        adapter.close()
        server.server_close()


if __name__ == "__main__":
    main()
