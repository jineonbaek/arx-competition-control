#!/usr/bin/env python3
"""LAN web control server: static UI + JSON commands + server-sent feedback."""
import argparse
import base64
import http.cookies
import json
from placement_policy import CHASSIS_FORWARD_SDK, CHASSIS_FORWARD_SECONDS, CHASSIS_COMPLETION_BASIS
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
from visual_docking_live import LiveDockTask
from fast_visual_level import FastVisualLevel
from d435_live_overlay import LiveOverlay
from plan_d_vision import PlanDVision
from navigation_placement import NavigationPlacement,CurrentPositionPlacement
from placement_debug import PlacementDebug
from d405_placement_preview import D405PlacementPreview
from current_axis_push import CurrentAxisPush
from placement_stages import PlacementStages
from placement_reference import PlacementReferenceController
from placement_task import make_placement_task
from competition_task import CompetitionTask
from t2_test_task import make_t2_navigation_task
from t2_timed_placement import TimedT2Placement
from placement_mode_switch import PlacementModeSwitch
from competition_modules import GraspModule, CarryReceiptModule, PlacementModule, QwenRecovery
from t2_preview_backend import make_t2_placement
from agent.battery_production import workflow_factory

WEB_ROOT = Path(__file__).resolve().parents[1]
PUBLIC = WEB_ROOT / "public"


def make_battery_task(controller,port,policy_path,depth_control=None):
    if policy_path is None:return None
    limits=controller.config['arms']['right']['gripper_limits_raw']
    factory=workflow_factory(policy_path,console_url=f'http://127.0.0.1:{port}',
                             gripper_limits=limits)
    from fast_grasp_dock import FastDock
    return BatteryTask(WEB_ROOT.parent/'agent/runs',controller,factory,depth_control=depth_control,
                       fast_dock=FastDock(),
                       fast_start_path=WEB_ROOT.parent/'config/fast-grasp-start.local.json',
                       plan_c_path=WEB_ROOT.parent/'config/battery-plan-c.local.json',
                       recognition_profiles_path=WEB_ROOT/'config/battery-recognition-profiles.json')


def make_handler(controller, arxgent=None, depth_control=None, vision=None, foundation=None, grasp_integration=None, battery_task=None, competition_task=None, placement_task=None, placement_preview=None, cabinet_preview=None, cabinet_rehearsal=None, cabinet_execution=None, visual_docking=None, fast_level=None, t2_preview=None, d435_overlay=None, placement_reference=None, placement_stages=None, t2_test_task=None, placement_modes=None, navigation=None):
    depth_control=depth_control or DepthControl()
    placement_preview=placement_preview or PlacementPreview(controller,depth_control)
    cabinet_preview=cabinet_preview or CabinetPreview(controller,depth_control)
    cabinet_rehearsal=cabinet_rehearsal or CabinetRehearsal(controller)
    cabinet_execution=cabinet_execution or CabinetExecutionTask(controller,cabinet_preview)
    visual_docking=visual_docking or LiveDockTask(controller,cabinet_preview)
    fast_level=fast_level or FastVisualLevel(controller,depth_control)
    placement_debug=PlacementDebug(controller,cabinet_preview,fast_level,WEB_ROOT.parent)
    steps = Steps(controller, depth_control)
    grasps = GraspPreview(controller, depth_control)
    d405_preview=D405PlacementPreview(controller,grasps,WEB_ROOT.parent)
    current_push=CurrentAxisPush(controller,WEB_ROOT.parent)
    head_scene = GraspPreview(controller, depth_control, camera="d435")
    captures={'d435':head_scene,'d405':grasps}
    def observe_mouth(guard,*,seat_only=False):
        from mouth_push import measure_approach
        return measure_approach(captures,controller,placement_stages.tcp,guard,seat_only=seat_only)
    current_push.mouth_observer=observe_mouth
    fast_level.mouth_observer=observe_mouth
    fast_level.wrist_capture=grasps
    def check_depth(task,observation,guard):
        from fast_visual_level import guarded_read
        from mouth_push import reobserve_seat,verify_fixed_depth
        if not observation.get('rail_guided'):
            return verify_fixed_depth(observation,lambda:guarded_read(lambda:observe_mouth(guard),guard,timeout_s=4.5),guard)
        def read():
            return verify_fixed_depth(observation,lambda:observe_mouth(guard,seat_only=True),guard)
        def report(attempt,reason):
            task.save(d405_seat_attempt=attempt,verification_reason=reason,reason='保持夹持，D405 正在重新识别：'+reason)
        return reobserve_seat(read,guard,report,task.cancel.wait)
    from seated_reference import monitor_stroke
    current_push.stroke_monitor=lambda:monitor_stroke(current_push,grasps,controller.models['right'],placement_stages.tcp,WEB_ROOT.parent)
    fast_level.stroke_monitor=lambda:monitor_stroke(fast_level,grasps,controller.models['right'],placement_stages.tcp,WEB_ROOT.parent)
    current_push.check_fixed_depth=lambda observation,guard:check_depth(current_push,observation,guard)
    fast_level.check_fixed_depth=lambda observation,guard:check_depth(fast_level,observation,guard)
    placement_stages=placement_stages or PlacementStages(controller,grasps,current_push,WEB_ROOT.parent)
    fast_level.saved_observation = getattr(placement_stages, 'saved_observation', lambda: None)
    plan_d_vision = PlanDVision(head_scene, controller)
    d435_overlay=d435_overlay or LiveOverlay(head_scene)
    vision = vision or VisionPreview()
    foundation = foundation or FoundationPosePreview(controller,depth_control)
    tcp_calibration = TcpCalibration(controller)
    reference_root=WEB_ROOT/'hardware/runtime/placement-references'
    def capture_reference(include_artifacts):
        state=controller.snapshot();artifacts={};capture={
            'navigation_pose':None,'chassis_pose':None,'chassis_velocity':None,
            'lift_rad':state.get('body',{}).get('height'),
            'body_configuration':{k:state.get('body',{}).get(k) for k in ('height','waist','head_yaw','head_pitch')},
            'left_joints_rad':state.get('arms',{}).get('left',{}).get('joints'),
            'right_joints_rad':state.get('arms',{}).get('right',{}).get('joints'),
            'right_wrist_base_mm_deg':state.get('arms',{}).get('right',{}).get('base_wrist'),
            'command_epoch':state.get('command_epoch')}
        try:
            nav=read_snapshot();capture['navigation_pose']=nav
            if isinstance(nav,dict):
                capture['chassis_pose']=nav.get('pose') or nav.get('robot_pose')
                capture['chassis_velocity']=nav.get('velocity') or nav.get('twist')
        except Exception as error:capture['navigation_error']=str(error)
        try:
            result=cabinet_preview.action({'camera':'d435','model':'demo'},include_arm_preview=False)
            window=result.get('window') or {};image=result.get('image','')
            capture['d435']={'valid':result.get('status')=='localized','captured_at':result.get('image_time'),
                'center_base_m':window.get('center_base_m'),'inside_axis_base':window.get('inside_axis_base'),
                'measured_size_mm':window.get('measured_size_mm'),'rim_plane_p90_mm':window.get('rim_plane_p90_mm'),
                'detected_quad_px':result.get('detected_quad_px'),'image_file':'d435.jpg','image_url':image,
                'status':result.get('status'),'reason':result.get('reason','')}
            if include_artifacts and isinstance(image,str) and image.startswith('data:image/jpeg;base64,'):
                artifacts['d435.jpg']=base64.b64decode(image.split(',',1)[1],validate=True)
        except Exception as error:capture['d435']={'valid':False,'reason':str(error)}
        return capture,artifacts
    placement_reference=placement_reference or PlacementReferenceController(reference_root,capture_reference)
    def competition_view_unlocked(view):
        if placement_modes is not None:
            view['placement_mode']=placement_modes.snapshot()
        owner=t2_test_task if view.get('scope')=='t2_test' else competition_task
        module=owner.modules.get('place') if owner is not None else None
        module=getattr(module,'placement_module',module)
        if isinstance(module,NavigationPlacement):
            view['placement_backend']='lidar_locked_d405_seat_body' if module.lidar_locked else 'visual_chassis_d405_seat_body'
            if isinstance(module,CurrentPositionPlacement):view['placement_backend']='current_position_d405_seat_body'
            view['placement_readiness']=module.readiness()
            view['insertion_policy']=dict(cabinet_depth_mm=150.,distance='minimum_measured_from_stroke_origin',minimum_measured_travel_mm=150.,axis_source='right_tool_positive_x',direct_start_mm=150.,standalone_probe_mm=5.,main_speed_factor=3.,terminal_speed_factor=5.,terminal_max_mm=10.,visual_check='live_reobserve_until_recognized',seat_standard='d405_operator_seated_reference',rotation_drift_limit_deg=None,handle_outside=True,
                visual_verification=True,compensation_step_mm=10.,max_total_travel_mm=300.,unverified_after_attempt='stop_and_hold_closed')
            if module.distance_only:
                view['placement_backend']='current_position_fixed_stroke' if isinstance(module,CurrentPositionPlacement) else 'lidar_locked_fixed_stroke'
                view['insertion_policy'].update(alignment='saved_preobservation',d405_horizontal_enabled=False,
                    visual_check='disabled',seat_standard=None,visual_verification=False,seat_verification_enabled=False,
                    handle_outside=None,completion_basis='measured_axial_travel',compensation_step_mm=None,
                    max_tracking_finish_mm=1.75,max_tracking_finish_attempts=1,max_total_travel_mm=151.75,
                    unverified_after_attempt='not_applicable',short_stroke_action='stop_and_hold_closed')
            if module.chassis_only:
                view['placement_backend']='lidar_special_point_chassis'
                view['insertion_policy']=dict(alignment='saved_preobservation',d405_horizontal_enabled=False,
                    chassis_recheck_enabled=False,chassis_pose_source='navigation_arrival_receipt',
                    seat_verification_enabled=False,visual_verification=False,visual_check='disabled',
                    arm_push_mm=0.,chassis_target='saved_special_route_endpoint',
                    terminal_forward_s=CHASSIS_FORWARD_SECONDS,terminal_forward_sdk=CHASSIS_FORWARD_SDK,
                    release_after='terminal_forward_completed_and_output_ended',
                    retreat='recorded_t2_arrival_pose',completion_basis=CHASSIS_COMPLETION_BASIS)
                for task in view.get('tasks',[]):
                    if task['id']=='place':task['label']='预观察点及 J4 补偿，保持夹持交给底盘'
                view['target']=f'T2 → 预观察点保持 → 已保存特殊点 → 底盘前进{CHASSIS_FORWARD_SECONDS:g}秒 → 松爪、底盘退出、机械臂返回'
            if view.get('run_id'):
                outcome=module.inspect({'request_id':view['run_id']+':place'})
                if outcome.get('arm_progress'):view['placement_progress']=outcome['arm_progress']
                if outcome.get('degraded'):
                    view.update(placement_confirmed=False,completion_quality='unverified',warnings=[outcome['reason']])
                    if view.get('status')=='completed':view['reason']=outcome['reason']
                    for task in view.get('tasks',[]):
                        if task.get('id')=='place':task.update(reason=outcome['reason'],placement_confirmed=False)

        if t2_preview is not None:
            view['t2_preview']={'mode':'recorded_preview','preview_only':True,
                                'notice':'T2 仅历史数据预览，不执行放置，不发送完成回执'}
            for task in view.get('tasks',[]):
                if task.get('id')=='place':task['label']='T2 历史数据预览（不执行放置）'
        return view
    def competition_view(view):
        if placement_modes is None:return competition_view_unlocked(view)
        with placement_modes.lock:return competition_view_unlocked(view)
    def task_action(task,client,data):
        if placement_modes is None:return competition_view(task.action(client,data))
        with placement_modes.lock:
            return competition_view(placement_modes.action(task,client,data))
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
            if path == '/api/placement-debug':
                if not self.client():return self.json(403,{'message':'请先打开视觉放置调试页面'})
                return self.json(200,placement_debug.snapshot())
            if path == '/api/plan-d-vision':
                if not self.client():return self.json(403,{'message':'请先打开自动抓取页面'})
                return self.json(200,plan_d_vision.snapshot())
            if path == '/api/placement-reference':
                if not self.client():return self.json(403,{'message':'请先打开视觉放置页面'})
                try:return self.json(200,placement_reference.snapshot())
                except ValueError as error:return self.json(404,{'message':str(error)})
            if path.startswith('/placement-reference/records/'):
                relative=unquote(path.removeprefix('/placement-reference/'))
                target=(reference_root/relative).resolve()
                if not target.is_relative_to(reference_root.resolve()) or not target.is_file():return self.json(404,{'message':'参考画面不存在'})
                body=target.read_bytes();kind=mimetypes.guess_type(str(target))[0] or 'application/octet-stream'
                self.response_headers(200,kind,len(body));self.end_headers();self.wfile.write(body);return
            if path == '/api/t2-preview':
                if not self.client():return self.json(403,{'message':'请先打开地图与任务页面'})
                return self.json(200,t2_preview.snapshot() if t2_preview else
                    dict(enabled=False,mode='existing_placement',preview_only=False))
            if path == '/api/fast-level':
                if not self.client():return self.json(403,{'message':'请先打开控制页面'})
                return self.json(200,fast_level.snapshot())
            if path == '/api/current-axis-push':
                if not self.client():return self.json(403,{'message':'请先打开控制页面'})
                return self.json(200,current_push.snapshot())
            if path == '/api/placement-stages':
                if not self.client():return self.json(403,{'message':'请先打开放置网页'})
                return self.json(200,placement_stages.snapshot())
            if path == '/api/d435-overlay':
                if not self.client():return self.json(403,{'message':'请先打开视觉放置页面'})
                return self.json(200,d435_overlay.snapshot())
            if path == '/api/visual-docking':
                if not self.client():return self.json(403,{'message':'请先打开控制页面'})
                return self.json(200,visual_docking.snapshot())
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
            if path == '/api/t2-test':
                if not self.client():return self.json(403, {'message':'请先打开T2实测页面'})
                if t2_test_task is None:return self.json(503, {'message':'T2实测入口未启用'})
                return self.json(200,competition_view(t2_test_task.snapshot(self.client())))
            if path == '/api/competition-task':
                if not self.client():return self.json(403, {'message':'请先打开地图与任务页'})
                if competition_task is None:return self.json(503, {'message':'比赛总控未启用'})
                return self.json(200, competition_view(competition_task.snapshot(self.client())))
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
                data['t2_test'] = competition_view(t2_test_task.snapshot(self.client())) if t2_test_task else None
                data['competition'] = competition_view(competition_task.snapshot(self.client())) if competition_task else None
                return self.json(200, data)
            if path == '/api/cabinet-rehearsal':
                if not self.client():return self.json(403,{'message':'请先打开控制页'})
                return self.json(200,cabinet_rehearsal.snapshot())
            if path == "/api/state":
                state=controller.snapshot()
                if navigation is not None:state['terminal_forward']=navigation.terminal_forward_permit()
                return self.json(200,state)
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
            if path in ("/", "/map.html", "/t2-test.html", "/navigation-debug.html", "/handeye.html", "/handle.html", "/vision.html", "/foundationpose.html", "/tcp-calibration.html", "/grasp-integration.html", "/placement.html", "/visual-docking.html") and not self.client():
                self.send_header("Set-Cookie", "arx_web_session=%s; Path=/; HttpOnly; SameSite=Strict" % uuid.uuid4())
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            path = urlsplit(self.path).path
            if path == '/api/plan-d-vision':
                expected = 'http://' + self.headers.get('Host', '')
                if self.headers.get('Origin') != expected or not self.client():
                    self.close_connection = True
                    return self.json(403,{'message':'请求来源无效'})
                try:
                    if self.headers.get('Content-Type','').split(';')[0] != 'application/json':
                        raise ValueError('仅接受 JSON')
                    length = int(self.headers.get('Content-Length','0'))
                    if not 0 < length <= 8192:
                        raise ValueError('请求大小无效')
                    if strict_json(self.rfile.read(length)) != {'op':'capture'}:
                        raise ValueError('仅支持 PlanD 识别采集')
                    return self.json(202,plan_d_vision.refresh())
                except ValueError as error:
                    return self.json(400,{'message':str(error)})
            if path not in ('/api/placement-stages','/api/current-axis-push','/api/d405-placement-preview','/api/placement-debug','/api/placement-reference','/api/d435-overlay','/api/fast-level','/api/visual-docking','/api/cabinet-execution', '/api/cabinet-controller/probe', '/api/cabinet-rehearsal', '/api/cabinet-preview', '/api/placement-preview', '/api/placement-task', '/api/command', '/api/arxgent', '/api/grasp-integration', '/api/battery-task', '/api/competition-task', '/api/t2-test', '/api/camera/depth', '/api/handeye', '/api/handle-preview', '/api/grasp-scene', '/api/vision', '/api/foundationpose', '/api/tcp-calibration'):
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
                if path == '/api/placement-stages':
                    return self.json(202,placement_stages.action(strict_json(raw)))
                if path == '/api/current-axis-push':
                    return self.json(202,current_push.action(strict_json(raw)))
                if path == '/api/d405-placement-preview':
                    return self.json(200,d405_preview.action(strict_json(raw)))
                if path == '/api/placement-debug':
                    return self.json(202,placement_debug.action(self.client(),strict_json(raw)))
                if path == '/api/placement-reference':
                    return self.json(200,placement_reference.action(strict_json(raw)))
                if path == '/api/d435-overlay':
                    data=strict_json(raw)
                    if data not in ({},{'op':'refresh'}):raise ValueError('仅支持刷新 D435 叠加画面')
                    return self.json(202,d435_overlay.refresh())
                if path == '/api/fast-level':
                    return self.json(200,fast_level.action(strict_json(raw)))
                if path == '/api/visual-docking':
                    return self.json(200,visual_docking.action(strict_json(raw)))
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
                if path == '/api/t2-test':
                    if t2_test_task is None:return self.json(503, {'message':'T2实测入口未启用'})
                    return self.json(202,task_action(t2_test_task,self.client(),strict_json(raw)))
                if path == '/api/competition-task':
                    if competition_task is None:return self.json(503, {'message':'比赛总控未启用'})
                    return self.json(202, task_action(competition_task,self.client(),strict_json(raw)))
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
                if isinstance(data,dict) and str(data.get('type','')).startswith('chassis'):
                    kind=data.get('type')
                    if kind in ('chassis.nav_sdk','chassis.gamepad_sdk') and self.client_address[0] not in ('127.0.0.1','::1'):
                        raise Rejected('SDK 原值入口仅接受本机适配器')
                    if kind=='chassis.nav_sdk' or kind=='chassis.end_output' and str(data.get('source_id','')).startswith('navigation-'):
                        return self.json(200,controller.submit_navigation(data,self.client()))
                    return self.json(200,controller.submit(data,self.client()))
                return self.json(200, steps.manual_submit(data, self.client()))
            except ChatError as error:
                return self.json(error.status, {'ok': False, 'message': error.message})
            except (ValueError, TypeError, Rejected) as e:
                self.close_connection = True
                return self.json(400, {"ok": False, "message": str(e)})
            except Exception:
                controller.log("控制接口异常，未确认执行结果", "warning")
                return self.json(503, {"ok": False, "message": "控制接口异常，请查看反馈"})
    Handler.placement_debug_task=placement_debug
    Handler.current_push_task=current_push
    Handler.placement_stages_task=placement_stages
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
    visual_docking=LiveDockTask(controller,cabinet_preview)
    fast_level=FastVisualLevel(controller,depth_control)
    placement_preview=PlacementPreview(controller,depth_control)
    placement_task=make_placement_task(controller,placement_preview,args.placement_policy,depth_control,WEB_ROOT.parent)
    battery_task=make_battery_task(controller,args.port,args.battery_policy,depth_control)
    navigation=NavigationSession(controller,RouteMailbox(STATE_FILE.parent,read_snapshot))
    from chassis_placement import ChassisReleaseModule
    t2_placement=NavigationPlacement(controller,cabinet_preview,fast_level,navigation,WEB_ROOT.parent,lidar_locked=True)
    if not t2_placement.chassis_only:t2_placement=TimedT2Placement(t2_placement,navigation,WEB_ROOT.parent)
    modules={'navigate_pick':navigation.stage('point_1'),'navigate_place':navigation.stage('point_2'),
             'place':t2_placement}
    if t2_placement.chassis_only:modules['release_after_chassis']=ChassisReleaseModule(t2_placement)
    if battery_task:
        grasp=GraspModule(battery_task)
        modules.update(pick=grasp,carry=CarryReceiptModule(grasp))
    competition_task=CompetitionTask(controller, WEB_ROOT.parent/'agent/competition-runs',
        modules=modules,navigation=navigation,recovery=QwenRecovery(arxgent.config))
    single=(NavigationPlacement(controller,cabinet_preview,fast_level,navigation,WEB_ROOT.parent,lidar_locked=True)
            if t2_placement.chassis_only else TimedT2Placement(CurrentPositionPlacement(controller,fast_level,WEB_ROOT.parent),navigation,WEB_ROOT.parent))
    t2_test_task=make_t2_navigation_task(controller,WEB_ROOT.parent/'agent/t2-test-runs',navigation,single)
    if single.chassis_only:t2_test_task.modules['release_after_chassis']=ChassisReleaseModule(single)
    placement_modes=PlacementModeSwitch(controller,cabinet_preview,fast_level,navigation,WEB_ROOT.parent,competition_task,t2_test_task)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(controller, arxgent,
        depth_control=depth_control,vision=vision,foundation=foundation,grasp_integration=grasp_integration,
        battery_task=battery_task,competition_task=competition_task,t2_test_task=t2_test_task,placement_modes=placement_modes,navigation=navigation,placement_task=placement_task,placement_preview=placement_preview,cabinet_rehearsal=cabinet_rehearsal,cabinet_preview=cabinet_preview,cabinet_execution=cabinet_execution,visual_docking=visual_docking,fast_level=fast_level,
        t2_preview=t2_placement if getattr(t2_placement,'preview_only',False) else None))
    server.daemon_threads = True
    server.stopping = threading.Event()
    adapter.start(controller)
    from codex_advice_server import serve as start_advice_mailbox
    try:advice_server=start_advice_mailbox()
    except (OSError,ValueError) as error:
        advice_server=None
        print('可选 Codex 纠错暂不可用：'+str(error),flush=True)
    print('T2 backend: '+('recorded_preview; no placement or completion receipt' if
        getattr(t2_placement,'preview_only',False) else 'existing_placement'),flush=True)
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
        server.RequestHandlerClass.placement_debug_task.close()
        server.RequestHandlerClass.placement_stages_task.close()
        server.RequestHandlerClass.current_push_task.close()
        fast_level.close()
        visual_docking.close()
        cabinet_execution.close()
        cabinet_rehearsal.close()
        if placement_task is not None:placement_task.close()
        t2_test_task.close()
        competition_task.close()
        if getattr(t2_placement,'preview_only',False):t2_placement.close()
        foundation.close()
        vision.close()
        arxgent.close()
        grasp_integration.close()
        if battery_task is not None:battery_task.close()
        if advice_server is not None:
            advice_server.shutdown();advice_server.server_close()
        adapter.close()
        server.server_close()


if __name__ == "__main__":
    main()
