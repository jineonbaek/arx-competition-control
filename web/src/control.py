"""Pure validation and control state. No SDK construction or device access."""
import copy
import json
import math
import threading
import time
from collections import deque
from functools import lru_cache

import numpy as np
from scipy.spatial.transform import Rotation
from handeye_column import ColumnModel
from chassis_scheduler import ChassisScheduler, InvalidChassis, IDENTITY_FIELDS

DIRECTIONS = {
    "forward": (1, 0, 0), "backward": (-1, 0, 0),
    "left": (0, 1, 0), "right": (0, -1, 0),
    "yaw-left": (0, 0, 1), "yaw-right": (0, 0, -1),
}

ARM_MODES = {0: "力矩清零", 1: "机械臂复位", 2: "阻尼模式", 3: "重力补偿"}
LEFT_HOLD_JOINTS_DEG = [0., 0., 0., 73., 0., 0.]
LEFT_HOLD_2_JOINTS_DEG = [0., 0., 90., 0., 0., 0.]
RIGHT_AVOIDANCE_JOINTS_DEG = [0., 0., 65., -65., 0., 0.]
LEFT_HOLD_TOLERANCE_DEG = 2.


def number(value):
    return ((type(value) is int and value.bit_length() <= 1023)
            or (isinstance(value, float) and math.isfinite(value)))


def bounds(pair):
    return isinstance(pair, list) and len(pair) == 2 and all(number(x) for x in pair) and pair[0] < pair[1]


def vector(value, length):
    return isinstance(value, list) and len(value) == length and all(number(x) for x in value)


class Rejected(ValueError):
    pass


class _MotionToken:
    """Process-local capability; never supplied by a browser or serialized."""
    def __init__(self, prestarted_lift_target=None):
        self.prestarted_lift_target = prestarted_lift_target
        self.contact_stop_pending = False


class _CompetitionToken:
    def __init__(self):
        self.interrupted = False
        self.navigation = False
        self.nav_session = None
        self.nav_client = _NavigationToken()
        self.prestarted_lift_target = None


class _NavigationToken:
    pass


@lru_cache(maxsize=2)
def arm_model(side):
    return ColumnModel(side=side)


class Controller:
    def __init__(self, config, adapter, clock=time.monotonic):
        self.config = copy.deepcopy(config)
        self.adapter = adapter
        self.clock = clock
        self.lock = threading.RLock()
        self.arms = {side: None for side in ("left", "right")}
        self.body = None
        self.lift_history = deque()
        self.logs = deque(maxlen=100)
        self.serial = 0
        self.command_epoch = 0
        self.motion_owner = None
        self.competition_owner = None
        self.motion_interrupted = False
        self.motion_trajectory = None
        self.models = {}
        for side in self.arms:
            try:
                self.models[side] = arm_model(side)
            except (OSError, ValueError) as error:
                self.log(f"{side} 整车模型不可用：{error}", "warning")
        self.chassis_owner = None
        self.chassis_deadline = 0
        self.chassis_message = None
        self.base_sequences = {}
        self.chassis_errors=(InvalidChassis,)
        if config.get('chassis_backend') == 'ros':
            import sys
            from pathlib import Path
            sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'chassis_runtime'))
            from arx_chassis.proxy import ChassisProxy
            from arx_chassis.core import Rejected as RemoteChassisRejected
            self.chassis_errors=(InvalidChassis,RemoteChassisRejected)
            self.chassis=ChassisProxy()
        else:
            self.chassis = ChassisScheduler(lambda output:self.adapter.chassis_output(output), clock=clock,
                                           lease=config["chassis_lease_s"], lock=self.lock)

    def reserve_competition(self, expected_epoch):
        with self.lock:
            if (type(expected_epoch) is not int or expected_epoch != self.command_epoch
                    or not self.config['execution_enabled'] or self.competition_owner is not None
                    or self.motion_owner is not None or self.chassis_owner is not None):
                raise Rejected('已有任务或控制状态已变化，不能开始整场比赛')
            self.competition_owner = _CompetitionToken()
            self.command_epoch += 1
            return self.competition_owner

    def competition_valid(self, token):
        with self.lock:
            return token is self.competition_owner and token is not None and not token.interrupted

    def set_competition_navigation(self, token, enabled):
        with self.lock:
            if token is None or token is not self.competition_owner:
                raise Rejected('整场任务控制权已失效')
            if enabled and token.interrupted:
                raise Rejected('整场任务已被明确停止')
            token.navigation = bool(enabled)

    def submit_competition_lift(self, data, token):
        """Start only the existing body lift while competition navigation owns the chassis."""
        with self.lock:
            if token is None or token is not self.competition_owner or token.interrupted:
                raise Rejected('整场任务控制权已失效')
            kind = data.get('type') if isinstance(data, dict) else None
            if (kind == 'lift' and set(data) == {'type', 'target'}):
                result = self.submit(data, token.nav_client)
                token.prestarted_lift_target = float(data['target'])
                return result
            if kind == 'lift.stop' and set(data) == {'type'}:
                result = self.submit(data, token.nav_client)
                token.prestarted_lift_target = None
                return result
            raise Rejected('比赛预升降仅接受现有升降与停止指令')

    def stop_competition_navigation(self, token):
        with self.lock:
            self.set_competition_navigation(token, False)
            # The route cancellation ends its captured output identity in ROS.
            # Do not close every generation belonging to the HTTP caller.

    def release_competition(self, token):
        with self.lock:
            if token is None or token is not self.competition_owner:
                raise Rejected('整场任务控制权已失效')
            if self.motion_owner is not None:
                raise Rejected('机械臂作业令牌尚未清理')
            self.competition_owner = None
            self.command_epoch += 1

    def submit_navigation(self, data, client):
        """Called only by the existing loopback SDK channel; standalone behavior stays intact."""
        return self.submit(data,self.navigation_client(data,client))

    def is_navigation_client(self, client):
        with self.lock:
            return self.competition_owner is not None and self.competition_owner.nav_session == client

    def navigation_client(self, data, client):
        # Identity translation is not a chassis ownership grant.
        with self.lock:
            if not isinstance(data, dict) or data.get('type') not in (
                    'chassis.nav_sdk', 'chassis.end_output', 'chassis.stop'):
                raise Rejected('导航通道仅接受底盘执行或结束输出')
            owner = self.competition_owner
            if owner is not None:
                if owner.nav_session is None:owner.nav_session = client
            return client

    def reserve_motion(self, expected_epoch, *, competition=None):
        """Reserve this controller only; the executor must also verify physical rest."""
        with self.lock:
            if self.competition_owner is not None and not (
                    competition is self.competition_owner and self.competition_valid(competition)):
                raise Rejected('整场自动任务占用控制权')
            if (type(expected_epoch) is not int or expected_epoch != self.command_epoch
                    or not self.config["execution_enabled"]):
                raise Rejected("控制权申请时指令状态已变化或真实控制未启用")
            if self.motion_owner is not None or self.chassis_owner is not None:
                raise Rejected("已有任务或底盘运动占用控制权")
            inherited = competition is not None and competition is self.competition_owner
            prestarted = competition.prestarted_lift_target if inherited else None
            self.motion_owner = _MotionToken(prestarted)
            if inherited:
                competition.prestarted_lift_target = None
            self.motion_interrupted = False
            self.command_epoch += 1
            return self.motion_owner

    def take_prestarted_lift(self, token, target):
        """Transfer the already-issued lift target into the original measured check."""
        with self.lock:
            if not self.motion_valid(token) or not number(target):
                return False
            started = token.prestarted_lift_target
            if not number(started) or not math.isclose(float(started), float(target), abs_tol=1e-9):
                return False
            token.prestarted_lift_target = None
            return True

    def motion_valid(self, token):
        with self.lock:
            return isinstance(token, _MotionToken) and token is self.motion_owner and not self.motion_interrupted

    def interrupt_motion(self, token):
        """Latch an executor fault without surrendering ownership before cleanup."""
        with self.lock:
            if not isinstance(token, _MotionToken) or token is not self.motion_owner:
                raise Rejected("任务控制权已失效")
            self.motion_interrupted = True
            self.command_epoch += 1

    def begin_contact_stop(self, token):
        """Retain ownership across child, debug and competition task cleanup."""
        with self.lock:
            if not isinstance(token, _MotionToken) or token is not self.motion_owner:
                raise Rejected('阻塞停止缺少任务控制权')
            token.contact_stop_pending = True

    def complete_contact_stop(self, token):
        """Internal executor calls this only after stop ack and measured rest."""
        with self.lock:
            if not self.motion_valid(token) or not token.contact_stop_pending:
                raise Rejected('阻塞停止控制权已变化')
            token.contact_stop_pending = False

    def release_motion(self, token):
        """Stop any leftover trajectory and return ownership when the executor exits."""
        with self.lock:
            if not isinstance(token, _MotionToken) or token is not self.motion_owner:
                raise Rejected("任务控制权已失效")
            if token.contact_stop_pending:
                raise Rejected('阻塞停止尚未确认，保留控制权和夹持')
            if self.motion_trajectory is not None:
                self._trajectory_abort(self.motion_trajectory['side'],token)
                if self.motion_trajectory is not None:
                    raise Rejected("轨迹停止请求未发出，不能释放控制权")
            self.end_chassis_output(token)
            self.motion_owner = None
            self.motion_interrupted = False
            self.command_epoch += 1

    def _check_motion_owner(self, kind, client):
        # Called under the same lock as dispatch, including after unlocked wrist IK.
        owner = self.competition_owner
        if isinstance(client,_NavigationToken) and (owner is None or client is not owner.nav_client):
            raise Rejected('旧导航授权已失效')
        if owner is not None and not (client is not None and client is self.motion_owner):
            stops = ('arm.stop', 'lift.stop', 'chassis.stop')
            if client is owner.nav_client:
                if kind != 'chassis.stop' and (not owner.navigation or owner.interrupted):
                    raise Rejected('当前阶段未授权导航运动')
            elif kind in stops:
                owner.interrupted = True
                self.command_epoch += 1
            else:
                raise Rejected('整场自动任务占用控制权，手动运动暂不可用')
        if isinstance(client, _MotionToken) and client is not self.motion_owner:
            raise Rejected("旧任务控制权已失效")
        if self.motion_owner is None:
            return
        if kind in ("arm.stop", "lift.stop", "chassis.stop"):
            if client is not self.motion_owner:
                self.motion_interrupted = True
                self.command_epoch += 1
            return
        if client is not self.motion_owner or self.motion_interrupted:
            raise Rejected("自动任务占用控制权或已被人工停止；禁止继续运动")
        if self.motion_owner.contact_stop_pending:
            raise Rejected('阻塞停止尚未确认，禁止继续运动')
        if self.motion_trajectory is not None:
            raise Rejected("轨迹尚未完成或停止核验，禁止发送其他运动")

    def _trajectory_abort(self, side, token):
        # Cancel ownership before requesting stop; never release after an uncertain dispatch.
        with self.lock:
            if token is not self.motion_owner:
                return
            self.motion_interrupted = True
            self.command_epoch += 1
            if self.motion_trajectory is not None:
                if self.motion_trajectory.get('stop_requested'):
                    return
                self.motion_trajectory['stop_requested'] = True
            try:
                if not self.adapter.arm_stop_ready(side):
                    raise Rejected("轨迹停止服务不可用")
                self.adapter.arm_stop(side)
                self.motion_trajectory = None
            except Exception as error:
                if self.motion_trajectory is not None:
                    self.motion_trajectory.pop('stop_requested',None)
                self.log("轨迹停止请求失败：" + str(error), "warning")

    def start_trajectory(self, side, plan, token, *, preflight, require_lift_settle=True,
                         expected_driver=None):
        """Process-only entry: planner checks scene, calibrated TCP and rest in preflight."""
        with self.lock:
            if side not in self.arms or not self.motion_valid(token) or not callable(preflight):
                raise Rejected("轨迹需要有效任务控制权和执行前复查")
            if token.contact_stop_pending:
                raise Rejected('阻塞停止尚未确认，禁止启动轨迹')
            if self.motion_trajectory is not None:
                raise Rejected("已有轨迹待完成或清理")
            reason = (self.wrist_reason(side) if require_lift_settle
                      else self._wrist_reason(side,require_settle=False))
            if reason:
                raise Rejected(reason)
            if preflight(self.snapshot(), self.clock()) is not True or not self.motion_valid(token):
                raise Rejected("轨迹执行前复查未许可")
            # Reserve before publish: failure can mean accepted but acknowledgement lost.
            self.motion_trajectory = {"side": side, "handle": None, "started_at": self.clock(),
                                      "require_lift_settle": bool(require_lift_settle)}
            try:
                expected = self.adapter.trajectory_status(side)
                if expected_driver is not None:
                    if (not isinstance(expected_driver,dict) or not isinstance(expected,dict)
                            or any(expected.get(k)!=expected_driver.get(k)
                                   for k in ('driver_instance','command_epoch'))):
                        raise Rejected('审批绑定的轨迹驱动/代次已变化')
                    # Keep the approved driver epoch through the atomic start check.
                    expected = copy.deepcopy(expected_driver)
                handle, confirmation = self.adapter.trajectory_start(side, plan, expected)
                self.motion_trajectory["handle"] = copy.deepcopy(handle)
                self.command_epoch += 1
            except Exception:
                self._trajectory_abort(side, token)
                raise
        try:
            confirmation()
            with self.lock:
                if not self.motion_valid(token):
                    raise Rejected("确认期间任务已被停止")
                return copy.deepcopy(handle)
        except Exception:
            self._trajectory_abort(side, token)
            raise

    def poll_trajectory(self, token, *, preflight):
        """Caller polls explicitly; no independent heartbeat can outlive its task checks."""
        with self.lock:
            if not isinstance(token, _MotionToken) or token is not self.motion_owner or self.motion_trajectory is None:
                raise Rejected("没有匹配的轨迹任务")
            side,handle = self.motion_trajectory['side'],self.motion_trajectory['handle']
            try:
                require_settle=self.motion_trajectory.get("require_lift_settle",True)
                reason=(self.wrist_reason(side) if require_settle
                        else self._wrist_reason(side,require_settle=False))
                if not self.motion_valid(token) or not callable(preflight) or reason:
                    raise Rejected("轨迹任务中断或反馈失效")
                if preflight(self.snapshot(), self.clock()) is not True or not self.motion_valid(token):
                    raise Rejected("轨迹持续复查未许可")
                state = self.adapter.trajectory_status(side)
                if (handle and state and state['ready'] and state['enabled']
                        and state['status'] in ('idle','failed','reference_complete')
                        and state['driver_instance']==handle['driver_instance']
                        and state['command_epoch']==handle['command_epoch']
                        and state['request_id']!=handle['request_id']
                        and 0<=self.clock()-self.motion_trajectory['started_at']<.3):
                    return {"status":"awaiting_driver_status", "arrived":False}
                if (not handle or not state or not state['ready'] or not state['enabled']
                        or any(state[k] != handle[k] for k in ('driver_instance','request_id'))
                        or state['command_epoch'] != handle['command_epoch']+1):
                    raise Rejected("轨迹回执缺失、过期或驱动已变化")
                if state['status'] == 'reference_complete':
                    self.motion_trajectory = None
                    return {"status":"reference_complete", "arrived":False}
                if state['status'] != 'running':
                    raise Rejected("轨迹终止：" + state.get('reason','unknown'))
                self.adapter.trajectory_keepalive(side, handle)
                return {"status":"running", "arrived":False}
            except Exception:
                self._trajectory_abort(side, token)
                raise

    def pause_trajectory_for_feedback(self, token):
        """Keep ownership only after the existing stop service acknowledges the pause."""
        with self.lock:
            if not self.motion_valid(token):raise Rejected('任务控制权已失效')
            if self.motion_trajectory is None:return False
            side=self.motion_trajectory['side']
            try:
                if not self.config['arms'][side]['stop_verified'] or not self.adapter.arm_stop_ready(side):
                    raise Rejected('轨迹停止服务不可用')
                self.adapter.arm_stop(side)
                self.motion_trajectory=None
                return True
            except Exception:
                self.begin_contact_stop(token);self._trajectory_abort(side,token)
                raise

    def clear_stopped_trajectory(self, token, *, verify_stopped):
        """Failure cleanup requires a caller's measured-stop check, never just a sent stop."""
        with self.lock:
            if (not isinstance(token,_MotionToken) or token is not self.motion_owner
                    or not self.motion_interrupted or self.motion_trajectory is None
                    or not callable(verify_stopped)
                    or verify_stopped(self.snapshot(),self.clock()) is not True):
                raise Rejected("尚未核验轨迹停止，保留任务控制权")
            state = self.adapter.trajectory_status(self.motion_trajectory['side'])
            if not state or state['status'] not in ('idle','failed','reference_complete'):
                raise Rejected("驱动仍在运行或状态未知，保留任务控制权")
            self.motion_trajectory = None

    def log(self, text, level="info"):
        with self.lock:
            self.serial += 1
            self.logs.append({"id": self.serial, "time": time.time(), "level": level, "text": text})

    def update_arm(self, side, joints, gripper, tcp, *, currents=None):
        if side not in self.arms or not vector(joints, 6) or not number(gripper) or not vector(tcp, 6):
            return
        with self.lock:
            self.arms[side] = {"joints": joints[:], "gripper": gripper, "tcp": tcp[:],
                               "currents": currents[:] if vector(currents, 6) else None,
                               "received": self.clock(), "timestamp": time.time()}

    def update_body(self, height, wheels, waist, yaw, pitch):
        if not all(number(x) for x in [height, waist, yaw, pitch]) or not vector(wheels, 4):
            return
        with self.lock:
            now = self.clock()
            self.lift_history.append((now, height))
            while len(self.lift_history)>1 and self.lift_history[0][0] < now - .5:
                self.lift_history.popleft()
            self.body = {"height": height, "wheels": wheels[:], "waist": waist,
                         "head_yaw": yaw, "head_pitch": pitch,
                         "received": self.clock(), "timestamp": time.time()}

    def fresh(self, state):
        return state is not None and self.clock() - state["received"] <= self.config["feedback_timeout_s"]

    def arm_reason(self, side, kind, mode=None):
        c = self.config["arms"][side]
        if not self.config["execution_enabled"]:
            return "观察模式，真实控制尚未启用"
        if not c["verified"]:
            return "机械臂坐标、限位和控制接口待验证"
        if not self.fresh(self.arms[side]):
            return "机械臂反馈缺失或已过期"
        if c.get("ready_topic") and not self.adapter.arm_ready(side):
            return "机械臂驱动未就绪、电机故障或 CAN 反馈已过期"
        if kind == "arm.stop" and c.get("stop_service") and not self.adapter.arm_stop_ready(side):
            return "机械臂停止接口未连接"
        if not isinstance(c["joint_limits_deg"], list) or len(c["joint_limits_deg"]) != 6 or not all(bounds(x) for x in c["joint_limits_deg"]):
            return "六轴安全范围未配置"
        if not bounds(c["gripper_limits_raw"]):
            return "夹爪开合范围未配置"
        if not number(c["max_delta_deg"]) or c["max_delta_deg"] <= 0:
            return "单次关节偏差限制未配置"
        if kind == "arm.tcp" and (not c["tcp_verified"] or not isinstance(c["tcp_limits_mm"], list) or len(c["tcp_limits_mm"]) != 3 or not all(bounds(x) for x in c["tcp_limits_mm"])):
            return "末端坐标、可达范围和执行行为待验证"
        if kind == "arm.mode":
            verified = c.get("verified_modes", [])
            if not isinstance(verified, list) or not any(type(value) is int and value == mode for value in verified):
                return "该机械臂 mode %s 切换行为待实机验证" % mode
        if kind == "arm.stop" and not c["stop_verified"]:
            return "机械臂停止／保持行为待验证"
        if not self.adapter.has_subscriber(c["command_topic"], "arm"):
            return "专用机械臂控制节点未连接"
        return ""

    def body_reason(self, kind):
        if kind=="chassis.stop":return "" # Explicit all-car stop bypasses ordinary motion gates.
        c = self.config["body"]
        if not self.config["execution_enabled"]:
            return "观察模式，真实控制尚未启用"
        if not self.fresh(self.body):
            return "机体反馈缺失或已过期"
        if c.get("calibrated_topic") and not self.adapter.body_calibrated():
            return "机体回零尚未确认或回零状态已过期"
        if kind.startswith("lift"):
            if not c["lift_verified"] or not bounds(c["lift_limits_raw"]):
                return "升降坐标、零位参考和范围待验证"
            if kind == "lift.stop" and not c["lift_stop_verified"]:
                return "升降停止／保持行为待验证"
            if kind == "lift.stop" and c.get("lift_stop_service") and not self.adapter.lift_stop_ready():
                return "升降停止接口未连接"
            if not self.adapter.lift_ready():
                return "专用升降参数接口未连接"
        else:
            if not c["chassis_verified"] or not c["chassis_stop_verified"]:
                return "底盘方向、速度和超时停车待验证"
            if not all(number(c[x]) and c[x] > 0 for x in ("max_linear_m_s", "max_angular_rad_s")):
                return "底盘速度限制未配置"
            if not self.adapter.chassis_ready():
                return "专用底盘控制节点未连接"
        return ""

    def wrist_feedback_reason(self, side):
        if side not in self.models:
            return "整车 CAD 模型缺失"
        if not self.fresh(self.arms[side]) or not self.fresh(self.body):
            return "等待新鲜机械臂和升降反馈"
        if abs(self.body["received"] - self.arms[side]["received"]) > .20:
            return "手臂和升降反馈时间未配对"
        if not self.adapter.body_calibrated():
            return "等待升降回零确认"
        limits = self.config["body"].get("lift_limits_raw")
        if not bounds(limits) or not limits[0] - .05 <= self.body["height"] <= limits[1] + .05:
            return "升降反馈超出模型范围"
        return ""

    def wrist_reason(self, side):
        return self._wrist_reason(side,require_settle=True)

    def _wrist_reason(self, side, *, require_settle):
        reason = self.arm_reason(side, "arm.joints") or self.wrist_feedback_reason(side)
        if reason:
            return reason
        if self.chassis_owner is not None:
            return "底盘正在运动"
        if require_settle:
            samples = self.lift_history
            if (len(samples)<3 or samples[-1][0]-samples[0][0]<.25
                    or max(b[0]-a[0] for a,b in zip(samples,list(samples)[1:]))>.20
                    or max(h for _,h in samples)-min(h for _,h in samples)>.05):
                return "等待升降停稳"
        return ""

    def wrist_command(self, data, client):
        fields = {"type","arm","frame","position_mm","orientation_deg"}
        side = data.get("arm")
        if set(data) != fields or side not in ("left","right") or data["frame"] != "base_link":
            raise Rejected("腕部目标需要指定左右臂和整车 base_link")
        xyz, rpy = data["position_mm"], data["orientation_deg"]
        if not vector(xyz, 3) or not vector(rpy, 3) or max(map(abs,rpy)) > 360:
            raise Rejected("腕部位置或姿态角无效")
        with self.lock:
            self._check_motion_owner("arm.wrist", client)
            reason = self.wrist_reason(side)
            if reason:
                raise Rejected(reason)
            q = self.arms[side]["joints"][:]
            h = self.body["height"]
            epoch = self.command_epoch
            limits = self.config["arms"][side]["joint_limits_deg"]
        try:
            goal = self.models[side].inverse_wrist(q, h, xyz, rpy, limits)
        except ValueError as error:
            raise Rejected(str(error)) from error
        with self.lock:
            self._check_motion_owner("arm.wrist", client)
            reason = self.wrist_reason(side)
            if reason:
                raise Rejected(reason)
            if (self.command_epoch != epoch or abs(self.body["height"]-h) > .02 or
                    max(abs(a-b) for a,b in zip(self.arms[side]["joints"], q)) > math.radians(.1)):
                raise Rejected("规划期间指令或起姿已变化，请重新读取反馈")
            command = {"type":"arm.joints", "arm":side, "joints":np.degrees(goal).tolist()}
            result = self._submit(command, client)
            self.command_epoch += 1
        confirmation = (result or {}).get("_confirmation")
        if confirmation:
            confirmation()
        self.log(f"{side} 腕部 base_link 点位目标已下发：{xyz} mm，RPY {rpy}°")
        return {"ok":True, "message":"腕部点位目标已下发，等待到位反馈"}

    def snapshot(self):
        with self.lock:
            arms = {}
            for side, state in self.arms.items():
                arms[side] = copy.deepcopy(state)
                if state:
                    arms[side]["fresh"] = self.fresh(state)
                    arms[side]["age_s"] = max(0, self.clock() - state["received"])
                arms[side] = arms[side] or {"fresh": False}
                arms[side]["base_wrist"] = None
                if not self.wrist_feedback_reason(side):
                    pose = self.models[side].wrist_transform(state["joints"], self.body["height"])
                    rpy = Rotation.from_matrix(pose[:3,:3]).as_euler("xyz", degrees=True)
                    arms[side]["base_wrist"] = (pose[:3,3]*1000).tolist() + rpy.tolist()
            body = copy.deepcopy(self.body) or {"fresh": False}
            if self.body:
                body["fresh"] = self.fresh(self.body)
                body["age_s"] = max(0, self.clock() - self.body["received"])
            capabilities = {}
            for side in ("left", "right"):
                for kind in ("arm.joints", "arm.tcp", "gripper", "arm.stop"):
                    capabilities[side + ":" + kind] = self.arm_reason(side, kind)
                capabilities[side + ":arm.wrist"] = self.wrist_reason(side)
                for mode in ARM_MODES:
                    capabilities["%s:arm.mode.%d" % (side, mode)] = self.arm_reason(side, "arm.mode", mode)
            for kind in ("chassis", "chassis.stop", "lift", "lift.stop"):
                capabilities[kind] = self.body_reason(kind)
            left = arms["left"]
            left_errors = ([abs(math.degrees(value) - target)
                            for value, target in zip(left["joints"], LEFT_HOLD_JOINTS_DEG)]
                           if left.get("fresh") and vector(left.get("joints"), 6) else None)
            left_errors_2 = ([abs(math.degrees(value) - target)
                              for value, target in zip(left["joints"], LEFT_HOLD_2_JOINTS_DEG)]
                             if left.get("fresh") and vector(left.get("joints"), 6) else None)
            right = arms["right"]
            right_avoidance_errors = ([abs(math.degrees(value) - target)
                                        for value, target in zip(right["joints"], RIGHT_AVOIDANCE_JOINTS_DEG)]
                                       if right.get("fresh") and vector(right.get("joints"), 6) else None)
            capabilities["left:arm.fixed_hold"] = self.arm_reason("left", "arm.joints")
            capabilities["left:arm.fixed_hold2"] = self.arm_reason("left", "arm.joints")
            capabilities["right:arm.avoidance_preset"] = self.arm_reason("right", "arm.joints")
            return {"time": time.time(), "ros": self.adapter.status(),
                    "execution_enabled": self.config["execution_enabled"],
                    "arms": arms, "body": body, "capabilities": capabilities,
                    "limits": {"arms": {s: {k: self.config["arms"][s][k] for k in
                               ("joint_limits_deg", "gripper_limits_raw", "gripper_open_raw", "gripper_closed_raw", "tcp_limits_mm")} for s in arms},
                               "lift": self.config["body"]["lift_limits_raw"],
                               "linear": self.config["body"]["max_linear_m_s"],
                               "angular": self.config["body"]["max_angular_rad_s"],
                               "forward": self.config["body"].get("manual_sdk_axes",{}).get("x",self.config["body"]["max_linear_m_s"]),
                               "lateral": self.config["body"].get("manual_sdk_axes",{}).get("y",self.config["body"]["max_linear_m_s"]),
                               "chassis_input_to_sdk": self.config["body"].get("chassis_input_scale",2.5)/2.5},
                    "chassis_active": self.chassis_owner is not None,
                    "chassis_scheduler": self.chassis.snapshot(),
                    "motion_reserved": self.motion_owner is not None,
                    "competition_reserved": self.competition_owner is not None,
                    "left_fixed_hold": {"joints_deg": LEFT_HOLD_JOINTS_DEG[:],
                        "gripper_raw": self.config["arms"]["left"]["gripper_closed_raw"],
                        "tolerance_deg": LEFT_HOLD_TOLERANCE_DEG,
                        "max_error_deg": max(left_errors) if left_errors else None,
                        "arrived": bool(left_errors and max(left_errors) <= LEFT_HOLD_TOLERANCE_DEG)},
                    "left_fixed_hold2": {"joints_deg": LEFT_HOLD_2_JOINTS_DEG[:],
                        "gripper_raw": self.config["arms"]["left"]["gripper_closed_raw"],
                        "tolerance_deg": LEFT_HOLD_TOLERANCE_DEG,
                        "max_error_deg": max(left_errors_2) if left_errors_2 else None,
                        "arrived": bool(left_errors_2 and max(left_errors_2) <= LEFT_HOLD_TOLERANCE_DEG)},
                    "right_avoidance_preset": {"joints_deg": RIGHT_AVOIDANCE_JOINTS_DEG[:],
                        "tolerance_deg": LEFT_HOLD_TOLERANCE_DEG,
                        "max_error_deg": max(right_avoidance_errors) if right_avoidance_errors else None,
                        "arrived": bool(right_avoidance_errors and max(right_avoidance_errors) <= LEFT_HOLD_TOLERANCE_DEG)},
                    "motion_interrupted": self.motion_interrupted, "command_epoch": self.command_epoch,
                    "contact_stop_pending": bool(self.motion_owner and self.motion_owner.contact_stop_pending),
                    "nav_sdk_ready": self.adapter.chassis_ready(),
                    "logs": list(self.logs)}

    def submit(self, data, client, *, preflight=None, automatic_joint_speed=False):
        try:
            if preflight is not None and (not callable(preflight) or not isinstance(data, dict)
                    or data.get("type") not in ("gripper", "lift")):
                raise Rejected("执行复查仅用于任务夹爪或升降指令")
            if isinstance(data, dict) and data.get("type") == "arm.wrist":
                return self.wrist_command(data, client)
            with self.lock:
                if preflight is not None:
                    if not self.motion_valid(client):
                        raise Rejected("执行复查需要有效任务控制权")
                    if preflight(self.snapshot(), self.clock()) is not True:
                        return {"ok": False, "deferred": True, "message": "执行复查未许可，未下发"}
                was_moving = self.chassis_owner is not None
                result = self._submit(data, client, automatic_joint_speed=automatic_joint_speed)
                self.command_epoch += 1
                confirmation = (result or {}).pop("_confirmation", None)
            # ROS feedback callbacks need this lock while the driver confirms.
            if confirmation:
                confirmation()
            with self.lock:
                if data["type"] == "arm.mode":
                    self.log(result["message"], "warning" if data["mode"] in (0, 1) else "info")
                    return {"ok": True, **result}
                if data["type"] not in ("chassis","chassis.nav_sdk") or not was_moving:
                    self.log("已下发：" + json.dumps(data, ensure_ascii=False, allow_nan=False) + "（未确认到位）")
                return {"ok": True, "message": "已下发，等待实际反馈；不代表已到位", **(result or {})}
        except Rejected as e:
            self.log("拒绝：" + str(e), "warning")
            raise

    def _submit(self, data, client, *, automatic_joint_speed=False):
        if not isinstance(data, dict):
            raise Rejected("指令必须是对象")
        if isinstance(data.get("type"),str) and data["type"].startswith("chassis"):
            return self._submit_chassis(data, client)
        sequence_client = client
        if data.get("type") in ("chassis", "chassis.stop") and "control_id" in data:
            control_id = data["control_id"]
            if (not isinstance(control_id, str) or len(control_id) != 32
                    or any(c not in "0123456789abcdef" for c in control_id)):
                raise Rejected("底盘页面控制 ID 无效")
            sequence_client = (client, control_id)
            data = {k: v for k, v in data.items() if k != "control_id"}
        schemas = {"arm.joints": {"type", "arm", "joints"}, "arm.fixed_hold": {"type", "arm"},
                   "arm.fixed_hold2": {"type", "arm"},
                   "arm.avoidance_preset": {"type", "arm"},
                   "arm.tcp": {"type", "arm", "position_mm", "orientation_deg"},
                   "gripper": {"type", "arm", "position"}, "arm.stop": {"type", "arm"},
                   "arm.mode": {"type", "arm", "mode"},
                   "chassis": {"type", "direction", "speed", "turn_speed", "sequence"},
                   "chassis.stop": {"type", "sequence"}, "lift": {"type", "target"}, "lift.stop": {"type"}}
        kind = data.get("type")
        nav_sdk = kind == "chassis.nav_sdk"
        chassis_vector = kind in ("chassis","chassis.nav_sdk") and set(data) == {"type", "x", "y", "omega", "sequence"}
        if not chassis_vector and (kind not in schemas or set(data) != schemas[kind]):
            raise Rejected("指令类型或字段无效")
        self._check_motion_owner(kind, client)
        if "arm" in data:
            side = data["arm"]
            if side not in self.arms:
                raise Rejected("机械臂选择无效")
            requested_mode = data.get("mode") if kind == "arm.mode" else None
            if kind == "arm.mode" and (type(requested_mode) is not int or requested_mode not in ARM_MODES):
                raise Rejected("手动模式必须为整数 0、1、2 或 3")
            reason = self.arm_reason(side, kind, requested_mode)
            if reason:
                raise Rejected(reason)
            c, state = self.config["arms"][side], self.arms[side]
            joints, gripper, tcp = state["joints"][:], state["gripper"], state["tcp"][:]
            if kind == "arm.stop" and c.get("stop_service"):
                self.adapter.arm_stop(side)
                if self.motion_trajectory is not None and self.motion_trajectory['side']==side:
                    self.motion_trajectory = None
                return
            if kind in ("arm.fixed_hold", "arm.fixed_hold2"):
                if side != "left":
                    raise Rejected("固定保持点位仅用于左臂")
                target = LEFT_HOLD_JOINTS_DEG if kind == "arm.fixed_hold" else LEFT_HOLD_2_JOINTS_DEG
                if any(not pair[0] <= value <= pair[1]
                       for value, pair in zip(target, c["joint_limits_deg"])):
                    raise Rejected("左臂固定保持点位超出已配置范围")
                joints = [math.radians(value) for value in target]
                gripper = c["gripper_closed_raw"]
            elif kind == "arm.avoidance_preset":
                if side != "right":
                    raise Rejected("避障预设点位仅用于右臂")
                if any(not pair[0] <= value <= pair[1]
                       for value, pair in zip(RIGHT_AVOIDANCE_JOINTS_DEG, c["joint_limits_deg"])):
                    raise Rejected("右臂避障预设点位超出已配置范围")
                joints = [math.radians(value) for value in RIGHT_AVOIDANCE_JOINTS_DEG]
            elif kind == "arm.joints":
                if not vector(data["joints"], 6):
                    raise Rejected("需要六个有限关节角度")
                for i, (v, pair) in enumerate(zip(data["joints"], c["joint_limits_deg"])):
                    if not pair[0] <= v <= pair[1]:
                        raise Rejected("J%d 超出已配置范围" % (i + 1))
                joints = [math.radians(v) for v in data["joints"]]
            elif kind == "gripper":
                v = data["position"]
                if not number(v) or not c["gripper_limits_raw"][0] <= v <= c["gripper_limits_raw"][1]:
                    raise Rejected("夹爪目标超出已配置范围")
                gripper = v
            elif kind == "arm.tcp":
                if not vector(data["position_mm"], 3) or not vector(data["orientation_deg"], 3):
                    raise Rejected("末端位姿需要六个有限数值")
                if any(not lo <= v <= hi for v, (lo, hi) in zip(data["position_mm"], c["tcp_limits_mm"])):
                    raise Rejected("末端位置超出已配置范围")
                if any(abs(v) > 360 for v in data["orientation_deg"]):
                    raise Rejected("姿态角超出输入范围")
                tcp = [v / 1000 for v in data["position_mm"]] + [math.radians(v) for v in data["orientation_deg"]]
            # Keep unused targets from measured feedback instead of default zeros.
            mode = requested_mode if kind == "arm.mode" else (4 if kind == "arm.tcp" else 5)
            confirmation = None
            if c.get("command_result_topic"):
                driver_kind = ('arm.joints.auto' if automatic_joint_speed and kind == 'arm.joints' and isinstance(client, _MotionToken)
                               and c.get('automatic_joint_speed_deg_s') else
                               'arm.joints' if kind in ('arm.fixed_hold','arm.fixed_hold2','arm.avoidance_preset') else kind)
                confirmation = self.adapter.arm(side, joints, gripper, tcp, mode, kind=driver_kind, wait=False)
            else:
                self.adapter.arm(side, joints, gripper, tcp, mode)
            if kind == "arm.mode":
                name = "左臂" if side == "left" else "右臂"
                return {"message": "已向%s发送 mode %d（%s）请求；实际模式未确认" % (name, mode, ARM_MODES[mode]),
                        "mode_status": "requested_unconfirmed", "_confirmation": confirmation}
            return {"_confirmation": confirmation}
        reason = self.body_reason(kind)
        if reason:
            raise Rejected(reason)
        if kind == "lift.stop" and self.config["body"].get("lift_stop_service"):
            self.adapter.lift_stop()
            return
        if kind == "lift" or kind == "lift.stop":
            v = data.get("target", self.body["height"])
            lo, hi = self.config["body"]["lift_limits_raw"]
            if not number(v) or not lo <= v <= hi:
                raise Rejected("升降目标超出已配置范围或反馈参考无效")
            self.adapter.lift(v)
            return
        raise Rejected("指令类型无效")

    def _sync_chassis(self):
        if self.config.get('chassis_backend')=='ros' and not self.chassis.snapshot().get('ready'):
            self.chassis_owner='ros-state-unknown';self.chassis_deadline=0
            return
        a=self.chassis.active
        self.chassis_owner=None if a is None or not any(a['sdk']) else a['client']
        self.chassis_deadline=0 if a is None else a['deadline']

    def _submit_chassis(self, data, client):
        kind=data.get('type')
        if kind=='chassis.stop':
            if set(data)-{'type','sequence','control_id'}:raise Rejected('全车停车字段无效')
            if type(data.get('sequence')) is not int or data['sequence']<=0:raise Rejected('停车序号无效')
            # Explicit operator stop keeps the existing manipulator interruption.
            self._check_motion_owner(kind,client)
            try:return self.chassis.stop('explicit_stop')
            finally:self._sync_chassis()
        if kind not in ('chassis','chassis.nav_sdk','chassis.gamepad_sdk','chassis.end_output'):
            raise Rejected('底盘指令类型无效')
        fields={'type'}|set(IDENTITY_FIELDS)
        if kind!='chassis.end_output':fields|={'x','y','omega'}
        if set(data)!=fields:raise Rejected('底盘身份、时效或执行字段无效')
        if kind!='chassis.end_output':
            # Manipulator capability validates its own stale callbacks, but never
            # excludes ordinary chassis callers while the manipulator is working.
            if isinstance(client,_MotionToken) and not self.motion_valid(client):
                raise Rejected('作业身份已结束或被停止')
            reason=self.body_reason(kind)
            if reason:raise Rejected(reason)
            c=self.config['body']
            linear,angular=(c['max_linear_m_s'],c['max_angular_rad_s'])
            if kind=='chassis.nav_sdk':linear,angular=.8,.8
            if kind=='chassis.gamepad_sdk':linear,angular=2.,2.
            values=tuple(data[k] for k in ('x','y','omega'))
            if not all(number(v) for v in values):raise Rejected('底盘执行值必须为有限数值')
            # Ordinary chassis callers keep their historical input units. The
            # browser adapter converts its displayed SDK values at the boundary.
            sdk=tuple(v*c.get('chassis_input_scale',2.5)/2.5 for v in values) if kind=='chassis' else values
            if math.hypot(*sdk[:2])>linear+1e-12 or abs(sdk[2])>angular+1e-12:
                raise Rejected('底盘 SDK 执行值超出限制')
            if kind=='chassis':
                axes=c.get('manual_sdk_axes',{})
                if abs(sdk[0])>axes.get('x',linear)+1e-12 or abs(sdk[1])>axes.get('y',linear)+1e-12:
                    raise Rejected('底盘 SDK 执行值超出当前机型分轴限制')
        else:sdk=None
        try:
            return self.chassis.submit(data,client,sdk,
                maintenance=client if client is self.chassis.maintenance else None)
        except self.chassis_errors as error:raise Rejected(str(error)) from error
        finally:self._sync_chassis()

    def end_chassis_output(self,client):
        with self.lock:
            try:self.chassis.end_client(client)
            finally:self._sync_chassis()

    def reserve_chassis_maintenance(self):
        with self.lock:
            try:return self.chassis.reserve_maintenance()
            except self.chassis_errors as error:raise Rejected(str(error)) from error
            finally:self._sync_chassis()

    def release_chassis_maintenance(self,token):
        with self.lock:
            try:self.chassis.release_maintenance(token)
            except self.chassis_errors as error:raise Rejected(str(error)) from error
            finally:self._sync_chassis()

    def _stop_chassis(self, reason, force=False):
        # Internal expiry is scoped to the current accepted output sequence.
        if self.chassis.active is not None:
            self.chassis.expire(self.chassis.active['output_seq'])
        elif force:self.chassis.stop(reason)
        self._sync_chassis()

    def tick(self):
        with self.lock:
            self.chassis.tick(feedback_fresh=self.fresh(self.body))
            self._sync_chassis()

    def disconnected(self, client, identity=None):
        with self.lock:
            self.chassis.disconnect(client,identity)
            self._sync_chassis()
