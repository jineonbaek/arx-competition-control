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

DIRECTIONS = {
    "forward": (1, 0, 0), "backward": (-1, 0, 0),
    "left": (0, 1, 0), "right": (0, -1, 0),
    "yaw-left": (0, 0, 1), "yaw-right": (0, 0, -1),
}

ARM_MODES = {0: "力矩清零", 1: "机械臂复位", 2: "阻尼模式", 3: "重力补偿"}


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

    def reserve_motion(self, expected_epoch):
        """Reserve this controller only; the executor must also verify physical rest."""
        with self.lock:
            if (type(expected_epoch) is not int or expected_epoch != self.command_epoch
                    or not self.config["execution_enabled"]):
                raise Rejected("控制权申请时指令状态已变化或真实控制未启用")
            if self.motion_owner is not None or self.chassis_owner is not None:
                raise Rejected("已有任务或底盘运动占用控制权")
            self.motion_owner = _MotionToken()
            self.motion_interrupted = False
            self.command_epoch += 1
            return self.motion_owner

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

    def release_motion(self, token):
        """Stop any leftover trajectory and return ownership when the executor exits."""
        with self.lock:
            if not isinstance(token, _MotionToken) or token is not self.motion_owner:
                raise Rejected("任务控制权已失效")
            if self.motion_trajectory is not None:
                self._trajectory_abort(self.motion_trajectory['side'],token)
                if self.motion_trajectory is not None:
                    raise Rejected("轨迹停止请求未发出，不能释放控制权")
            self.motion_owner = None
            self.motion_interrupted = False
            self.command_epoch += 1

    def _check_motion_owner(self, kind, client):
        # Called under the same lock as dispatch, including after unlocked wrist IK.
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

    def start_trajectory(self, side, plan, token, *, preflight, require_lift_settle=True):
        """Process-only entry: planner checks scene, calibrated TCP and rest in preflight."""
        with self.lock:
            if side not in self.arms or not self.motion_valid(token) or not callable(preflight):
                raise Rejected("轨迹需要有效任务控制权和执行前复查")
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

    def update_arm(self, side, joints, gripper, tcp):
        if side not in self.arms or not vector(joints, 6) or not number(gripper) or not vector(tcp, 6):
            return
        with self.lock:
            self.arms[side] = {"joints": joints[:], "gripper": gripper, "tcp": tcp[:],
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
            if not self.adapter.has_subscriber(c["command_topic"], "body"):
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
            return {"time": time.time(), "ros": self.adapter.status(),
                    "execution_enabled": self.config["execution_enabled"],
                    "arms": arms, "body": body, "capabilities": capabilities,
                    "limits": {"arms": {s: {k: self.config["arms"][s][k] for k in
                               ("joint_limits_deg", "gripper_limits_raw", "gripper_open_raw", "gripper_closed_raw", "tcp_limits_mm")} for s in arms},
                               "lift": self.config["body"]["lift_limits_raw"],
                               "linear": self.config["body"]["max_linear_m_s"],
                               "angular": self.config["body"]["max_angular_rad_s"]},
                    "chassis_active": self.chassis_owner is not None,
                    "motion_reserved": self.motion_owner is not None,
                    "motion_interrupted": self.motion_interrupted, "command_epoch": self.command_epoch,
                    "nav_sdk_ready": self.adapter.has_subscriber("/web/nav/sdk_command","body"),
                    "logs": list(self.logs)}

    def submit(self, data, client, *, preflight=None):
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
                result = self._submit(data, client)
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

    def _submit(self, data, client):
        if not isinstance(data, dict):
            raise Rejected("指令必须是对象")
        schemas = {"arm.joints": {"type", "arm", "joints"}, "arm.tcp": {"type", "arm", "position_mm", "orientation_deg"},
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
            if kind == "arm.joints":
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
                confirmation = self.adapter.arm(side, joints, gripper, tcp, mode, kind=kind, wait=False)
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
        sequence = data.get("sequence")
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= self.base_sequences.get(client, 0):
            raise Rejected("底盘指令序号已过期")
        if self.chassis_owner and self.chassis_owner != client and kind != "chassis.stop":
            raise Rejected("底盘正在由另一网页控制")
        self.base_sequences[client] = sequence
        if kind == "chassis.stop":
            self._stop_chassis("底盘停止", force=True)
            return
        c = self.config["body"]
        max_linear, max_angular = c["max_linear_m_s"], c["max_angular_rad_s"]
        if nav_sdk:
            if not self.adapter.has_subscriber("/web/nav/sdk_command", "body"):
                raise Rejected("导航 SDK 接收端未启动")
            max_linear, max_angular = 1.0, .112
        if chassis_vector:
            x, y, omega = data["x"], data["y"], data["omega"]
            if (not all(type(v) in (int, float) and -limit <= v <= limit
                        for v, limit in ((x, max_linear), (y, max_linear), (omega, max_angular)))
                    or math.hypot(x, y) > max_linear + (1e-12 if nav_sdk else 0.) or (x, y, omega) == (0, 0, 0)):
                raise Rejected("底盘三轴速度无效或超出已配置范围")
        else:
            direction, speed, turn = data["direction"], data["speed"], data["turn_speed"]
            if direction not in DIRECTIONS or not number(speed) or not number(turn) or not 0 < speed <= max_linear or not 0 < turn <= max_angular:
                raise Rejected("方向或速度超出已配置范围")
            dx, dy, rotation = DIRECTIONS[direction]
            x, y, omega = dx * speed, dy * speed, rotation * turn
        self.chassis_message = copy.deepcopy(self.body)
        if nav_sdk:
            self.adapter.nav_chassis_sdk(x, y, omega)
        else:
            self.adapter.chassis(x, y, omega, self.chassis_message)
        self.chassis_owner = client
        self.chassis_deadline = self.clock() + self.config["chassis_lease_s"]

    def _stop_chassis(self, reason, force=False):
        if self.chassis_owner is not None or force:
            self.adapter.chassis(0, 0, 0, self.chassis_message or self.body)
            self.chassis_owner = None
            self.chassis_message = None
            self.log(reason)

    def tick(self):
        with self.lock:
            if self.chassis_owner is not None and (self.clock() > self.chassis_deadline or not self.fresh(self.body)):
                self._stop_chassis("底盘指令超时或反馈过期，已请求停车")

    def disconnected(self, client):
        with self.lock:
            if self.chassis_owner == client:
                self._stop_chassis("网页连接断开，已请求停车")
