"""Pure validation and control state. No SDK construction or device access."""
import copy
import json
import math
import threading
import time
from collections import deque

DIRECTIONS = {
    "forward": (1, 0, 0), "backward": (-1, 0, 0),
    "left": (0, 1, 0), "right": (0, -1, 0),
    "yaw-left": (0, 0, 1), "yaw-right": (0, 0, -1),
}

ARM_MODES = {0: "力矩清零", 1: "机械臂复位", 2: "阻尼模式", 3: "重力补偿"}


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def bounds(pair):
    return isinstance(pair, list) and len(pair) == 2 and all(number(x) for x in pair) and pair[0] < pair[1]


def vector(value, length):
    return isinstance(value, list) and len(value) == length and all(number(x) for x in value)


class Rejected(ValueError):
    pass


class Controller:
    def __init__(self, config, adapter, clock=time.monotonic):
        self.config = copy.deepcopy(config)
        self.adapter = adapter
        self.clock = clock
        self.lock = threading.RLock()
        self.arms = {side: None for side in ("left", "right")}
        self.body = None
        self.logs = deque(maxlen=100)
        self.serial = 0
        self.chassis_owner = None
        self.chassis_deadline = 0
        self.chassis_message = None
        self.base_sequences = {}

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

    def snapshot(self):
        with self.lock:
            arms = {}
            for side, state in self.arms.items():
                arms[side] = copy.deepcopy(state)
                if state:
                    arms[side]["fresh"] = self.fresh(state)
                    arms[side]["age_s"] = max(0, self.clock() - state["received"])
                arms[side] = arms[side] or {"fresh": False}
            body = copy.deepcopy(self.body) or {"fresh": False}
            if self.body:
                body["fresh"] = self.fresh(self.body)
                body["age_s"] = max(0, self.clock() - self.body["received"])
            capabilities = {}
            for side in ("left", "right"):
                for kind in ("arm.joints", "arm.tcp", "gripper", "arm.stop"):
                    capabilities[side + ":" + kind] = self.arm_reason(side, kind)
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
                    "logs": list(self.logs)}

    def submit(self, data, client):
        try:
            with self.lock:
                was_moving = self.chassis_owner is not None
                result = self._submit(data, client)
                confirmation = (result or {}).pop("_confirmation", None)
            # ROS feedback callbacks need this lock while the driver confirms.
            if confirmation:
                confirmation()
            with self.lock:
                if data["type"] == "arm.mode":
                    self.log(result["message"], "warning" if data["mode"] in (0, 1) else "info")
                    return {"ok": True, **result}
                if data["type"] != "chassis" or not was_moving:
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
        if kind not in schemas or set(data) != schemas[kind]:
            raise Rejected("指令类型或字段无效")
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
        if self.chassis_owner and self.chassis_owner != client:
            raise Rejected("底盘正在由另一网页控制")
        self.base_sequences[client] = sequence
        if kind == "chassis.stop":
            self._stop_chassis("底盘停止")
            return
        direction, speed, turn = data["direction"], data["speed"], data["turn_speed"]
        c = self.config["body"]
        if direction not in DIRECTIONS or not number(speed) or not number(turn) or not 0 < speed <= c["max_linear_m_s"] or not 0 < turn <= c["max_angular_rad_s"]:
            raise Rejected("方向或速度超出已配置范围")
        x, y, z = DIRECTIONS[direction]
        self.chassis_message = copy.deepcopy(self.body)
        self.adapter.chassis(x * speed, y * speed, z * turn, self.chassis_message)
        self.chassis_owner = client
        self.chassis_deadline = self.clock() + self.config["chassis_lease_s"]

    def _stop_chassis(self, reason):
        if self.chassis_owner is not None:
            self.adapter.chassis(0, 0, 0, self.chassis_message)
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
