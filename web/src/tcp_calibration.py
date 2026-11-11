"""Read-only fixed-point TCP data collection and pivot calibration."""
import json
from pathlib import Path
import secrets
import threading
import time

import numpy as np


OUT = Path(__file__).resolve().parents[1] / "hardware/runtime/tcp-calibration/candidates"


def _matrix(value):
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("样本必须是有限的 4×4 齐次变换")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError("样本不是有效齐次变换")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)):
        raise ValueError("样本旋转矩阵无效")
    return transform


def solve_pivot(transforms):
    """Solve R_i * tcp + t_i = pivot using algebraic one-step least squares."""
    if len(transforms) < 6:
        raise ValueError("至少记录 6 个不同腕部姿态")
    matrices = [_matrix(value) for value in transforms]
    identity = np.eye(3)
    system = np.vstack([np.hstack((item[:3, :3], -identity)) for item in matrices])
    target = np.concatenate([-item[:3, 3] for item in matrices])
    solution, _, rank, singular = np.linalg.lstsq(system, target, rcond=None)
    if rank < 6 or singular[-1] <= np.finfo(float).eps:
        raise ValueError("腕部姿态变化不足，无法稳定求解 TCP")
    tcp, pivot = solution[:3], solution[3:]
    errors = np.array([
        np.linalg.norm(item[:3, :3] @ tcp + item[:3, 3] - pivot)
        for item in matrices
    ]) * 1000.0
    return {
        "tcp_offset_mm": np.round(tcp * 1000.0, 6).tolist(),
        "pivot_base_mm": np.round(pivot * 1000.0, 6).tolist(),
        "rank": int(rank),
        "condition": round(float(singular[0] / singular[-1]), 6),
        "rms_mm": round(float(np.sqrt(np.mean(errors ** 2))), 6),
        "max_error_mm": round(float(np.max(errors)), 6),
        "errors_mm": np.round(errors, 6).tolist(),
    }


class TcpCalibration:
    """Per-browser TCP capture sessions. No method dispatches robot motion."""
    def __init__(self, controller, *, out=OUT, clock=time.time):
        self.controller = controller
        self.out = Path(out)
        self.clock = clock
        self.lock = threading.RLock()
        self.sessions = {}

    @staticmethod
    def _empty_arm():
        return {"samples": [], "solution": None, "candidate_path": None}

    def _session(self, client):
        if not client:
            raise ValueError("TCP 标定页面会话无效")
        with self.lock:
            return self.sessions.setdefault(client, {
                "left": self._empty_arm(), "right": self._empty_arm()
            })

    def _feedback(self, side):
        snapshot = self.controller.snapshot()
        if snapshot.get("chassis_active"):
            raise ValueError("底盘正在运动，不能记录 TCP 样本")
        if not (snapshot.get("ros") or {}).get("connected"):
            raise ValueError("ROS 未连接")
        arm = (snapshot.get("arms") or {}).get(side) or {}
        body = snapshot.get("body") or {}
        if (not arm.get("fresh") or arm.get("age_s", 99) > 0.3
                or not body.get("fresh") or body.get("age_s", 99) > 0.3):
            raise ValueError("机械臂或升降反馈缺失或已过期")
        joints = np.asarray(arm.get("joints"), dtype=float)
        lift = body.get("height")
        if joints.shape != (6,) or not np.isfinite(np.r_[joints, lift]).all():
            raise ValueError("关节或升降反馈数值无效")
        model = self.controller.models.get(side)
        if model is None:
            raise ValueError("该机械臂的腕部模型不可用")
        transform = _matrix(model.wrist_transform(joints, lift))
        pose = arm.get("base_wrist")
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError("base_link 到 J6 腕部位姿不可用")
        return snapshot, joints, float(lift), transform, [float(value) for value in pose]

    def _arm_state(self, record):
        return {
            "sample_count": len(record["samples"]),
            "samples": [dict(sample) for sample in record["samples"]],
            "solution": dict(record["solution"]) if record["solution"] else None,
            "candidate_path": record["candidate_path"],
        }

    def state(self, client):
        with self.lock:
            session = self._session(client)
            current = {}
            for side in ("left", "right"):
                try:
                    _, _, _, _, pose = self._feedback(side)
                    current[side] = {"available": True, "pose": pose, "reason": ""}
                except (ValueError, TypeError, OSError) as error:
                    current[side] = {"available": False, "pose": None, "reason": str(error)}
            return {
                "time": self.clock(),
                "method": "fixed_point_pivot",
                "frame": "base_link_to_j6_wrist",
                "minimum_samples": 6,
                "current": current,
                "arms": {side: self._arm_state(session[side]) for side in ("left", "right")},
            }

    def action(self, client, data):
        if not isinstance(data, dict):
            raise ValueError("需要 TCP 标定操作对象")
        op = data.get("op")
        side = data.get("arm")
        if side not in ("left", "right"):
            raise ValueError("请选择左臂或右臂")
        with self.lock:
            record = self._session(client)[side]
            if op == "capture":
                if data.get("confirmed") is not True:
                    raise ValueError("请确认工具尖端保持在同一个固定点")
                _, joints, lift, transform, pose = self._feedback(side)
                record["samples"].append({
                    "index": len(record["samples"]) + 1,
                    "captured_at": self.clock(),
                    "joints_rad": joints.tolist(),
                    "lift_raw": lift,
                    "wrist_matrix": transform.tolist(),
                    "wrist_pose": pose,
                })
                record["solution"] = None
                record["candidate_path"] = None
            elif op == "remove":
                index = data.get("index")
                if not isinstance(index, int) or isinstance(index, bool):
                    raise ValueError("样本编号无效")
                before = len(record["samples"])
                record["samples"] = [sample for sample in record["samples"] if sample["index"] != index]
                if len(record["samples"]) == before:
                    raise ValueError("样本不存在")
                for number, sample in enumerate(record["samples"], 1):
                    sample["index"] = number
                record["solution"] = None
                record["candidate_path"] = None
            elif op == "reset":
                self._session(client)[side] = self._empty_arm()
            elif op == "solve":
                record["solution"] = solve_pivot(
                    [sample["wrist_matrix"] for sample in record["samples"]]
                )
                record["candidate_path"] = None
            elif op == "save":
                if record["solution"] is None:
                    raise ValueError("请先求解 TCP")
                label = data.get("label", "")
                if not isinstance(label, str) or len(label.strip()) > 80:
                    raise ValueError("工具名称最多 80 个字符")
                self.out.mkdir(parents=True, exist_ok=True)
                filename = "%s-%d-%s.json" % (
                    side, int(self.clock()), secrets.token_hex(4)
                )
                path = self.out / filename
                payload = {
                    "schema": "arx.tcp-pivot-candidate.v1",
                    "created_at": self.clock(),
                    "arm": side,
                    "label": label.strip(),
                    "method": "fixed_point_pivot",
                    "wrist_frame": "j6_wrist",
                    "base_frame": "base_link",
                    "translation_only": True,
                    "samples": record["samples"],
                    "solution": record["solution"],
                }
                with path.open("x", encoding="utf-8") as stream:
                    json.dump(payload, stream, ensure_ascii=False, allow_nan=False, indent=2)
                    stream.write("\n")
                record["candidate_path"] = str(path)
            else:
                raise ValueError("未知 TCP 标定操作")
            return self.state(client)
