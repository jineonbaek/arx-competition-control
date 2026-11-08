"""Read-only physical check of the calibrated D405 color camera relative to body4."""
import json
import time
from pathlib import Path
from uuid import uuid4

import numpy as np
from handeye_observation import WaitForFrame, board_spec, observe

CANDIDATE = (Path(__file__).resolve().parents[2] / "vision/handeye/results"
             / "d405-right-cc200-ee0ff2fa49e9e56285d271d6.json")
CHECKS = Path(__file__).resolve().parents[1] / "hardware/runtime/handeye-checks"
BOARD = board_spec(dict(dictionary="5X5_100", marker=26, mm=11.25, layout="CC200-15-11.25"))


def validate_reference(value):
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("需要 X、Y、Z 三个坐标栏（毫米）")
    if any(v is not None and (type(v) not in (int,float) or not np.isfinite(v)) for v in value):
        raise ValueError("实测坐标必须为有限数值")
    return value if any(v is not None for v in value) else None


def marker_center(sample, candidate):
    if (sample["color_intrinsics"]["serial"] != candidate["camera"]["serial"]
            or candidate["parent_frame"] != "right_link26" or candidate["child_frame"] != "D405 color optical frame"):
        raise ValueError("候选外参的相机序列号或坐标系与当前采集不匹配")
    body_from_wrist = np.asarray(sample["T_body4_from_right_link26"])
    wrist_from_camera = np.asarray(candidate["T_right_link26_from_D405_color"])
    camera_from_marker = np.asarray(sample["T_camera_from_marker"])
    return (body_from_wrist @ wrist_from_camera @ camera_from_marker)[:3, 3] * 1000


def check_point(snapshot, model, reference):
    measured = validate_reference(reference)
    candidate = json.loads(CANDIDATE.read_text())
    start = time.time()
    deadline = time.monotonic() + 4
    history = []
    frames = []
    reason = "等待新鲜相机帧和右臂反馈"
    first_q = None
    while time.monotonic() < deadline:
        state, q, h = snapshot()
        arm = state["arms"]["right"]
        if first_q is None:
            first_q, first_h = q.copy(), h
        if np.max(np.abs(q-first_q)) > np.radians(.18) or abs(h-first_h) > .05:
            raise ValueError("右臂或升降在移动，请停稳后重测")
        if not history or arm["timestamp"] > history[-1][0]:
            history.append((arm["timestamp"], q.copy()))
        history = [row for row in history if arm["timestamp"] - row[0] <= .65]
        if len(history)>=4 and history[-1][0]-history[0][0]>=.4 and np.max(np.diff([row[0] for row in history]))<=.25:
            try:
                sample, _ = observe(BOARD, start, arm, model, history, color_only=False)
            except WaitForFrame:
                pass
            except (ValueError, OSError) as err:
                frames.clear()
                reason = str(err)
            else:
                if frames and (sample["camera_session"] != frames[-1][0]["camera_session"]
                               or sample["image_time"]-frames[-1][0]["image_time"] > .65):
                    frames.clear()
                if not frames or sample["image_time"] > frames[-1][0]["image_time"]:
                    frames.append((sample, marker_center(sample, candidate)))
                    frames = frames[-3:]
                if len(frames)==3:
                    points = np.stack([row[1] for row in frames])
                    spread = float(np.max(np.linalg.norm(points[:,None]-points,axis=2)))
                    if spread <= 5:
                        center = points.mean(axis=0)
                        # J1 origin in body4; axes stay parallel to body4 even when J1 rotates.
                        relative = center-model.parts[0][0][:3,3]*1000
                        delta = ([round(float(x-v), 3) if v is not None else None
                                  for x,v in zip(relative, measured)] if measured is not None else None)
                        result = dict(body4_mm=center.tolist(), shoulder_mm=relative.tolist(),
                                    reference_mm=measured,
                                    delta_mm=delta,
                                    error_mm=float(np.linalg.norm(delta)) if delta is not None and None not in delta else None,
                                    spread_mm=spread, image_times=[row[0]["image_time"] for row in frames],
                                    session=candidate["source_session"])
                        if measured is not None:
                            result["check_id"] = uuid4().hex
                            CHECKS.mkdir(parents=True, exist_ok=True)
                            path = CHECKS / (result["check_id"] + ".json")
                            path.write_text(json.dumps(result, indent=2, allow_nan=False))
                        return result
                    reason = f"标记中心三帧波动 {spread:.1f} mm，等候停稳"
        time.sleep(.045)
    raise ValueError(reason)
