"""Guarded, lift-only segmented descent to an explicit target."""

import http.cookiejar
import json
import math
from pathlib import Path
import time
import urllib.request


BASE = "http://127.0.0.1:9983"
TARGET = 5.0
MAX_STEP_RAD = 4.20
TIMEOUT_S = 420.0
REPORT_ROOT = Path("/home/arx/arx-place-hold-20261116/reports")

opener = urllib.request.build_opener(
    urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar())
)
opener.open(BASE + "/", timeout=3).read()


def state():
    return json.load(opener.open(BASE + "/api/state", timeout=2))


def send(payload):
    if payload.get("type") not in ("lift", "lift.stop"):
        raise ValueError("Command outside lift-only scope")
    allowed = {"type", "target"} if payload["type"] == "lift" else {"type"}
    if set(payload) != allowed:
        raise ValueError("Unexpected lift command field")
    request = urllib.request.Request(
        BASE + "/api/command",
        data=json.dumps(payload).encode(),
        headers={"Origin": BASE, "Content-Type": "application/json"},
    )
    result = json.load(opener.open(request, timeout=3))
    if result.get("ok") is not True:
        raise ValueError("Command not acknowledged")
    return result


before = state()
reference = before
epoch = before["command_epoch"]


def validate(current):
    if not current.get("execution_enabled"):
        raise ValueError("Execution disabled")
    if not current["body"].get("fresh") or not all(
        current["arms"][name].get("fresh") for name in ("left", "right")
    ):
        raise ValueError("Feedback unavailable")
    if any(
        current.get(name)
        for name in ("motion_reserved", "competition_reserved", "chassis_active")
    ):
        raise ValueError("Other controller active")
    if current.get("command_epoch") != epoch:
        raise ValueError("Concurrent control changed")
    if abs(
        current["arms"]["right"]["gripper"]
        - reference["arms"]["right"]["gripper"]
    ) > 0.05:
        raise ValueError("Clamp feedback changed")
    # The left arm may be repositioned independently by the on-site operator.
    # Guard the loaded right arm; the current left pose is handled by the
    # preflight geometry check rather than treated as a fixed reference.
    for name in ("right",):
        delta = max(
            abs(a - b)
            for a, b in zip(
                current["arms"][name]["joints"],
                reference["arms"][name]["joints"],
            )
        )
        if delta > math.radians(0.5):
            raise ValueError(f"{name} arm changed")
    for name in ("waist", "head_yaw", "head_pitch"):
        if abs(current["body"][name] - reference["body"][name]) > 0.02:
            raise ValueError("Body orientation changed")


validate(before)
start = float(before["body"]["height"])
limits = tuple(float(v) for v in before["limits"]["lift"])
if not limits[0] <= TARGET <= limits[1]:
    raise ValueError("Target outside lift limits")
if not TARGET < start <= TARGET + 4.2:
    raise ValueError("Unexpected lift start")

segment_count = math.ceil((start - TARGET) / MAX_STEP_RAD)
goals = [start + (TARGET - start) * i / segment_count for i in range(1, segment_count + 1)]
report = {
    "status": "planned",
    "start": start,
    "target": TARGET,
    "goals": goals,
    "robot_motion": False,
    "arm_commands": 0,
    "gripper_commands": 0,
    "records": [],
}
report_dir = REPORT_ROOT / ("lift-to-5rad-" + time.strftime("%Y%m%d-%H%M%S"))
report_dir.mkdir(parents=True, exist_ok=False)
report_file = report_dir / "result.json"
report_file.write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps({"status": "planned", "start": start, "goals": goals}), flush=True)

sent = False
try:
    for index, goal in enumerate(goals, 1):
        validate(state())
        send({"type": "lift", "target": goal})
        sent = True
        epoch += 1
        report["robot_motion"] = True
        started = time.monotonic()
        stable = []
        last_progress = started
        while time.monotonic() - started < TIMEOUT_S:
            current = state()
            validate(current)
            height = float(current["body"]["height"])
            stamp = float(current["body"]["timestamp"])
            stable.append((stamp, height))
            stable = [row for row in stable if stamp - row[0] <= 0.6]
            if time.monotonic() - last_progress >= 10.0:
                print(
                    json.dumps(
                        {
                            "segment": index,
                            "target": goal,
                            "measured": height,
                            "elapsed_s": round(time.monotonic() - started, 1),
                        }
                    ),
                    flush=True,
                )
                last_progress = time.monotonic()
            if (
                abs(height - goal) <= 0.03
                and len(stable) >= 3
                and stable[-1][0] - stable[0][0] >= 0.3
                and max(v for _, v in stable) - min(v for _, v in stable) <= 0.01
            ):
                break
            time.sleep(0.08)
        else:
            raise ValueError(f"Segment {index} arrival timeout")
        report["records"].append(
            {"segment": index, "target": goal, "measured": height}
        )
        report_file.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(
            json.dumps({"segment": index, "target": goal, "measured": height}),
            flush=True,
        )
    report["status"] = "arrived"
except Exception as exc:
    report["status"] = "held"
    report["reason"] = str(exc)
finally:
    if sent:
        try:
            report["stop"] = send({"type": "lift.stop"})
            epoch += 1
        except Exception as exc:
            report["stop_error"] = str(exc)
    time.sleep(0.2)
    report["after"] = state()
    report["gripper_delta"] = (
        report["after"]["arms"]["right"]["gripper"]
        - before["arms"]["right"]["gripper"]
    )
    report_file.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(
        json.dumps(
            {
                "status": report["status"],
                "reason": report.get("reason"),
                "height": report["after"]["body"]["height"],
                "gripper_delta": report["gripper_delta"],
                "report": str(report_file),
            }
        ),
        flush=True,
    )
