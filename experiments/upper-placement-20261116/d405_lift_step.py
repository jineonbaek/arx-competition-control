"""Bounded loaded lift correction with measured-state guards.

Dry-run by default.  This module never sends arm or gripper commands.
"""
import argparse
import http.cookiejar
import json
import math
from pathlib import Path
import time
import urllib.request


NOMINAL_RAD_PER_M = 41.54
MAX_STEP_MM = 15.0


def lift_goal(height, up_mm, *, limits=(0.0, 18.0)):
    values = (height, up_mm, *limits)
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError("Non-finite lift request")
    if not 0.0 < float(up_mm) <= MAX_STEP_MM:
        raise ValueError("Lift step must be positive and bounded to 15 mm")
    goal = float(height) + float(up_mm) / 1000.0 * NOMINAL_RAD_PER_M
    if not float(limits[0]) <= goal <= float(limits[1]):
        raise ValueError("Lift goal exceeds reported limits")
    return goal


def validate_state(current, reference, *, expected_epoch):
    if not current.get("execution_enabled"):
        raise ValueError("Execution disabled")
    if not current["body"].get("fresh") or not all(
        current["arms"][name].get("fresh") for name in ("left", "right")
    ):
        raise ValueError("Feedback unavailable")
    if any(current.get(name) for name in (
        "motion_reserved", "competition_reserved", "chassis_active"
    )):
        raise ValueError("Other controller active")
    if current.get("command_epoch") != expected_epoch:
        raise ValueError("Concurrent control changed")
    if abs(current["arms"]["right"]["gripper"] - reference["arms"]["right"]["gripper"]) > 0.05:
        raise ValueError("Clamp feedback changed")
    for name in ("left", "right"):
        delta = max(abs(a - b) for a, b in zip(
            current["arms"][name]["joints"], reference["arms"][name]["joints"]
        ))
        if delta > math.radians(0.5):
            raise ValueError(f"{name} arm changed")
    for name in ("waist", "head_yaw", "head_pitch"):
        if abs(current["body"][name] - reference["body"][name]) > 0.02:
            raise ValueError("Body orientation changed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--up-mm", type=float, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=20.0)
    args = parser.parse_args()
    if not 2.0 <= args.timeout_s <= 30.0:
        raise ValueError("Timeout outside bounded range")

    base = "http://127.0.0.1:9983"
    opener = urllib.request.build_opener(
        urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar())
    )
    opener.open(base + "/", timeout=3).read()

    def state():
        return json.load(opener.open(base + "/api/state", timeout=2))

    def send(payload):
        if payload.get("type") not in ("lift", "lift.stop"):
            raise ValueError("Command outside lift-only scope")
        if payload.get("type") == "lift" and set(payload) != {"type", "target"}:
            raise ValueError("Unexpected lift command field")
        if payload.get("type") == "lift.stop" and set(payload) != {"type"}:
            raise ValueError("Unexpected lift stop field")
        request = urllib.request.Request(
            base + "/api/command",
            data=json.dumps(payload).encode(),
            headers={"Origin": base, "Content-Type": "application/json"},
        )
        result = json.load(opener.open(request, timeout=3))
        if result.get("ok") is not True:
            raise ValueError("Command not acknowledged")
        return result

    before = state()
    epoch = before["command_epoch"]
    validate_state(before, before, expected_epoch=epoch)
    goal = lift_goal(
        before["body"]["height"], args.up_mm,
        limits=tuple(before["limits"]["lift"]),
    )
    report = {
        "status": "planned",
        "robot_motion": False,
        "gripper_commands": 0,
        "arm_commands": 0,
        "before": before,
        "requested_up_mm": args.up_mm,
        "target_lift_rad": goal,
        "nominal_rad_per_m": NOMINAL_RAD_PER_M,
    }
    report_dir = Path("/home/arx/arx-place-hold-20261116/reports") / (
        "d405-lift-step-" + time.strftime("%Y%m%d-%H%M%S")
    )
    report_dir.mkdir(parents=True, exist_ok=False)
    report_file = report_dir / "result.json"
    report_file.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({
        "status": "planned", "target_lift_rad": goal,
        "requested_up_mm": args.up_mm, "report": str(report_file),
    }), flush=True)
    if not args.execute:
        return

    sent = False
    try:
        validate_state(state(), before, expected_epoch=epoch)
        send({"type": "lift", "target": goal})
        sent = True
        epoch += 1
        report["robot_motion"] = True
        started = time.monotonic()
        stable = []
        while time.monotonic() - started < args.timeout_s:
            current = state()
            validate_state(current, before, expected_epoch=epoch)
            height = float(current["body"]["height"])
            stamp = float(current["body"]["timestamp"])
            stable.append((stamp, height))
            stable = [row for row in stable if stamp - row[0] <= 0.6]
            if (abs(height - goal) <= 0.03 and len(stable) >= 3
                    and stable[-1][0] - stable[0][0] >= 0.3
                    and max(v for _, v in stable) - min(v for _, v in stable) <= 0.01):
                break
            time.sleep(0.08)
        else:
            raise ValueError("Measured lift arrival timeout")
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
        report["actual_up_mm_nominal"] = (
            report["after"]["body"]["height"] - before["body"]["height"]
        ) / NOMINAL_RAD_PER_M * 1000.0
        report["gripper_delta"] = (
            report["after"]["arms"]["right"]["gripper"]
            - before["arms"]["right"]["gripper"]
        )
        report_file.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps({
            "status": report["status"],
            "reason": report.get("reason"),
            "actual_up_mm_nominal": report["actual_up_mm_nominal"],
            "gripper_delta": report["gripper_delta"],
            "report": str(report_file),
        }), flush=True)


if __name__ == "__main__":
    main()
