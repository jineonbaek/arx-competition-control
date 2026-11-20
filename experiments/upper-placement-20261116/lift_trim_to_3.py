"""Overcome low-position stiction while stopping on measured 3 rad height."""

import http.cookiejar
import json
import time
import urllib.request


BASE = "http://127.0.0.1:9983"
DRIVE_TARGET = 2.5
STOP_AT = 3.05
LOW_ABORT = 2.90

opener = urllib.request.build_opener(
    urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar())
)
opener.open(BASE + "/", timeout=3).read()


def state():
    return json.load(opener.open(BASE + "/api/state", timeout=2))


def send(payload):
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
height = float(before["body"]["height"])
if not 3.20 <= height <= 3.80:
    raise ValueError(f"Unexpected trim start {height}")
if not before["body"]["fresh"] or not before["arms"]["right"]["fresh"]:
    raise ValueError("Feedback unavailable")
if any(before.get(name) for name in ("motion_reserved", "competition_reserved", "chassis_active")):
    raise ValueError("Other controller active")

epoch = before["command_epoch"]
gripper = float(before["arms"]["right"]["gripper"])
sent = False
status = "held"
reason = None
try:
    send({"type": "lift", "target": DRIVE_TARGET})
    sent = True
    epoch += 1
    started = time.monotonic()
    last_progress = started
    while time.monotonic() - started < 120.0:
        current = state()
        if current["command_epoch"] != epoch:
            raise ValueError("Concurrent control changed")
        if not current["body"]["fresh"] or not current["arms"]["right"]["fresh"]:
            raise ValueError("Feedback unavailable")
        if abs(float(current["arms"]["right"]["gripper"]) - gripper) > 0.05:
            raise ValueError("Clamp feedback changed")
        height = float(current["body"]["height"])
        if height < LOW_ABORT:
            raise ValueError("Lift passed low abort threshold")
        if height <= STOP_AT:
            status = "reached_stop_threshold"
            break
        if time.monotonic() - last_progress >= 5.0:
            print(json.dumps({"height": height, "elapsed_s": round(time.monotonic() - started, 1)}), flush=True)
            last_progress = time.monotonic()
        time.sleep(0.02)
    else:
        reason = "Trim timeout"
finally:
    if sent:
        send({"type": "lift.stop"})
    time.sleep(0.25)
    after = state()
    print(
        json.dumps(
            {
                "status": status,
                "reason": reason,
                "height": after["body"]["height"],
                "gripper": after["arms"]["right"]["gripper"],
            }
        ),
        flush=True,
    )
