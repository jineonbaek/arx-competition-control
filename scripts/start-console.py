#!/usr/bin/env python3
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "web/hardware/runtime"
URL = "http://127.0.0.1:9983"
TITLE = "ARX \u4e00\u952e\u542f\u52a8\u4e0e\u81ea\u68c0"
LABELS = {"left": "\u5de6\u81c2", "right": "\u53f3\u81c2", "body": "\u5e95\u76d8\u548c\u5347\u964d"}
ARM = ROOT / "web/hardware/build/x5_controller_web"
BODY = ROOT / "web/hardware/build/lift_controller_web"
SERVER = ROOT / "web/src/server.py"


def processes():
    commands = []
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args = path.read_bytes().decode(errors="replace").rstrip("\0").split("\0")
        except (PermissionError, ProcessLookupError, FileNotFoundError):
            continue
        if args[0]:
            commands.append(args)
    return commands


def start(name, command, env=None):
    with (RUNTIME / f"launcher-{name}.log").open("ab") as log:
        return subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True,
                                stdin=subprocess.DEVNULL, stdout=log, stderr=log)


def check_exit(children):
    for name, child in children.items():
        if child.poll() is not None:
            raise StartupError(name, f"\u8fdb\u7a0b\u9000\u51fa\uff08\u9000\u51fa\u7801 {child.returncode}\uff09\u3002\u65e5\u5fd7\uff1alauncher-{name}.log")


def wait_for(check, seconds, children):
    deadline = time.monotonic() + seconds
    while True:
        check_exit(children)
        value = check()
        if value is not None:
            return value
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.4)


def link(channel):
    result = subprocess.run(["ip", "-j", "link", "show", channel], capture_output=True, text=True)
    return json.loads(result.stdout)[0] if result.returncode == 0 else None


def ensure_can(channel, commands, children):
    device = Path("/dev/arx" + channel)
    if not device.is_char_device():
        raise RuntimeError(f"\u672a\u68c0\u6d4b\u5230 USB \u8f6c CAN \u8bbe\u5907 {device}")
    adapters = [args for args in commands if Path(args[0]).name == "slcand" and args[-1] == channel]
    existing = link(channel)
    if existing or adapters:
        if len(adapters) != 1 or os.path.realpath(adapters[0][-2]) != str(device.resolve()):
            raise RuntimeError(f"{channel} \u7684\u9002\u914d\u5668\u6620\u5c04\u4e0d\u4e00\u81f4")
    else:
        owner = subprocess.run(["sudo", "-n", "fuser", "-s", str(device)], capture_output=True)
        if owner.returncode != 1 or owner.stderr:
            raise RuntimeError(f"\u65e0\u6cd5\u786e\u8ba4 {device} \u7a7a\u95f2\uff0c\u8bf7\u68c0\u67e5\u8bbe\u5907\u5360\u7528\u548c sudo \u6743\u9650")
        children[channel] = start(channel, ["sudo", "-n", "slcand", "-F", "-o", "-s8",
                                            "-S115200", str(device), channel])
    existing = wait_for(lambda: link(channel), 5, children)
    if existing is None or existing.get("link_type") != "can":
        raise RuntimeError(f"\u672a\u80fd\u5efa\u7acb CAN \u63a5\u53e3 {channel}")
    if "UP" not in existing["flags"]:
        subprocess.run(["sudo", "-n", "ip", "link", "set", channel, "up"], check=True)


def state():
    try:
        with urlopen(URL + "/api/state", timeout=2) as response:
            value = json.load(response)
        if not isinstance(value, dict):
            raise ValueError("expected an object")
        return value
    except (OSError, URLError, ValueError):
        return None


CHECKS = {
    "left": ("left", ("left:arm.joints", "left:gripper", "left:arm.stop")),
    "right": ("right", ("right:arm.joints", "right:gripper", "right:arm.stop")),
    "chassis": ("body", ("chassis", "chassis.stop")),
    "lift": ("body", ("lift", "lift.stop")),
}


def check_fault(snapshot, name):
    if snapshot is None:
        return "9983 \u72b6\u6001\u63a5\u53e3\u4e0d\u53ef\u7528\uff0c\u65e0\u6cd5\u81ea\u68c0"
    ros = snapshot.get("ros", {})
    if not ros.get("connected") or ros.get("domain_id") != "191":
        return ros.get("error") or "ROS \u57df 191 \u672a\u8fde\u63a5"
    topic, keys = CHECKS[name]
    source = ros.get("feedback_sources", {}).get(topic, {})
    if (source.get("publishers") or 0) > 1:
        return "\u68c0\u6d4b\u5230\u591a\u4e2a\u53cd\u9988\u53d1\u5e03\u8005\uff0c\u8bf7\u68c0\u67e5\u662f\u5426\u91cd\u590d\u542f\u52a8\u9a71\u52a8"
    if source.get("other_publishers"):
        return "\u53cd\u9988\u8bdd\u9898\u5b58\u5728\u7c7b\u578b\u4e0d\u5339\u914d\u7684\u53d1\u5e03\u8005"
    if (source.get("status"), source.get("publishers")) != ("fresh", 1):
        return source.get("message") or "\u6ca1\u6709\u5b9e\u65f6\u53cd\u9988"
    caps = snapshot.get("capabilities", {})
    return next((caps.get(key) or f"\u672a\u62a5\u544a\u63a7\u5236\u63a5\u53e3 {key} \u7684\u72b6\u6001"
                 for key in keys if caps.get(key) != ""), "")


def component_status(snapshot, failed=None, reason=""):
    faults = {name: check_fault(snapshot, name) for name in CHECKS}
    if failed in ("left", "can1"):
        faults["left"] = reason
    elif failed in ("right", "can3"):
        faults["right"] = reason
    elif failed in ("body", "can5"):
        faults["chassis"] = faults["lift"] = reason
    arm_faults = [f"{LABELS[side]}\uff1a{faults[side]}" for side in ("left", "right") if faults[side]]
    return {"\u673a\u68b0\u81c2": "\n".join(arm_faults), "\u5e95\u76d8": faults["chassis"], "\u5347\u964d": faults["lift"]}


class StartupError(RuntimeError):
    def __init__(self, stage, message):
        super().__init__(message)
        self.stage = stage


def launch():
    children = {}
    commands = processes()
    for path in (ROOT / ".venv/bin/python", SERVER, ARM, BODY):
        if not path.is_file():
            raise RuntimeError(f"Missing {path}")
    web = [args for args in commands if len(args) > 1 and args[1] == str(SERVER)]
    listeners = subprocess.check_output(["ss", "-H", "-ltn", "sport = :9983"], text=True)
    if len(web) > 1 or (not web and listeners.strip()):
        raise RuntimeError("9983 \u7aef\u53e3\u5df2\u88ab\u5176\u4ed6\u8fdb\u7a0b\u5360\u7528")
    if not web:
        children["web"] = start("web", ["bash", "web/start.sh"],
                                dict(os.environ, WEB_HOST="0.0.0.0", WEB_PORT="9983",
                                     WEB_ROS_DOMAIN_ID="191", ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"))
    snapshot = wait_for(state, 10, children)
    if snapshot is None or snapshot.get("ros", {}).get("domain_id") != "191":
        raise RuntimeError("9983 \u72b6\u6001\u63a5\u53e3\u4e0d\u53ef\u7528\u6216 ROS \u57df\u4e0d\u5339\u914d")

    missing = []
    for name in LABELS:
        matching = [args for args in commands if
                    (name == "body" and args[0] == str(BODY)) or
                    (name != "body" and args[0] == str(ARM) and f"side:={name}" in args)]
        source = snapshot["ros"].get("feedback_sources", {}).get(name, {})
        if len(matching) > 1 or (not matching and (source.get("publishers") or source.get("other_publishers"))):
            raise StartupError(name, "\u5df2\u6709\u53e6\u4e00\u4e2a\u9a71\u52a8\u8fdb\u7a0b\u6216\u53cd\u9988\u53d1\u5e03\u8005\uff0c\u8bf7\u68c0\u67e5\u662f\u5426\u91cd\u590d\u542f\u52a8")
        if not matching:
            missing.append(name)

    for channel in ("can1", "can3", "can5"):
        try:
            ensure_can(channel, commands, children)
        except StartupError:
            raise
        except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
            raise StartupError(channel, f"{channel}: {exc}") from exc
    for name in missing:
        command = ["bash", "web/hardware/start-body.sh"] if name == "body" else [
            "bash", "web/hardware/start-arm.sh", name]
        children[name] = start(name, command)
        (RUNTIME / (("body" if name == "body" else f"{name}-arm") + ".pid")).write_text(
            f"{children[name].pid}\n")

    def ready():
        snapshot = state()
        statuses = component_status(snapshot)
        return statuses if not any(statuses.values()) else None

    statuses = wait_for(ready, 75 if missing else 8, children)
    if statuses is None:
        return component_status(state())

    with urlopen(URL + "/", timeout=2) as page:
        if page.status != 200:
            raise RuntimeError("9983 homepage is unavailable")
    return statuses


def show_status(statuses, warning):
    import gi
    gi.require_version("Gtk", "3.0")
    from gi.repository import Gtk

    dialog = Gtk.Dialog(title="ARX \u8bbe\u5907\u81ea\u68c0")
    dialog.set_default_size(530, -1)
    dialog.add_button("\u5173\u95ed", Gtk.ResponseType.CLOSE)
    box = dialog.get_content_area()
    box.set_border_width(16)
    box.set_spacing(12)
    for name, fault in statuses.items():
        row = Gtk.Box(spacing=12)
        name_label = Gtk.Label(xalign=0)
        name_label.set_markup(f"<b>{name}</b>")
        status_label = Gtk.Label(xalign=1)
        status_label.set_markup('<span foreground="#b42318"><b>\u4e0d\u597d</b></span>' if fault else
                                '<span foreground="#067647"><b>\u597d</b></span>')
        row.pack_start(name_label, True, True, 0)
        row.pack_end(status_label, False, False, 0)
        box.pack_start(row, False, False, 0)
        if fault:
            detail = Gtk.Label(label=fault, xalign=0)
            detail.set_line_wrap(True)
            detail.set_selectable(True)
            box.pack_start(detail, False, False, 0)
        box.pack_start(Gtk.Separator(), False, False, 0)
    if warning:
        detail = Gtk.Label(label=warning, xalign=0)
        detail.set_line_wrap(True)
        detail.set_selectable(True)
        box.pack_start(detail, False, False, 0)
    dialog.show_all()
    dialog.run()
    dialog.destroy()


def main():
    desktop = sys.argv[1:] == ["--desktop"]
    progress = None
    warning = ""
    try:
        if desktop:
            progress = subprocess.Popen(
                ["zenity", "--progress", "--pulsate", "--no-cancel", "--title", TITLE,
                 "--text", "\u6b63\u5728\u68c0\u67e5\u673a\u68b0\u81c2\u3001\u5e95\u76d8\u548c\u5347\u964d\u3002\u542f\u52a8\u65f6\u53cc\u81c2\u53ef\u80fd\u4f7f\u80fd\uff0c\u5347\u964d\u53ef\u80fd\u56de\u96f6\u3002"],
                stdin=subprocess.PIPE)
        RUNTIME.mkdir(parents=True, exist_ok=True)
        with (RUNTIME / "launcher.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            statuses = launch()
    except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
        stage = getattr(exc, "stage", None)
        statuses = component_status(state(), stage, str(exc))
        if stage in (None, "web"):
            warning = f"\u542f\u52a8\u68c0\u67e5\u9047\u5230\u95ee\u9898\uff1a{exc}"
    finally:
        if progress is not None:
            progress.terminate()
            progress.communicate()
    if desktop:
        show_status(statuses, warning)
    return int(bool(warning) or any(statuses.values()))


if __name__ == "__main__":
    sys.exit(main())
