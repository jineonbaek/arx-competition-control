#!/usr/bin/env python3
import fcntl
import json
import math
import os
import select
import socket
import struct
import threading
from http.cookiejar import CookieJar
from pathlib import Path
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen, Request, build_opener, HTTPCookieProcessor

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "web/hardware/runtime"
URL = "http://127.0.0.1:9983"
TITLE = "ARX \u4e00\u952e\u542f\u52a8\u4e0e\u81ea\u68c0"
LABELS = {"left": "\u5de6\u81c2", "right": "\u53f3\u81c2", "body": "\u5e95\u76d8\u548c\u5347\u964d"}
ARM = ROOT / "web/hardware/build/x5_controller_web"
BODY = ROOT / "web/hardware/build/lift_controller_web"
SERVER = ROOT / "web/src/server.py"
CAMERA = ROOT / "web/src/camera_preview.py"
CAMERA_OUTPUT = ROOT / "web/public/camera-live"
CAMERA_NAMES = {"d435": "D435", "d405": "D405", "astra": "Astra Pro"}


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



def camera_fault(not_before=0.0):
    faults = []
    now = time.time()
    for key, label in CAMERA_NAMES.items():
        try:
            meta = json.loads((CAMERA_OUTPUT / (key + ".json")).read_text())
            captured = meta.get("captured_at")
            if (meta.get("state") != "online" or type(captured) not in (int, float)
                    or not math.isfinite(captured) or captured < not_before
                    or not -1.5 <= now-captured <= 3):
                faults.append(label + "：" + (meta.get("message") or "画面未更新"))
        except (OSError, ValueError, AttributeError):
            faults.append(label + "：等待相机画面")
    return "\n".join(faults)



def whole_battery_fault():
    faults=[]
    for key,label in [("d435","D435"),("d405","D405"),("astra","Astra Pro")]:
        try:
            meta=json.loads((CAMERA_OUTPUT/(key+"-battery.json")).read_text())
            captured=meta.get("captured_at")
            if (meta.get("state")!="online" or not isinstance(meta.get("whole_battery"),dict) or not isinstance(meta.get("whole_battery",{}).get("holes"),list)
                    or type(captured) not in (int,float) or not math.isfinite(captured)
                    or not -1.5<=time.time()-captured<=3):
                faults.append(label+"："+(meta.get("message") or "整电池识别未更新"))
        except (OSError,ValueError,TypeError,AttributeError):
            faults.append(label+"：等待整电池识别")
    return "\n".join(faults)


def ensure_cameras():
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / "camera-launcher.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        matching = [args for args in processes() if len(args)>1 and args[1]==str(CAMERA)]
        if len(matching)>1:
            return "发现多个相机采集进程，请检查重复启动"
        children = {}
        not_before = 0.0 if matching else time.time()
        try:
            if not matching:
                if not CAMERA.is_file():
                    return "相机采集程序不存在"
                children["cameras"] = start("cameras", ["/usr/bin/python3", str(CAMERA)])
                (RUNTIME / "cameras.pid").write_text(str(children["cameras"].pid) + "\n")
            ready = wait_for(lambda: True if not camera_fault(not_before) else None, 15, children)
            return "" if ready else camera_fault(not_before)
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            return "相机启动失败：" + str(exc)


def link(channel):
    result = subprocess.run(["ip", "-details", "-j", "link", "show", channel], capture_output=True, text=True, timeout=2)
    return json.loads(result.stdout)[0] if result.returncode == 0 else None


def ensure_can(channel, commands, children):
    device = Path("/dev/arx" + channel)
    if not device.is_char_device():
        raise RuntimeError(f"\u672a\u68c0\u6d4b\u5230 USB \u8f6c CAN \u8bbe\u5907 {device}")
    adapters = [args for args in commands if Path(args[0]).name == "slcand" and args[-1] == channel]
    existing = link(channel)
    if existing or adapters:
        info = device.stat()
        fault = adapter_fault(channel, adapter_records(), [info.st_dev, info.st_ino, info.st_rdev])
        if fault:
            raise RuntimeError(fault)
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
        if not source.get("publishers"):
            return "驱动未启动或已退出；" + (source.get("message") or "未发现反馈发布者")
        return source.get("message") or "\u6ca1\u6709\u5b9e\u65f6\u53cd\u9988"
    caps = snapshot.get("capabilities", {})
    return next((caps.get(key) or f"\u672a\u62a5\u544a\u63a7\u5236\u63a5\u53e3 {key} \u7684\u72b6\u6001"
                 for key in keys if caps.get(key) != ""), "")


CHANNELS = {"left": "can1", "right": "can3", "body": "can5"}
MOTOR_ERRORS = {8: "过压", 9: "欠压", 10: "过流", 11: "MOS 过温",
                12: "线圈过温", 13: "通信丢失", 14: "过载"}
# This is a software-zero tolerance, not a calibrated ground clearance.
BOTTOM_MAX_RAD = 0.6


def process_records():
    records = []
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args = path.read_bytes().decode().rstrip("\0").split("\0")
            if args[0]:
                records.append({"pid": int(path.parent.name), "args": args,
                                "start": (path.parent / "stat").read_text().rsplit(")", 1)[1].split()[19]})
        except (OSError, ValueError, UnicodeError):
            continue
    return records


def owned(name, records=None):
    def matches(args):
        if name in ("left", "right"):
            return args[0] == str(ARM) and f"side:={name}" in args
        if name == "body":
            return args[0] == str(BODY)
        target = SERVER if name == "web" else CAMERA
        return len(args) > 1 and args[1] == str(target)
    return [record for record in (process_records() if records is None else records)
            if matches(record["args"])]


def adapter_records():
    # Reading the actual root-owned open FD catches USB reconnects hidden by /dev aliases.
    script = r"""
import json, os
from pathlib import Path
records=[]
for path in Path('/proc').glob('[0-9]*/cmdline'):
    try:
        args=path.read_bytes().decode().rstrip('\0').split('\0')
        if not args[0] or Path(args[0]).name!='slcand': continue
        serial=[]
        for fd in (path.parent/'fd').iterdir():
            try:
                target=os.readlink(fd)
                if target.startswith('/dev/tty'):
                    s=fd.stat()
                    serial.append({'path':target,'identity':[s.st_dev,s.st_ino,s.st_rdev]})
            except OSError: pass
        records.append({'pid':int(path.parent.name),'args':args,'serial':serial,
                        'start':(path.parent/'stat').read_text().rsplit(')',1)[1].split()[19]})
    except (OSError,ValueError,UnicodeError): pass
print(json.dumps(records))
"""
    result = subprocess.run(["sudo", "-n", "/usr/bin/python3", "-c", script],
                            capture_output=True, text=True, timeout=3)
    if result.returncode:
        raise RuntimeError("无法检查 CAN 适配器的实际设备连接：" + result.stderr.strip())
    return json.loads(result.stdout)


def adapter_fault(channel, records, identity):
    matching = [r for r in records if r["args"][-1] == channel]
    if len(matching) != 1:
        return f"{channel}：适配器进程数量 {len(matching)}，应为 1"
    serial = matching[0]["serial"]
    if not serial or any("(deleted)" in fd["path"] or fd["identity"] != identity for fd in serial):
        return f"{channel}：适配器仍连接旧设备或设备已断开（USB 重连后需恢复）"
    return ""


def motor_feedback(seconds=0.35):
    sockets, seen, errors, lift = {}, {}, {}, []
    chassis_seen = False
    try:
        for channel in CHANNELS.values():
            try:
                sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                sock.setblocking(False)
                sock.bind((channel,))
                sockets[sock] = channel
                seen[channel] = set()
            except OSError as exc:
                errors[channel] = f"{channel}：无法只读接收 CAN：{exc}"
                if "sock" in locals():
                    sock.close()
        deadline = time.monotonic() + seconds
        while sockets and time.monotonic() < deadline:
            ready, _, _ = select.select(list(sockets), [], [], max(0, deadline-time.monotonic()))
            for sock in ready:
                frame, _, flags, _ = sock.recvmsg(16)
                if len(frame) != 16 or flags & socket.MSG_DONTROUTE:
                    continue
                cid, size, data = struct.unpack("=IB3x8s", frame)
                if cid & 0xE0000000:
                    continue
                channel = sockets[sock]
                # Vendor ChassisType1: wheel feedback 702, periodic IMU 706/707/708.
                if channel == "can5" and size == 8 and cid in (0x702, 0x706, 0x707, 0x708):
                    chassis_seen = True
                    continue
                motor = data[0] & 15
                if channel != "can5" and size >= 6 and cid in (1, 2, 4):
                    seen[channel].add(cid)
                elif size == 8 and cid in (0, motor):
                    valid = motor == 1 if channel == "can5" else 5 <= motor <= 8
                    if not valid:
                        continue
                    seen[channel].add(motor)
                    status = data[0] >> 4
                    if status != 1:
                        errors[channel] = f"{channel}：电机 {motor} " + MOTOR_ERRORS.get(status, f"未使能或未知状态 0x{status:X}")
                    if channel == "can5":
                        position = ((data[1] << 8) | data[2]) / 65535 * 25 - 12.5
                        speed = ((data[3] << 4) | (data[4] >> 4)) / 4095 * 90 - 45
                        lift.append((position, speed))
    except OSError as exc:
        for channel in CHANNELS.values():
            errors[channel] = f"{channel}：CAN 采样中断：{exc}"
    finally:
        for sock in sockets:
            sock.close()
    for channel in CHANNELS.values():
        required = {1} if channel == "can5" else {1, 2, 4, 5, 6, 7, 8}
        missing = required - seen.get(channel, set())
        if missing:
            errors.setdefault(channel, f"{channel}：未收到电机 {sorted(missing)} 的真实反馈；检查供电、急停、接线或驱动")
    if not chassis_seen:
        errors["chassis"] = "未收到底盘控制器反馈（0x702/706/707/708）；检查机体急停、底盘供电或 CAN 支路"
    sample = None
    if lift:
        positions = [item[0] for item in lift]
        sample = {"position": positions[-1], "span": max(positions)-min(positions),
                  "speed": max(abs(item[1]) for item in lift), "fault": errors.get("can5", "")}
    return errors, sample


def can_health():
    faults = {}
    try:
        adapters = adapter_records()
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        adapters = None
        faults = {channel: str(exc) for channel in CHANNELS.values()}
    for channel in CHANNELS.values():
        reasons = []
        device = Path("/dev/arx" + channel)
        try:
            if not device.is_char_device():
                reasons.append(f"{channel}：USB 设备 {device} 不存在")
            elif adapters is not None:
                s = device.stat()
                fault = adapter_fault(channel, adapters, [s.st_dev, s.st_ino, s.st_rdev])
                if fault:
                    reasons.append(fault)
            interface = link(channel)
            if interface is None:
                reasons.append(f"{channel}：CAN 接口不存在（可能 USB 掉线）")
            elif interface.get("link_type") != "can" or "UP" not in interface.get("flags", []):
                reasons.append(f"{channel}：CAN 接口未启用")
            elif interface.get("linkinfo", {}).get("info_data", {}).get("state") in ("BUS-OFF", "ERROR-PASSIVE", "STOPPED"):
                reasons.append(f"{channel}：总线状态 " + interface["linkinfo"]["info_data"]["state"])
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            reasons.append(f"{channel}：接口检查失败：{exc}")
        faults[channel] = "\n".join(filter(None, [faults.get(channel, ""), *reasons]))
    received, sample = motor_feedback()
    for channel in CHANNELS.values():
        faults[channel] = "\n".join(filter(None, [faults[channel], received.get(channel, "")]))
    faults["chassis"] = received.get("chassis", "")
    return faults, sample


def recovery_plan(snapshot, buses=None):
    if (snapshot is None or not snapshot.get("ros", {}).get("connected")
            or snapshot.get("ros", {}).get("domain_id") != "191"):
        result = ["web"]
    else:
        buses = can_health()[0] if buses is None else buses
        result = [name for name, channel in CHANNELS.items()
                  if buses[channel] or check_fault(snapshot, "lift" if name == "body" else name)
                  or (name == "body" and (buses.get("chassis") or check_fault(snapshot, "chassis")))]
    if camera_fault() or whole_battery_fault():
        result.append("cameras")
    return result


def bottom_ok(snapshot, sample):
    if snapshot is None or sample is None or sample["fault"] or check_fault(snapshot, "lift"):
        return False
    body = snapshot.get("body", {})
    height = body.get("height")
    return (body.get("fresh") and type(height) in (int, float) and math.isfinite(height)
            and -0.1 <= height <= BOTTOM_MAX_RAD and sample["span"] <= 0.025
            and sample["speed"] <= 0.08 and abs(sample["position"] + height) <= 0.06
            and not snapshot.get("chassis_active"))


def command(data):
    jar = CookieJar()
    client = build_opener(HTTPCookieProcessor(jar))
    with client.open(URL + "/", timeout=3):
        pass
    request = Request(URL + "/api/command", json.dumps(data).encode(),
                      {"Origin": URL, "Content-Type": "application/json"})
    with client.open(request, timeout=3) as response:
        result = json.load(response)
    if not result.get("ok"):
        raise RuntimeError(result.get("message") or "控制请求被拒绝")


def native_lift_state(operation="read"):
    # Talk only to the existing ROS owner; never construct a second SDK/CAN controller.
    script = r"""
import json,sys,time,uuid
import rclpy
from arm_control.msg import PosCmd
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
rclpy.init()
node=rclpy.create_node('launcher_lift_'+uuid.uuid4().hex[:8])
latest={}
def body(msg): latest['body']=(msg,time.monotonic())
def calibrated(msg): latest['calibrated']=(msg.data,time.monotonic())
node.create_subscription(PosCmd,'/body_information',body,rclpy.qos.qos_profile_sensor_data)
node.create_subscription(Bool,'/web/body/calibrated',calibrated,rclpy.qos.qos_profile_sensor_data)
params=node.create_client(SetParameters,'/lift_controller/set_parameters')
stop=node.create_client(Trigger,'/web/body/stop_lift')
operation=sys.argv[1];future=None;deadline=time.monotonic()+4
try:
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        now=time.monotonic()
        source=node.get_publishers_info_by_topic('/body_information')
        fresh=(len(source)==1 and source[0].topic_type=='arm_control/msg/PosCmd'
               and 'body' in latest and now-latest['body'][1]<.5
               and 'calibrated' in latest and latest['calibrated'][0] and now-latest['calibrated'][1]<.5)
        if operation=='stop' and future is None and stop.service_is_ready():
            future=stop.call_async(Trigger.Request())
        if not fresh or not stop.service_is_ready(): continue
        if operation=='lower' and future is None and params.service_is_ready():
            request=SetParameters.Request()
            request.parameters=[Parameter(name='fixed_body_height',value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,double_value=0.0))]
            future=params.call_async(request)
        if operation!='read':
            if future is None or not future.done(): continue
            response=future.result()
            if operation=='stop':
                if not response.success: raise RuntimeError(response.message)
            elif not response.results or not all(r.successful for r in response.results):
                raise RuntimeError('Lift driver rejected the zero target')
        print(json.dumps({'ros':{'connected':True,'domain_id':'191','feedback_sources':{
            'body':{'status':'fresh','publishers':1,'other_publishers':0}}},
            'body':{'height':float(latest['body'][0].height),'fresh':True},
            'capabilities':{'lift':'','lift.stop':''},'chassis_active':False}))
        break
    else: raise RuntimeError('No fresh calibrated lift feedback / ROS service timeout')
finally:
    node.destroy_node()
    rclpy.shutdown()
"""
    result = subprocess.run(["bash", "-c",
        'source "$1"; export ROS_DOMAIN_ID=191 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; exec /usr/bin/python3 -c "$2" "$3"',
        "launcher-lift", str(ROOT / "env.sh"), script, operation],
        capture_output=True, text=True, timeout=7)
    if result.returncode:
        raise RuntimeError("现有机体驱动的底层 ROS 升降通道不可用：" + result.stderr.strip()[-1200:])
    return json.loads(result.stdout.strip().splitlines()[-1])


def lift_state():
    snapshot = state()
    return snapshot if not check_fault(snapshot, "lift") else native_lift_state()


def lift_command(kind):
    if not check_fault(state(), "lift"):
        command({"type": "lift", "target": 0.0} if kind == "lower" else {"type": "lift.stop"})
    else:
        native_lift_state(kind)


def prepare_recovery(progress, cancel=None):
    buses, sample = can_health()
    if buses["can5"]:
        return False  # A disconnected CAN bus cannot carry a lowering command.
    snapshot = lift_state()
    if snapshot.get("chassis_active"):
        raise RuntimeError("底盘正在运动，先在网页停车再恢复")
    if cancel is not None and cancel.is_set():
        raise RuntimeError("已取消恢复，未发下降指令")
    progress("正在通过现有驱动缓慢降至软件零位；请现场观察，保持下方清空")
    deadline, stable, previous, last_progress = time.monotonic()+55, None, None, time.monotonic()
    try:
        lift_command("lower")
        while time.monotonic() < deadline:
            if cancel is not None and cancel.is_set():
                raise RuntimeError("已取消下降和恢复，已请求保持升降位置")
            snapshot = lift_state()
            buses, sample = can_health()
            if buses["can5"] or check_fault(snapshot, "lift"):
                raise RuntimeError("下降中反馈失效，已中止恢复；保持现场监护，禁止断电或重启")
            height = snapshot.get("body", {}).get("height")
            if type(height) not in (int, float) or not math.isfinite(height):
                raise RuntimeError("升降高度无效，不能继续恢复")
            progress(f"升降实际反馈 {height:.3f} SDK rad；正在检查最低附近并停稳")
            now = time.monotonic()
            if previous is None or height < previous-0.05:
                previous, last_progress = height, now
            if bottom_ok(snapshot, sample):
                stable = now if stable is None else stable
                if now-stable >= 1.5:
                    return True
            else:
                stable = None
            if now-last_progress > 8:
                raise RuntimeError("升降下降无有效进展，可能受阻；恢复已中止，请现场检查")
            time.sleep(0.15)
        raise RuntimeError("升降未在 55 秒内到达零位附近，禁止重启")
    except Exception as exc:
        # Hold via the existing controller; never disable the lift to abort recovery.
        try:
            lift_command("stop")
        except Exception as hold_error:
            raise RuntimeError(f"{exc}；保持请求也未确认：{hold_error}。保持现场监护，禁止高位断电或重启") from exc
        raise


def stop_owned(name, record=None):
    records = owned(name) if record is None else [record]
    if len(records) > 1:
        raise RuntimeError(f"{name} 有重复进程，不能自动选择并终止")
    for item in records:
        script = r"""
import json,os,signal,sys,time
from pathlib import Path
r=json.loads(sys.argv[1]); p=Path('/proc')/str(r['pid'])
try:
    fd=os.pidfd_open(r['pid'])
    args=(p/'cmdline').read_bytes().decode().rstrip('\0').split('\0')
    start=(p/'stat').read_text().rsplit(')',1)[1].split()[19]
    if args!=r['args'] or start!=r['start']: raise RuntimeError('Process identity changed; refusing stop')
    signal.pidfd_send_signal(fd,signal.SIGTERM)
    import select
    if not select.select([fd],[],[],4)[0]: raise RuntimeError('Process did not exit; no forced kill')
    os.close(fd)
except ProcessLookupError: pass
"""
        result = subprocess.run(["sudo", "-n", "/usr/bin/python3", "-c", script, json.dumps(item)],
                                capture_output=True, text=True, timeout=6)
        if result.returncode:
            raise RuntimeError(f"{name} 停止失败：" + result.stderr.strip()[-1500:])


def restart_faults(physical_bottom=False, manual=False, selected=None, progress=lambda message: None):
    if not physical_bottom:
        raise RuntimeError("尚未现场确认升降最低且稳定，禁止重启")
    with (RUNTIME / "launcher.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        snapshot = state()
        buses, sample = can_health()
        lift_snapshot = snapshot
        if not manual and not buses["can5"]:
            lift_snapshot = lift_state()
        if not manual and (buses["can5"] or not bottom_ok(lift_snapshot, sample)):
            raise RuntimeError("最低位置证明已失效，禁止重启")
        if manual and not buses["can5"] and not check_fault(snapshot, "lift") and not bottom_ok(snapshot, sample):
            raise RuntimeError("实时反馈与人工最低确认不一致，禁止重启")
        names = recovery_plan(snapshot, buses)
        if selected is not None:
            names = [name for name in names if name in selected]
        # Refuse foreign publishers, duplicates and missing USB devices before stopping anything.
        for name in names:
            if len(owned(name)) > 1:
                raise RuntimeError(f"{name}：存在重复进程，不能自动恢复")
            if name in CHANNELS:
                source = (snapshot or {}).get("ros", {}).get("feedback_sources", {}).get(name, {})
                if source.get("other_publishers") or (source.get("publishers") or 0) > len(owned(name)):
                    raise RuntimeError(f"{name}：存在其他驱动发布者，不能自动恢复")
                if not Path("/dev/arx"+CHANNELS[name]).is_char_device():
                    raise RuntimeError(f"{CHANNELS[name]}：USB 设备仍不在，先恢复接线/供电；未停止任何驱动")
                path = BODY if name == "body" else ARM
                if not path.is_file():
                    raise RuntimeError(f"驱动文件不存在：{path}")
                if len([r for r in adapter_records() if r["args"][-1] == CHANNELS[name]]) > 1:
                    raise RuntimeError(f"{name}：存在重复 CAN 适配器，不能自动恢复")
        children = {}
        for name in names:
            # Recheck immediately before each stop in case another client moved the lift.
            current, (current_buses, sample) = state(), can_health()
            if not manual and not current_buses["can5"]:
                current = lift_state()
            if ((not manual or not check_fault(current, "lift")) and not bottom_ok(current, sample)):
                raise RuntimeError("升降最低状态已变化，剩余服务未重启")
            progress("正在恢复：" + LABELS.get(name, {"web": "网页", "cameras": "相机及识别"}.get(name, name)))
            stop_owned(name)
            if name in CHANNELS:
                channel = CHANNELS[name]
                if buses[channel]:
                    for record in adapter_records():
                        if record["args"][-1] == channel:
                            stop_owned(channel, record)
                ensure_can(channel, processes(), children)
                cmd = ["bash", "web/hardware/start-body.sh"] if name == "body" else ["bash", "web/hardware/start-arm.sh", name]
                children[name] = start(name, cmd)
                (RUNTIME / (("body" if name == "body" else f"{name}-arm") + ".pid")).write_text(
                    str(children[name].pid) + "\n")
                if name == "body":
                    if wait_for(lambda: True if not check_fault(state(), "lift") else None,
                                75, children) is None:
                        raise RuntimeError("机体恢复后尚未就绪，剩余服务未重启")
            elif name == "web":
                children[name] = start("web", ["bash", "web/start.sh"],
                    dict(os.environ, WEB_HOST="0.0.0.0", WEB_PORT="9983",
                         WEB_ROS_DOMAIN_ID="191", ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"))
                if wait_for(state, 12, children) is None:
                    raise RuntimeError("网页重启失败，请检查 launcher-web.log")
            else:
                fault = ensure_cameras()
                if fault:
                    raise RuntimeError(fault)
        def ready():
            snapshot = state()
            buses, _ = can_health()
            return True if not any(name in recovery_plan(snapshot, buses) for name in names) else None
        if names and wait_for(ready, 75, children) is None:
            raise RuntimeError("恢复后仍有异常；请查看各项故障和 launcher-*.log")
    return component_status(state())


def component_status(snapshot, failed=None, reason=""):
    buses, _ = can_health()
    faults = {}
    for name in CHECKS:
        channel = CHANNELS["body" if name in ("chassis", "lift") else name]
        faults[name] = "\n".join(filter(None, [buses[channel],
            buses.get("chassis", "") if name == "chassis" else "", check_fault(snapshot, name)]))
    if failed in ("left", "can1"):
        faults["left"] = reason
    elif failed in ("right", "can3"):
        faults["right"] = reason
    elif failed in ("body", "can5"):
        faults["chassis"] = faults["lift"] = reason
    arm_faults = [f"{LABELS[side]}\uff1a{faults[side]}" for side in ("left", "right") if faults[side]]
    return {"\u673a\u68b0\u81c2": "\n".join(arm_faults), "\u5e95\u76d8": faults["chassis"], "\u5347\u964d": faults["lift"], "相机": camera_fault(), "电池框与孔洞": whole_battery_fault()}


class StartupError(RuntimeError):
    def __init__(self, stage, message):
        super().__init__(message)
        self.stage = stage


def launch():
    ensure_cameras()
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

    # Never restart or re-enable hardware during status checks.
    # All hardware starts, including cold boots, go through confirmed recovery.
    return component_status(snapshot)


def show_status(statuses, warning):
    import gi
    gi.require_version("Gtk", "3.0")
    from gi.repository import Gtk, GLib

    dialog = Gtk.Dialog(title="ARX 设备自检")
    dialog.set_default_size(620, 680)
    retry = dialog.add_button("恢复 / 重启故障服务", Gtk.ResponseType.APPLY)
    close = dialog.add_button("关闭", Gtk.ResponseType.CLOSE)
    cancel_button = dialog.add_button("停止下降 / 取消恢复", Gtk.ResponseType.REJECT)
    cancel = threading.Event()
    outer = dialog.get_content_area()
    outer.set_border_width(16)
    outer.set_spacing(10)
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
    scroll = Gtk.ScrolledWindow()
    scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
    scroll.add(box)
    outer.pack_start(scroll, True, True, 0)
    rows = {}
    state_ui = {"busy": False, "checking": False, "alive": True,
                "statuses": statuses, "warning": warning}
    for name in statuses:
        row = Gtk.Box(spacing=12)
        name_label = Gtk.Label(xalign=0)
        name_label.set_markup(f"<b>{name}</b>")
        status_label = Gtk.Label(xalign=1)
        row.pack_start(name_label, True, True, 0)
        row.pack_end(status_label, False, False, 0)
        box.pack_start(row, False, False, 0)
        detail = Gtk.Label(xalign=0)
        detail.set_line_wrap(True)
        detail.set_selectable(True)
        box.pack_start(detail, False, False, 0)
        box.pack_start(Gtk.Separator(), False, False, 0)
        rows[name] = (status_label, detail)
    note = Gtk.Label(xalign=0)
    note.set_line_wrap(True)
    note.set_selectable(True)
    outer.pack_start(note, False, False, 0)

    def render(values):
        state_ui["statuses"] = values
        for name, fault in values.items():
            status_label, detail = rows[name]
            status_label.set_markup('<span foreground="#b42318"><b>不好</b></span>' if fault else
                                    '<span foreground="#067647"><b>好</b></span>')
            detail.set_text(fault)
            detail.set_visible(bool(fault))
        retry.set_sensitive(any(values.values()) and not state_ui["busy"])
        note.set_text(state_ui["warning"] or
                      ("检测到异常，是否恢复 / 重启故障服务？正常服务继续运行。"
                       if any(values.values()) else "各项检查通过；每 2 秒更新。关闭窗口后服务继续运行。"))

    def message(text, confirm=False):
        popup = Gtk.MessageDialog(transient_for=dialog, modal=True,
            message_type=Gtk.MessageType.WARNING if confirm else Gtk.MessageType.ERROR,
            buttons=Gtk.ButtonsType.YES_NO if confirm else Gtk.ButtonsType.CLOSE, text=text)
        result = popup.run()
        popup.destroy()
        return result == Gtk.ResponseType.YES

    def progress(text):
        GLib.idle_add(note.set_text, text)

    def idle():
        state_ui["busy"] = False
        close.set_sensitive(True)
        cancel_button.hide()
        render(state_ui["statuses"])

    def worker(function, done):
        state_ui["busy"] = True
        retry.set_sensitive(False)
        close.set_sensitive(False)
        def run():
            try:
                result, error = function(), None
            except Exception as exc:
                result, error = None, str(exc)
                try:
                    RUNTIME.mkdir(parents=True, exist_ok=True)
                    with (RUNTIME / "launcher-recovery.log").open("a") as log:
                        log.write(time.strftime("%F %T ") + str(exc) + "\n")
                except OSError:
                    pass
            def finished():
                if not state_ui["alive"]:
                    return False
                if error:
                    state_ui["warning"] = "恢复未完成：" + error
                    message(state_ui["warning"])
                    idle()
                else:
                    done(result)
                return False
            GLib.idle_add(finished)
        threading.Thread(target=run, daemon=True).start()

    def recover():
        selected = recovery_plan(state())
        if not selected:
            return
        labels = [LABELS.get(name, {"web": "网页", "cameras": "相机及识别"}.get(name, name))
                  for name in selected]
        if not message("是否恢复 / 重启以下异常服务？\n" + "、".join(labels) +
                       "\n\n先确认底盘已停、升降下方和双臂周围清空，现场有人监护。"
                       "允许先通过现有驱动低速下降；未确认最低前，不会停止任何服务。", True):
            return
        cancel.clear()
        cancel_button.show()
        def prepared(automatic):
            cancel_button.hide()
            text = ("软件反馈已在零位附近停稳。请现场确认升降确实处于最低位置，且下方无夹物。"
                    if automatic else
                    "CAN5 升降链路不可用，无法下发可靠的下降指令，也没有停止驱动。\n"
                    "若仍在高位，请取消并先恢复连接；不要通过高位断电来降低。"
                    "只有平台已经处于最低且有稳定支撑时，才可现场确认后恢复。")
            if not message(text + "\n\n你已亲眼确认最低且稳定，并允许重启上述异常服务吗？", True):
                state_ui["warning"] = "已取消恢复；未停止服务。请检查升降目标和现场状态。"
                idle()
                return
            def completed(values):
                state_ui["warning"] = "" if not any(values.values()) else "仍有异常，见各项原因；未自动反复重启。"
                state_ui["statuses"] = values
                idle()
            worker(lambda: restart_faults(True, manual=not automatic,
                                          selected=selected, progress=progress), completed)
        worker(lambda: prepare_recovery(progress, cancel), prepared)

    def refresh():
        if not state_ui["alive"]:
            return False
        if state_ui["busy"] or state_ui["checking"]:
            return True
        state_ui["checking"] = True
        def run():
            try:
                values = component_status(state())
            except Exception as exc:
                values = {name: "自检失败：" + str(exc) for name in rows}
            def finished():
                state_ui["checking"] = False
                if state_ui["alive"] and not state_ui["busy"]:
                    render(values)
                return False
            GLib.idle_add(finished)
        threading.Thread(target=run, daemon=True).start()
        return True

    dialog.connect("delete-event", lambda *_: state_ui["busy"])
    dialog.show_all()
    cancel_button.hide()
    render(statuses)
    timer = GLib.timeout_add_seconds(2, refresh)
    while True:
        response = dialog.run()
        if response == Gtk.ResponseType.REJECT:
            cancel.set()
            note.set_text("正在取消下降并请求保持；不会停止驱动")
        elif response == Gtk.ResponseType.APPLY:
            if not state_ui["busy"]:
                recover()
        elif not state_ui["busy"]:
            break
    state_ui["alive"] = False
    GLib.source_remove(timer)
    dialog.destroy()
    return state_ui["statuses"], state_ui["warning"]


def main():
    if sys.argv[1:] == ["--cameras-only"]:
        fault = ensure_cameras()
        print(json.dumps({"相机": fault or "三路实时画面正常", "robot_startup": False}, ensure_ascii=False))
        return int(bool(fault))
    if sys.argv[1:] == ["--check-only"]:
        statuses = component_status(state())
        print(json.dumps(statuses, ensure_ascii=False))
        return int(any(statuses.values()))
    desktop = sys.argv[1:] == ["--desktop"]
    progress = None
    warning = ""
    try:
        if desktop:
            progress = subprocess.Popen(
                ["zenity", "--progress", "--pulsate", "--no-cancel", "--title", TITLE,
                 "--text", "正在检查真实 CAN 连接、电机反馈和 ROS 状态；异常服务需确认后恢复。"],
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
        statuses, warning = show_status(statuses, warning)
    return int(bool(warning) or any(statuses.values()))


if __name__ == "__main__":
    sys.exit(main())
