"""User-service integration for the ARX desktop launcher; no hardware startup."""
import json
import os
import subprocess
from urllib.request import urlopen

VIEWER='arx-gamepad-viewer.service'
CONTROL='arx-gamepad-teleop.service'
URL='http://127.0.0.1:9984/api/state'
START_ERRORS={}


def systemctl(*args):
    env=dict(os.environ)
    env.setdefault('XDG_RUNTIME_DIR',f'/run/user/{os.getuid()}')
    return subprocess.run(['systemctl','--user',*args],capture_output=True,text=True,timeout=8,env=env)


def request_gamepad_start(runtime):
    """Submit optional jobs to systemd without waiting for either service."""
    env=dict(os.environ)
    env.setdefault('XDG_RUNTIME_DIR',f'/run/user/{os.getuid()}')
    try:
        runtime.mkdir(parents=True,exist_ok=True)
        with (runtime/'launcher-gamepad.log').open('ab') as log:
            subprocess.Popen(['systemctl','--user','--no-block','start',VIEWER,CONTROL],
                stdin=subprocess.DEVNULL,stdout=log,stderr=log,env=env,start_new_session=True)
        return ''
    except (OSError,subprocess.SubprocessError) as exc:
        fault=f'手柄任务启动请求失败：{exc}'
        START_ERRORS[VIEWER]=fault
        return fault


def service_active(unit):
    try:
        result=systemctl('is-active','--quiet',unit)
        if result.returncode==0:
            START_ERRORS.pop(unit,None)
            return True
        if result.stderr.strip():START_ERRORS[unit]=result.stderr.strip()
        return False
    except (OSError,subprocess.SubprocessError) as exc:
        START_ERRORS[unit]=str(exc)
        return False


def run_actions(actions):
    for action,unit in actions:
        try:
            result=systemctl(action,unit)
            if result.returncode:
                fault=f'{unit} 启动失败：'+(result.stderr.strip() or result.stdout.strip() or '请查看用户服务日志')
                START_ERRORS[unit]=fault
                return fault
            START_ERRORS.pop(unit,None)
        except (OSError,subprocess.SubprocessError) as exc:
            fault=f'{unit} 启动失败：{exc}'
            START_ERRORS[unit]=fault
            return fault
    return ''


def ensure_gamepad_services():
    return run_actions([('start',VIEWER),('start',CONTROL)])


def read_gamepad_state():
    with urlopen(URL,timeout=.6) as response:
        data=json.load(response)
    if not isinstance(data,dict):
        raise ValueError('手柄状态格式错误')
    return data


def repair_plan():
    viewer=service_active(VIEWER);control=service_active(CONTROL)
    actions=[];data=None
    if not viewer:
        actions.append(('start',VIEWER))
    else:
        try:
            data=read_gamepad_state()
        except (OSError,ValueError,TypeError,AttributeError):
            actions.append(('restart',VIEWER))
    if not control:
        actions.append(('start',CONTROL))
    elif data and data.get('connected') and data.get('initialized') and data.get('mode') in ('NS','XINPUT'):
        report=data.get('control')
        if not isinstance(report,dict) or report.get('execute') is not True:
            actions.append(('restart',CONTROL))
    return actions


def gamepad_restart_needed():
    return bool(repair_plan())


def repair_gamepad_services():
    return run_actions(repair_plan())


def gamepad_status():
    faults={'手柄可视化':'','手柄底盘控制':''}
    viewer=service_active(VIEWER);control=service_active(CONTROL)
    if not viewer:
        faults['手柄可视化']=START_ERRORS.get(VIEWER) or '手柄可视化服务未运行（端口 9984）'
    if not control:
        faults['手柄底盘控制']=START_ERRORS.get(CONTROL) or '手柄底盘传输服务未运行'
    if not viewer:
        faults['手柄底盘控制']=faults['手柄底盘控制'] or '等待手柄可视化服务启动'
        return faults
    try:
        data=read_gamepad_state()
        if not data.get('connected') or not data.get('initialized') or data.get('mode') not in ('NS','XINPUT'):
            faults['手柄可视化']=data.get('error') or '手柄未就绪或模式不支持'
        if control:
            if faults['手柄可视化']:
                faults['手柄底盘控制']='手柄输入不可用，禁止底盘输出'
            elif not isinstance(data.get('control'),dict) or data['control'].get('execute') is not True:
                faults['手柄底盘控制']='控制进程尚未就绪或处于预览模式'
    except (OSError,ValueError,AttributeError,TypeError) as exc:
        faults['手柄可视化']=f'手柄状态接口不可用：{exc}'
        faults['手柄底盘控制']=faults['手柄底盘控制'] or '无法确认手柄传输状态'
    return faults
