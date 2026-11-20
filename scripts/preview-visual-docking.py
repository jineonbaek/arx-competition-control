#!/usr/bin/env python3
"""Current-camera planning adapter. GET-only transport, no actuator client/SDK."""
import argparse
import copy
import html
import json
from pathlib import Path
import sys
import re
import time
import urllib.request
import numpy as np
import cv2

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'web/src'),str(ROOT)]
from cabinet_execution import CabinetExecutionTask
from cabinet_preview import CabinetPreview
from handeye_column import ColumnModel
from agent.cabinet_controller_bridge import digest
from agent.docking_fit import evaluate_fit
from agent.docking_grip import GripTracker
from agent.storage import atomic_json


class ReadOnlyController:
    def __init__(self,runtime,url):
        self.url=url;self.config=json.loads((runtime/'web/config/control.json').read_text())
        self.models={side:ColumnModel(side=side) for side in ('left','right')};self.marker=object()
    def snapshot(self):
        return json.load(urllib.request.urlopen(self.url+'/api/state',timeout=4))
    def motion_valid(self,token):return token is self.marker
    def reserve_motion(self,*args,**kwargs):raise RuntimeError('Read-only adapter: hardware ownership forbidden')


def relative_grip_image(controller,serial):
    """Image-plane monitoring needs no pinhole/PnP distortion conversion.

    Keep identity and exposure pairing; do not turn inverse-Brown pixels into a
    metric pose. The fixed lens distortion is shared by the bound reference.
    """
    state=controller.snapshot();meta=json.loads((ROOT/'web/public/camera-live/d405.json').read_text())
    stamp=meta['captured_at'];arm=state['arms']['right'];name=meta['raw_image_file']
    if (meta['state']!='online' or not arm['fresh'] or not 0<=time.time()-stamp<=.8
            or abs(stamp-arm['timestamp'])>.2 or meta['color_intrinsics']['serial']!=serial
            or not re.fullmatch(r'd405-v-[a-f0-9]+-\d+\.jpg',name)):
        raise ValueError('D405 image identity/exposure and gripper feedback not paired')
    rgb=cv2.imdecode(np.frombuffer((ROOT/'web/public/camera-live'/name).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    k=meta['color_intrinsics']
    if rgb is None or rgb.shape!=(k['height'],k['width'],3):raise ValueError('D405 RGB dimensions invalid')
    return dict(rgb=rgb[:,:,::-1].copy(),image_time=stamp,right_gripper=arm['gripper'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime',type=Path,default=Path('/home/arx/arx-competition-control'))
    parser.add_argument('--url',default='http://127.0.0.1:9983')
    parser.add_argument('--model',choices=('demo','competition'),default='demo')
    parser.add_argument('--output',type=Path,default=ROOT/'reports/visual-docking-live')
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    obs=ReadOnlyController(args.runtime,args.url);before=obs.snapshot()
    preview=CabinetPreview(obs,None,root=ROOT)
    tracker=GripTracker();grip_rows=[];frame405=None
    try:
        from cabinet_capture import CALIBRATIONS
        serial=json.loads((ROOT/CALIBRATIONS['d405']).read_text())['camera']['serial']
        deadline=time.monotonic()+2.5
        while time.monotonic()<deadline:
            try:frame405=relative_grip_image(obs,serial);break
            except ValueError:time.sleep(.03)
        if frame405 is None:raise ValueError('No paired D405 reference within capture window')
        tracker.bind(frame405['rgb'],frame405['image_time'],frame405['right_gripper'],now=time.time())
        deadline=time.monotonic()+2.5;last_stamp=frame405['image_time']
        while time.monotonic()<deadline:
            time.sleep(.03)
            try:latest405=relative_grip_image(obs,serial)
            except ValueError:continue  # Wait for a newly paired exposure; do not relax timestamps.
            if latest405['image_time']<=last_stamp:continue
            frame405=latest405;last_stamp=frame405['image_time']
            grip_rows.append(tracker.update(frame405['rgb'],frame405['image_time'],frame405['right_gripper'],now=time.time()))
            if grip_rows[-1]['stable'] or grip_rows[-1]['status']=='slip_suspected':break
        if not grip_rows:raise ValueError('No independent paired D405 frames in bounded capture window')
        grip=grip_rows[-1]
    except ValueError as error:grip=dict(stable=False,status='unknown',reason=str(error))
    frame=preview.captures['d435'].capture()
    detection=preview.analyze_frame('d435',args.model,frame,include_arm_preview=False)
    initial=obs.snapshot();result=dict(mode='live_read_only_planning',physical_execution=False,
        no_actuator_transport=True,model=args.model,grip=grip,grip_history=grip_rows,
        camera_image_time=frame['image_time'],source_hashes=detection.get('source_hashes',{}),
        current_joints_rad=initial['arms']['right']['joints'],
        phases=['observe','align_and_level','reobserve_fit','bounded_correct','insert_fixed_z','withdraw','hold_clamped'],
        strategies=dict(arm='existing checked timed trajectories',
                        lift='backend contract; no physical execution adapter installed in this pass',
                        base='backend contract requires measured odometry and checked sweep; no blind timed drive'),
        no_release_operation=True)
    task=None;bundle=None
    try:
        class FrozenPreview:
            root=ROOT
            def action(self,*a,**kw):return copy.deepcopy(detection)
            def _source_hashes(self,*a,**kw):return preview._source_hashes(*a,**kw)
        task=CabinetExecutionTask(obs,FrozenPreview(),directory=out/'unexecuted-task',auto_worker=False)
        task.token=obs.marker;task.initial=initial;task.source_hash=digest(obs.config)
        task.state.update(keep_grip=True,align_only=True,model=args.model,release_requested=False,motion_started=False)
        bundle=task._plan(args.model)
        slot=preview.profiles[args.model]['openings'][0]
        if args.model=='competition':slot=next(v for v in preview.profiles[args.model]['openings'] if v['slot_id']=='upper_2')
        tcp=bundle['tcp_matrix'];current=obs.models['right'].wrist_transform(initial['arms']['right']['joints'],initial['body']['height'])@tcp
        endq=bundle['segments'][-1]['plan']['joint_waypoints_rad'][-1]
        target=obs.models['right'].wrist_transform(endq,initial['body']['height'])@tcp
        pivot=json.loads((ROOT/'config/right-grip-tcp.json').read_text())['provenance']['solution']['max_error_mm']
        def fit(transform,uncertainty):
            return evaluate_fit(transform,bundle['observation']['window'],slot,preview.battery,
                preview.grip['points_object_mm']['tip'],uncertainty_mm=uncertainty)
        result.update(status='outside_path_checked',loaded_sweep=bundle['loaded_sweep'],
            current_fit=fit(current,0.),aligned_nominal_fit=fit(target,0.),
            aligned_fit_with_tcp_error_lower_bound=fit(target,float(pivot)),
            uncertainty_note='TCP maximum pivot residual only; D435 CAD extrinsics and object-to-tool pose add unquantified error',
            joint_adjustment_deg=np.degrees(np.asarray(endq)-initial['arms']['right']['joints']).tolist(),
            segments=bundle['segments'],entry_plan=bundle['observation']['plan'],
            expected_duration_s=sum(p['plan']['time_from_start_s'][-1] for p in bundle['segments']),
            nominal_vertical_total_clearance_mm=float(slot['size_mm'][1]-75.))
    except (ValueError,KeyError,RuntimeError) as error:result.update(status='planning_rejected',reason=str(error))
    after=obs.snapshot()
    if before['command_epoch']!=after['command_epoch'] or any(after[k] for k in ('chassis_active','motion_reserved','competition_reserved')):
        result.update(status='planning_invalidated',reason='Concurrent control or ownership changed during preview')
    if (np.max(np.abs(np.asarray(after['arms']['right']['joints'])-initial['arms']['right']['joints']))>.01
            or abs(after['body']['height']-initial['body']['height'])>.05):
        result.update(status='planning_invalidated',reason='Start configuration moved during planning')
    image=frame['rgb'][:,:,::-1].copy()
    if detection.get('detected_quad_px'):
        cv2.polylines(image,[np.rint(detection['detected_quad_px']).astype(np.int32)],True,(0,220,255),2)
    if bundle:
        T=np.linalg.inv(frame['base_from_color']);k=json.loads(str(frame['intrinsics']))
        K=np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1.]])
        points=[]
        for segment in bundle['segments']:
            for q in segment['plan']['joint_waypoints_rad']:
                t=obs.models['right'].wrist_transform(q,initial['body']['height'])@bundle['tcp_matrix']
                p=T@np.r_[t[:3,3],1.]
                if p[2]>.07:points.append(p[:3])
        if points:
            uv=cv2.projectPoints(np.array(points),np.zeros(3),np.zeros(3),K,np.array(k['coeffs']))[0].reshape(-1,2)
            uv=np.rint(uv).astype(np.int32)
            cv2.polylines(image,[uv],False,(255,170,30),2);cv2.circle(image,tuple(uv[-1]),6,(0,255,0),2)
    cv2.putText(image,'NOMINAL PLAN - NO MOTION',(12,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2)
    cv2.imwrite(str(out/'d435-plan.jpg'),image)
    if frame405 is not None:cv2.imwrite(str(out/'d405-grip.jpg'),frame405['rgb'][:,:,::-1])
    atomic_json(out/'result.json',result)
    brief={
        '规划结果':result['status'],
        '夹持相对画面': '稳定可见' if grip.get('stable') else grip.get('status','未知'),
        '实机动作':'未执行，夹爪不松开',
        '仓外路径预计秒数':round(result.get('expected_duration_s',0),1),
        '目标名义最小间隙_mm':round(result.get('aligned_nominal_fit',{}).get('nominal_clearance_mm',0),3),
        '计入已知TCP残差后的间隙_mm':round(result.get('aligned_fit_with_tcp_error_lower_bound',{}).get('available_clearance_mm',0),3),
        '执行适配进度':'真实控制接入待完成；底盘、升降当前为接口与模拟验证',
        '实机验证状态':'名义路径可算出，当前精度尚未验证',
    }
    if result.get('reason'):brief['原因']=result['reason']
    content='''<!doctype html><meta charset="utf-8"><title>视觉入仓框架</title>
<style>body{font:16px system-ui;background:#10202b;color:#ecf2f6;margin:32px;max-width:1200px}h1{font-size:28px}.flow{display:flex;gap:8px;flex-wrap:wrap}.flow span{background:#234458;padding:12px;border-radius:8px}img{max-width:48%;border-radius:10px}pre{white-space:pre-wrap;background:#172e3e;padding:20px;border-radius:10px}.note{color:#ffce83}</style>
<h1>一次运行的视觉入仓框架</h1><p class="note">当前是实拍画面与模型路径预览，没有发送运动或松爪指令。</p>
<div class="flow"><span>D435 识别仓口</span><span>D405 监测夹持</span><span>整段调平与对位</span><span>复看间隙／自动纠偏</span><span>固定 Z 浅入</span><span>撤回并保持夹紧</span></div>
<p>蓝线：仓外 TCP 路径；绿点：仓外对齐位置；黄框：识别到的仓口。</p>
<img src="d435-plan.jpg"><img src="d405-grip.jpg">
<p>当前 CAD 仓口高 79mm，电池厚 75mm，总竖向余量 4mm。名义通过与实机精度验证分开显示。底盘／升降替换执行器必须提供测量反馈和整机扫掠检查，本预览不会下发速度命令。</p>
<p>复用入口：DockingRun(backend).run()。执行器接收整段计划并返回实测回执；上层不逐点调用控制接口。发生失联、夹持变化或结果不确定时保持并进入恢复状态，全程没有松爪阶段。</p>
<pre>DETAILS</pre><a href="result.json" style="color:#9edcff">完整路径与结果 JSON</a>'''
    (out/'index.html').write_text(content.replace('DETAILS',html.escape(json.dumps(brief,ensure_ascii=False,indent=2))))
    print(json.dumps(dict(status=result['status'],report=str(out),physical_execution=False,
        nominal_fit=result.get('aligned_nominal_fit',{}).get('nominal_fit'),
        qualified_fit=result.get('aligned_fit_with_tcp_error_lower_bound',{}).get('qualified_fit'),
        grip=grip,expected_duration_s=result.get('expected_duration_s')),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
