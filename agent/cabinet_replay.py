"""Offline RGB/gate integration demos and archived-frame reports. Never captures devices."""
from dataclasses import asdict
import base64
import hashlib
import html
import json
from pathlib import Path
import time
import uuid
import cv2
import numpy as np
from .held_rgb import ReferenceMatcher
from .held_battery import HeldBatteryMonitor
from .cabinet_gates import Context,Evidence,Gatekeeper,identity_hash
from .cabinet_autonomy import OfflineCabinetRun
from .storage import atomic_json

ROOT=Path(__file__).resolve().parents[1]


def synthetic_reference():
    identity=dict(reference_id='synthetic-qualified-reference',camera='d405',serial='SIM_D405',
        handeye_sha256='a'*64,tcp_sha256='b'*64,battery_sha256='c'*64,grasp_id='synthetic-grasp')
    rng=np.random.default_rng(84);empty=np.full((240,320,3),75,np.uint8);texture=rng.integers(30,245,(240,320,3),dtype=np.uint8)
    jaw=np.zeros((240,320),np.uint8);jaw[25:70,20:85]=255;jaw[25:70,235:300]=255
    battery=np.zeros((240,320),np.uint8);battery[95:205,90:230]=255
    empty[jaw>0]=texture[jaw>0];rgb=empty.copy();rgb[battery>0]=texture[battery>0]
    cv2.rectangle(rgb,(105,110),(215,180),(230,230,230),3)
    return identity,rgb,empty,battery,jaw


def simulation_evidence(context,now,*,precision=True):
    """Explicit synthetic oracle, not a physical calibration/obstacle certificate."""
    rows={}
    for i in range(1,21):
        name=f'G{i:02d}';d={'provenance':'synthetic_scenario_oracle_not_physical'}
        if i==7:d.update(si_mapping_verified=True,braking_verified=True,odometry_control_verified=True)
        if i==8:d.update(normal_clear=True,stopping_clear=True,coverage_complete=True)
        if i==12:d.update(bound_kind='validated_end_to_end_bound',clearance_mm=1.2,error_upper_mm=.25 if precision else 2.,reserve_mm=.2,
            retreat_verified=True,contact_free=True,target_static=True,current_pose_known=True)
        if i==13:d.update(actual_velocity_known=True,fresh_post_stop=True)
        if i==15:d.update(support_verified=True,insertion_depth_verified=True)
        if i==16:d.update(empty_grip_verified=True,release_verified=True)
        if i==17:d.update(battery_left_in_slot=True,stable_verified=True)
        rows[name]=Evidence(name,'verified',now,now+2.,context.run_id,context.boot_id,context.generation,context.asset_hash,
            details=d,state_seq=context.state_seq)
    return rows


def _annotate(image,result,path):
    bgr=image[:,:,::-1].copy();affine=result.get('battery_affine')
    if affine is not None:
        quad=cv2.transform(np.float32([[[90,95],[230,95],[230,205],[90,205]]]),np.asarray(affine,float))[0]
        cv2.polylines(bgr,[np.rint(quad).astype(np.int32)],True,(30,220,30),2)
    cv2.putText(bgr,'OFFLINE / '+str(result.get('presence','unknown')),(8,18),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,190,255),1)
    cv2.imwrite(str(path),bgr)


def _write_report(output,report,images=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);atomic_json(output/'report.json',report)
    cards=[]
    for name,row in report.get('scenarios',{}).items():
        cards.append('<article><h2>'+html.escape(name)+'</h2><p>'+html.escape(row['state']['status'])+'</p><p>'+html.escape(row['state'].get('reason',''))+'</p><pre>'+html.escape(json.dumps(row.get('last',{}).get('decision'),ensure_ascii=False,indent=2))+'</pre></article>')
    image_tags=''.join('<figure><img src="'+html.escape(p,quote=True)+'"><figcaption>'+html.escape(label)+'</figcaption></figure>' for p,label in (images or []))
    document='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>电池夹持与找仓门控 · 离线报告</title><style>body{font:16px system-ui;background:#f4f6f8;color:#182431;max-width:1180px;margin:auto;padding:24px}header,article,figure{background:white;padding:18px;border-radius:12px;margin:16px 0}header{border-left:6px solid #d28a00}img{max-width:100%;height:auto}pre{overflow:auto;font-size:12px;max-height:380px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:16px}a{color:#125db0}</style><header><h1>D405夹持与自主找仓门控</h1><p>离线验证 · 不连接机器人 · 所有执行端均为内存模拟器</p><p>仿真门控可继续不代表实机获准；历史图像不代表当前位置。页面没有控制按钮或运动接口。</p></header>'''
    document+='<p>'+html.escape(report.get('description',''))+'</p>'+image_tags+'<div class="grid">'+''.join(cards)+'</div><details><summary>完整证据与阻断原因</summary><pre>'+html.escape(json.dumps(report,ensure_ascii=False,indent=2))+'</pre></details><p><a href="report.json">下载JSON报告</a></p></html>'
    (output/'index.html').write_text(document)


def run_demo(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    identity,rgb,empty,battery,jaw=synthetic_reference();matcher=ReferenceMatcher(rgb,battery,jaw,identity=identity,reference_kind='synthetic',empty_rgb=empty)
    report=dict(mode='offline',executable=False,physical_accuracy_verified=False,historical=False,
        description='合成场景测试：D405使用真实RGB特征匹配算法；D435仓位、驱动状态、障碍与承载证据使用显式仿真真值，不代表实物验收。',scenarios={})
    for scenario in ('success','slip','occlusion','precision','manual'):
        now=10.;seq=0;ctx=Context('sim-'+scenario,'sim-boot',1,'d'*64,identity_hash(identity))
        run=OfflineCabinetRun(output/'runs'/(scenario+'-'+uuid.uuid4().hex[:8]),ctx,identity,now=now,
            initial_state=dict(scope='simulation',lift_rad=5.,base_xy_m=[0.,0.],base_yaw_rad=0.))
        def prime():
            nonlocal now,seq
            for _ in range(3):
                now+=.15;seq+=1;released=run.snapshot()['release']=='released_verified'
                o=matcher.analyze(empty if released else rgb,frame_id=f'd405-{seq}',captured_at=now)
                c=dict(scope='simulation',reference_id=identity['reference_id'],grasp_id=identity['grasp_id'],identity_sha256=identity_hash(identity),reference_sha256=matcher.digest,
                    issued_at=now-.5,expires_at=now+10.,load_validated=True,unobserved_axes_bounded=True)
                run.observe_held(o,dict(captured_at=now,position_raw=-3.14 if released else -1.,fresh=True),now=now,carry_certificate=c)
                run.observe_target(dict(camera='d435',frame='local_gravity',scope='simulation',target_id='demo:left',model_sha256='e'*64,
                    frame_id=f'd435-{seq}',captured_at=now,center_local_m=[.55,-.2,.65],normal_local=[-1.,0.,0.],ambiguous=False,
                    provenance='synthetic_scene_truth_not_real_camera_measurement'),now=now)
        def check(c):return dict(ik_clear=True,normal_clear=True,stopping_clear=True,coverage_complete=True,retreat_clear=True,
            min_clearance_mm=40.+c['lift_rad'],joint_margin_deg=20.,provenance='synthetic_scene_oracle')
        last={}
        try:
            for _ in range(10):
                prime();rows=simulation_evidence(run.context,now,precision=scenario!='precision')
                last=run.step(rows,now=now,check_candidate=check)
                if last['next_action']=='complete':
                    for i in range(1,4):
                        now+=.15;run.observe_rest(dict(scope='simulation',generation=run.executor.generation,sample_id=f'rest-{i}',
                            axes={a:dict(seq=i,captured_at=now,speed=0.) for a in run.executor.axes}),now=now)
                    break
                segment=last.get('segment')
                if not segment:break
                if scenario=='manual':run.pause(now=now,reason='manual');break
                if scenario in ('slip','occlusion'):
                    now+=.1;seq+=1
                    if scenario=='slip':
                        shifted=cv2.warpAffine(rgb,np.float32([[1,0,12],[0,1,0]]),(320,240));image=empty.copy();mask=cv2.warpAffine(battery,np.float32([[1,0,12],[0,1,0]]),(320,240))>0;image[mask]=shifted[mask]
                    else:image=rgb.copy();image[battery>0]=140
                    o=matcher.analyze(image,frame_id=f'd405-{seq}',captured_at=now)
                    run.observe_held(o,dict(captured_at=now,position_raw=-1.,fresh=True),now=now)
                    last=run.step(simulation_evidence(run.context,now),now=now,check_candidate=check);break
                now+=.05;actual=run.snapshot()['actual_state'].copy();pending=run.snapshot()['inflight']
                actual.update(pending['goal'])
                run.complete_segment(segment['segment_id'],now=now,completion=dict(scope='simulation',segment_id=segment['segment_id'],
                    actual_state_seq=run.context.state_seq+1,actual_pose_verified=True,actual_state=actual,rest_verified=True,completed_at=now))
            report['scenarios'][scenario]=dict(state=run.snapshot(),last=last,commands=copy_commands(run.executor.commands),
                stopping=run.executor.stopping,held=run.held)
        finally:
            run.close()
            if scenario in report['scenarios']:report['scenarios'][scenario]['state']=json.loads((run.directory/'state.json').read_text())
    result=matcher.analyze(rgb,frame_id='illustration',captured_at=10.);_annotate(rgb,result,output/'synthetic-d405.jpg')
    _write_report(output,report,[('synthetic-d405.jpg','合成特征图，仅用于验证匹配与门控，不是实拍电池')]);return report


def copy_commands(rows):return json.loads(json.dumps(rows,allow_nan=False))


def replay_recorded(output,*,d435_frame=None,d405_image=None,d405_meta=None,d405_reference=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);images=[];now=time.time()
    report=dict(mode='offline',executable=False,historical=True,physical_accuracy_verified=False,
        description='历史文件回放：保持原曝光时间；无新相机采集，未验证当前夹持或位置。缺失参考、反馈或精度证据时保持阻断。',sources={})
    if d435_frame is not None:
        from scripts.replay_cabinet import replay
        report['cabinet']=replay(d435_frame,'demo',output/'d435');images.append(('d435/preview.jpg','D435历史仓口识别与模型路径；不是当前执行目标'))
        report['sources']['d435_sha256']=hashlib.sha256(Path(d435_frame).read_bytes()).hexdigest()
    held=dict(mode='offline',executable=False,presence='unknown',engagement='unknown',pose='unobservable',stability='unknown',
        holding_eligibility='unknown',metric_pose=None,reason='Qualified D405 reference missing')
    if d405_image is not None:
        image=cv2.imread(str(d405_image))
        if image is None:raise ValueError('Cannot decode archived D405 image')
        rgb=image[:,:,::-1];report['sources']['d405_sha256']=hashlib.sha256(Path(d405_image).read_bytes()).hexdigest()
        meta=json.loads(Path(d405_meta).read_text()) if d405_meta else {}
        report['d405_capture']=meta;stamp=meta.get('image_time');report['d405_actual_age_s']=now-stamp if type(stamp) in (int,float) and np.isfinite(stamp) else None
        if d405_reference is not None:
            with np.load(d405_reference,allow_pickle=False) as ref:
                identity=json.loads(str(ref['identity']));kind=str(ref['reference_kind'])
                matcher=ReferenceMatcher(ref['rgb'],ref['battery_mask'],ref['jaw_mask'],identity=identity,reference_kind=kind,
                    empty_rgb=ref['empty_rgb'] if 'empty_rgb' in ref else None)
            if meta.get('camera_id')!='d405' or meta.get('calibration_sha256')!=identity['handeye_sha256']:
                held['reason']='Archived camera/calibration identity missing or mismatched'
            elif type(stamp) not in (float,int) or not np.isfinite(stamp):held['reason']='Archived exposure timestamp missing'
            else:
                obs=matcher.analyze(rgb,frame_id=str(meta.get('id','')),captured_at=stamp)
                held=HeldBatteryMonitor(identity).update(obs,{},now=stamp)
        _annotate(rgb,held,output/'d405.jpg');images.append(('d405.jpg','D405历史图像；没有合格参考和新反馈时，不宣称夹持正常'))
    report['held']=held
    identity,_,_,_,_=synthetic_reference();ctx=Context('historical-replay','offline-replay',0,'d'*64,identity_hash(identity))
    report['decision']=Gatekeeper().evaluate('base_align',{},ctx,now=now,held=held).as_dict()
    _write_report(output,report,images);return report
