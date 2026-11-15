"""Opt-in placement over the existing controller. No threads, devices or imports start motion."""
import copy
import hashlib
import itertools
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .placement_geometry import analyze, rigid, policy as geometry_policy
from .storage import atomic_json


def corners(bounds):
    b=np.asarray(bounds,float)
    if b.shape!=(2,3) or not np.isfinite(b).all() or np.any(b[1]<=b[0]):raise ValueError('CAD边界无效')
    return np.array(list(itertools.product(*zip(b[0],b[1]))))


def release_geometry(base_from_battery,region,bounds):
    t=rigid(base_from_battery);b=np.asarray(bounds,float);corners(b)
    n=np.asarray(region['normal_base']);left=np.asarray(region['left_axis_base']);forward=np.asarray(region['forward_axis_base'])
    bottom=np.array([[x,y,b[0,2]] for x in b[:,0] for y in b[:,1]])@t[:3,:3].T+t[:3,3]
    actual=np.array([(b[0,0]+b[1,0])/2,(b[0,1]+b[1,1])/2,b[0,2]])@t[:3,:3].T+t[:3,3]
    delta=actual-np.asarray(region['target_base_m']);xy=np.array([delta@forward,delta@left])*1000
    heights=(bottom-np.asarray(region['target_base_m']))@n*1000
    desired=np.column_stack((forward,left,n));angle=float(np.degrees(Rotation.from_matrix(desired.T@t[:3,:3]).magnitude()))
    z=float(np.max(abs(heights)))
    return dict(ready=bool(np.all(abs(xy)<=region['xy_tolerance_mm']) and z<=region['z_tolerance_mm']
                and angle<=region['yaw_tolerance_deg']),xy_error_mm=xy.tolist(),height_error_mm=z,orientation_error_deg=angle)


def _read_bound(root,relative,digest=None):
    p=(Path(root)/relative).resolve()
    if not p.is_relative_to(Path(root).resolve()):raise ValueError('抓取证据路径越界')
    raw=p.read_bytes()
    if digest is not None and hashlib.sha256(raw).hexdigest()!=digest:raise ValueError('抓取证据已被改写')
    return json.loads(raw)


def load_held(root,model,mesh,profile,now=None):
    """Use only the most recent task; never fall back past a failed/newer grasp."""
    now=time.time() if now is None else now;states=sorted(Path(root).glob('run-*/state.json'),key=lambda p:p.stat().st_mtime,reverse=True)
    if not states:raise ValueError('没有本机已完成的抓取证据')
    path=states[0];state_blob=path.read_bytes();s=json.loads(state_blob);flow=s.get('battery_flow',{})
    if not -1<=now-path.stat().st_mtime<=1200 or flow.get('status')!='completed' or flow.get('grasp_confirmed') is not True or s.get('pending'):
        raise ValueError('最新抓取未完成、过期或回执未知，不能推断仍在持物')
    preview=_read_bound(path.parent,s['grasp_preview'],s['grasp_preview_sha256'])
    plan=_read_bound(path.parent,s['battery_pick_plan'],s['battery_pick_plan_sha256'])
    site=Path(root).resolve().parent.parent
    checks={'control_config_sha256':'web/config/control.json',
            'urdf_sha256':'web/public/models/lift2/diy-20261105/urdf/lift2_with_sensors.urdf',
            'profile_sha256':'web/public/cad/battery-grip.json'}
    for key,relative in checks.items():
        if hashlib.sha256((site/relative).read_bytes()).hexdigest()!=plan.get(key):raise ValueError('抓取后的配置、模型或标定版本已变化')
    tcp_hash=hashlib.sha256((site/'config/right-grip-tcp.json').read_bytes()).hexdigest()
    if not str(plan.get('calibration_id','')).endswith(':'+tcp_hash):raise ValueError('抓取后的TCP标定已变化')
    mesh_blob=Path(mesh).read_bytes();sha=hashlib.sha256(mesh_blob).hexdigest()
    if sha!=profile['mesh_sha256']['battery-print.stl'] or preview['pose'].get('mesh_sha256')!=sha:
        raise ValueError('电池CAD与抓取证据不一致')
    if preview.get('status')!='preview_complete' or preview['quality'].get('status')!='consistent':raise ValueError('抓取位姿证据无效')
    pose=rigid(preview['pose']['T_base_from_object']);end=model.wrist_transform(plan['joint_waypoints_rad'][-1],plan['lift_rad'])
    held=rigid(np.linalg.inv(end)@pose)
    dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('extra','<u2')])
    count=int.from_bytes(mesh_blob[80:84],'little')
    if len(mesh_blob)!=84+count*50 or count<4:raise ValueError('需要已校验的binary STL')
    vertices=np.frombuffer(mesh_blob,dtype=dtype,offset=84)['vertices'].reshape(-1,3).astype(float)*.001
    bounds=np.array([vertices.min(0),vertices.max(0)]);corners(bounds)
    if path.read_bytes()!=state_blob:raise ValueError('读取期间抓取证据发生变化')
    return dict(run_id=path.parent.name,T_wrist_from_battery=held.tolist(),bounds_m=bounds.tolist(),mesh_sha256=sha,
        source='FoundationPose + planned insertion endpoint; model-derived, not a new measurement',
        grasp_state_sha256=hashlib.sha256(state_blob).hexdigest())


class PlacementBackend:
    def __init__(self,store,controller,token,settings,pick_policy,preview,*,environment_clear):
        self.store,self.controller,self.token=store,controller,token
        self.settings=copy.deepcopy(settings);self.pick_policy=copy.deepcopy(pick_policy);self.preview=preview
        self.environment_clear=environment_clear;self.held=None;self.region=None;self.head=None;self.released=False
        self.model=controller.models['right'];self.lift=None;self.left_joints=None
        self.hold_range=pick_policy['pick']['hold_gripper_range'];self.geometry=geometry_policy(settings.get('geometry'))
        self.clock=getattr(controller,"clock",time.monotonic);self.wall_clock=time.time;self.sleep=time.sleep;self.rest_evidence=None

    def _check(self,holding=True):
        if self.environment_clear is not True:raise ValueError('放置通路未确认')
        if self.store.cancelled or not self.controller.motion_valid(self.token):raise ValueError('任务取消或控制权失效')
        s=self.controller.snapshot();a=s['arms']['right'];body=s['body']
        if (not a.get('fresh') or not body.get('fresh') or s.get('chassis_active') is not False
            or s.get('motion_interrupted') is not False):raise ValueError('反馈失效、底盘运动或任务中断')
        if self.lift is not None and abs(body['height']-self.lift)>.02:raise ValueError('放置期间升降发生变化')
        if self.left_joints is not None:
            other=s['arms']['left']
            if not other.get('fresh') or np.max(abs(np.asarray(other['joints'])-self.left_joints))>np.radians(.5):raise ValueError('左臂反馈缺失或姿态变化')
        if holding and not self.hold_range[0]<=a['gripper']<=self.hold_range[1]:raise ValueError('夹爪持物反馈不在已配置范围')
        return s

    def _observe(self,camera,*,verify=False):
        self._check(holding=not self.released)
        r=self.preview.capture(camera,self.store,allow_occupied=verify,settings=self.geometry)
        self._check(holding=not self.released)
        return r

    def _match(self,new):
        if (np.linalg.norm(np.asarray(new['center_base_m'])-self.head['center_base_m'])>.04
            or abs(new['radius_m']-self.head['radius_m'])>.025
            or np.dot(new['normal_base'],self.head['normal_base'])<np.cos(np.radians(5))):raise ValueError('两相机红圈或支撑平面不一致')

    def _object_target(self,height):
        r=self.region;rotation=np.column_stack((r['forward_axis_base'],r['left_axis_base'],r['normal_base']))
        bounds=np.array(self.held['bounds_m']);bottom=np.r_[bounds[:,:2].mean(0),bounds[0,2]]
        t=np.eye(4);t[:3,:3]=rotation;t[:3,3]=np.asarray(r['target_base_m'])+np.asarray(r['normal_base'])*height-rotation@bottom
        return t

    def _base_battery(self,s):return rigid(self.model.wrist_transform(s['arms']['right']['joints'],s['body']['height'])@self.held['T_wrist_from_battery'])

    def _sweep(self,plan,holding):
        # Adds the carried battery's full CAD bounding box against the measured support plane.
        if not holding:return
        verts=corners(self.held['bounds_m']);floor=np.asarray(self.region['target_base_m']);normal=np.asarray(self.region['normal_base'])
        qs=np.asarray(plan['joint_waypoints_rad']);minimum=10.
        for a,b in zip(qs[:-1],qs[1:]):
            for t in np.linspace(0,1,max(2,int(np.ceil(np.max(abs(b-a))/np.radians(.2)))+1)):
                transform=self.model.wrist_transform(a+t*(b-a),self.lift)@self.held['T_wrist_from_battery']
                distance=((verts@transform[:3,:3].T+transform[:3,3]-floor)@normal).min()
                minimum=min(minimum,float(distance))
        if minimum<-.001:raise ValueError('持物路径穿过实测支撑面')
        plan['payload_plane_lower_m']=minimum

    def _fresh(self,region,age=None):
        limit=self.geometry['max_frame_age_s'] if age is None else age
        elapsed=self.wall_clock()-float(region.get('image_time',0))
        if not -.1<=elapsed<=limit:raise ValueError('支撑面或占位观测已过期，需要重新采集')

    def _refresh_support(self):
        region=self._observe('d435',verify=True);self._fresh(region);self._match(region)
        if (np.linalg.norm(np.asarray(region['center_base_m'])-self.region['center_base_m'])>.005
            or abs(float(np.dot(np.asarray(region['center_base_m'])-self.region['center_base_m'],self.region['normal_base'])))>.002):
            raise ValueError('松爪前支撑面发生变化')
        self.preview.check_payload_support(region,self._base_battery(self._check(holding=not self.released)),self.held['bounds_m'])
        self.region=region

    def _feedback(self,snapshot,target):
        from .motion_feedback import tool_feedback
        p=self.pick_policy['pick']
        return tool_feedback(snapshot,self.model,self.held['T_wrist_from_battery'],target,
            now=self.clock(),max_age_s=p['arm_policy']['max_age_s'],max_pair_skew_s=p['max_pair_skew_s'],
            expected_lift_rad=self.lift,lift_tolerance_rad=p['lift_tolerance_rad'])

    def _settle(self,target,region,holding,deadline):
        from .motion_feedback import ArrivalMonitor
        p=copy.deepcopy(self.pick_policy['pick']['arm_policy'])
        p['tolerance']=[region['xy_tolerance_mm'],region['xy_tolerance_mm'],region['z_tolerance_mm']]+[region['yaw_tolerance_deg']]*3
        p['settle_s']=max(.5,p['settle_s']);p['timeout_s']=min(p['timeout_s'],deadline-self.clock())
        monitor=ArrivalMonitor(np.zeros(6),started_at=self.clock(),**p)
        while self.clock()<deadline:
            snapshot=self._check(holding);feedback=self._feedback(snapshot,target)
            value=np.r_[feedback['position_error_mm'],feedback['orientation_error_rotvec_deg']]
            receipt=monitor.update(value,feedback['sample_time'],self.clock(),ready=True,cancelled=self.store.cancelled)
            if receipt['status']=='arrived':
                check=release_geometry(feedback['base_from_tool'],region,self.held['bounds_m'])
                if not check['ready']:raise ValueError('电池底面实际高度或姿态未到位')
                self.rest_evidence=dict(target=target.copy(),value=value.copy(),sample_time=feedback['sample_time'])
                return check
            if receipt['status']!='waiting':raise ValueError('放置到位检查失败：'+receipt['status'])
            self.sleep(.05)
        raise ValueError('放置实际姿态或支撑高度未稳定到位')

    def _rest_still_valid(self,snapshot):
        try:
            e=self.rest_evidence;p=self.pick_policy['pick']['arm_policy'];feedback=self._feedback(snapshot,e['target'])
            value=np.r_[feedback['position_error_mm'],feedback['orientation_error_rotvec_deg']];dt=feedback['sample_time']-e['sample_time']
            if not 0<=dt<=p['max_age_s']:return False
            if dt==0:return bool(np.allclose(value,e['value'],atol=1e-9))
            return bool(np.all(abs(value-e['value'])/dt<=np.asarray(p['max_speed'])))
        except Exception:return False

    def _move(self,height,*,holding=True):
        from .grasp_path import plan_correction,TimedJointPath
        self._fresh(self.region)
        s=self._check(holding);object_target=self._object_target(height)
        wrist=object_target@np.linalg.inv(rigid(self.held['T_wrist_from_battery']))
        current=self.model.wrist_transform(s['arms']['right']['joints'],self.lift)
        if np.linalg.norm(current[:3,3]-wrist[:3,3])>.65:raise ValueError('目标超出单次放置接近范围')
        r=copy.deepcopy(self.region);r['target_base_m']=(np.asarray(r['target_base_m'])+np.asarray(r['normal_base'])*height).tolist()
        existing=release_geometry(self._base_battery(s),r,self.held['bounds_m'])
        deadline=self.clock()+45.
        if not existing['ready']:
            timing=copy.deepcopy(self.pick_policy['timing_limits'])
            timing['tcp_speed_mm_s']=min(timing['tcp_speed_mm_s'],5. if height==0 else 10.)
            timing['tcp_acceleration_mm_s2']=min(timing['tcp_acceleration_mm_s2'],10.)
            plan=plan_correction(self.model,s['arms']['right']['joints'],self.lift,np.eye(4),wrist,
                self.controller.config['arms']['right']['joint_limits_deg'],timing_limits=timing)
            self._sweep(plan,holding)
            # Bound carried corner speeds as well as the wrist, using the same quintic transport.
            reach=self.model.reach+np.linalg.norm(np.asarray(self.held['T_wrist_from_battery'])[:3,3])+np.linalg.norm(corners(self.held['bounds_m']),axis=1).max()
            path=TimedJointPath(plan['joint_waypoints_rad'],reach,**timing)
            plan['timing']=path.receipt()
            if self.environment_clear is not True:raise ValueError('放置通路未确认')
            ident='place-'+self.store.state['placement_flow']['phase'];atomic_json(self.store.path/(ident+'.json'),plan)
            start=np.asarray(s['arms']['right']['joints']);sent=False
            def preflight(snapshot,now):
                try:
                    self._check(holding)
                    self._fresh(self.region,self.settings.get('execution_observation_max_age_s',120.))
                    return (self.environment_clear is True and (sent or np.max(abs(np.asarray(snapshot['arms']['right']['joints'])-start))<np.radians(.1)))
                except Exception:return False
            self._refresh_support()
            self._sweep(plan,holding)
            duration=float(plan['timing']['duration_s'])
            if duration>min(555.,self.settings.get('execution_observation_max_age_s',120.)-max(0,self.wall_clock()-self.region['image_time'])-1.):raise ValueError('放置轨迹超过观测有效执行时限')
            self._fresh(self.region)
            wire=dict(interpolation='segment_quintic_scalar',time_from_start_s=path.times.tolist(),joint_waypoints_rad=path.q.tolist())
            self.controller.start_trajectory('right',wire,self.token,preflight=preflight);sent=True
            deadline=self.clock()+duration+45.
            while self.clock()<deadline:
                self._check(holding)
                if self.controller.poll_trajectory(self.token,preflight=preflight)['status']=='reference_complete':break
                self.sleep(.05)
            else:raise ValueError('放置轨迹超时')
        return self._settle(object_target,r,holding,deadline)

    def _release(self):
        self._refresh_support()
        self._settle(self._object_target(0.),self.region,True,self.clock()+15.)
        self._fresh(self.region)
        s=self._check();r=release_geometry(self._base_battery(s),self.region,self.held['bounds_m'])
        if not r['ready']:raise ValueError('未到支撑高度，禁止松爪')
        target=self.controller.config['arms']['right']['gripper_open_raw']
        def gate(snapshot,now):
            try:
                self._fresh(self.region)
                return self._check() is not None and self._rest_still_valid(snapshot) and release_geometry(self._base_battery(snapshot),self.region,self.held['bounds_m'])['ready']
            except Exception:return False
        result=self.controller.submit(dict(type='gripper',arm='right',position=target),self.token,preflight=gate)
        if result.get('ok') is not True:raise ValueError('松爪请求回执未知')
        # Set immediately: failure now is a recovery case, never an automatic re-close/release retry.
        self.released=True
        from .motion_feedback import ArrivalMonitor
        p=copy.deepcopy(self.pick_policy['pick']['gripper_policy']);p['settle_s']=max(.5,p['settle_s'])
        monitor=ArrivalMonitor([target],started_at=self.clock(),**p)
        deadline=self.clock()+min(15.,p['timeout_s'])
        while self.clock()<deadline:
            a=self._check(False)['arms']['right']
            receipt=monitor.update([a['gripper']],a['received'],self.clock(),ready=True,cancelled=self.store.cancelled)
            if receipt['status']=='arrived':return r
            if receipt['status']!='waiting':raise ValueError('夹爪松开检查失败：'+receipt['status'])
            self.sleep(.05)
        raise ValueError('夹爪张开未获新鲜静止反馈确认')

    def call(self,name):
        if name=='held':
            if self.settings.get('execution_enabled') is not True or self.settings.get('placement_geometry_verified') is not True:raise ValueError('当前仅离线开发，未启用经现场核验的放置执行')
            s=self._check();self.lift=s['body']['height'];left=s['arms']['left']
            if not left.get('fresh'):raise ValueError('需要左臂新鲜反馈')
            self.left_joints=np.asarray(left['joints'],float)
            root=Path(self.settings['asset_root']);profile=json.loads((root/'web/public/cad/battery-grip.json').read_text())
            self.held=load_held(root/'agent/runs',self.model,root/'web/cad/gripper-v2/battery-print.stl',profile)
            actual_size=np.diff(np.asarray(self.held['bounds_m']),axis=0)[0]
            self.geometry['battery_size_m']=actual_size.tolist();atomic_json(self.store.path/'held.json',self.held)
        elif name=='head':self.head=self.region=self._observe('d435')
        elif name=='preplace':self._move(.080)
        elif name=='wrist':
            r=self._observe('d405');self._match(r);self.region=r
        elif name=='correct':self._move(.080)
        elif name=='reobserve':
            r=self._observe('d405');self._match(r)
            if np.linalg.norm(np.asarray(r['target_base_m'])-self.region['target_base_m'])>.010:raise ValueError('复核落点变化超过10mm，需要重新规划')
            self.region=r
        elif name=='descend':self._move(0.)
        elif name=='release':self._release()
        elif name=='withdraw':
            if not self.released:raise ValueError('尚未确认松爪')
            self._move(.080,holding=False)
        elif name=='verify':
            # Recognize once, then check a fresh geometry sequence; GPU time cannot masquerade as stability.
            r=self._observe('d435',verify=True);self._match(r)
            recognized=self.preview.verify_battery(self.store,r,self.region,self.geometry)
            self._check(False)
            def observe():
                r=self._observe('d435',verify=True);self._match(r);self._fresh(r)
                evidence=self.preview.verify_battery(self.store,r,self.region,self.geometry,recognize=False)
                return dict(image_time=r['image_time'],center_base_m=evidence['center_base_m'])
            verify_stability(observe,lambda:self._check(False),self.sleep,self.clock,
                             reference=recognized['center_base_m'])
        else:raise ValueError('未知放置阶段')
        return dict(status='confirmed',phase=name,side='left',placement_confirmed=name=='verify')

    def stop(self):
        try:
            if self.controller.motion_valid(self.token):self.controller.submit(dict(type='arm.stop',arm='right'),self.token)
        finally:self.controller.interrupt_motion(self.token)


def verify_stability(observe,check,sleep,clock=time.monotonic,*,reference=None):
    """Existing placement criterion: four fresh frames over three seconds, <=10 mm drift."""
    points=[];started=clock();first_stamp=None;last_stamp=None
    while len(points)<4 or first_stamp is None or last_stamp-first_stamp<3.:
        check()
        if clock()-started>30:raise ValueError('放置视觉稳定性复核超时')
        evidence=observe();stamp=float(evidence['image_time'])
        center=np.asarray(evidence['center_base_m'],float)
        if center.shape!=(3,) or not np.isfinite(center).all() or not np.isfinite(stamp):
            raise ValueError('放置稳定性反馈无效')
        if last_stamp is not None and not 0<stamp-last_stamp<=2.:
            raise ValueError('放置后反馈冻结或稳定性采样中断')
        if first_stamp is None:first_stamp=stamp
        if reference is None:reference=center.copy()
        if np.linalg.norm(center-reference)>.010:raise ValueError('电池放置后仍在移动')
        points.append(center.tolist());last_stamp=stamp;sleep(.5)
    return dict(placement_confirmed=True,battery_recognized=True,
                stable_duration_s=last_stamp-first_stamp,frames=len(points))
