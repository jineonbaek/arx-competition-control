"""Live vision backend; uses running Controller, never initializes motor SDKs."""
import copy
import json
from pathlib import Path
import re
import threading
import time
import uuid
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from agent.visual_docking import DockingRun
from agent.docking_fit import evaluate_fit,OBJECT_TO_TOOL
from agent.docking_grip import GripTracker
from agent.cabinet_preview_geometry import horizontal_plan,opening,detect
from agent.cabinet_loaded_sweep import check_outside_sweep,_boxes,_root,_gap
from agent.grasp_path import plan_correction,plan_insert,TimedJointPath
from agent.storage import atomic_json
from agent.cabinet_controller_bridge import digest,wheels_stopped
from docking_motion import DockingMotion

ROOT=Path(__file__).resolve().parents[2]


def split_plan(points,times):
    parts=[];begin=0
    while begin<len(points)-1:
        end=begin+1
        if times[end]-times[begin]>45:raise ValueError('Trajectory interval exceeds 45 seconds')
        while end+1<len(points) and times[end+1]-times[begin]<=45:end+=1
        parts.append(dict(interpolation='segment_quintic_scalar',
            joint_waypoints_rad=copy.deepcopy(points[begin:end+1]),
            time_from_start_s=[float(t-times[begin]) for t in times[begin:end+1]]))
        begin=end
    return parts


def validate_arrival(phase,actual,nominal,fit,*,start_standoff=None):
    if phase=='align':
        error=float(np.linalg.norm(actual[:3,3]-nominal[:3,3])*1000)
        angle=float(np.degrees(Rotation.from_matrix(nominal[:3,:3].T@actual[:3,:3]).magnitude()))
        if error>15. or angle>3.:raise ValueError('Outside measured coarse arrival not reached')
        return
    if abs(actual[2,3]-nominal[2,3])>.0005:raise ValueError('Measured fixed-Z arrival not reached')
    if fit.get('nominal_clearance_mm',0)<.5:raise ValueError('Measured payload aperture clearance insufficient')
    if phase=='insert' and abs(fit['leading_standoff_mm']+5.)>1.:
        raise ValueError('Measured shallow penetration has not reached 5mm +/-1mm')
    if phase=='withdraw' and (start_standoff is None or abs(fit['leading_standoff_mm']-start_standoff)>1.):
        raise ValueError('Measured original outside stand-off not restored')


def scene_drift_allowance(sweep):
    # Base, left-arm and own-arm gaps are robot-relative. A new cabinet estimate
    # cannot change them; their original swept checks remain in force.
    lower=sweep['lower_mm']
    return min(5.,lower['cabinet_standoff']-50.,lower['table']-10.)


def requires_high_stage(current,high):
    distance=float(np.linalg.norm(current[:3,3]-high[:3,3]))
    angle=float(Rotation.from_matrix(high[:3,:3].T@current[:3,:3]).magnitude())
    return bool(distance>.015 or angle>np.radians(3.))


def level_first_path(model,q,h,tcp,target,limits,timing):
    q=np.asarray(q,float);limits=np.asarray(limits,float)
    # The CAD held-battery convention requires J6=-90deg. Complete that
    # physical wrist roll before adding any other joint correction/translation.
    rolled=q.copy();rolled[5]=-np.pi/2
    if not limits[5,0]<=-90.<=limits[5,1]:raise ValueError('Required horizontal J6 outside joint limits')
    points=[q.tolist()];times=[0.];legs=[]
    if abs(rolled[5]-q[5])>np.radians(.1):
        count=max(1,int(np.ceil(abs(rolled[5]-q[5])/np.radians(2.))))
        roll=np.linspace(q,rolled,count+1)
        receipt=TimedJointPath(roll,model.reach+np.linalg.norm(tcp[:3,3]),
            tcp_position=lambda a:(model.wrist_transform(a,h)@tcp)[:3,3],**timing).receipt()
        points=roll.tolist();times=receipt['time_from_start_s']
        legs.append(dict(kind='roll_j6_outside',target=(model.wrist_transform(rolled,h)@tcp).tolist(),end_waypoint=len(points)-1))
    current=model.wrist_transform(points[-1],h)@tcp
    level=current.copy();level[:3,:3]=target[:3,:3]
    raised=level.copy();raised[2,3]=max(current[2,3],target[2,3])
    for kind,pose in [('level_outside',level),('raise_outside',raised),('approach_high',target)]:
        start=model.wrist_transform(points[-1],h)@tcp
        if np.allclose(start,pose,atol=1e-8,rtol=0):continue
        try:path=plan_correction(model,points[-1],h,tcp,pose,limits,timing_limits=timing,translation_step_mm=5.,rotation_step_deg=2.)
        except ValueError as error:
            if 'IK branch jump' not in str(error):raise
            path=plan_correction(model,points[-1],h,tcp,pose,limits,timing_limits=timing)
        offset=times[-1];points.extend(path['joint_waypoints_rad'][1:]);times.extend(offset+t for t in path['timing']['time_from_start_s'][1:])
        legs.append(dict(kind=kind,target=pose.tolist()))
    if len(points)<2:raise ValueError('No level-first motion needed')
    return dict(joint_waypoints_rad=points,timing=dict(time_from_start_s=times),legs=legs)


def _insert_sweep(ctrl,state,points,tcp,battery,contact,window,slot,table):
    """Nominal complete payload/arm/table sweep; aperture uncertainty is reported separately."""
    model=ctrl.models['right'];left=ctrl.models['left'];lift=state['body']['height']
    left_boxes=_boxes(left,state['arms']['left']['joints'],lift,_root(left,lift));root=_root(model,lift)
    bounds=np.asarray(battery['bounds_mm'],float);corners=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
    payload=(corners-contact)@OBJECT_TO_TOOL.T/1000
    center=np.asarray(window['center_base_m']);out=np.asarray(window['outward_normal_base'])
    normal=np.asarray(table['front_normal_base']);radius=model.reach+np.linalg.norm(tcp[:3,3])+np.max(np.linalg.norm(payload,axis=1))
    lower={k:float('inf') for k in ('aperture','table','arm_standoff','left_arm','base','own_arm')};count=0;bound=.00005
    for a,b in zip(np.asarray(points),np.asarray(points)[1:]):
        steps=max(1,int(np.ceil(radius*np.abs(b-a).sum()/(2*bound))))
        for f in np.linspace(0,1,steps+1):
            q=a+f*(b-a);T=model.wrist_transform(q,lift)@tcp;box=payload@T[:3,:3].T+T[:3,3]
            parts=_boxes(model,q,lift,root);count+=1
            fit=evaluate_fit(T,window,slot,battery,contact)
            lower['aperture']=min(lower['aperture'],fit['nominal_clearance_mm']-bound*1000)
            lower['arm_standoff']=min(lower['arm_standoff'],min(float(((p-center)@out).min())*1000 for p in parts)-bound*1000)
            for p in [box,*parts]:
                gap=max(0.,float(table['front_offset_m']-(p@normal).max()),float(p[:,2].min()-table['top_m']),float(table['bottom_m']-p[:,2].max()))
                lower['table']=min(lower['table'],gap*1000-bound*1000)
                lower['left_arm']=min(lower['left_arm'],*(_gap(p,l)*1000-bound*1000 for l in left_boxes))
            lower['base']=min(lower['base'],_gap(box,np.array(model.column))*1000-bound*1000)
            lower['own_arm']=min(lower['own_arm'],*(_gap(box,p)*1000-bound*1000 for p in parts[:6]))
    required=dict(aperture=.5,table=10.,arm_standoff=20.,left_arm=20.,base=20.,own_arm=10.)
    if not count:raise ValueError('No insertion sweep samples')
    for k,v in required.items():
        if lower[k]<v:raise ValueError('Insertion '+k+' nominal gap %.3fmm below %.3fmm'%(lower[k],v))
    return dict(clear=True,nominal_only=True,physical_accuracy_verified=False,samples=count,lower_mm=lower,required_mm=required)


class LiveDockBackend:
    def __init__(self,ctrl,preview,token,maintenance,model,cancel,event):
        self.ctrl=ctrl;self.preview=preview;self.root=preview.root;self.token=token;self.model=model
        self.cancel_event=cancel;self.event=event;self.initial=ctrl.snapshot();self.config_hash=digest(ctrl.config)
        self.driver=self._driver()['driver_instance'];self.clamp=self._driver()['gripper_target_raw']
        self.raw=self.initial['arms']['right']['gripper'];self.tracker=GripTracker();self.grip=None;self.grip_stamp=-1.
        self.bias=np.zeros(6);self.nominal_goal=None;self.align_count=0;self.engagement='outside';self.phase='observe'
        self.trimmed_epoch=None
        self.cached=None;self.last_observation=None;self.inserted=None;self.moved=[]
        self.finished=False
        self.high_aligned=False
        self.last_geometry_check=0.
        self.grip_lock=threading.RLock();self.monitor_stop=threading.Event();self.after_motion_wall=0.
        self.motion=DockingMotion(ctrl,token,guard=self.guard,base_client=maintenance,check_lift=False,require_lift_settle=False)
        self.timing=dict(joint_speed_deg_s=[3.]*6,joint_acceleration_deg_s2=[6.]*6,tcp_speed_mm_s=12.,tcp_acceleration_mm_s2=25.)
        self.tcp=np.asarray(json.loads((self.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
        self.battery=json.loads((self.root/'config/battery-envelope.json').read_text())
        self.contact=np.asarray(json.loads((self.root/'web/public/cad/battery-grip.json').read_text())['points_object_mm']['tip'])
        self.loaded_battery=self.battery|dict(contact_mm=self.contact.tolist())
        self.slot=opening(json.loads((self.root/'config/cabinet-profiles.json').read_text()),model)
        self.assets=preview._source_hashes('d435')
        cal=json.loads((self.root/'vision/handeye/results/d405-right-cc200-ee0ff2fa49e9e56285d271d6.json').read_text())
        self.serial=cal['camera']['serial']
        cv2.setNumThreads(1)
        self._grip(settle=True)
        def monitor():
            while not self.monitor_stop.is_set():
                try:self._grip(settle=False)
                except (ValueError,OSError):pass  # Guard consumes current valid evidence or rejects.
                self.monitor_stop.wait(.03)
        self.monitor_thread=threading.Thread(target=monitor,daemon=True,name='docking-grip-watch');self.monitor_thread.start()

    def cancelled(self):return self.cancel_event.is_set()
    def _driver(self):
        row=self.ctrl.adapter.trajectory_status('right')
        if not row or not row['ready'] or not row['enabled'] or 'joint_reference_rad' not in row or 'gripper_target_raw' not in row:
            raise ValueError('Fresh running driver with retained clamp/reference telemetry required')
        return row

    def _grip(self,*,settle=False):
        deadline=time.monotonic()+(2.5 if settle else .001)
        while True:
            # Controller preflight already holds its own RLock. The watcher must
            # take that lock BEFORE grip_lock too, never in the opposite order.
            state=self.ctrl.snapshot();meta=json.loads((self.root/'web/public/camera-live/d405.json').read_text())
            with self.grip_lock:
                stamp=meta['captured_at'];arm=state['arms']['right'];name=meta['raw_image_file']
                if meta['state']!='online' or not 0<=time.time()-stamp<=.8 or meta['color_intrinsics']['serial']!=self.serial:
                    raise ValueError('D405 camera unavailable or changed')
                if stamp>self.grip_stamp and abs(stamp-arm['timestamp'])<=.2:
                    if not re.fullmatch(r'd405-v-[a-f0-9]+-\d+\.jpg',name):raise ValueError('D405 image identity invalid')
                    bgr=cv2.imdecode(np.frombuffer((self.root/'web/public/camera-live'/name).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
                    if bgr is None:raise ValueError('D405 image unreadable')
                    rgb=bgr[:,:,::-1]
                    if self.tracker.reference is None:self.tracker.bind(rgb,stamp,arm['gripper'],now=time.time())
                    self.grip=self.tracker.update(rgb,stamp,arm['gripper'],now=time.time());self.grip_stamp=stamp
                    if self.grip['status']=='slip_suspected':raise ValueError('D405 relative grip change detected')
                if self.grip and self.grip.get('stable') and time.time()-self.grip_stamp<.5:return self.grip
            if not settle or time.monotonic()>=deadline:
                detail=(self.grip or {}).get('reason',(self.grip or {}).get('status','no paired exposure'))
                raise ValueError('Waiting for stable independent D405 grip frames: '+detail)
            if self.cancelled():raise InterruptedError('Cancelled while observing')
            time.sleep(.03)

    def guard(self,state=None,now=None):
        state=state or self.ctrl.snapshot();active=self.motion.active
        if self.cancelled() or not self.ctrl.motion_valid(self.token) or digest(self.ctrl.config)!=self.config_hash:
            raise ValueError('Task cancelled, ownership or configuration changed')
        if any(not state['arms'][s]['fresh'] for s in ('left','right')) or not state['body']['fresh']:
            raise ValueError('Robot feedback stale')
        if self._driver()['driver_instance']!=self.driver or self._driver()['gripper_target_raw']!=self.clamp:
            raise ValueError('Driver or retained clamp target changed')
        if abs(state['arms']['right']['gripper']-self.raw)>.05:raise ValueError('Gripper feedback changed')
        if max(abs(a-b) for a,b in zip(state['arms']['left']['joints'],self.initial['arms']['left']['joints']))>np.radians(.5):raise ValueError('Left arm changed')
        if any(abs(state['body'][k]-self.initial['body'][k])>.02 for k in ('waist','head_yaw','head_pitch')):raise ValueError('Camera/body configuration changed')
        if active!='base' and (state['chassis_active'] or not wheels_stopped(state['body']['wheels'])):raise ValueError('Unexpected chassis movement')
        meta=json.loads((self.root/'web/public/camera-live/d435.json').read_text())
        if meta['state']!='online' or not 0<=time.time()-meta['captured_at']<=1.:raise ValueError('Primary D435 camera stale')
        self._grip(settle=False)
        if self.phase=='align' and active in ('arm','lift') and hasattr(self,'window'):
            tick=self.ctrl.clock()
            if tick-self.last_geometry_check>=.08:
                q=state['arms']['right']['joints']
                self.measured_clearance=self._outside_sweep(state,[q,q],dict(window=self.window,table_estimate=self.table))
                self.last_geometry_check=tick
        if (self.engagement=='inside' or self.phase in ('insert','withdraw')) and hasattr(self,'window'):
            T=self.ctrl.models['right'].wrist_transform(state['arms']['right']['joints'],state['body']['height'])@self.tcp
            fit=evaluate_fit(T,self.window,self.slot,self.battery,self.contact)
            if fit['nominal_clearance_mm']<.5 or fit['leading_standoff_mm']< -10.:
                raise ValueError('Measured payload left shallow aperture envelope')
            if abs(T[2,3]-self.fixed_z)>.0005:raise ValueError('Measured insertion Z changed by over 0.5mm')
        return True

    def _outside_sweep(self,state,points,scene):
        return check_outside_sweep(self.ctrl.models['right'],self.ctrl.models['left'],state['arms']['left']['joints'],
            state['body']['height'],points,self.tcp,self.loaded_battery,scene['window'],scene['table_estimate'])

    def observe(self):
        self.phase='observe';self._grip(settle=True)
        deadline=time.monotonic()+2.5
        while self.grip_stamp<=self.after_motion_wall+.02:
            if time.monotonic()>deadline:raise ValueError('No post-motion D405 exposure')
            self._grip(settle=True);time.sleep(.03)
        self.guard();state=self.ctrl.snapshot();planned_epoch=self._driver()['command_epoch']
        self.initial=state;self.event(phase='observe',reason='D435/D405 vision planning',grip=self.grip)
        if self.engagement=='inside':
            # D405 relative evidence does not need unsupported inverse-Brown PnP.
            self._grip(settle=True)
            obs=dict(id=uuid.uuid4().hex,captured_at=self.grip_stamp,driver_id=self.driver,grip_stable=True,
                     outside=False,at_entry=False,fit={})
            self.last_observation=obs;return obs
        scene=self.preview.action(dict(camera='d435',model=self.model),include_arm_preview=False,token=self.token)
        if scene.get('status')!='localized' or scene.get('table_estimate',{}).get('status')!='inferred':raise ValueError('D435 cabinet/table not localized')
        self.window=scene['window'];self.table=scene['table_estimate']
        p=horizontal_plan(self.window,self.slot,self.battery,tip_point_object_mm=self.contact,preclearance_mm=55.,max_insertion_depth_mm=5.)
        q=np.asarray(state['arms']['right']['joints']);h=state['body']['height'];model=self.ctrl.models['right'];T=model.wrist_transform(q,h)@self.tcp
        fit=evaluate_fit(T,self.window,self.slot,self.battery,self.contact)
        if self.finished:
            obs=dict(id=scene['snapshot_id'],captured_at=scene['image_time'],driver_id=self.driver,
                grip_stable=True,outside=fit['leading_standoff_mm']>20.,at_entry=False,fit=fit)
            self.last_observation=obs;return obs
        at_entry=45.<=fit['leading_standoff_mm']<=65. and fit['qualified_fit']
        target=np.eye(4);target[:3,:3]=p['R_base_from_tool'];target[:3,3]=np.asarray(p['tcp_pre_base_mm'])/1000
        first_high=False
        if self.nominal_goal is None:
            high=horizontal_plan(self.window,self.slot,self.battery,tip_point_object_mm=self.contact,preclearance_mm=100.,max_insertion_depth_mm=5.)
            high_pose=target.copy();high_pose[:3,3]=np.asarray(high['tcp_pre_base_mm'])/1000;high_pose[2,3]+=.03
            first_high=requires_high_stage(T,high_pose)
            if first_high:target=high_pose
            else:self.bias=np.asarray(self._driver()['joint_reference_rad'])-q
        if at_entry:
            self.fixed_z=float(T[2,3]);axis=np.asarray(p['insertion_axis_base'],float)
            desired_rotation=np.asarray(p['R_base_from_tool'],float)
            if abs(T[2,0])>.003:raise ValueError('Current tool insertion axis needs outside leveling')
            depth=5.+fit['leading_standoff_mm'];end=T[:3,3]*1000+axis*depth
            path=plan_insert(model,q,h,T[:3,3]*1000,end,desired_rotation,self.tcp,
                self.ctrl.config['arms']['right']['joint_limits_deg'],timing_limits=dict(self.timing,tcp_speed_mm_s=3.,tcp_acceleration_mm_s2=6.))
            if path['interpolation_position_bound_mm']>fit['available_clearance_mm']:
                raise ValueError('Insertion interpolation bound exceeds remaining nominal aperture margin')
            nominal=np.asarray(path['joint_waypoints_rad']);offset=np.asarray(self._driver()['joint_reference_rad'])-q
            if np.max(np.abs(offset))>np.radians(2):raise ValueError('Holding compensation exceeds 2deg bound')
            commands=nominal+offset
            if np.max(np.abs(nominal[0]-q))>np.radians(.25):raise ValueError('Insertion approach requires a separate outside correction')
            sweep=_insert_sweep(self.ctrl,state,np.vstack((q,nominal)),self.tcp,self.battery,self.contact,self.window,self.slot,self.table)
            self.cached=dict(strategy='arm',points=commands.tolist(),times=path['timing']['time_from_start_s'],
                             nominal_points=nominal.tolist(),sweep=sweep,kind='insert',holding_offset=offset.tolist(),
                             nominal_end=nominal[-1].tolist(),start_standoff=fit['leading_standoff_mm'])
        else:
            if self.nominal_goal is not None and self.trimmed_epoch!=planned_epoch:
                if self.high_aligned:
                    # This is the offset already holding the payload, not a new
                    # torque/gain or extra initial joint jump.
                    self.bias=np.asarray(self._driver()['joint_reference_rad'])-q
                    self.high_aligned=False
                else:self.bias+=np.clip(self.nominal_goal-q,-np.radians(.75),np.radians(.75))
                self.bias=np.clip(self.bias,-np.radians(2),np.radians(2))
                self.trimmed_epoch=planned_epoch
            try:
                if first_high:path=level_first_path(model,q,h,self.tcp,target,self.ctrl.config['arms']['right']['joint_limits_deg'],self.timing)
                else:path=plan_correction(model,q,h,self.tcp,target,self.ctrl.config['arms']['right']['joint_limits_deg'],timing_limits=self.timing)
                nominal=np.asarray(path['joint_waypoints_rad'])
                holding=np.asarray(self._driver()['joint_reference_rad'])-q
                if np.max(np.abs(holding))>np.radians(2):raise ValueError('Existing load holding offset exceeds2deg')
                fractions=np.linspace(0,1,len(nominal))[:,None];commands=nominal+(1-fractions)*holding+fractions*self.bias
                sweep=self._outside_sweep(state,commands.tolist(),scene)
                timing=path['timing'] if np.max(np.abs(commands-nominal))<1e-12 else TimedJointPath(commands,model.reach+np.linalg.norm(self.tcp[:3,3]),
                    tcp_position=lambda a:(model.wrist_transform(a,h)@self.tcp)[:3,3],**self.timing).receipt()
                self.cached=dict(strategy='arm',points=commands.tolist(),times=timing['time_from_start_s'],sweep=sweep,kind='align',nominal_end=nominal[-1].tolist(),high_stage=first_high,legs=path.get('legs'))
            except ValueError as error:
                # Native auxiliary groups are chosen only for a checked useful displacement.
                from handeye_column import NOMINAL_RAD_PER_M
                dz=float(target[2,3]-T[2,3]);dh=float(np.clip(dz*NOMINAL_RAD_PER_M,-.5,.5))
                if abs(dh)>.05:
                    goal=float(np.clip(h+dh,0,18))
                    checks=[self._outside_sweep(state|dict(body=state['body']|dict(height=float(v))),[q.tolist(),q.tolist()],scene)
                            for v in np.linspace(h,goal,8)]
                    height_bound=abs(goal-h)/NOMINAL_RAD_PER_M*1000/14
                    if min(c['lower_mm']['table'] for c in checks)-height_bound<10.:
                        raise ValueError('Lift swept table margin insufficient between samples')
                    self.cached=dict(strategy='lift',target=goal,sweep=checks,kind='align')
                else:
                    delta=(target[:2,3]-T[:2,3]);delta*=min(1.,.03/max(np.linalg.norm(delta),1e-9))
                    if np.linalg.norm(delta)<.003:raise ValueError('No useful bounded base/lift alternative: '+str(error))
                    moved=copy.deepcopy(scene);moved['window']['center_base_m']=(np.asarray(self.window['center_base_m'])-np.r_[delta,0]).tolist()
                    moved['table_estimate']['front_offset_m']-=float(np.asarray(self.table['front_normal_base'])[:2]@delta)
                    sweep=self._outside_sweep(state,[q.tolist(),q.tolist()],moved)
                    self.cached=dict(strategy='base',delta=[*map(float,delta),0.],sweep=sweep,kind='align')
        # Reobserve after slow IK. The target may drift slightly outside, but a
        # moved cabinet invalidates the old path rather than executing stale IK.
        self._grip(settle=True)
        fresh=self.preview.action(dict(camera='d435',model=self.model),include_arm_preview=False,token=self.token)
        if fresh.get('status')!='localized':raise ValueError('Cabinet lost during planning')
        drift=np.linalg.norm(np.asarray(fresh['window']['center_base_m'])-self.window['center_base_m'])*1000
        if drift>(.3 if at_entry else 5.):raise ValueError('Cabinet pose changed during planning %.3fmm'%drift)
        current=self.ctrl.snapshot()
        if np.max(np.abs(np.asarray(current['arms']['right']['joints'])-q))>np.radians(.1):
            raise ValueError('Robot start state changed during planning')
        self.window=fresh['window'];self.table=fresh['table_estimate']
        if self.cached['strategy']=='arm':
            if at_entry:self.cached['sweep']=_insert_sweep(self.ctrl,current,np.vstack((q,self.cached['nominal_points'])),self.tcp,self.battery,self.contact,self.window,self.slot,self.table)
            else:self.cached['sweep']=self._outside_sweep(current,self.cached['points'],fresh)
        elif self.cached['strategy']=='lift':
            self.cached['sweep']=[self._outside_sweep(current|dict(body=current['body']|dict(height=float(v))),[q.tolist(),q.tolist()],fresh)
                for v in np.linspace(h,self.cached['target'],8)]
        elif self.cached['strategy']=='base':
            moved=copy.deepcopy(fresh);delta=np.asarray(self.cached['delta'])
            moved['window']['center_base_m']=(np.asarray(fresh['window']['center_base_m'])-delta).tolist()
            moved['table_estimate']['front_offset_m']-=float(np.asarray(self.table['front_normal_base'])@delta)
            self.cached['sweep']=self._outside_sweep(current,[q.tolist(),q.tolist()],moved)
        # Fast final exposure checks after sweep, without rerunning IK.
        frame=self.preview.captures['d435'].capture(token=self.token);detected=detect(frame['rgb'])
        if detected['status']!='complete' or np.max(np.linalg.norm(np.asarray(detected['inner_quad_px'])-fresh['window']['inner_quad_px'],axis=1))>(.5 if at_entry else 2.):
            raise ValueError('Cabinet marker moved before dispatch')
        last=self.preview.analyze_frame('d435',self.model,frame,include_arm_preview=False)
        if last.get('status')!='localized':raise ValueError('Final cabinet exposure not localized')
        corner_change=float(np.max(np.linalg.norm(np.asarray(last['window']['corners_base_m'])-self.window['corners_base_m'],axis=1)))*1000
        if at_entry:remaining=self.cached['sweep']['lower_mm']['aperture']-.5
        elif self.cached['strategy']=='arm':
            remaining=scene_drift_allowance(self.cached['sweep'])
        elif self.cached['strategy']=='lift':remaining=min(scene_drift_allowance(s) for s in self.cached['sweep'])
        else:remaining=scene_drift_allowance(self.cached['sweep'])
        self.event(phase='dispatch_validation',corner_change_mm=corner_change,checked_scene_drift_allowance_mm=remaining)
        if corner_change>remaining:raise ValueError('Final cabinet exposure exceeds checked geometric drift allowance')
        self.window=last['window']
        if self.cached['strategy']=='arm':
            degrees=np.degrees(np.asarray(self.cached['points']));limits=np.asarray(self.ctrl.config['arms']['right']['joint_limits_deg'])
            if np.any(degrees<limits[:,0]) or np.any(degrees>limits[:,1]):raise ValueError('Compensated plan outside joint limits')
        self.guard();T=model.wrist_transform(current['arms']['right']['joints'],h)@self.tcp
        fit=evaluate_fit(T,self.window,self.slot,self.battery,self.contact)
        obs=dict(id=frame['id'],captured_at=frame['image_time'],driver_id=self.driver,grip_stable=True,
            outside=True,at_entry=bool(at_entry and fit['qualified_fit']),fit=fit)
        if self._driver()['command_epoch']!=planned_epoch:raise ValueError('Driver command epoch changed during planning')
        self.cached['start_q']=q.tolist();self.cached['height']=h;self.cached['epoch']=planned_epoch
        self.last_observation=obs
        self.event(phase='plan_ready',fit=fit,grip=self.grip,strategy=self.cached['strategy'],reason='Supervised nominal-model trial; no physical precision certification',
                   trial_depth_mm=5.,known_tcp_max_residual_mm=4.181058160075474)
        return obs

    def plan(self,phase,obs,*,reverse_of=None):
        if obs!=self.last_observation:raise ValueError('Observation identity changed')
        if phase=='withdraw':
            if self.inserted is None or reverse_of!=self.inserted['id']:raise ValueError('No confirmed insertion to reverse')
            source=self.inserted['payload'];end=source['times'][-1]
            payload=source|dict(points=list(reversed(source['points'])),times=[end-t for t in reversed(source['times'])],
                nominal_end=source['nominal_points'][0],start_q=self.inserted['measured_end_q'],epoch=self.inserted['completed_epoch'])
        else:
            if self.cached is None or self.cached['kind']!=phase:raise ValueError('No matching fresh checked plan')
            payload=copy.deepcopy(self.cached)
        plan=dict(id=uuid.uuid4().hex,observation_id=obs['id'],driver_id=self.driver,phase=phase,strategy=payload['strategy'],
            checked_sweep=True,measured_feedback=True,max_duration_s=(payload['times'][-1] if payload['strategy']=='arm' else 30.),
            fixed_z=phase in ('insert','withdraw'),reverse_of=reverse_of,gripper_action='retain',payload=payload)
        return plan

    def execute(self,plan,check_cancel):
        self.phase=plan['phase'];p=plan['payload'];self.guard()
        if self.preview._source_hashes('d435')!=self.assets:raise ValueError('Cabinet/TCP/calibration assets changed')
        if self._driver()['driver_instance']!=plan['driver_id']:raise ValueError('Driver changed before dispatch')
        self.event(phase=self.phase,reason='Executing one checked '+p['strategy']+' plan',plan_id=plan['id'])
        if p['strategy']=='arm':
            # Reversing uses the already applied compensation, not a new grip or mode.
            self.motion.arm(split_plan(p['points'],p['times']),check_cancel,
                expected_start=dict(joints=p['start_q'],height=p['height']),
                expected_driver=dict(driver_instance=self.driver,command_epoch=p['epoch']))
        elif p['strategy']=='lift':self.motion.lift(p['target'],check_cancel,
            expected_start=dict(joints=p['start_q'],height=p['height']),expected_driver=dict(driver_instance=self.driver,command_epoch=p['epoch']))
        elif p['strategy']=='base':self.motion.base(p['delta'],check_cancel,
            expected_start=dict(joints=p['start_q'],height=p['height']),expected_driver=dict(driver_instance=self.driver,command_epoch=p['epoch']))
        self.moved.append(p['strategy']);self.initial=self.ctrl.snapshot()
        self.guard()
        if p['strategy']=='arm':
            actual=self.ctrl.models['right'].wrist_transform(self.initial['arms']['right']['joints'],self.initial['body']['height'])@self.tcp
            nominal=self.ctrl.models['right'].wrist_transform(p['nominal_end'],p['height'])@self.tcp
            fit=evaluate_fit(actual,self.window,self.slot,self.battery,self.contact)
            validate_arrival(self.phase,actual,nominal,fit,start_standoff=p.get('start_standoff'))
            if self.phase=='align':self.nominal_goal=np.asarray(p['nominal_end'])
            if p.get('high_stage'):self.high_aligned=True
        if self.phase=='insert':
            self.engagement='inside';self.inserted=copy.deepcopy(plan)
            self.inserted.update(measured_end_q=self.initial['arms']['right']['joints'],completed_epoch=self.motion.completed_driver['command_epoch'])
        elif self.phase=='withdraw':self.engagement='outside';self.finished=True
        self.event(phase=self.phase,reason='Reference complete and measured rest confirmed',measured_state=self.initial)
        self.after_motion_wall=time.time()
        return dict(plan_id=plan['id'],driver_id=self.driver,measured_complete=True,gripper_released=False,
                    fixed_z=plan['fixed_z'],engagement=self.engagement)

    def hold(self):self.motion.hold()
    def close(self):self.monitor_stop.set();self.monitor_thread.join(1.)


class LiveDockTask:
    def __init__(self,controller,preview,*,directory=None,auto_worker=True):
        self.ctrl=controller;self.preview=preview;self.directory=Path(directory or ROOT/'web/hardware/runtime/visual-docking')
        self.auto_worker=auto_worker;self.lock=threading.RLock();self.cancel=threading.Event();self.thread=None;self.backend=None
        self.state=dict(status='idle',phase='idle',released=False,reason='',execution_backend='existing_drivers',trial_depth_mm=5.)
        if (self.directory/'receipt.json').exists():
            try:
                old=json.loads((self.directory/'receipt.json').read_text())
                if old.get('status') not in ('idle','blocked','held_outside','cancelled'):self.state.update(status='recovery_required',reason='Previous physical task not replayed')
            except (ValueError,OSError):self.state.update(status='recovery_required',reason='Receipt unreadable')
    def snapshot(self):
        with self.lock:
            state=copy.deepcopy(self.state)
            if self.backend is not None:
                with self.backend.grip_lock:
                    state.update(grip=copy.deepcopy(self.backend.grip),grip_captured_at=self.backend.grip_stamp)
            return state
    def save(self,**changes):
        with self.lock:
            self.state.update(changes);self.directory.mkdir(parents=True,exist_ok=True);atomic_json(self.directory/'receipt.json',self.state)
    def action(self,data):
        if data==dict(op='cancel'):
            self.cancel.set()
            if self.backend is not None:self.backend.hold()
            return self.snapshot()
        if type(data) is not dict or set(data)!={'op','model'} or data['op']!='start' or data['model'] not in ('demo','competition'):
            raise ValueError('Only fixed no-release start/model or cancel is accepted')
        with self.lock:
            if self.state['status'] not in ('idle','blocked','held_outside','cancelled') or self.thread and self.thread.is_alive():raise ValueError('Task active or physical recovery unresolved')
            self.cancel=threading.Event();self.save(status='planning',phase='observe',reason='',model=data['model'],run_id=uuid.uuid4().hex,released=False)
            if self.auto_worker:
                self.thread=threading.Thread(target=self._run,daemon=True,name='visual-docking-live');self.thread.start()
            return self.snapshot()
    def _run(self):
        token=maintenance=None;runner=None
        try:
            state=self.ctrl.snapshot()
            if state['chassis_active']:raise ValueError('Chassis moving')
            token=self.ctrl.reserve_motion(state['command_epoch']);maintenance=self.ctrl.reserve_chassis_maintenance()
            self.backend=LiveDockBackend(self.ctrl,self.preview,token,maintenance,self.state['model'],self.cancel,self.save)
            self.save(status='running',control_owned=True)
            runner=DockingRun(self.backend,journal=self.directory/'runs'/self.state['run_id']/'receipt.json')
            result=runner.run();self.save(**result,groups_moved=self.backend.moved)
            if result['status']=='recovery_required' or result['status']=='cancelled' and result['motion_started']:
                self.save(status='recovery_required',control_retained=True);return
        except Exception as error:self.save(status='blocked' if not runner or not runner.state['motion_started'] else 'recovery_required',reason=str(error))
        finally:
            if self.backend is not None:self.backend.close()
            if self.state['status']!='recovery_required':
                try:
                    if maintenance is not None:self.ctrl.release_chassis_maintenance(maintenance)
                    if token is not None:self.ctrl.release_motion(token)
                    self.save(control_owned=False)
                except Exception as error:self.save(status='recovery_required',control_retained=True,reason='Control handoff failed: '+str(error))
    def close(self):
        if self.thread and self.thread.is_alive():self.action(dict(op='cancel'));self.thread.join(3.)
