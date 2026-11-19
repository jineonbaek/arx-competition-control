"""One fixed entry/height/visual-level task on the EXISTING motor controllers."""
import copy,hashlib,json,time,threading,uuid,subprocess,urllib.request,http.cookiejar,sys
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
from agent.visual_battery_level import entry_joints_deg,motion_limits,d435_observation,visual_level_step
from agent.grasp_path import TimedJointPath,plan_correction
from agent.held_insertion_path import plan_insert
from agent.docking_fit import OBJECT_TO_TOOL
from agent.docking_grip import GripTracker
from agent.cabinet_loaded_sweep import _boxes,_root,_gap
from held_insertion_sweep import check_outside_sweep
from agent.cabinet_preview_geometry import localize,opening,horizontal_plan,detect
from agent.locked_cabinet_marker import marker_drift_mm
from agent.held_placement_geometry import effective_tcp,prepared_height,scene_allowance,compensate_path,updated_bias,retained_reference_path,capture_motion_bound,aperture_recenter
from agent.docking_fit import evaluate_fit
from agent.shallow_aperture import evaluate_shallow_fit,aperture_relevant
from agent.storage import atomic_json
from docking_motion import DockingMotion
from agent.taught_placement import (PhaseTimeline,load_preobservation_waypoint,load_taught_route,
    outer_alignment_error,reverse_reached_route,validate_preobservation_pose,validate_preobservation_waypoint)
from agent.d405_aperture_alignment import estimate_horizontal_alignment,estimate_vertical_alignment
from agent.visual_chassis_correction import load_chassis_policy,load_locked_handoff
from d405_alignment_overlay import write_horizontal_overlay
from handeye_validation import CANDIDATE as D405_CALIBRATION

ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT

def d435_estimator_command():
    return [sys.executable,str(ROOT/'scripts/estimate-held-d435.py')]

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

def camera_ready(meta,now):
    stamp=meta.get('captured_at')
    return meta.get('state')=='online' and isinstance(stamp,(int,float)) and 0<=now-stamp<1.

def load_fixed_entry(path,current_lift):
    try:data=json.loads(Path(path).read_text())
    except (OSError,ValueError) as error:raise ValueError('Invalid fixed entry waypoint') from error
    q=np.asarray(data.get('joints_rad'),float);w=np.asarray(data.get('wrist_base_mm_deg'),float)
    lift=data.get('lift_rad');grip=data.get('gripper_command_target')
    if (data.get('arm')!='right' or q.shape!=(6,) or w.shape!=(6,)
            or not np.isfinite(np.r_[q,w,lift,current_lift]).all() or grip!=0
            or abs(float(lift)-float(current_lift))>.10):
        raise ValueError('Invalid fixed entry waypoint')
    return data

def required_cameras(mode):
    if mode=='auto_insert_160':return ()
    return ('d435',) if mode=='d435_only' else ('d435','d405')

def requires_three_rad(mode,height):
    return mode not in ('d435_only','auto_insert_160') and not prepared_height(height)

class ContinuousMonitor:
    def __init__(self,sample):
        self.stop=threading.Event();self.failed=threading.Event();self.error=None
        def work():
            while not self.stop.is_set():
                try:sample()
                except Exception as e:self.error=e;self.failed.set();return
                self.stop.wait(.03)
        self.thread=threading.Thread(target=work,daemon=True);self.thread.start()
    def check(self):
        if self.failed.is_set():raise self.error
    def close(self):self.stop.set();self.thread.join(2.)

def guarded_read(read,guard,*,timeout_s):
    """Keep exposure/holding checks live while a read-only capture blocks."""
    guard();done=threading.Event();result={}
    def worker():
        try:result['value']=read()
        except Exception as e:result['error']=e
        finally:done.set()
    threading.Thread(target=worker,daemon=True).start();deadline=time.monotonic()+timeout_s
    while not done.wait(.05):
        guard()
        if time.monotonic()>deadline:raise ValueError('Read-only capture deadline')
    guard()
    if 'error' in result:raise result['error']
    return result['value']

def require_joint_arrival(actual,target,tolerance_deg):
    if np.max(np.abs(np.array(actual)-target))>np.radians(tolerance_deg):
        raise ValueError('Measured joint endpoint not attained')

def require_stationary_pose(state,q,height,epoch,expected_epoch):
    if (np.max(np.abs(np.array(state['arms']['right']['joints'])-q))>np.radians(.1)
            or abs(state['body']['height']-height)>.02 or epoch!=expected_epoch):
        raise ValueError('Robot moved during body pose estimation')

def require_grip(evidence):
    if not evidence or evidence.get('stable') is not True or evidence.get('status')!='stable_visible':
        raise ValueError('Held battery image evidence unavailable: '+str((evidence or {}).get('reason',(evidence or {}).get('status','none'))))

def require_gripper_retained(feedback,baseline,*,tolerance=.25):
    if (not np.isfinite([feedback,baseline,tolerance]).all() or not 0<tolerance<=.25
            or abs(float(feedback)-float(baseline))>tolerance):
        raise ValueError('Gripper feedback changed while battery must remain held')
    return True

def require_loaded_gripper(feedback):
    if not np.isfinite(feedback) or not -1.5<float(feedback)<-.25:
        raise ValueError('Loaded battery clamp feedback unavailable')
    return True

def d405_correction_target(tool,measurement,*,limit_mm=5.):
    T=np.asarray(tool,float);step=float(measurement.get('step_mm',float('nan')))
    if (T.shape!=(4,4) or not np.isfinite(T).all() or measurement.get('status')!='visual_guidance_only'
            or not np.isfinite(step) or abs(step)>limit_mm):
        raise ValueError('D405 correction unavailable or out of range')
    result=T.copy();result[2,3]+=step/1000.;return result

def d405_horizontal_correction_target(tool,measurement,*,limit_mm=3.):
    T=np.asarray(tool,float);step=float(measurement.get('step_mm',float('nan')))
    axis=np.asarray(measurement.get('axis_base'),float)
    if (T.shape!=(4,4) or not np.isfinite(T).all() or measurement.get('status')!='visual_guidance_only'
            or axis.shape!=(3,) or not np.allclose(axis,[0.,1.,0.],atol=1e-9)
            or not np.isfinite([step,limit_mm]).all() or not 0<limit_mm<=3. or abs(step)>limit_mm):
        raise ValueError('D405 horizontal correction unavailable or out of range')
    result=T.copy();result[1,3]+=step/1000.;return result

def validate_lateral_endpoint(start_tool,end_tool,end_joints,route,*,y_bounds_mm=None,
                              position_margin_mm=0.,rotation_margin_deg=0.):
    start=np.asarray(start_tool,float);end=np.asarray(end_tool,float);q=np.asarray(end_joints,float)
    if (start.shape!=(4,4) or end.shape!=(4,4) or q.shape!=(6,)
            or not np.isfinite(np.r_[start.ravel(),end.ravel(),q,position_margin_mm,rotation_margin_deg]).all()
            or not 0<=position_margin_mm<route.preobservation_z_tolerance_mm
            or not 0<=rotation_margin_deg<min(route.preobservation_rotation_tolerance_deg,route.preobservation_axis_tolerance_deg)):
        raise ValueError('Invalid D405 lateral endpoint')
    delta=(end[:3,3]-start[:3,3])*1000.
    relative=start[:3,:3].T@end[:3,:3]
    rotation_error=float(np.degrees(np.arccos(np.clip((np.trace(relative)-1.)/2.,-1.,1.))))
    axis=end[:3,0]/np.linalg.norm(end[:3,0])
    axis_error=float(np.degrees(np.arccos(np.clip(axis@[1.,0.,0.],-1.,1.))))
    position_limit=route.preobservation_z_tolerance_mm-position_margin_mm
    rotation_limit=route.preobservation_rotation_tolerance_deg-rotation_margin_deg
    axis_limit=route.preobservation_axis_tolerance_deg-rotation_margin_deg
    if (abs(delta[0])>position_limit or abs(delta[2])>position_limit
            or rotation_error>rotation_limit or axis_error>axis_limit):
        raise ValueError('D405 lateral endpoint changed locked X/Z/rotation/axis')
    if abs(delta[1])>route.d405_total_limit_mm:
        raise ValueError('D405 lateral endpoint exceeded total Y bound')
    if y_bounds_mm is not None:
        bounds=np.asarray(y_bounds_mm,float)
        if bounds.shape!=(2,) or not np.isfinite(bounds).all():raise ValueError('Invalid authorized Y bounds')
        low,high=sorted(bounds.tolist());tolerance=route.preobservation_z_tolerance_mm-position_margin_mm
        if not low-tolerance<=delta[1]<=high+tolerance:
            raise ValueError('D405 lateral endpoint exceeded authorized Y interval')
    j6_error=abs(float(np.degrees(q[5])-route.preobservation_j6_target_deg))
    if j6_error>route.preobservation_j6_tolerance_deg:
        raise ValueError('D405 lateral endpoint left J6 branch')
    return dict(delta_y_mm=float(delta[1]),x_drift_mm=float(delta[0]),z_drift_mm=float(delta[2]),
        rotation_drift_deg=rotation_error,axis_drift_deg=axis_error,j6_error_deg=j6_error,
        position_margin_mm=float(position_margin_mm),rotation_margin_deg=float(rotation_margin_deg))

def alignment_goal(fit,T,window,plan):
    if not fit['qualified_fit']:
        try:
            delta,gap=aperture_recenter(fit)
        except ValueError as error:
            if str(error)!='No positive clearance within 5mm correction':raise
        else:
            goal=T.copy()
            goal[:3,3]+=(delta[0]*np.array(window['left_axis_base'])+delta[1]*np.array(window['up_axis_base']))/1000.
            return goal,delta,gap
    goal=np.eye(4);goal[:3,:3]=plan['R_base_from_tool'];goal[:3,3]=np.array(plan['tcp_pre_base_mm'])/1000
    return goal,None,None

def retry_cabinet_observation(read):
    for attempt in range(3):
        try:return read()
        except ValueError as error:
            if str(error)!='红框前沿深度不符合单一平面' or attempt==2:raise

def validate_scene_drift(proof,scene_drift,table_drift):
    lower=proof['lower_mm'];required=proof['required_mm']
    # Bound the rim's observed variation and keep table changes within their
    # own, often much smaller, clearance. Limit rim jitter to the trial's 5mm.
    scene_limit=max(0.,min(5.,lower['cabinet_standoff']-required['cabinet_standoff'],
                           lower['arm_standoff']-required['arm_standoff']))
    table_limit=max(0.,lower['table']-required['table']) if 'table' in required else None
    if not np.isfinite([scene_drift,table_drift]).all() or scene_drift>scene_limit or (table_limit is not None and table_drift>table_limit):
        raise ValueError('Fresh scene or table exceeds its remaining path clearance')
    return scene_limit,table_limit

def raised_tool_target(T):
    result=np.array(T,dtype=float,copy=True)
    if result.shape!=(4,4) or not np.isfinite(result).all():raise ValueError('Invalid tool pose')
    result[2,3]+=.010
    return result

def rotated_tool_target(T,left,up):
    result=np.array(T,dtype=float,copy=True);axis=np.cross(left,up);length=np.linalg.norm(axis)
    if result.shape!=(4,4) or not np.isfinite(result).all() or not np.isfinite(length) or length<.9:raise ValueError('Invalid rotation reference')
    result[:3,:3]=Rotation.from_rotvec(axis/length*np.radians(1.5)).as_matrix()@result[:3,:3]
    return result

def recovery_tool_target(T,direction,distance_mm):
    result=np.array(T,dtype=float,copy=True);axis=np.asarray(direction,dtype=float);norm=np.linalg.norm(axis)
    if result.shape!=(4,4) or axis.shape!=(3,) or not np.isfinite(np.r_[result.ravel(),axis,norm,distance_mm]).all() or norm<.9 or not 0<distance_mm<=12.:
        raise ValueError('Invalid bounded recovery target')
    result[:3,3]+=axis/norm*distance_mm/1000.
    return result

def continuous_correction(planner,*args,timing_limits):
    for step in (5.,2.,1.,.5,.25):
        try:
            return planner(*args,timing_limits=timing_limits,translation_step_mm=step,rotation_step_deg=min(2.,step))
        except ValueError as error:
            if str(error)!='Correction IK branch jump or singular configuration' or step==.25:raise

def no_retreat_target(start,target,outward):
    start=np.asarray(start,float);result=np.array(target,dtype=float,copy=True);normal=np.asarray(outward,float)
    if start.shape!=(4,4) or result.shape!=(4,4) or normal.shape!=(3,) or not np.isfinite(np.r_[start.ravel(),result.ravel(),normal]).all() or np.linalg.norm(normal)<.9:
        raise ValueError('Invalid no-retreat reference')
    normal=normal/np.linalg.norm(normal)
    outward_delta=float((result[:3,3]-start[:3,3])@normal)
    if outward_delta>0:result[:3,3]-=outward_delta*normal
    return result

def d435_incremental_target(T,error,window,*,max_step_mm=10.,standoff_range_mm=(55.,65.)):
    result=np.array(T,dtype=float,copy=True);residual=np.asarray(error.get('residual_uv_mm'),float)
    standoff=float(error.get('leading_standoff_mm',float('nan')));limits=np.asarray(standoff_range_mm,float)
    left=np.asarray(window.get('left_axis_base'),float);up=np.asarray(window.get('up_axis_base'),float);out=np.asarray(window.get('outward_normal_base'),float)
    values=np.r_[result.ravel(),residual,standoff,limits,left,up,out,max_step_mm]
    if (result.shape!=(4,4) or residual.shape!=(2,) or limits.shape!=(2,) or left.shape!=(3,) or up.shape!=(3,) or out.shape!=(3,)
            or not np.isfinite(values).all() or not 0<max_step_mm<=10. or not 0<=limits[0]<limits[1]
            or min(np.linalg.norm(left),np.linalg.norm(up),np.linalg.norm(out))<.9):
        raise ValueError('Invalid D435 incremental correction')
    left/=np.linalg.norm(left);up/=np.linalg.norm(up);out/=np.linalg.norm(out)
    depth=limits[0]-standoff if standoff<limits[0] else limits[1]-standoff if standoff>limits[1] else 0.
    delta=residual[0]*left+residual[1]*up+depth*out
    length=float(np.linalg.norm(delta))
    if length>max_step_mm:delta*=max_step_mm/length
    result[:3,3]+=delta/1000.
    return result

def preflight_outside(observe,sweep,guard,save,*,clock=time.time,attempts=3):
    """Retry changing/stale observations before dispatch, never a collision rejection."""
    last_error=None
    for retry in range(attempts):
        guard();fresh,fresh_table=observe();proof=sweep(fresh,fresh_table)
        last,last_table=observe()
        drift=float(np.max(np.linalg.norm(np.array(last['corners_base_m'])-fresh['corners_base_m'],axis=1)))*1000
        table_drift=max(abs(last_table[k]-fresh_table[k]) for k in ('front_offset_m','top_m','bottom_m'))*1000
        try:
            scene_limit,table_limit=validate_scene_drift(proof,drift,table_drift)
        except ValueError as error:
            last_error=error
            save(scene_delta_mm=drift,table_delta_mm=table_drift,scene_retry=retry+1)
            continue
        last_proof=sweep(last,last_table)
        # The geometric sweep may itself take longer than a camera frame.
        # Re-observe after it and bind the checked plane to that new exposure.
        guard();latest,latest_table=observe()
        late_drift=float(np.max(np.linalg.norm(np.array(latest['corners_base_m'])-last['corners_base_m'],axis=1)))*1000
        late_table_drift=max(abs(latest_table[k]-last_table[k]) for k in ('front_offset_m','top_m','bottom_m'))*1000
        try:
            validate_scene_drift(last_proof,late_drift,late_table_drift)
            if not 0<=clock()-latest['image_time']<=1.:raise ValueError('Final cabinet exposure stale')
        except ValueError as error:
            last_error=error
            save(scene_delta_mm=late_drift,table_delta_mm=late_table_drift,scene_retry=retry+1)
            continue
        save(scene_delta_mm=drift,table_delta_mm=table_drift,remaining_scene_clearance_mm=scene_limit,
             remaining_table_clearance_mm=table_limit,scene_retry=retry,final_scene_delta_mm=late_drift,
             final_table_delta_mm=late_table_drift)
        return last,last_table,last_proof
    raise last_error or ValueError('Outside scene preflight unavailable')

def payload_in_tool(bounds_mm,attachment):
    corners=np.array([[x,y,z] for x in bounds_mm[:,0] for y in bounds_mm[:,1] for z in bounds_mm[:,2]])/1000
    return corners@attachment[:3,:3].T+attachment[:3,3]

def table_front_x(frame,*,details=False):
    rgb=frame['rgb'];xyz=frame['xyz'];T=frame['base_from_color'];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    selected=(hsv[:,:,0]>=8)&(hsv[:,:,0]<=45)&(hsv[:,:,1]>=12)&(hsv[:,:,1]<=140)&(hsv[:,:,2]>120)&(xyz[:,:,2]>.1)
    selected[int(rgb.shape[0]*.45):]=False
    points=xyz[selected]@T[:3,:3].T+T[:3,3]
    points=points[(points[:,2]>.35)&(points[:,2]<.85)]
    if len(points)<500:raise ValueError('Table front not visible for fixed entry/lift')
    points=points[::max(1,len(points)//3000)];rng=np.random.default_rng(7);best=None
    for _ in range(200):
        a,b,c=points[rng.choice(len(points),3,replace=False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-7:continue
        n/=length
        if abs(n[2])<.95:continue
        inliers=np.abs((points-a)@n)<.006
        if best is None or inliers.sum()>best.sum():best=inliers
    if best is None or best.sum()<500 or min(np.ptp(points[best,:2],axis=0))<.2:
        raise ValueError('Table plane evidence insufficient')
    front=float(np.quantile(points[best,0],.02)-.02)
    if details:
        top=float(np.quantile(points[best,2],.98))
        return dict(status='inferred',front_normal_base=[1.,0.,0.],front_offset_m=front,
                    top_m=top+.001,bottom_m=top-.06,source='paired_rgbd_table_plane',support_pixels=int(best.sum()))
    return front

class FastVisualLevel:
    def __init__(self,ctrl,depth):
        self.ctrl=ctrl;self.depth=depth;self.lock=threading.RLock();self.cancel=threading.Event();self.thread=None
        self.state=dict(status='idle',phase='idle',released=False,insertion=False)
        self.token=None;self.motion=None;self.perception=None
    def snapshot(self):
        with self.lock:
            result=dict(self.state)
            if hasattr(self,'timeline'):result.update(self.timeline.snapshot())
            return result
    def save(self,**kw):
        with self.lock:
            self.state.update(kw)
            if hasattr(self,'run'):atomic_json(self.run/'receipt.json',self.state)
    def action(self,data):
        if data==dict(op='cancel'):
            self.cancel.set()
            if self.motion:self.motion.hold()
            return self.snapshot()
        if data not in (dict(op='start'),dict(op='place'),dict(op='place_raised'),dict(op='place_rotate_no_table'),dict(op='place_no_table'),dict(op='place_recover_no_table'),dict(op='place_up_no_table'),dict(op='auto_full'),dict(op='auto_insert_160',visual_chassis_handoff='locked_admitted'),dict(op='d435_only')):raise ValueError('Only fixed task operations accepted')
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('Leveling already active')
            s=self.ctrl.snapshot()
            if s['motion_reserved'] or s['competition_reserved'] or s['chassis_active']:raise ValueError('Other control active')
            prefix='automatic-placement-' if data['op'] in ('auto_full','auto_insert_160') else 'fast-level-'
            self.cancel=threading.Event();self.run=WORK/'reports'/(prefix+time.strftime('%Y%m%d-%H%M%S'));self.run.mkdir()
            self.mode=data['op'];self.auto_full=data['op']=='auto_full';self.auto_insert_160=data['op']=='auto_insert_160';self.d435_only=data['op']=='d435_only';self.navigation_handoff=data.get('visual_chassis_handoff')
            self.prepared_only=data['op'] in ('place','place_raised','place_rotate_no_table','place_no_table','place_recover_no_table','place_up_no_table','auto_full','auto_insert_160');self.raise_before_place=data['op']=='place_raised';self.rotate_before_place=data['op']=='place_rotate_no_table';self.recover_before_place=data['op']=='place_recover_no_table';self.small_up_before_place=data['op']=='place_up_no_table';self.skip_table=data['op'] in ('place_rotate_no_table','place_no_table','place_recover_no_table','place_up_no_table','auto_full','auto_insert_160','d435_only');self.outside_margin_mm=.1 if self.small_up_before_place else .5;self.inserting=False
            self.save(table_checked=not self.skip_table)
            self.save(status='running',phase='entry',reason='',released=False,insertion=False,mode=self.mode,navigation_handoff=self.navigation_handoff)
            self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
            return self.snapshot()
    def guard(self,s=None,now=None):
        s=self.ctrl.snapshot() if s is None else s
        if self.cancel.is_set() or not self.ctrl.motion_valid(self.token):raise ValueError('Task stopped')
        if any(not s['arms'][side]['fresh'] for side in ('left','right')) or not s['body']['fresh'] or s['chassis_active']:raise ValueError('Feedback stale or chassis moved')
        if np.max(np.abs(np.array(s['arms']['left']['joints'])-self.left_q))>np.radians(.5):raise ValueError('Left arm changed')
        driver=self.ctrl.adapter.trajectory_status('right')
        if not driver or driver['driver_instance']!=self.driver or not driver['ready'] or not driver['enabled']:raise ValueError('Arm driver changed')
        if self.initializing:
            require_stationary_pose(s,self.initial_q,self.height,driver['command_epoch'],self.initial_epoch)
        if not getattr(self,'released',False):
            require_gripper_retained(s['arms']['right']['gripper'],self.raw)
        if self.motion.active!='lift' and abs(s['body']['height']-self.height)>.05:raise ValueError('Unexpected lift change')
        for camera in required_cameras(self.mode):
            m=json.loads((ROOT/'web/public/camera-live'/(camera+'.json')).read_text())
            if self.initializing and camera=='d435':continue
            if not camera_ready(m,time.time()):raise ValueError(camera+' camera stale')
        if not self.d435_only and not self.auto_insert_160 and not getattr(self,'locked_inside',False) and not getattr(self,'released',False):
            self.monitor.check();self.read_grip(s)
            with self.grip_lock:evidence=self.grip_evidence
            if not self.initializing:require_grip(evidence)
        if any(abs(s['body'][k]-v)>.02 for k,v in self.body_fixed.items()):raise ValueError('Camera/body configuration changed')
        if self.inserting:
            T=self.ctrl.models['right'].wrist_transform(s['arms']['right']['joints'],self.height)@self.insertion_tcp
            fit=evaluate_shallow_fit(T,self.window,self.slot,self.battery,self.contact,depth_mm=7.)
            if ((aperture_relevant(fit['leading_standoff_mm']) and fit['nominal_clearance_mm']<.5)
                    or fit['leading_standoff_mm']< -7. or abs(T[2,3]-self.fixed_z)>.005):
                raise ValueError('Measured shallow insertion aperture/Z bound exceeded')
        if self.motion.active=='arm' and getattr(self,'scene',None) and not getattr(self,'returning',False) and time.monotonic()-self.geometry_tick>.08:
            self.geometry_tick=time.monotonic();window,table,tcp=self.scene;q=s['arms']['right']['joints']
            if not self.ctrl.models['right'].check(q,q,self.height)['clear']:raise ValueError('Measured arm/base clearance exceeded')
            if getattr(self,'full_inserting',False):
                if not self.ctrl.models['right'].check(q,q,self.height)['clear']:raise ValueError('Measured arm/base clearance exceeded')
            elif self.inserting:
                from held_insertion_sweep import _insert_sweep
                _insert_sweep(self.ctrl,s,[q,q],tcp,self.battery,self.contact,window,self.slot,table,table_margin_mm=.5,measured=True,skip_table=self.skip_table)
            else:
                check_outside_sweep(self.ctrl.models['right'],self.ctrl.models['left'],s['arms']['left']['joints'],self.height,
                    [q,q],tcp,self.battery|dict(contact_mm=self.contact.tolist()),window,table,cabinet_margin_mm=self.outside_margin_mm,_table_margin_mm=.5,skip_table=self.skip_table)
        return True
    def read_grip(self,s=None):
        s=self.ctrl.snapshot() if s is None else s
        m=json.loads((ROOT/'web/public/camera-live/d405.json').read_text())
        if not camera_ready(m,time.time()):raise ValueError('d405 camera stale')
        with self.grip_lock:
            if m['captured_at']>self.grip_stamp:
                bgr=cv2.imread(str(ROOT/'web/public/camera-live'/m['raw_image_file']))
                if bgr is None:raise ValueError('Held-battery image unavailable')
                rgb=bgr[:,:,::-1]
                if self.grip.reference is None:self.grip.bind(rgb,m['captured_at'],self.raw,now=time.time())
                evidence=self.grip.update(rgb,m['captured_at'],s['arms']['right']['gripper'],now=time.time());self.grip_stamp=m['captured_at']
                if evidence['status']=='slip_suspected':raise ValueError('Battery relative grip changed')
                self.grip_evidence=evidence
    def local_sweep(self,a,b,s):
        model=self.ctrl.models['right'];left=self.ctrl.models['left'];h=s['body']['height']
        if not model.check(a,b,h)['clear']:raise ValueError('Arm/base CAD sweep rejected')
        local=payload_in_tool(self.bounds,self.attachment)
        left_boxes=_boxes(left,s['arms']['left']['joints'],h,_root(left,h));root=_root(model,h)
        radius=model.reach+np.linalg.norm(self.tcp[:3,3])+np.linalg.norm(local,axis=1).max()
        count=max(2,int(np.ceil(radius*np.abs(np.asarray(b)-a).sum()/.002))+1)
        bound=radius*np.abs(np.asarray(b)-a).sum()/(2*(count-1))
        minimum=float('inf')
        for i,q in enumerate(np.linspace(a,b,count)):
            if i%16==0:self.guard()
            T=model.wrist_transform(q,h)@self.tcp;p=local@T[:3,:3].T+T[:3,3];parts=_boxes(model,q,h,root)
            gaps=[_gap(p,np.array(model.column)),*[_gap(p,x) for x in parts[:6]],*[_gap(x,y) for x in [p,*parts] for y in left_boxes]]
            minimum=min(minimum,*gaps)
            if minimum-bound<.02 or min(x[:,2].min() for x in [p,*parts])-bound<.05:raise ValueError('Held payload local clearance insufficient')
            if (not self.skip_table and max(x[:,0].max() for x in [p,*parts])+bound>self.front_x-.02):
                raise ValueError('Fixed entry/lift would approach the observed table')
        return dict(minimum_mm=(minimum-bound)*1000,samples=count,scope='observed_payload_all_arm_base_left_floor_table_front')
    def arm_to(self,target,*,tolerance_deg=2.):
        s=self.ctrl.snapshot();q=np.array(s['arms']['right']['joints']);target=np.array(target,float)
        proof=self.local_sweep(q,target,s);self.save(local_path=proof)
        timing=TimedJointPath([q,target],self.ctrl.models['right'].reach+np.linalg.norm(self.tcp[:3,3]),
            tcp_position=lambda a:(self.ctrl.models['right'].wrist_transform(a,self.height)@self.tcp)[:3,3],**self.limits)
        plan=dict(interpolation='segment_quintic_scalar',joint_waypoints_rad=timing.q.tolist(),time_from_start_s=timing.times.tolist())
        d=self.ctrl.adapter.trajectory_status('right');self.save(planned_seconds=float(timing.times[-1]))
        self.motion.arm([plan],lambda:self.guard(),expected_start=dict(joints=q.tolist(),height=self.height),
            expected_driver=dict(driver_instance=self.driver,command_epoch=d['command_epoch']))
        require_joint_arrival(self.ctrl.snapshot()['arms']['right']['joints'],target,tolerance_deg)
        if hasattr(self,'reached_route'):self.reached_route.extend(timing.q[1:].tolist())
    def capture(self):
        self.guard()
        opener=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()));base='http://127.0.0.1:9983'
        opener.open(base+'/',timeout=3).read()
        request=urllib.request.Request(base+'/api/grasp-scene',json.dumps(dict(camera='d435')).encode(),{'Origin':base,'Content-Type':'application/json'})
        return guarded_read(lambda:json.load(opener.open(request,timeout=25)),self.guard,timeout_s=27.)
    def ensure_model(self):
        run=WORK/'reports/d435-visual-level-active';owner=run/'model-owner.json'
        run.mkdir(parents=True,exist_ok=True)
        try:
            info=json.loads(owner.read_text());args=(Path('/proc')/str(info['pid'])/'cmdline').read_bytes().decode().split('\0')
            if 'agent.perception_worker' in args and str(run/'perception') in args:return
        except (OSError,ValueError,KeyError):pass
        from agent.perception_session import PerceptionSession
        self.perception=PerceptionSession(run)
        warm=guarded_read(lambda:self.perception.prewarm(lambda:self.cancel.is_set()),self.guard,timeout_s=90.)
        atomic_json(owner,dict(pid=self.perception.child.pid,directory=str(self.perception.directory),warmup=warm,issued_at=time.time()))
    def pose(self):
        if self.motion.active is not None:raise ValueError('Visual planning requires a held arm')
        captured=self.capture()
        with np.load(ROOT/'web/hardware/runtime/grasp-head/frame.npz',allow_pickle=False) as f:
            capture_q=f['joints'].copy();capture_h=float(f['lift_rad'])
            if str(f['id'])!=captured['id']:raise ValueError('Captured body frame changed')
        captured_epoch=self.ctrl.adapter.trajectory_status('right')['command_epoch']
        captured_tool=self.ctrl.models['right'].wrist_transform(capture_q,capture_h)@self.tcp
        def exposure_check():
            s=self.ctrl.snapshot();current_tool=self.ctrl.models['right'].wrist_transform(s['arms']['right']['joints'],s['body']['height'])@self.tcp
            return capture_motion_bound(captured_tool,current_tool,captured_epoch,self.ctrl.adapter.trajectory_status('right')['command_epoch'])
        if hasattr(self,'attachment'):
            expected=self.ctrl.models['right'].wrist_transform(capture_q,capture_h)@self.tcp@self.attachment
            atomic_json(WORK/'reports/d435-visual-level-active/pose-seed.json',dict(frame_id=captured['id'],base_from_object=expected.tolist()))
        command=d435_estimator_command()
        child=subprocess.Popen(command,cwd=ROOT,stdout=open(self.run/'vision.log','ab'),stderr=subprocess.STDOUT)
        deadline=time.monotonic()+25
        while child.poll() is None:
            self.guard()
            exposure_check()
            if time.monotonic()>deadline:child.terminate();raise ValueError('Visual pose deadline')
            time.sleep(.08)
        if child.returncode:raise ValueError('Body vision worker failed')
        path=Path((WORK/'reports/d435-visual-level-active/latest-pose-path.txt').read_text());report=json.loads(path.read_text())
        with np.load(path.parent/'frame.npz',allow_pickle=False) as f:frame={k:f[k] for k in f.files}
        if str(frame['id'])!=captured['id']:raise ValueError('Visual result is not this exposure')
        s=self.ctrl.snapshot()
        motion_bound=exposure_check()
        observation=d435_observation(frame,report,self.mesh_sha)
        self.pose_frame=frame
        tool=self.ctrl.models['right'].wrist_transform(frame['joints'],float(frame['lift_rad']))@self.tcp
        self.attachment=np.linalg.inv(tool)@np.array(observation['T_base_from_object'])
        observation['source_T_base_from_object']=observation['T_base_from_object']
        observation['T_base_from_object']=(self.ctrl.models['right'].wrist_transform(s['arms']['right']['joints'],s['body']['height'])@self.tcp@self.attachment).tolist()
        observation['pose_propagation']='paired visual exposure plus measured joint settling delta'
        self.save(visual_frame_id=observation['id'],pose_age_s=time.time()-observation['image_time'],vision_method='D435 FoundationPose battery CAD',capture_motion_bound_mm=motion_bound)
        return observation
    def cabinet(self):
        def observe():
            captured=self.capture()
            with np.load(ROOT/'web/hardware/runtime/grasp-head/frame.npz',allow_pickle=False) as f:frame={k:f[k] for k in f.files}
            if str(frame['id'])!=captured['id']:raise ValueError('Cabinet exposure identity changed')
            window=localize(frame,self.slot)
            table=table_front_x(frame,details=True)
            self.save(cabinet_frame_id=captured['id'],cabinet_size_mm=window['measured_size_mm'])
            return window,table
        return retry_cabinet_observation(observe)
    def cabinet_path(self,path,tcp,window,table,*,inside=False,sample_bound_mm=1.,locked_scene=False):
        from held_insertion_sweep import _insert_sweep
        s=self.ctrl.snapshot();points=np.array(path['joint_waypoints_rad']);q=np.array(s['arms']['right']['joints'])
        nominal_end=points[-1].copy()
        if not inside and np.max(np.abs(self.bias))>1e-8:
            points=compensate_path(points,self.bias,self.ctrl.config['arms']['right']['joint_limits_deg'])
            path['timing']=TimedJointPath(points,self.ctrl.models['right'].reach+np.linalg.norm(tcp[:3,3]),
                tcp_position=lambda a:(self.ctrl.models['right'].wrist_transform(a,self.height)@tcp)[:3,3],**self.limits).receipt()
            for a,b in zip(points,points[1:]):
                if not self.ctrl.models['right'].check(a,b,self.height)['clear']:raise ValueError('Compensated arm/base path rejected')
        if np.max(np.abs(points[0]-q))>np.radians(.25):raise ValueError('Cabinet path start mismatch')
        def sweep(w,t):
            if inside:return _insert_sweep(self.ctrl,self.ctrl.snapshot(),points,tcp,self.battery,self.contact,w,self.slot,t,table_margin_mm=.5,skip_table=self.skip_table)
            return check_outside_sweep(self.ctrl.models['right'],self.ctrl.models['left'],s['arms']['left']['joints'],
                self.height,points,tcp,self.battery|dict(contact_mm=self.contact.tolist()),w,t,cabinet_margin_mm=self.outside_margin_mm,_table_margin_mm=.5,sample_bound_mm=sample_bound_mm,skip_table=self.skip_table)
        proof=sweep(window,table)
        if inside:
            # Lock the checked static rim. A new noisy depth fit is not physical
            # marker motion; fresh color must still agree within checked slack.
            fresh,fresh_table=window,table;nominal_slack=scene_allowance(proof,inside=True)
            # Operator-authorized, attended prototype trial: permit the stated
            # 5mm image-estimation variation. This is not a precision certificate
            # or an expansion of the checked physical/model aperture.
            allowance=5.;drift=float('inf')
            for _ in range(3):
                captured=self.capture()
                with np.load(ROOT/'web/hardware/runtime/grasp-head/frame.npz',allow_pickle=False) as f:frame={k:f[k] for k in f.files}
                if str(frame['id'])!=captured['id']:raise ValueError('Final marker exposure changed')
                d=detect(frame['rgb'])
                if d['status']!='complete':raise ValueError('Fresh observed rim edges unavailable')
                drift=marker_drift_mm(window,frame,d['inner_quad_px'])
                if drift<=allowance and time.time()-float(frame['image_time'])<1.:break
            else:raise ValueError('Fresh marker variation exceeds authorized 5mm trial tolerance')
            self.save(scene_delta_mm=drift,nominal_aperture_slack_mm=nominal_slack,supervised_marker_tolerance_mm=allowance,
                scene_validation='attended_5mm_trial_on_locked_depth_plane')
        elif locked_scene:
            fresh,fresh_table=window,table
        else:
            fresh,fresh_table,proof=preflight_outside(self.cabinet,sweep,self.guard,self.save)
        if locked_scene:proof=sweep(fresh,fresh_table)
        self.scene=(fresh,fresh_table,tcp);self.geometry_tick=0.
        if inside:
            self.window=fresh;self.insertion_tcp=tcp;self.fixed_z=float((self.ctrl.models['right'].wrist_transform(q,self.height)@tcp)[2,3]);self.inserting=True
        self.save(path_check=proof,planned_seconds=path['timing']['time_from_start_s'][-1])
        d=self.ctrl.adapter.trajectory_status('right')
        commands=points
        if inside:
            if self.last_reference is None or d['command_epoch']!=self.last_reference_epoch:raise ValueError('Loaded reference continuity unavailable')
            commands=retained_reference_path(points,self.last_reference,q)
            limits=np.radians(self.ctrl.config['arms']['right']['joint_limits_deg'])
            if np.any(commands<limits[:,0]) or np.any(commands>limits[:,1]):raise ValueError('Loaded command references exceed joint limits')
            self.save(retained_reference_offset_deg=np.degrees(self.last_reference-q).tolist())
        name='insert-plan.json' if inside else 'alignment-%d-plan.json'%self.snapshot().get('alignment_attempt',0)
        atomic_json(self.run/name,dict(nominal_joint_waypoints_rad=points.tolist(),command_joint_waypoints_rad=commands.tolist(),
            time_from_start_s=path['timing']['time_from_start_s'],window=fresh,table=fresh_table,effective_tcp=tcp.tolist(),
            expected_start=dict(joints=q.tolist(),height=self.height),driver=d,collision_proof=proof,gripper_action='retain'))
        dispatch=self.ctrl.snapshot();dispatch_q=np.asarray(dispatch['arms']['right']['joints'],float)
        if np.max(np.abs(dispatch_q-q))>np.radians(.25):raise ValueError('Cabinet path start changed before dispatch')
        self.motion.arm(split_plan(commands.tolist(),path['timing']['time_from_start_s']),lambda:self.guard(),
            expected_start=dict(joints=dispatch_q.tolist(),height=dispatch['body']['height']),expected_driver=dict(driver_instance=self.driver,command_epoch=d['command_epoch']))
        require_joint_arrival(self.ctrl.snapshot()['arms']['right']['joints'],points[-1],2.)
        if hasattr(self,'reached_route'):self.reached_route.extend(np.asarray(self.ctrl.snapshot()['arms']['right']['joints'],float)[None,:].tolist())
        if not inside:
            actual=np.array(self.ctrl.snapshot()['arms']['right']['joints']);self.bias=updated_bias(self.bias,nominal_end,actual)
            self.last_reference=commands[-1].copy();self.last_reference_epoch=self.ctrl.adapter.trajectory_status('right')['command_epoch']
            self.save(arrival_error_deg=np.degrees(nominal_end-actual).tolist(),compensation_deg=np.degrees(self.bias).tolist())

    def _timed(self,phase,call,*,planned_duration_s=None):
        self.timeline.start(phase,planned_duration_s=planned_duration_s);self.save(phase=phase)
        try:
            value=call();self.timeline.finish('ok');self.save(**self.timeline.snapshot());return value
        except Exception as error:
            self.timeline.finish('failed',str(error));self.save(**self.timeline.snapshot());raise

    def capture_pair(self,timeout_s=47.):
        self.guard();base='http://127.0.0.1:9987'
        request=urllib.request.Request(base+'/api/capture',data=b'',headers={'Origin':base})
        timeout=float(timeout_s)
        if not np.isfinite(timeout) or not 0<timeout<=47.:raise ValueError('Invalid D405 capture timeout')
        result=guarded_read(lambda:json.load(urllib.request.urlopen(request,timeout=max(1.,timeout-.5))),self.guard,timeout_s=timeout)
        folder=Path(result['capture_directory'])
        with np.load(folder/'d405.npz',allow_pickle=False) as data:frame={k:data[k] for k in data.files}
        return folder,frame

    def auto_d405_horizontal_trim(self,model,tcp,preobservation):
        total=0.;after_image_time=None;last=None
        expected=hashlib.sha256(D405_CALIBRATION.read_bytes()).hexdigest()
        output=ROOT/'web/public/camera-overlay'
        for attempt in range(1,self.route.d405_attempts+1):
            frame=None
            try:
                _,frame=self.capture_pair(self.route.d405_capture_timeout_s)
                frame_q=np.asarray(frame.get('joints'),float);frame_lift=float(np.asarray(frame.get('lift_rad')).item())
                if frame_q.shape!=(6,) or not np.isfinite(np.r_[frame_q,frame_lift]).all():
                    raise ValueError('D405 paired tool pose unavailable')
                frame_tool=model.wrist_transform(frame_q,frame_lift)@tcp
                bounds=np.asarray(self.battery['bounds_mm'],float)
                front_object=np.array([[bounds[0,0],0.,bounds[1,2]],[bounds[1,0],0.,bounds[1,2]]])
                front_tool=(front_object-np.asarray(self.contact,float))@OBJECT_TO_TOOL.T/1000.
                expected_battery_front=front_tool@frame_tool[:3,:3].T+frame_tool[:3,3]
                measurement=estimate_horizontal_alignment(frame,
                    target_offset_mm=self.route.d405_target_offset_mm,
                    correction_axis_base=(0.,1.,0.),max_step_mm=self.route.d405_step_limit_mm,
                    expected_calibration_sha256=expected,
                    expected_battery_front_base_m=expected_battery_front,after_image_time=after_image_time)
            except (OSError,ValueError,TimeoutError) as error:
                reason=str(error);result=dict(converged=False,degraded=True,attempt=attempt,
                    cumulative_correction_mm=total,reason=reason)
                if frame is not None:
                    unavailable=dict(status='unavailable',reason=reason,
                        frame_id=str(np.asarray(frame.get('id','')).item()),
                        image_time=float(np.asarray(frame.get('image_time',0.)).item()),
                        calibration_sha256=str(np.asarray(frame.get('calibration_sha256','')).item()))
                    write_horizontal_overlay(frame,unavailable,output)
                self.save(d405_attempt=attempt,d405_raw_error_mm=None,d405_step_mm=None,
                    d405_cumulative_correction_mm=total,d405_converged=False,d405_degraded=True,d405_reason=reason)
                return result
            raw=float(measurement['raw_error_mm']);step=float(measurement['step_mm'])
            converged=abs(raw)<self.route.d405_convergence_mm
            last=measurement|dict(attempt=attempt,cumulative_correction_mm=total,converged=converged)
            write_horizontal_overlay(frame,last,output)
            self.save(d405_attempt=attempt,d405_raw_error_mm=raw,d405_step_mm=step,
                d405_cumulative_correction_mm=total,d405_converged=converged,d405_degraded=False,
                d405_reason='',d405_frame_id=measurement['frame_id'],d405_image_time=measurement['image_time'])
            if converged:
                return dict(converged=True,degraded=False,attempt=attempt,
                    cumulative_correction_mm=total,reason='',measurement=last)
            if attempt==self.route.d405_attempts:
                reason='D405 horizontal alignment exhausted observations'
                self.save(d405_degraded=True,d405_reason=reason)
                return dict(converged=False,degraded=True,attempt=attempt,
                    cumulative_correction_mm=total,reason=reason,measurement=last)
            if abs(total+step)>self.route.d405_total_limit_mm:
                reason='D405 cumulative horizontal correction exceeds taught bound'
                self.save(d405_degraded=True,d405_reason=reason)
                return dict(converged=False,degraded=True,attempt=attempt,
                    cumulative_correction_mm=total,reason=reason,measurement=last)
            s=self.ctrl.snapshot();q=np.asarray(s['arms']['right']['joints'],float)
            current=model.wrist_transform(q,self.height)@tcp
            target=d405_horizontal_correction_target(current,measurement,limit_mm=self.route.d405_step_limit_mm)
            path=continuous_correction(plan_correction,model,q,self.height,tcp,target,
                self.ctrl.config['arms']['right']['joint_limits_deg'],timing_limits=self.limits)
            authorized=total+step
            self.execute_d405_lateral_path(path,model,tcp,preobservation,
                y_start_mm=total,y_end_mm=authorized)
            end_q=np.asarray(self.ctrl.snapshot()['arms']['right']['joints'],float)
            endpoint=model.wrist_transform(end_q,self.height)@tcp
            endpoint_check=validate_lateral_endpoint(preobservation,endpoint,end_q,self.route,
                y_bounds_mm=(authorized,authorized))
            total=float(endpoint_check['delta_y_mm']);after_image_time=float(measurement['image_time'])
            self.save(d405_cumulative_correction_mm=total,d405_endpoint=endpoint_check)
        raise AssertionError('Unreachable D405 correction state')

    def execute_d405_lateral_path(self,path,model,tcp,preobservation,*,y_start_mm=None,y_end_mm=None):
        points=np.asarray(path.get('joint_waypoints_rad'),float)
        times=np.asarray(path.get('timing',{}).get('time_from_start_s'),float)
        q=np.asarray(self.ctrl.snapshot()['arms']['right']['joints'],float)
        if (points.ndim!=2 or points.shape[1:]!=(6,) or len(points)<2 or times.shape!=(len(points),)
                or not np.isfinite(np.r_[points.ravel(),times]).all() or times[0]!=0.
                or np.any(np.diff(times)<=0) or np.max(np.abs(points[0]-q))>np.radians(.25)):
            raise ValueError('Invalid D405 lateral path')
        y_bounds=None if y_start_mm is None or y_end_mm is None else (float(y_start_mm),float(y_end_mm))
        checks=[];sample_count=0;max_position_bound=0.;max_rotation_bound=0.
        for point in points:
            tool=model.wrist_transform(point,self.height)@tcp
            checks.append(validate_lateral_endpoint(preobservation,tool,point,self.route,y_bounds_mm=y_bounds))
        for a,b in zip(points,points[1:]):
            if not model.check(a,b,self.height)['clear']:
                raise ValueError('D405 lateral path approaches central column')
            joint_length=float(np.abs(b-a).sum());radius=float(model.reach+np.linalg.norm(tcp[:3,3]))
            samples=max(2,int(np.ceil(radius*joint_length*1000./(.1))),
                int(np.ceil(np.degrees(joint_length)/.04)))
            position_bound=radius*joint_length*1000./(2*samples)
            rotation_bound=np.degrees(joint_length)/(2*samples)
            max_position_bound=max(max_position_bound,position_bound)
            max_rotation_bound=max(max_rotation_bound,rotation_bound)
            for t in np.linspace(0.,1.,samples+1)[1:-1]:
                point=a+t*(b-a);tool=model.wrist_transform(point,self.height)@tcp
                validate_lateral_endpoint(preobservation,tool,point,self.route,y_bounds_mm=y_bounds,
                    position_margin_mm=position_bound,rotation_margin_deg=rotation_bound);sample_count+=1
        driver=self.ctrl.adapter.trajectory_status('right')
        self.motion.arm(split_plan(points.tolist(),times.tolist()),lambda:self.guard(),
            expected_start=dict(joints=q.tolist(),height=self.height),
            expected_driver=dict(driver_instance=self.driver,command_epoch=driver['command_epoch']))
        require_joint_arrival(self.ctrl.snapshot()['arms']['right']['joints'],points[-1],.5)
        self.reached_route.extend(points[1:].tolist())
        self.save(d405_planned_endpoint=checks[-1],d405_interpolation_samples=sample_count,
            d405_interpolation_position_bound_mm=max_position_bound,
            d405_interpolation_rotation_bound_deg=max_rotation_bound,
            d405_planned_duration_s=float(times[-1]))

    def auto_d405_trim(self,model,tcp,window,table,nominal_z):
        total=0.;measurement=None;aligned=False
        for attempt in range(1,self.route.d405_attempts+1):
            try:
                _,frame=self.capture_pair();measurement=estimate_vertical_alignment(frame,max_step_mm=min(3.,self.route.d405_step_limit_mm))
            except ValueError as error:
                self.save(d405_attempt=attempt,d405_reason=str(error));continue
            step=float(measurement['step_mm'])
            if abs(step)<.75:aligned=True;break
            if abs(total+step)>self.route.d405_total_limit_mm:raise ValueError('D405 cumulative vertical correction exceeds taught bound')
            s=self.ctrl.snapshot();q=np.array(s['arms']['right']['joints']);T=model.wrist_transform(q,self.height)@tcp
            target=d405_correction_target(T,measurement,limit_mm=self.route.d405_step_limit_mm)
            path=continuous_correction(plan_correction,model,q,self.height,tcp,target,self.ctrl.config['arms']['right']['joint_limits_deg'],timing_limits=self.limits)
            self.cabinet_path(path,tcp,window,table,sample_bound_mm=.1,locked_scene=True);total+=step
        if measurement is None:
            raise ValueError('D405 vertical evidence unavailable; clamp retained')
        if not aligned:raise ValueError('D405 vertical alignment did not converge; clamp retained')
        self.save(vertical_correction_mm=total,d405_measurement=measurement)
        return total

    def execute_locked_axis(self,model,tcp,axis,distance_mm,*,fixed_z=True):
        s=self.ctrl.snapshot();q=np.array(s['arms']['right']['joints']);T=model.wrist_transform(q,self.height)@tcp
        end=T[:3,3]*1000+np.asarray(axis,float)*float(distance_mm)
        timing=(dict(self.limits) if self.auto_insert_160 else
                self.limits|dict(tcp_speed_mm_s=8.,tcp_acceleration_mm_s2=4.))
        path=plan_insert(model,q,self.height,T[:3,3]*1000,end,T[:3,:3],tcp,self.ctrl.config['arms']['right']['joint_limits_deg'],
            timing_limits=timing,fixed_z_axis=axis if fixed_z else None,max_step_mm=2.)
        points=np.asarray(path['joint_waypoints_rad']);
        for a,b in zip(points,points[1:]):
            if not model.check(a,b,self.height)['clear']:raise ValueError('Full insertion arm/base path rejected')
        self.timeline.plan(float(path['timing']['time_from_start_s'][-1]))
        self.full_inserting=True;self.locked_inside=True;self.fixed_z=float(T[2,3]);d=self.ctrl.adapter.trajectory_status('right')
        self.motion.arm(split_plan(points.tolist(),path['timing']['time_from_start_s']),lambda:self.guard(),
            expected_start=dict(joints=q.tolist(),height=self.height),expected_driver=dict(driver_instance=self.driver,command_epoch=d['command_epoch']))
        require_joint_arrival(self.ctrl.snapshot()['arms']['right']['joints'],points[-1],2.)
        self.reached_route.extend(points[1:].tolist());self.save(depth_mm=float(distance_mm),planned_duration_s=path['timing']['time_from_start_s'][-1])

    def d435_outer_align(self,model,limits):
        def observe(label):
            self.pose();window,table=self.cabinet();q=np.array(self.ctrl.snapshot()['arms']['right']['joints'])
            tcp=effective_tcp(self.tcp,self.attachment,self.contact);T=model.wrist_transform(q,self.height)@tcp
            fit=evaluate_shallow_fit(T,window,self.slot,self.battery,self.contact)
            error=outer_alignment_error(fit,window,self.slot,standoff_range_mm=(55.,65.))
            self.save(d435_stage=label,d435_outer_error=error,aperture_fit=fit)
            return window,table,tcp,T,error
        window,table,tcp,T,error=self._timed('d435_initial_localize',lambda:observe('initial'),planned_duration_s=55.)
        attempt=0
        while not error['converged']:
            attempt+=1
            def move():
                nonlocal window,table,tcp,T
                q=np.array(self.ctrl.snapshot()['arms']['right']['joints'])
                target=d435_incremental_target(T,error,window)
                path=continuous_correction(plan_correction,model,q,self.height,tcp,target,limits,timing_limits=self.limits)
                self.timeline.plan(float(path['timing']['time_from_start_s'][-1]));self.cabinet_path(path,tcp,window,table)
            self._timed('d435_outer_move_%d'%attempt,move)
            window,table,tcp,T,error=self._timed('d435_outer_verify_%d'%attempt,lambda:observe('verify_%d'%attempt),planned_duration_s=55.)
        def preposition():
            nonlocal T
            q=np.array(self.ctrl.snapshot()['arms']['right']['joints'])
            p=horizontal_plan(window,self.slot,self.battery,tip_point_object_mm=self.contact,preclearance_mm=60.,max_insertion_depth_mm=5.)
            target=np.eye(4);target[:3,:3]=p['R_base_from_tool'];target[:3,3]=np.asarray(p['tcp_pre_base_mm'])/1000.
            path=continuous_correction(plan_correction,model,q,self.height,tcp,target,limits,timing_limits=self.limits)
            self.timeline.plan(float(path['timing']['time_from_start_s'][-1]));self.cabinet_path(path,tcp,window,table)
            T=target
        self._timed('d435_preposition',preposition)
        return window,table,tcp,T[2,3]

    def release_and_return(self):
        target=float(self.ctrl.config['arms']['right']['gripper_open_raw'])
        reply=self.ctrl.submit(dict(type='gripper',arm='right',position=target),self.token,
            preflight=lambda s,n:self.ctrl.motion_valid(self.token) and s['arms']['right']['fresh'])
        if reply.get('ok') is not True:raise ValueError('Gripper release command rejected')
        self.released=True
        deadline=time.monotonic()+6.
        while time.monotonic()<deadline:
            s=self.ctrl.snapshot()
            if s['arms']['right']['fresh'] and abs(s['arms']['right']['gripper']-target)<=.35:break
            time.sleep(.08)
        else:raise ValueError('Gripper open feedback not reached')
        self.save(released=True,release_gripper=s['arms']['right']['gripper'])
        points=np.asarray(reverse_reached_route(self.reached_route),float);self.returning=True;self.full_inserting=False;self.inserting=False
        receipt=TimedJointPath(points,self.ctrl.models['right'].reach+np.linalg.norm(self.tcp[:3,3]),
            tcp_position=lambda a:(self.ctrl.models['right'].wrist_transform(a,self.height)@self.tcp)[:3,3],**self.limits).receipt()
        self.timeline.plan(float(receipt['time_from_start_s'][-1])+6.)
        d=self.ctrl.adapter.trajectory_status('right');q=self.ctrl.snapshot()['arms']['right']['joints']
        self.motion.arm(split_plan(points.tolist(),receipt['time_from_start_s']),lambda:self.guard(),
            expected_start=dict(joints=q,height=self.height),expected_driver=dict(driver_instance=self.driver,command_epoch=d['command_epoch']))
        require_joint_arrival(self.ctrl.snapshot()['arms']['right']['joints'],self.initial_q,2.)

    def close_auto_gripper(self):
        target=float(self.ctrl.config['arms']['right']['gripper_closed_raw'])
        reply=self.ctrl.submit(dict(type='gripper',arm='right',position=target),self.token,
            preflight=lambda s,n:self.ctrl.motion_valid(self.token) and s['arms']['right']['fresh'])
        if reply.get('ok') is not True:raise ValueError('Gripper close command rejected')
        deadline=time.monotonic()+3.;stable_since=None;previous=None
        while time.monotonic()<deadline:
            s=self.ctrl.snapshot()
            value=float(s['arms']['right']['gripper']);now=time.monotonic()
            try:loaded=s['arms']['right']['fresh'] and require_loaded_gripper(value)
            except ValueError:loaded=False
            if loaded:
                if previous is None or abs(value-previous)>.03:stable_since=now
                elif stable_since is not None and now-stable_since>=.35:break
            else:stable_since=None
            previous=value
            time.sleep(.05)
        else:raise ValueError('Loaded battery clamp feedback not reached')
        require_loaded_gripper(value)
        self.raw=float(s['arms']['right']['gripper']);self.save(clamped=True,clamp_gripper=self.raw)

    def auto_insert_160_place(self):
        self.battery=json.loads((ROOT/'config/battery-envelope.json').read_text())
        self.slot=opening(json.loads((ROOT/'config/cabinet-profiles.json').read_text()),'demo')
        model=self.ctrl.models['right'];limits=self.ctrl.config['arms']['right']['joint_limits_deg']
        tcp=self.tcp
        policy=load_chassis_policy(ROOT/self.route.visual_chassis_policy)
        self.chassis_handoff=load_locked_handoff(ROOT/self.route.visual_chassis_handoff,
            now=time.time(),max_age_s=policy['handoff_max_age_s'])
        self.save(chassis_handoff=self.chassis_handoff,entry_source='visual_chassis+d405_preobservation')
        preobservation=load_preobservation_waypoint(ROOT/self.route.d405_preobservation_waypoint,self.height)
        contact=np.asarray(self.contact,float);self.attachment=np.eye(4);self.attachment[:3,:3]=OBJECT_TO_TOOL
        self.attachment[:3,3]=-(OBJECT_TO_TOOL@contact)/1000.
        target_q=np.asarray(preobservation['joints_rad'],float)
        planned_wrist=model.wrist_transform(target_q,self.height);planned_tool=planned_wrist@tcp
        tcp_sha256=hashlib.sha256((ROOT/'config/right-grip-tcp.json').read_bytes()).hexdigest()
        planned_lock=validate_preobservation_waypoint(preobservation,planned_wrist,planned_tool,target_q,limits,self.route,tcp_sha256)
        self.save(planned_preobservation_lock=planned_lock)
        self._timed('d405_preobservation',lambda:self.arm_to(target_q))
        q=np.asarray(self.ctrl.snapshot()['arms']['right']['joints'],float);T=model.wrist_transform(q,self.height)@tcp
        self.preobservation_lock=validate_preobservation_waypoint(preobservation,
            model.wrist_transform(q,self.height),T,q,limits,self.route,tcp_sha256)
        self.save(preobservation_lock=self.preobservation_lock)
        self._timed('d405_horizontal_trim',lambda:self.auto_d405_horizontal_trim(model,tcp,T),
            planned_duration_s=self.route.d405_capture_timeout_s*self.route.d405_attempts)
        self._timed('full_insert',lambda:self.execute_locked_axis(model,tcp,np.array([1.,0.,0.]),160.0,fixed_z=True))
        self._timed('release_and_return',self.release_and_return)
        self.timeline.complete();self.save(status='complete',phase='complete',reason='160mm automatic placement completed and returned',insertion=True,depth_mm=160.,**self.timeline.snapshot())
        atomic_json(self.run/'timeline.json',self.timeline.snapshot());atomic_json(self.run/'executed-route.json',dict(joint_waypoints_rad=self.reached_route))

    def auto_full_place(self):
        self.route=load_taught_route(ROOT/'config/taught-placement.json');self.timeline=PhaseTimeline();self.reached_route=[self.initial_q.tolist()]
        self.battery=json.loads((ROOT/'config/battery-envelope.json').read_text())
        self.slot=opening(json.loads((ROOT/'config/cabinet-profiles.json').read_text()),'demo')
        model=self.ctrl.models['right'];limits=self.ctrl.config['arms']['right']['joint_limits_deg']
        window,table,tcp,nominal_z=self.d435_outer_align(model,limits)
        self._timed('d405_vertical_trim',lambda:self.auto_d405_trim(model,tcp,window,table,nominal_z),planned_duration_s=47.*self.route.d405_attempts)
        def mouth():
            fresh,table2=window,table;q=np.array(self.ctrl.snapshot()['arms']['right']['joints']);T=model.wrist_transform(q,self.height)@tcp
            p=horizontal_plan(fresh,self.slot,self.battery,tip_point_object_mm=self.contact,preclearance_mm=5.,max_insertion_depth_mm=15.)
            target=np.eye(4);target[:3,:3]=p['R_base_from_tool'];target[:3,3]=np.asarray(p['tcp_pre_base_mm'])/1000.;target[2,3]+=(T[2,3]-nominal_z)
            path=continuous_correction(plan_correction,model,q,self.height,tcp,target,limits,timing_limits=self.limits)
            self.timeline.plan(float(path['timing']['time_from_start_s'][-1]));self.cabinet_path(path,tcp,fresh,table2,sample_bound_mm=.1,locked_scene=True);return np.asarray(p['insertion_axis_base'],float)
        axis=self._timed('engage',mouth)
        self._timed('full_insert',lambda:self.execute_locked_axis(model,tcp,axis,self.route.full_depth_mm))
        self._timed('release_and_return',self.release_and_return)
        self.timeline.complete();self.save(status='complete',phase='complete',reason='Full taught placement completed and returned',insertion=True,**self.timeline.snapshot())
        atomic_json(self.run/'timeline.json',self.timeline.snapshot());atomic_json(self.run/'executed-route.json',dict(joint_waypoints_rad=self.reached_route))
    def place(self):
        self.battery=json.loads((ROOT/'config/battery-envelope.json').read_text())
        self.slot=opening(json.loads((ROOT/'config/cabinet-profiles.json').read_text()),'demo')
        model=self.ctrl.models['right'];limits=self.ctrl.config['arms']['right']['joint_limits_deg']
        for attempt in range(9):
            self.save(phase='cabinet_alignment',alignment_attempt=attempt)
            if attempt or not self.prepared_only:self.pose()
            window,table=self.cabinet();q=np.array(self.ctrl.snapshot()['arms']['right']['joints'])
            tcp=effective_tcp(self.tcp,self.attachment,self.contact);T=model.wrist_transform(q,self.height)@tcp
            fit=evaluate_shallow_fit(T,window,self.slot,self.battery,self.contact);self.save(aperture_fit=fit)
            if fit['leading_standoff_mm']<0:raise ValueError('Battery already beyond rim; no unverified replay')
            p=horizontal_plan(window,self.slot,self.battery,tip_point_object_mm=self.contact,
                preclearance_mm=float(np.clip(fit['leading_standoff_mm'],5.,100.)),max_insertion_depth_mm=5.)
            left_center=min(window['measured_size_mm'][0],self.slot['size_mm'][0])/4
            left_error=abs(float(np.array(fit['footprint_uv_mm'])[:,0].mean())-left_center)
            self.save(left_target_error_mm=left_error)
            if fit['qualified_fit'] and left_error<=5. and self.last_reference is not None and 1.<=fit['leading_standoff_mm']<=100.:
                break
            if attempt==8:raise ValueError('Outside alignment budget reached; battery held outside')
            goal,delta,gap=alignment_goal(fit,T,window,p)
            goal=no_retreat_target(T,goal,window['outward_normal_base'])
            if delta is not None:self.save(recenter_uv_mm=delta.tolist(),recenter_nominal_gap_mm=gap)
            else:self.save(recenter_uv_mm=None,recenter_nominal_gap_mm=None)
            path=continuous_correction(plan_correction,model,q,self.height,tcp,goal,limits,timing_limits=self.limits)
            self.cabinet_path(path,tcp,window,table)
        else:raise ValueError('Outside alignment budget reached; battery held outside')
        self.save(phase='insert',reason='Fixed-Z 5mm shallow test; clamp retained')
        start=T[:3,3]*1000;end=start+np.array(p['insertion_axis_base'])*(fit['leading_standoff_mm']+5.)
        timing=self.limits|dict(tcp_speed_mm_s=10.,tcp_acceleration_mm_s2=5.)
        for step in (2.,1.,.5,.25):
            path=plan_insert(model,q,self.height,start,end,T[:3,:3],tcp,limits,timing_limits=timing,
                fixed_z_axis=p['insertion_axis_base'],max_step_mm=step)
            if path['interpolation_position_bound_mm']<=fit['available_clearance_mm']:break
        else:raise ValueError('Insertion interpolation exceeds aperture margin')
        self.save(interpolation_bound_mm=path['interpolation_position_bound_mm'],insertion_sampling_mm=step)
        self.cabinet_path(path,tcp,window,table,inside=True)
        s=self.ctrl.snapshot();actual=model.wrist_transform(s['arms']['right']['joints'],self.height)@tcp
        reached=evaluate_shallow_fit(actual,self.window,self.slot,self.battery,self.contact,depth_mm=7.)
        if not -7.<=reached['leading_standoff_mm']<=-1.:raise ValueError('Measured shallow penetration not attained')
        self.save(status='inserted_held',phase='hold',insertion=True,penetration_mm=-reached['leading_standoff_mm'],reason='Measured shallow insertion; clamp retained')
    def raise_for_place(self):
        self.battery=json.loads((ROOT/'config/battery-envelope.json').read_text())
        self.slot=opening(json.loads((ROOT/'config/cabinet-profiles.json').read_text()),'demo')
        window,table=self.cabinet();s=self.ctrl.snapshot();q=np.array(s['arms']['right']['joints'])
        model=self.ctrl.models['right'];tcp=effective_tcp(self.tcp,self.attachment,self.contact)
        goal=raised_tool_target(model.wrist_transform(q,self.height)@tcp)
        path=plan_correction(model,q,self.height,tcp,goal,self.ctrl.config['arms']['right']['joint_limits_deg'],timing_limits=self.limits,translation_step_mm=5.,rotation_step_deg=2.)
        self.save(phase='raise_clearance',reason='Raise held battery 10mm before realignment')
        self.cabinet_path(path,tcp,window,table,sample_bound_mm=.1)
        self.pose()
    def rotate_for_place(self):
        self.battery=json.loads((ROOT/'config/battery-envelope.json').read_text())
        self.slot=opening(json.loads((ROOT/'config/cabinet-profiles.json').read_text()),'demo')
        window,table=self.cabinet();s=self.ctrl.snapshot();q=np.array(s['arms']['right']['joints'])
        model=self.ctrl.models['right'];tcp=effective_tcp(self.tcp,self.attachment,self.contact)
        goal=rotated_tool_target(model.wrist_transform(q,self.height)@tcp,window['left_axis_base'],window['up_axis_base'])
        path=plan_correction(model,q,self.height,tcp,goal,self.ctrl.config['arms']['right']['joint_limits_deg'],timing_limits=self.limits,translation_step_mm=5.,rotation_step_deg=2.)
        self.save(phase='rotate_alignment',reason='Rotate held battery 1.5 degrees',table_checked=False)
        self.cabinet_path(path,tcp,window,table,sample_bound_mm=.1)
        self.pose()
    def recover_for_place(self):
        self.battery=json.loads((ROOT/'config/battery-envelope.json').read_text())
        self.slot=opening(json.loads((ROOT/'config/cabinet-profiles.json').read_text()),'demo')
        model=self.ctrl.models['right'];limits=self.ctrl.config['arms']['right']['joint_limits_deg']
        for stage in ('retreat','align_up'):
            window,table=self.cabinet();s=self.ctrl.snapshot();q=np.array(s['arms']['right']['joints'])
            tcp=effective_tcp(self.tcp,self.attachment,self.contact);start=model.wrist_transform(q,self.height)@tcp
            direction=window['outward_normal_base'] if stage=='retreat' else window['up_axis_base']
            goal=recovery_tool_target(start,direction,12. if stage=='retreat' else 10.)
            path=plan_correction(model,q,self.height,tcp,goal,limits,timing_limits=self.limits,translation_step_mm=5.,rotation_step_deg=2.)
            self.save(phase='recover_'+stage,reason='Retreat and correct held battery alignment',table_checked=False)
            self.cabinet_path(path,tcp,window,table,sample_bound_mm=.1)
            self.pose()
    def small_up_for_place(self):
        self.battery=json.loads((ROOT/'config/battery-envelope.json').read_text())
        self.slot=opening(json.loads((ROOT/'config/cabinet-profiles.json').read_text()),'demo')
        window,table=self.cabinet();s=self.ctrl.snapshot();q=np.array(s['arms']['right']['joints'])
        model=self.ctrl.models['right'];tcp=effective_tcp(self.tcp,self.attachment,self.contact)
        goal=recovery_tool_target(model.wrist_transform(q,self.height)@tcp,[0,0,1],3.)
        path=continuous_correction(plan_correction,model,q,self.height,tcp,goal,self.ctrl.config['arms']['right']['joint_limits_deg'],timing_limits=self.limits)
        self.save(phase='align_up',reason='Operator-confirmed outside; raise 3mm',table_checked=False)
        self.cabinet_path(path,tcp,window,table,sample_bound_mm=.1)
        self.outside_margin_mm=.5
        self.pose()
    def _run(self):
        maintenance=None;error=None;moving=False;previous_depth=None;monitor=None
        try:
            s=self.ctrl.snapshot();self.token=self.ctrl.reserve_motion(s['command_epoch'])
            maintenance=None if self.auto_insert_160 else self.ctrl.reserve_chassis_maintenance()
            self.driver=self.ctrl.adapter.trajectory_status('right')['driver_instance'];self.raw=s['arms']['right']['gripper'];self.height=s['body']['height']
            self.left_q=np.array(s['arms']['left']['joints']);self.scene=None;self.geometry_tick=0.
            self.bias=np.zeros(6)
            self.last_reference=None;self.last_reference_epoch=None
            self.motion=DockingMotion(self.ctrl,self.token,guard=self.guard,timeout_s=180.,lift_tolerance_rad=.35,require_lift_settle=False)
            self.body_fixed={k:s['body'][k] for k in ('waist','head_yaw','head_pitch')}
            self.grip=GripTracker();self.grip_stamp=-1.;self.grip_evidence=None;self.initializing=True
            self.grip_lock=threading.RLock()
            if not self.d435_only and not self.auto_insert_160:self.monitor=monitor=ContinuousMonitor(self.read_grip)
            self.initial_q=np.array(s['arms']['right']['joints']);self.initial_epoch=self.ctrl.adapter.trajectory_status('right')['command_epoch']
            previous_depth=self.depth.enabled('d435');self.depth.request(dict(camera='d435',enabled=True))
            deadline=time.monotonic()+(2. if self.auto_insert_160 else 8.);ready=False
            while time.monotonic()<deadline:
                self.guard()
                camera=json.loads((ROOT/'web/public/camera-live/d435.json').read_text())
                held_ready=self.d435_only or self.auto_insert_160 or (self.grip_evidence and self.grip_evidence.get('stable'))
                ready=(held_ready and camera_ready(camera,time.time()) and self.depth.snapshot()['cameras']['d435']['active'])
                if ready:break
                time.sleep(.05)
            self.d435_available=bool(ready)
            if not ready and not self.auto_insert_160:raise ValueError('Depth/held-object startup readiness deadline')
            self.initializing=False;self.guard()
            config=json.loads((ROOT/'config/battery-execution.json').read_text());self.limits=motion_limits(config,self.ctrl.config)
            self.tcp=np.array(json.loads((ROOT/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
            self.bounds=np.array(json.loads((ROOT/'config/battery-envelope.json').read_text())['bounds_mm'])
            self.contact=np.array(json.loads((ROOT/'web/public/cad/battery-grip.json').read_text())['points_object_mm']['tip'])
            import hashlib
            self.mesh_sha=hashlib.sha256((ROOT/'web/cad/gripper-v2/battery-print.stl').read_bytes()).hexdigest()
            if self.auto_insert_160:
                self.route=load_taught_route(ROOT/'config/taught-placement.json');self.timeline=PhaseTimeline();self.reached_route=[self.initial_q.tolist()]
                self._timed('clamp',self.close_auto_gripper)
            self.save(limits=self.limits,initial_gripper=self.raw)
            if not self.auto_insert_160:
                self.ensure_model()
                self.pose()
                self.front_x=table_front_x(self.pose_frame)
                self.save(table_front_x_m=self.front_x)
            if self.prepared_only:
                if not self.auto_insert_160:
                    down=self.attachment[:3,1]
                    observed=(self.ctrl.models['right'].wrist_transform(self.initial_q,self.height)@self.tcp)[:3,:3]@down
                    if not prepared_height(self.height) or observed@np.array([0.,0.,-1.])<np.cos(np.radians(20.)):
                        raise ValueError('Prepared placement requires observed near-horizontal battery and approximate 3rad height')
            else:
                moving=True;self.arm_to(np.radians(entry_joints_deg()))
                if requires_three_rad(self.mode,self.height):
                    self.save(phase='lift',reason='Fixed lift target 3rad')
                    q=self.ctrl.snapshot()['arms']['right']['joints']
                    for h in np.linspace(self.height,3.,12):
                        self.local_sweep(q,q,self.ctrl.snapshot()|dict(body=self.ctrl.snapshot()['body']|dict(height=float(h))))
                    d=self.ctrl.adapter.trajectory_status('right')
                    self.motion.lift(3.,lambda:self.guard(),expected_start=dict(joints=q,height=self.height),
                        expected_driver=dict(driver_instance=self.driver,command_epoch=d['command_epoch']))
            self.height=self.ctrl.snapshot()['body']['height'];self.save(phase='observe',height_rad=self.height,
                lift_policy='locked_current_height' if self.d435_only else 'three_rad_if_required')
            if self.d435_only:
                moving=True;self.timeline=PhaseTimeline();self.reached_route=[self.initial_q.tolist()]
                self.battery=json.loads((ROOT/'config/battery-envelope.json').read_text())
                self.slot=opening(json.loads((ROOT/'config/cabinet-profiles.json').read_text()),'demo')
                self.d435_outer_align(self.ctrl.models['right'],self.ctrl.config['arms']['right']['joint_limits_deg'])
                self.timeline.complete();self.save(status='complete_hold_outside',phase='hold',reason='D435 outside preposition complete; clamp retained',released=False,insertion=False,**self.timeline.snapshot())
                atomic_json(self.run/'timeline.json',self.timeline.snapshot());return
            if self.prepared_only:
                moving=True
                if self.auto_insert_160:self.auto_insert_160_place();return
                if self.auto_full:self.auto_full_place();return
                if self.raise_before_place:self.raise_for_place()
                if self.rotate_before_place:self.rotate_for_place()
                if self.recover_before_place:self.recover_for_place()
                if self.small_up_before_place:self.small_up_for_place()
                self.place();return
            for cycle in range(3):
                self.save(phase='observe',cycle=cycle);observation=self.pose();s=self.ctrl.snapshot()
                proposal=visual_level_step(observation,self.ctrl.models['right'],s['arms']['right']['joints'],self.height,self.tcp,
                    self.ctrl.config['arms']['right']['joint_limits_deg'],max_age_s=5.)
                self.save(tilt_deg=proposal['tilt_deg'])
                if proposal['action']=='hold':self.save(status='level',phase='hold',reason='Visual battery level confirmed; clamp retained');break
                self.save(phase='correct',reason='Small visual wrist correction')
                # Arrival is re-evaluated from the next body image. Small
                # gravity offsets do not replace that independent observation.
                self.arm_to(proposal['nominal_target_rad'],tolerance_deg=2.)
            else:self.save(status='held',phase='hold',reason='Bounded leveling cycles complete; visual residual retained')
            moving=True;self.save(status='running');self.place()
        except Exception as e:
            error=str(e)
            if self.motion:self.motion.hold()
            if getattr(self,'auto_insert_160',False):
                self.save(status='complete_degraded',phase='complete',reason=error,insertion_skipped=True,next_phase='complete')
            else:self.save(status='held' if moving else 'blocked',phase='hold',reason=error)
        finally:
            if monitor is not None:monitor.close()
            if previous_depth is not None:
                try:self.depth.request(dict(camera='d435',enabled=previous_depth))
                except Exception:pass
            if error is None or getattr(self,'auto_insert_160',False):
                if maintenance is not None:self.ctrl.release_chassis_maintenance(maintenance)
                if self.token is not None:self.ctrl.release_motion(self.token)
            else:self.save(control_retained=self.token is not None)
    def close(self):
        if self.thread and self.thread.is_alive():self.action(dict(op='cancel'));self.thread.join(3.)
        if self.perception:self.perception.close()
