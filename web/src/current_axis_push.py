"""Owned fixed travel or measured insertion, verification and release/exit.

Fixed-travel requests retain the clamp; placement uses explicit verified mode.
No navigation, lift, or left-arm commands are issued here.
"""
import copy,json,threading,time,uuid
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from agent.held_insertion_path import plan_insert
from agent.grasp_path import TimedJointPath
from agent.insertion_verification import run_verified_insertion
from agent.visual_battery_level import motion_limits
from agent.storage import atomic_json
from docking_motion import DockingMotion
from placement_clamp import close_for_placement,require_closed_target
from fast_visual_level import split_plan

def validate_request(data):
    required={'op','request_id','expected_epoch'};optional={'distance_mm','speed_mm_s','axis_base','height_up_base','verify_insertion','cabinet_depth_mm'}
    if (not isinstance(data,dict) or not required<=set(data) or set(data)-required-optional
        or data['op']!='start' or not isinstance(data['request_id'],str)
        or not 1<=len(data['request_id'])<=128 or type(data['expected_epoch']) is not int):
        raise ValueError('Invalid current-push request')
    if 'verify_insertion' in data and data['verify_insertion'] is not True:raise ValueError('Invalid verification mode')
    if 'cabinet_depth_mm' in data and ('verify_insertion' not in data or type(data['cabinet_depth_mm']) not in (int,float) or not 100<=data['cabinet_depth_mm']<=180):raise ValueError('Invalid cabinet depth')
    if data.get('verify_insertion') and set(data)&{'distance_mm','axis_base','height_up_base'}:raise ValueError('Verified insertion computes its own distance and axis')
    for key,default,lo,hi in [('distance_mm',160.,1.,200.),('speed_mm_s',8.,1.,30.)]:
        value=data.get(key,default)
        if type(value) not in (int,float) or not np.isfinite(value) or not lo<=value<=hi:raise ValueError('Invalid '+key)
    if ('axis_base' in data)!=('height_up_base' in data):raise ValueError('Insertion axis and height basis must be paired')
    if 'axis_base' in data:
        axis=np.asarray(data['axis_base'],float);up=np.asarray(data['height_up_base'],float)
        if (axis.shape!=(3,) or up.shape!=(3,) or not np.isfinite(np.r_[axis,up]).all()
            or abs(np.linalg.norm(axis)-1)>1e-6 or abs(np.linalg.norm(up)-1)>1e-6
            or abs(up[2])<.80 or abs(axis@up)>1e-6):raise ValueError('Invalid fixed-height insertion basis')

def measured_progress(anchor,actual,axis_base=None):
    delta=(actual[:3,3]-anchor[:3,3])*1000
    axis=np.asarray([1.,0.,0.] if axis_base is None else axis_base,float);progress=float(delta@axis)
    if np.linalg.norm(delta-progress*axis)>2.5:raise ValueError('Measured TCP left 2.5mm horizontal corridor')
    angle=np.degrees(Rotation.from_matrix(anchor[:3,:3].T@actual[:3,:3]).magnitude())
    if angle>1.:raise ValueError('Measured tool rotation changed more than 1deg')
    return progress

def require_arrival(anchor,actual,distance,axis_base=None):
    progress=measured_progress(anchor,actual,axis_base)
    if abs(progress-distance)>1.5:raise ValueError('Measured push endpoint not attained')
    return progress

class CurrentAxisPush:
    def __init__(self,ctrl,root,observer=None):
        self.ctrl=ctrl;self.root=Path(root);self.lock=threading.RLock();self.observer=observer
        self.thread=None;self.cancel=threading.Event();self.requests={};self.motion=None
        self.state={'status':'idle','released':False}

    def snapshot(self):
        with self.lock:return copy.deepcopy(self.state)

    def save(self,**values):
        with self.lock:
            self.state.update(values)
            atomic_json(self.run/'receipt.json',self.state)

    def action(self,data):
        with self.lock:
            if isinstance(data,dict) and data.get('op')=='cancel':
                if set(data)!={'op','request_id'} or data['request_id']!=self.state.get('request_id'):
                    raise ValueError('Cancel does not match current push')
                self.cancel.set();return self.snapshot()
            validate_request(data)
            request=data['request_id']
            if request in self.requests:
                if self.requests[request]!=data:raise ValueError('Replayed request changed')
                return copy.deepcopy(self.requests[request+'_result'])
            if self.thread and self.thread.is_alive():raise ValueError('Push already running')
            s=self.ctrl.snapshot()
            if any(s.get(k) for k in ('motion_reserved','competition_reserved','chassis_active')):
                raise ValueError('Another controller is active')
            run=self.root/'reports'/('automatic-current-push-'+uuid.uuid4().hex);run.mkdir(parents=True)
            token=self.ctrl.reserve_motion(data['expected_epoch']);self.run=run
            self.cancel=threading.Event();self.motion=None
            self.state=dict(status='running',phase='planning',request_id=request,run_id=self.run.name,
                verify_insertion=bool(data.get('verify_insertion')),cabinet_depth_mm=float(data.get('cabinet_depth_mm',150.)),
                requested_travel_mm=None if data.get('verify_insertion') else float(data.get('distance_mm',160.)),speed_mm_s=float(data.get('speed_mm_s',8.)),
                axis_base=data.get('axis_base',[1.,0.,0.]),height_up_base=data.get('height_up_base',[0.,0.,1.]),
                measured_travel_mm=0.,insertion_depth_verified=False,
                released=False,gripper_commands=0,lift_commands=0,chassis_commands=0,left_arm_commands=0,
                started_at=time.time(),stages=[],initial=s)
            self.requests[request]=copy.deepcopy(data)
            self.requests[request+'_result']=self.snapshot()
            self.thread=threading.Thread(target=self._run,args=(token,),daemon=True,name='current-axis-push')
            try:self.thread.start()
            except Exception:self.ctrl.release_motion(token);raise
            return self.snapshot()

    def _run(self,token):
        started=time.monotonic();moving=False;stopped=True;returning=False;releasing=False;stage_start=started
        try:
            s=self.ctrl.snapshot();q=np.array(s['arms']['right']['joints']);h=s['body']['height']
            raw=s['arms']['right']['gripper'];left=np.array(s['arms']['left']['joints'])
            model=self.ctrl.models['right'];tcp=np.asarray(json.loads((self.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
            anchor=model.wrist_transform(q,h)@tcp;route=[q.tolist()]
            axis=np.asarray(self.state['axis_base'],float);up=np.asarray(self.state['height_up_base'],float)
            distance=self.state['requested_travel_mm'] or 0.;planned_total=0.
            driver=self.ctrl.adapter.trajectory_status('right')
            verified=self.state['verify_insertion'];clamp=None
            if not driver or (not verified and 'joint_reference_rad' not in driver):raise ValueError('Continuous driver with retained load reference required')
            advance=[0.,time.monotonic()]
            def guard(state=None,now=None):
                state=self.ctrl.snapshot() if state is None else state
                if self.cancel.is_set():raise ValueError('Push cancelled')
                if not self.ctrl.motion_valid(token):raise ValueError('Push ownership interrupted')
                a=state['arms']['right'];b=state['body']
                if not a['fresh'] or not b['fresh'] or state['chassis_active']:raise ValueError('Fresh stationary feedback lost')
                if abs(b['height']-h)>.02 or (not releasing and (not verified or clamp) and abs(a['gripper']-raw)>.10):raise ValueError('Lift or retained clamp changed')
                if np.max(abs(np.asarray(state['arms']['left']['joints'])-left))>np.radians(.5):raise ValueError('Left arm moved')
                d=self.ctrl.adapter.trajectory_status('right')
                if not d or d['driver_instance']!=driver['driver_instance']:raise ValueError('Arm driver changed')
                if clamp and not releasing and not returning:require_closed_target(d)
                progress=measured_progress(anchor,model.wrist_transform(a['joints'],h)@tcp,axis)
                if not -2.5<=progress<=distance+2.5:raise ValueError('Measured travel outside bounded route')
                if progress>advance[0]+.25:advance[:]=[progress,time.monotonic()]
                if not moving or returning or d['status']!='running':advance[1]=time.monotonic()
                elif progress<distance-1.5 and time.monotonic()-advance[1]>2.:raise ValueError('Measured forward progress stalled; possible contact')
                self.state['measured_travel_mm']=progress
                return True
            if verified:
                self.save(phase='clamp')
                clamp=close_for_placement(self.ctrl,token,driver['driver_instance'],guard)
                raw=clamp['feedback_raw']
                self.save(clamp=clamp,gripper_commands=int(clamp['close_command_sent']))
                q=np.array(self.ctrl.snapshot()['arms']['right']['joints']);anchor=model.wrist_transform(q,h)@tcp;route=[q.tolist()]
            guard()
            limits=motion_limits(json.loads((self.root/'config/battery-execution.json').read_text()),self.ctrl.config)
            limits.update(tcp_speed_mm_s=self.state['speed_mm_s'],tcp_acceleration_mm_s2=4.)
            self.motion=DockingMotion(self.ctrl,token,guard=guard,require_lift_settle=False,timeout_s=90.)
            def move(length,reference=None):
                nonlocal moving,stopped,distance,axis,up,planned_total,stage_start
                guard();state=self.ctrl.snapshot();start_q=state['arms']['right']['joints'];tool=model.wrist_transform(start_q,h)@tcp
                if reference:
                    axis=np.asarray(reference['axis_base']);up=np.asarray(reference['up_base'])
                    validate_request(dict(op='start',request_id='basis',expected_epoch=0,axis_base=axis.tolist(),height_up_base=up.tolist()))
                    self.save(axis_base=axis.tolist(),height_up_base=up.tolist())
                start_progress=measured_progress(anchor,tool,axis);distance=start_progress+length
                path=plan_insert(model,np.asarray(start_q),h,tool[:3,3]*1000,tool[:3,3]*1000+length*axis,
                    tool[:3,:3],tcp,self.ctrl.config['arms']['right']['joint_limits_deg'],timing_limits=limits,
                    fixed_z_axis=axis,fixed_height_up=up,max_step_mm=2.)
                index=len(self.state['stages']);atomic_json(self.run/('plan.json' if not index else 'plan-%d.json'%index),path)
                guard();parts=split_plan(path['joint_waypoints_rad'],path['timing']['time_from_start_s'])
                planned_total+=float(path['timing']['time_from_start_s'][-1])
                self.save(phase='compensate' if index else 'push',planned_duration_s=planned_total,
                    planning_actual_s=time.monotonic()-started,segments=len(parts))
                moving=True;stopped=False;stage_start=time.monotonic();advance[1]=stage_start
                d=self.ctrl.adapter.trajectory_status('right')
                actual=self.motion.arm(parts,lambda:guard(),expected_start={'joints':start_q,'height':h},
                    expected_driver={k:d[k] for k in ('driver_instance','command_epoch')})
                stopped=True;progress=require_arrival(tool,model.wrist_transform(actual['arms']['right']['joints'],h)@tcp,length,axis)
                route.extend(path['joint_waypoints_rad'][1:])
                self.save(measured_travel_mm=start_progress+progress,stages=self.state['stages']+[dict(distance_mm=length,
                    actual_travel_mm=progress,planned_duration_s=path['timing']['time_from_start_s'][-1],duration_s=time.monotonic()-stage_start)])
            if self.state['verify_insertion']:
                if self.observer is None:raise ValueError('入仓深度观测未接入，本次未推入')
                def event(**value):
                    value.pop('status',None);self.save(**value)
                result=run_verified_insertion(self.observer,move,guard=guard,event=event,cabinet_depth_mm=self.state['cabinet_depth_mm'])
                self.save(**result,requested_travel_mm=result['planned_initial_travel_mm'])
                # Operator chose release and original-path exit after an attempted
                # insertion even if post-push visual evidence remains unavailable.
                if result['push_started'] or result['insertion_depth_verified']:
                    guard();self.save(status='running',phase='release',reason='正在松爪并原路退出；校验结论保留')
                    target=float(self.ctrl.config['arms']['right']['gripper_open_raw'])
                    reply=self.ctrl.submit(dict(type='gripper',arm='right',position=target),token,preflight=lambda s,n:guard(s,n))
                    if reply.get('ok') is not True:raise ValueError('Gripper release rejected')
                    releasing=True
                    self.save(gripper_commands=self.state['gripper_commands']+1,released=True)
                    deadline=time.monotonic()+6.
                    while True:
                        guard();state=self.ctrl.snapshot()
                        if abs(state['arms']['right']['gripper']-target)<=.35:break
                        if time.monotonic()>=deadline:raise ValueError('Gripper open feedback not reached; exit not attempted')
                        self.cancel.wait(.05)
                    raw=state['arms']['right']['gripper'];releasing=False;returning=True
                    if len(route)>1:
                        self.save(phase='exit');points=list(reversed(route))
                        timing=TimedJointPath(points,model.reach+np.linalg.norm(tcp[:3,3]),
                            tcp_position=lambda v:(model.wrist_transform(v,h)@tcp)[:3,3],**limits).receipt()
                        stopped=False;d=self.ctrl.adapter.trajectory_status('right')
                        self.motion.arm(split_plan(points,timing['time_from_start_s']),lambda:guard(),
                            expected_start=dict(joints=state['arms']['right']['joints'],height=h),
                            expected_driver={k:d[k] for k in ('driver_instance','command_epoch')})
                        stopped=True
                        if np.max(abs(np.asarray(self.ctrl.snapshot()['arms']['right']['joints'])-q))>np.radians(2.):raise ValueError('Exit endpoint not attained')
                    self.save(status='completed_verified' if result['insertion_depth_verified'] else 'completed_unverified',phase='hold',
                        returned=True,reason=('入仓已校验' if result['insertion_depth_verified'] else '入仓未确认')+'；已松爪并原路退出')
                else:self.save(status='insertion_unverified',phase='hold',reason=result['reason']+'；未推入，保持夹持')
            else:
                move(distance)
                self.save(status='completed_travel',phase='hold',reason='Measured travel completed; clamp retained; insertion depth unverified')
        except Exception as error:
            if self.motion:
                try:self.motion.hold()
                except Exception as stop_error:self.save(stop_error=str(stop_error))
            if moving:
                history=[];deadline=time.monotonic()+3.
                while time.monotonic()<deadline:
                    state=self.ctrl.snapshot();a=state['arms']['right'];d=self.ctrl.adapter.trajectory_status('right')
                    if a['fresh'] and d and d['status'] in ('idle','failed','reference_complete'):
                        history.append((a['received'],np.asarray(a['joints'])))
                        history=[v for v in history if history[-1][0]-v[0]<=.6]
                        if len(history)>=3 and history[-1][0]-history[0][0]>=.25 and np.max(np.ptp([v[1] for v in history],axis=0))<.002:
                            stopped=True;break
                    time.sleep(.08)
            self.save(status='held' if moving else 'blocked',phase='hold',reason=str(error))
        finally:
            if stopped:
                try:self.ctrl.release_motion(token)
                except Exception as error:stopped=False;self.save(cleanup_error=str(error))
            self.save(control_retained=not stopped,actual_total_s=time.monotonic()-started,
                trajectory_handoffs=self.motion.handoffs if self.motion else [],ended_at=time.time(),final=self.ctrl.snapshot())

    def close(self):
        self.cancel.set()
        if self.thread:self.thread.join(3.)
