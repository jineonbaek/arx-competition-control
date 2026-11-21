"""Owned fixed travel or measured insertion, verification and release/exit.

Fixed-travel requests retain the clamp; placement uses explicit verified mode.
No navigation, lift, or left-arm commands are issued here.
"""
import copy,json,threading,time,uuid
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from agent.held_insertion_path import plan_insert, ENTRY_ANGLE_DEG
from agent.insertion_verification import run_verified_insertion
from agent.visual_battery_level import motion_limits
from agent.storage import atomic_json
from docking_motion import DockingMotion, record_elapsed
from insertion_contact import ContactMonitor, InsertionContact
from mouth_push import placement_timing, DIRECT_MM, PUSH_MM, axis_basis, record_cross_track, record_rotation, require_seated_release, run_fixed_stroke
from placement_clamp import close_for_placement,require_closed_target
from fast_visual_level import StrokeTrace,return_after_release,split_plan,validate_rail_start

def validate_request(data):
    required={'op','request_id','expected_epoch'};optional={'distance_mm','speed_mm_s','axis_base','height_up_base','verify_insertion','cabinet_depth_mm','mouth_then_push','probe_only'}
    if (not isinstance(data,dict) or not required<=set(data) or set(data)-required-optional
        or data['op']!='start' or not isinstance(data['request_id'],str)
        or not 1<=len(data['request_id'])<=128 or type(data['expected_epoch']) is not int):
        raise ValueError('Invalid current-push request')
    if 'probe_only' in data and (data['probe_only'] is not True or data.get('mouth_then_push') is not True):
        raise ValueError('试推需使用夹爪纵轴')
    if 'mouth_then_push' in data and (data['mouth_then_push'] is not True or
            set(data)&{'verify_insertion','distance_mm','axis_base','height_up_base','cabinet_depth_mm'}):
        raise ValueError('固定入仓的轴和 150 mm 行程由程序设定')
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

def measured_progress(anchor,actual,axis_base=None,*,enforce_corridor=True,check_rotation=True):
    delta=(actual[:3,3]-anchor[:3,3])*1000
    axis=np.asarray([1.,0.,0.] if axis_base is None else axis_base,float);progress=float(delta@axis)
    cross=float(np.linalg.norm(delta-progress*axis))
    if enforce_corridor and cross>2.5:raise ValueError(f'末端偏离推进轴 {cross:.2f} mm，超过 2.5 mm；触发时轴向位移 {progress:.2f} mm')
    if check_rotation:
        angle=np.degrees(Rotation.from_matrix(anchor[:3,:3].T@actual[:3,:3]).magnitude())
        if angle>1.:raise ValueError(f'末端姿态相对起推时旋转 {angle:.2f}°，超过 1°；当时轴向推进 {progress:.2f} mm')
    return progress

def require_arrival(anchor,actual,distance,axis_base=None,*,enforce_corridor=True,check_rotation=True):
    progress=measured_progress(anchor,actual,axis_base,enforce_corridor=enforce_corridor,check_rotation=check_rotation)
    if abs(progress-distance)>1.5:raise ValueError('Measured push endpoint not attained')
    return progress

class CurrentAxisPush:
    def __init__(self,ctrl,root,observer=None):
        self.ctrl=ctrl;self.root=Path(root);self.lock=threading.RLock();self.observer=observer;self.mouth_observer=None
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
                mouth_then_push=bool(data.get('mouth_then_push')),probe_only=bool(data.get('probe_only')),verify_insertion=bool(data.get('verify_insertion')),cabinet_depth_mm=float(data.get('cabinet_depth_mm',150.)),
                requested_travel_mm=None if data.get('verify_insertion') or data.get('mouth_then_push') else float(data.get('distance_mm',160.)),speed_mm_s=float(data.get('speed_mm_s',8.)),
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
        from agent.codex_recovery import session
        with session(self.run.name,self.ctrl,lambda:not self.cancel.is_set() and self.ctrl.motion_valid(token),
                     lambda event,kind:self.save(codex_advice=dict(event=event,kind=kind))):
            return self._run_task(token)

    def _run_task(self,token):
        detail={}
        started=time.monotonic();moving=False;stopped=True;returning=False;releasing=False;stage_start=started
        try:
            s=self.ctrl.snapshot();q=np.array(s['arms']['right']['joints']);h=s['body']['height']
            raw=s['arms']['right']['gripper'];left=np.array(s['arms']['left']['joints'])
            model=self.ctrl.models['right'];tcp=np.asarray(json.loads((self.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
            anchor=model.wrist_transform(q,h)@tcp;route=[q.tolist()]
            axis=np.asarray(self.state['axis_base'],float);up=np.asarray(self.state['height_up_base'],float)
            distance=self.state['requested_travel_mm'] or 0.;planned_total=0.
            driver=self.ctrl.adapter.trajectory_status('right')
            fixed=self.state['mouth_then_push'];verified=self.state['verify_insertion'] or fixed;clamp=None
            if not driver or (not verified and 'joint_reference_rad' not in driver):raise ValueError('Continuous driver with retained load reference required')
            rail_guided=fixed and not self.state['probe_only']
            advance=[0.,time.monotonic()]
            trace=None;contact_monitor=None
            def guard(state=None,now=None):
                nonlocal h
                state=self.ctrl.snapshot() if state is None else state
                if self.cancel.is_set():raise ValueError('Push cancelled')
                if not self.ctrl.motion_valid(token):raise ValueError('Push ownership interrupted')
                a=state['arms']['right'];b=state['body']
                if not a['fresh'] or not b['fresh'] or state['chassis_active']:raise ValueError('Fresh stationary feedback lost')
                h=float(b['height'])
                if returning and abs(a['gripper']-target)>.35:raise ValueError('Open gripper feedback changed on exit')
                if not returning and not releasing and (not verified or clamp) and abs(a['gripper']-raw)>.10:raise ValueError('Retained clamp changed')
                if np.max(abs(np.asarray(state['arms']['left']['joints'])-left))>np.radians(.5):raise ValueError('Left arm moved')
                d=self.ctrl.adapter.trajectory_status('right')
                if not d or d['driver_instance']!=driver['driver_instance']:raise ValueError('Arm driver changed')
                if clamp and not releasing and not returning:require_closed_target(d)
                actual=model.wrist_transform(a['joints'],h)@tcp
                if fixed and moving and not returning and not releasing:
                    with self.lock:
                        record_cross_track(self.state,(actual[:3,3]-anchor[:3,3])*1000.,axis)
                        record_rotation(self.state,anchor[:3,:3],actual[:3,:3])
                progress=measured_progress(anchor,actual,axis,enforce_corridor=not fixed,check_rotation=not rail_guided)
                if not -2.5<=progress<=distance+2.5:raise ValueError('Measured travel outside bounded route')
                if trace is not None and moving and not stopped and not returning:trace.sample(progress,a['joints'])
                if progress>advance[0]+.25:advance[:]=[progress,time.monotonic()]
                if not moving or returning or d['status']!='running':advance[1]=time.monotonic()
                elif contact_monitor is None and progress<distance-1.5 and time.monotonic()-advance[1]>2.:raise ValueError('Measured forward progress stalled; possible contact')
                if not returning:self.state['measured_travel_mm']=progress
                return True
            if verified:
                self.save(phase='clamp')
                clamp=close_for_placement(self.ctrl,token,driver['driver_instance'],guard)
                raw=clamp['feedback_raw']
                self.save(clamp=clamp,gripper_commands=int(clamp['close_command_sent']))
                q=np.array(self.ctrl.snapshot()['arms']['right']['joints']);anchor=model.wrist_transform(q,h)@tcp;route=[q.tolist()]
            guard()
            limits=motion_limits(json.loads((self.root/'config/battery-execution.json').read_text()),self.ctrl.config)
            envelope=dict(limits)
            limits=(placement_timing(envelope,self.state['speed_mm_s'],False) if rail_guided else
                    dict(limits,tcp_speed_mm_s=self.state['speed_mm_s'],tcp_acceleration_mm_s2=4.))
            planning_total=0.
            self.motion=DockingMotion(self.ctrl,token,guard=guard,require_lift_settle=False,check_lift=False,timeout_s=90.)
            def move(length,reference=None,*,phase=None,rotation=None,mouth_gap=None):
                nonlocal moving,stopped,distance,axis,up,planned_total,stage_start,planning_total,trace,detail,contact_monitor
                detail={}
                guard();state=self.ctrl.snapshot();start_q=state['arms']['right']['joints'];tool=model.wrist_transform(start_q,h)@tcp
                if reference:
                    axis=np.asarray(reference['axis_base']);up=np.asarray(reference['up_base'])
                    validate_request(dict(op='start',request_id='basis',expected_epoch=0,axis_base=axis.tolist(),height_up_base=up.tolist()))
                    self.save(axis_base=axis.tolist(),height_up_base=up.tolist())
                start_progress=measured_progress(anchor,tool,axis,enforce_corridor=not fixed,check_rotation=not rail_guided);distance=start_progress+length
                timing=(placement_timing(envelope,self.state['speed_mm_s'],length<=10.) if rail_guided else limits)
                if phase=='direct_push':timing=dict(timing,tcp_speed_mm_s=min(5.,timing['tcp_speed_mm_s']))
                with record_elapsed(detail, 'planning'):
                    path=plan_insert(model,np.asarray(start_q),h,tool[:3,3]*1000,tool[:3,3]*1000+length*axis,
                        tool[:3,:3] if rotation is None else rotation,tcp,self.ctrl.config['arms']['right']['joint_limits_deg'],timing_limits=timing,
                        fixed_z_axis=axis,fixed_height_up=up,max_step_mm=.5 if phase=='direct_push' else 2.,
                        adaptive_stroke=rail_guided,
                        terminal_timing_limits=(placement_timing(envelope,self.state['speed_mm_s'],True)
                            if rail_guided and length>10. else None),
                        measured_gap_mm=max(0.,-reference['depth_mm']) if reference and length>200. else mouth_gap,
                        entry_tolerance_deg=None if rail_guided else (ENTRY_ANGLE_DEG if verified else 3.))
                planning_total+=detail['planning']
                index=len(self.state['stages']);atomic_json(self.run/('plan.json' if not index else 'plan-%d.json'%index),path)
                guard();parts=split_plan(path['joint_waypoints_rad'],path['timing']['time_from_start_s'])
                planned_total+=float(path['timing']['time_from_start_s'][-1])
                self.save(phase=phase or ('compensate' if index else 'push'),planned_duration_s=planned_total,
                    planning_actual_s=planning_total,effective_speed_mm_s=timing['tcp_speed_mm_s'],segments=len(parts))
                trace=StrokeTrace(start_q)
                trace.travel=start_progress
                moving=True;stopped=False;stage_start=time.monotonic();advance[1]=stage_start
                d=self.ctrl.adapter.trajectory_status('right')
                contact_monitor=(ContactMonitor(model,tcp,anchor,axis,state,d)
                    if rail_guided and 'joint_reference_rad' in d else None)
                def check_contact():
                    guard()
                    if contact_monitor is not None:
                        contact_monitor.sample(self.ctrl.snapshot(),self.ctrl.adapter.trajectory_status('right'),time.monotonic())
                checks=dict(timing_detail=detail,rail_path=rail_guided)
                if contact_monitor is not None:
                    checks['settled_when']=contact_monitor.settled
                if rail_guided:
                    checks['rail_start_check']=lambda state,initial,held:validate_rail_start(model,tcp,state,initial,held)
                try:
                    actual=self.motion.arm(parts,check_contact,expected_start={'joints':start_q,'height':h},
                        expected_driver={k:d[k] for k in ('driver_instance','command_epoch')},**checks)
                except InsertionContact as contact:
                    actual=self.ctrl.snapshot();stopped=contact.evidence.get('stop_confirmed') is True
                    end=model.wrist_transform(actual['arms']['right']['joints'],actual['body']['height'])@tcp
                    travel=float(1000.*(end[:3,3]-anchor[:3,3])@axis)
                    contact.evidence['stopped_travel_mm']=travel
                    route.extend(trace.stopped(actual['arms']['right']['joints']))
                    self.save(contact_detected=True,contact_stop=contact.evidence,measured_travel_mm=travel,
                        stages=self.state['stages']+[dict(phase=phase or 'push',distance_mm=length,
                            actual_travel_mm=travel-start_progress,push_timing_s=detail,
                            planned_duration_s=path['timing']['time_from_start_s'][-1],
                            duration_s=time.monotonic()-stage_start,cumulative_duration_s=time.monotonic()-started,
                            status='contact')])
                    raise
                stopped=True
                final_tool=model.wrist_transform(actual['arms']['right']['joints'],actual['body']['height'])@tcp
                if fixed:
                    with self.lock:
                        record_cross_track(self.state,(final_tool[:3,3]-anchor[:3,3])*1000.,axis)
                        record_rotation(self.state,anchor[:3,:3],final_tool[:3,:3])
                if rail_guided:
                    progress=measured_progress(tool,final_tool,axis,enforce_corridor=False,check_rotation=False)
                else:
                    progress=require_arrival(tool,final_tool,length,axis,enforce_corridor=not fixed)
                route.extend(trace.reached(path['joint_waypoints_rad'],actual['arms']['right']['joints']))
                self.save(measured_travel_mm=start_progress+progress,stages=self.state['stages']+[dict(phase=phase or ('compensate' if index else 'push'),distance_mm=length,
                    actual_travel_mm=progress,push_timing_s=detail,planned_duration_s=path['timing']['time_from_start_s'][-1],
                    duration_s=time.monotonic()-stage_start,cumulative_duration_s=time.monotonic()-started,status='ok')])
            if verified:
                if fixed:
                    axis,up=axis_basis(anchor[:3,0])
                    rotation=anchor[:3,:3].copy()
                    probe=self.state['probe_only']
                    self.save(phase='direct_push' if probe else 'seat_push',requested_travel_mm=DIRECT_MM if probe else PUSH_MM,axis_base=axis.tolist(),
                              height_up_base=up.tolist(),axis_source='right_tool_positive_x',
                              axis_yaw_deg=float(np.degrees(np.arctan2(axis[1],axis[0]))),
                              axis_pitch_deg=float(np.degrees(np.arcsin(np.clip(axis[2],-1.,1.)))))
                    if probe:
                        move(DIRECT_MM,phase='direct_push',rotation=rotation)
                        self.save(direct_travel_mm=self.state['stages'][-1]['actual_travel_mm'],
                                  status='completed_travel',phase='hold',reason='5 mm 纵轴试推结束，保持夹持')
                        return
                    def read_tool():
                        state=self.ctrl.snapshot()
                        return model.wrist_transform(state['arms']['right']['joints'],state['body']['height'])@tcp
                    def verify(obs,check):
                        self.save(phase='verify')
                        check_depth=getattr(self,'check_fixed_depth',None)
                        evidence=(check_depth(obs,check) if callable(check_depth) else
                                  dict(insertion_depth_verified=False,verification_reason='无法取得坐底观测'))
                        self.save(**evidence)
                        return evidence
                    result=run_fixed_stroke(anchor,read_tool,
                        lambda phase,length,orientation:move(length,phase=phase,rotation=orientation),guard,verify,
                        monitor=getattr(self,'stroke_monitor',None))
                    self.save(**result)
                    if not result['release_ready']:
                        self.save(status='held',phase='hold',reason=result['verification_reason']+'；已停推，夹爪保持闭合')
                        return
                else:
                    if self.observer is None:raise ValueError('入仓深度观测未接入，本次未推入')
                    def event(**value):
                        value.pop('status',None);self.save(**value)
                    result=run_verified_insertion(self.observer,move,guard=guard,event=event,cabinet_depth_mm=self.state['cabinet_depth_mm'])
                    self.save(**result,requested_travel_mm=result['planned_initial_travel_mm'])
                if result['insertion_depth_verified']:
                    guard();self.save(status='running',phase='release',reason='正在松爪并原路退出；校验结论保留')
                    target=float(self.ctrl.config['arms']['right']['gripper_open_raw'])
                    def release_preflight(state,now):
                        guard(state,now)
                        if fixed:
                            actual=model.wrist_transform(state['arms']['right']['joints'],state['body']['height'])@tcp
                            require_seated_release(anchor,actual,axis,result)
                        return True
                    reply=self.ctrl.submit(dict(type='gripper',arm='right',position=target),token,preflight=release_preflight)
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
                        stopped=False
                        return_after_release(self.ctrl,self.motion,model,tcp,points,limits,guard,driver['driver_instance'])
                        stopped=True
                        if np.max(abs(np.asarray(self.ctrl.snapshot()['arms']['right']['joints'])-q))>np.radians(2.):raise ValueError('Exit endpoint not attained')
                    self.save(status='completed_verified',phase='hold',
                        returned=True,reason='入仓已校验，已松爪并原路退出')
                else:
                    from agent.recovery_watch import report_fault
                    report_fault('推入坐底未确认',result['reason'],run=getattr(getattr(self,'run',None),'name',None))
                    self.save(status='held',phase='hold',reason=result['reason']+'；已停推，夹爪保持闭合')
            else:
                move(distance)
                self.save(status='completed_travel',phase='hold',reason='Measured travel completed; clamp retained; insertion depth unverified')
        except Exception as error:
            from agent.recovery_watch import report_fault
            if not self.cancel.is_set():report_fault('轴向推入/退出',str(error),run=getattr(getattr(self,'run',None),'name',None))
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
            if moving and stopped and not returning:
                state=self.ctrl.snapshot()
                if state['arms']['right']['fresh'] and state['body']['fresh']:
                    actual=model.wrist_transform(state['arms']['right']['joints'],state['body']['height'])@tcp
                    delta=(actual[:3,3]-anchor[:3,3])*1000.
                    self.save(measured_travel_mm=float(delta@axis),stop_offset_xyz_mm=delta.tolist(),
                              stopped_cross_track_mm=float(np.linalg.norm(delta-(delta@axis)*axis)))
            self.save(status='held' if moving else 'blocked',phase='hold',release_ready=False,reason=str(error))
        finally:
            stop_pending=bool(getattr(self.motion,'contact_stop_pending',False))
            if stop_pending:stopped=False
            if stopped:
                try:self.ctrl.release_motion(token)
                except Exception as error:stopped=False;self.save(cleanup_error=str(error))
            self.save(last_push_timing_s=detail,control_retained=not stopped,contact_stop_pending=stop_pending,actual_total_s=time.monotonic()-started,
                trajectory_handoffs=self.motion.handoffs if self.motion else [],ended_at=time.time(),final=self.ctrl.snapshot())

    def close(self):
        self.cancel.set()
        if self.thread:self.thread.join(3.)
