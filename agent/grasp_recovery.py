"""Plan F observation recovery. Constructing/importing sends no commands.

Only completed, open-jaw approaches may be reversed. Never replays an uncertain
motion or opens an uncertain grip. All effects use the existing reserved driver.
"""
import copy
import json
import time
import numpy as np
from .grasp_path import TimedJointPath
from .motion_feedback import ArrivalMonitor, tool_feedback
from .storage import atomic_json
from web.src.handeye_column import NOMINAL_RAD_PER_M

POINT3_JOINTS_DEG=(15.,32.,45.,-13.,16.,-90.)
MAX_ATTEMPTS=4
NON_RECOVERABLE={'execution_unknown','execution_timeout','cancelled','interrupted',
                 'control_lost','ownership_lost','capability_unavailable',
                 'recovery_required','invalid_flow'}
TERMINAL_STAGES={'PICK_EXECUTED','HOLD_CONFIRMED','EXTRACTED','CARRY_READY'}


def simplify_trace(points,tolerance=np.radians(.15)):
    """Keep measured endpoints/turns; any simplified connector is checked again."""
    q=np.asarray(points,float)
    if q.ndim!=2 or q.shape[1:]!=(6,) or not len(q) or not np.isfinite(q).all():
        raise ValueError('Finite measured six-axis trace required')
    # Iterative RDP avoids recursion limits on long approach histories.
    keep={0,len(q)-1};stack=[(0,len(q)-1)]
    while stack:
        a,b=stack.pop()
        if b-a<2:continue
        delta=q[b]-q[a];length=float(delta@delta)
        frac=np.clip((q[a+1:b]-q[a])@delta/length,0,1) if length else np.zeros(b-a-1)
        errors=np.max(np.abs(q[a+1:b]-(q[a]+frac[:,None]*delta)),axis=1)
        i=int(np.argmax(errors))+a+1
        if errors[i-a-1]>tolerance:keep.add(i);stack.extend(((a,i),(i,b)))
    result=[]
    for i in sorted(keep):
        if not result or not np.array_equal(result[-1],q[i]):result.append(q[i].tolist())
    return result


class GraspRecovery:
    def __init__(self,backend,*,environment_clear):
        self.backend=backend;self.store=backend.store;self.controller=backend.controller
        self.token=backend.token;self.environment_clear=environment_clear is True
        self.directory=backend.directory/'grasp-recovery';self.directory.mkdir(exist_ok=True)

    @property
    def state(self):return self.store.state['grasp_recovery']

    def _snapshot(self,*,open_grip=False):
        c=self.controller;s=c.snapshot();now=c.clock()
        if (self.store.cancelled or self.backend.cancelled() or getattr(self.backend,'cancelled_here',False)
                or not c.motion_valid(self.token)
                or s.get('contact_stop_pending') or s.get('motion_interrupted') is not False
                or s.get('chassis_active') is not False):raise ValueError('Recovery ownership or stop state invalid')
        arm=s.get('arms',{}).get('right',{});body=s.get('body',{});caps=s.get('capabilities',{})
        q=np.asarray(arm.get('joints'),float);lift=body.get('height')
        p=self.backend.policies['pick'];stamps=np.asarray([arm.get('received'),body.get('received')],float)
        if (q.shape!=(6,) or not np.isfinite(q).all() or type(lift) not in (int,float)
                or not np.isfinite(lift) or not 0<=lift<=18 or not np.isfinite(stamps).all()
                or arm.get('fresh') is not True or body.get('fresh') is not True
                or np.any(stamps>now) or np.any(now-stamps>p['arm_policy']['max_age_s'])
                or np.ptp(stamps)>p['max_pair_skew_s']
                or caps.get('right:arm.wrist')!='' or caps.get('right:gripper')!=''):
            raise ValueError('Fresh, ready right-arm and lift feedback required')
        grip=arm.get('gripper')
        if open_grip and (type(grip) not in (int,float) or not np.isfinite(grip) or not -3.2<=grip<=-2.9):
            raise ValueError('Grip is not confirmed open; automatic posture retry forbidden')
        return s,q,float(lift),float(stamps.min())

    def capture(self):
        if 'grasp_recovery' in self.store.state:
            if self.state.get('run_id')!=self.store.state['run_id']:raise ValueError('Point 2 belongs to another run')
            return copy.deepcopy(self.state['point2'])
        flow=self.store.state.get('battery_flow',{})
        if (self.store.state.get('pending') or self.controller.motion_trajectory is not None
                or flow.get('index',0)!=0 or flow.get('phase','head')!='head'
                or self.store.state['mission'].get('approach_plan')
                or any(self.backend.directory.glob('*.json'))):
            raise ValueError('Point 2 requires this runs initial observation, not a resumed pose')
        _,q,lift,stamp=self._snapshot()
        point=dict(joints_rad=q.tolist(),lift_rad=lift,sample_time=stamp)
        self.store.state['grasp_recovery']=dict(run_id=self.store.state['run_id'],point2=point,
            attempt=0,status='ready',outbound_trace=[q.tolist()])
        self.store.save();self.store.event('grasp_point2',dict(name='记录本轮抓取起始姿态，用于沿原路退出',**point))
        return copy.deepcopy(point)

    def record_approach(self,data):
        request=data.get('execution_id')
        if not isinstance(request,str) or not request:raise ValueError('Approach execution identity required')
        recorded=self.state.setdefault('recorded_approaches',{})
        if request in recorded:return recorded[request]
        trace=data.get('return_trace_rad')
        if not trace:raise ValueError('Completed approach lacks measured return trace')
        trace=simplify_trace(trace)
        previous=self.state.get('last_approach_end',self.state['point2']['joints_rad'])
        progressed=bool(np.max(np.abs(np.asarray(trace[-1])-previous))>np.radians(.1))
        count=0 if progressed else self.state.get('no_progress_count',0)+1
        self.state.update(outbound_trace=simplify_trace(self.state['outbound_trace']+trace),
                          last_approach_end=trace[-1],no_progress_count=count)
        recorded[request]=count<2
        self.store.save()
        self.store.event('grasp_arrival',dict(name='抓取轨迹结束，记录实际误差',
            **{k:data.get(k) for k in ('arrival_mode','pose_reached','reference_settled',
                                      'position_error_mm','orientation_error_deg','lift_offset_rad')}))
        return recorded[request]

    def _idle(self):
        if (self.store.state.get('pending') or self.controller.motion_trajectory is not None
                or self.state.get('status')!='ready'
                or self.store.state['mission'].get('stage') in TERMINAL_STAGES):
            raise ValueError('Recovery requires a known completed open-jaw approach')
        if any(not entry.done.is_set() and not entry.wait(.25) for entry in self.backend.entries.values()):
            raise ValueError('Execution thread has not completed')
        for journal in self.backend.directory.glob('*.json'):
            import json
            receipt=json.loads(journal.read_text())['receipt']
            if receipt.get('status') not in ('completed','rejected'):
                raise ValueError('Earlier execution result unknown')
        self._snapshot(open_grip=True)

    def recover(self,reason,request_id=None):
        self._idle()
        previous=self.state.get('last_recovery')
        if request_id is not None:
            for path in self.directory.glob('attempt-*.json'):
                saved=json.loads(path.read_text())
                if saved.get('request_id')!=request_id:continue
                if (saved.get('status')=='completed' and previous
                        and previous.get('request_id')==request_id):return copy.deepcopy(previous)
                raise ValueError('Stale or uncertain recovery request; will not replay')
        if self.state['attempt']>=MAX_ATTEMPTS:raise ValueError('Point 3 recovery exhausted four attempts')
        if not self.environment_clear:raise ValueError('Recovery needs existing clear-area confirmation')
        origin=np.asarray(self.state['point2']['joints_rad'],float)
        # Completed status is durable, but never grants permission to replay a command.
        _,current,lift,_=self._snapshot(open_grip=True)
        reverse=simplify_trace([current.tolist()]+list(reversed(self.state['outbound_trace']))+[origin.tolist()])
        self._validate_points(reverse,lift)
        attempt=self.state['attempt']+1
        target=np.radians(POINT3_JOINTS_DEG)
        self._validate_points([origin.tolist(),target.tolist()],lift)
        journal=self.directory/f'attempt-{attempt}.json'
        if journal.exists():raise ValueError('Recovery attempt already exists; will not replay')
        intent=dict(attempt=attempt,request_id=request_id,reason=str(reason)[:300],target_joints_rad=target.tolist())
        self.state.update(status='moving',attempt=attempt)
        self.store.save();atomic_json(journal,dict(status='unknown',**intent))
        self.store.event('grasp_pose_retry',dict(name=f'重试 {attempt}/4：返回预观察点 3',**intent))
        try:
            _,current,_,_=self._snapshot(open_grip=True)
            reverse=simplify_trace([current.tolist()]+list(reversed(self.state['outbound_trace']))+[origin.tolist()])
            self._move(reverse)
            _,at_origin,_,_=self._snapshot(open_grip=True)
            trace=self._move([at_origin.tolist(),target.tolist()])
            intent['completed_at']=time.time()
            self.state.update(status='ready',outbound_trace=simplify_trace([origin.tolist()]+trace),last_recovery=copy.deepcopy(intent),
                              last_approach_end=trace[-1],no_progress_count=0)
            atomic_json(journal,dict(status='completed',**intent));self.store.save()
            self.store.event('grasp_pose_retry',dict(name='预观察点 3 到位，从新画面重新开始抓取流程',**intent))
            return intent
        except Exception:
            self.state['status']='unknown'
            # Stop before persisting diagnostics: disk failure cannot suppress it.
            # An uncertain effect must not free the motion token for another command.
            self.controller.begin_contact_stop(self.token)  # shared unresolved-stop latch
            self.controller.interrupt_motion(self.token)
            try:self.controller.submit({'type':'arm.stop','arm':'right'},self.token)
            except Exception:pass
            try:self.store.save()
            except OSError:pass  # The pre-effect journal remains unknown.
            raise

    def _validate_points(self,points,lift):
        path=np.asarray(points,float)
        bounds=np.radians(np.asarray(self.controller.config['arms']['right']['joint_limits_deg'],float))
        if (path.ndim!=2 or path.shape[1:]!=(6,) or not len(path) or not np.isfinite(path).all()
                or bounds.shape!=(6,2) or not np.isfinite(bounds).all()
                or np.any(path<bounds[:,0]) or np.any(path>bounds[:,1])):
            raise ValueError('Recovery posture outside joint limits')
        model=self.controller.models['right']
        checks=[model.check(a,b,lift) for a,b in zip(path,path[1:])]
        if not checks:checks=[model.check(path[0],path[0],lift)]
        if any(c.get('clear') is not True for c in checks):raise ValueError('Recovery path is not CAD clear')
        lower=min(float(c['lower_mm']) for c in checks)
        if not np.isfinite(lower) or lower<0:raise ValueError('Invalid recovery clearance bound')
        return lower

    def _move(self,points):
        """One checked open-jaw route, reference receipt and measured rest."""
        _,q,lift,_=self._snapshot(open_grip=True)
        points=simplify_trace([q.tolist()]+points)
        path=np.asarray(points,float)
        model=self.controller.models['right'];certificate=[lift,self._validate_points(points,lift)]
        def certify(height):certificate[:]=[height,self._validate_points(points,height)]
        p=self.backend.policies['pick']
        # Use measured FK displacement, not an invented zero-error speed signal.
        tcp=np.asarray(self.backend.pick_setup['T_wrist_from_tool'],float)
        target=model.wrist_transform(path[-1],lift)@tcp
        def guard(snapshot,now):
            _,_,height,_=self._snapshot(open_grip=True)
            if abs(height-certificate[0])/NOMINAL_RAD_PER_M*1000>=max(0.,certificate[1]/2):certify(height)
            return True
        if len(path)>1:
            timed=TimedJointPath(path,model.reach+np.linalg.norm(tcp[:3,3]),
                tcp_position=lambda a:(model.wrist_transform(a,lift)@tcp)[:3,3],
                **self.backend.pick_setup['timing_limits'])
            if timed.times[-1]>60:raise ValueError('Recovery route exceeds driver duration')
            plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=timed.times.tolist(),joint_waypoints_rad=path.tolist())
            self.controller.start_trajectory('right',plan,self.token,preflight=guard)
            deadline=time.monotonic()+float(timed.times[-1])+p['arm_policy']['timeout_s']
            trace=[q.tolist()]
            while True:
                guard(None,None)
                if time.monotonic()>=deadline:raise ValueError('Recovery trajectory timeout')
                r=self.controller.poll_trajectory(self.token,preflight=guard)
                _,actual,_,_=self._snapshot(open_grip=True);trace.append(actual.tolist())
                if r['status']=='reference_complete':break
                if r['status'] not in ('running','awaiting_driver_status'):raise ValueError('Unknown recovery trajectory receipt')
                time.sleep(.02)
        else:trace=[q.tolist()]
        monitor=ArrivalMonitor(path[-1],started_at=self.controller.clock(),
            **dict(p['arm_policy'],tolerance=[float(np.radians(.5))]*6,max_speed=[float(np.radians(1.))]*6))
        deadline=time.monotonic()+p['arm_policy']['timeout_s']
        while True:
            guard(None,None);s,actual,_,_=self._snapshot(open_grip=True);now=self.controller.clock()
            measured=tool_feedback(s,model,tcp,target,now=now,max_age_s=p['arm_policy']['max_age_s'],
                max_pair_skew_s=p['max_pair_skew_s'],expected_lift_rad=lift,
                lift_tolerance_rad=p['lift_tolerance_rad'],enforce_lift=False)
            error=measured['position_error_mm']+measured['orientation_error_rotvec_deg']
            r=monitor.update(actual,measured['sample_time'],now)
            trace.append(actual.tolist())
            if r['stop_required'] or time.monotonic()>=deadline:raise ValueError('Recovery rest not confirmed: '+r['status'])
            if r['arrived']:return simplify_trace(trace)
            time.sleep(.02)
