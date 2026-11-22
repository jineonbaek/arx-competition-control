"""Plan F observation recovery. Constructing/importing sends no commands.

All effects use the existing reserved driver and its original execution checks.
"""
import copy
import time
import numpy as np
from .grasp_path import TimedJointPath
from .storage import atomic_json

POINT3_JOINTS_DEG=(15.,32.,45.,-13.,16.,-90.)
NON_RECOVERABLE={'execution_unknown','execution_timeout','cancelled','interrupted',
                 'control_lost','ownership_lost','capability_unavailable',
                 'recovery_required','invalid_flow'}


def simplify_trace(points,tolerance=np.radians(.15)):
    """Keep measured endpoints and turns for the return route."""
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

    def _snapshot(self):
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
        return s,q,float(lift),float(stamps.min())

    def capture(self):
        if 'grasp_recovery' in self.store.state:
            if self.state.get('run_id')!=self.store.state['run_id']:raise ValueError('Point 2 belongs to another run')
            return copy.deepcopy(self.state['point2'])
        _,q,lift,stamp=self._snapshot()
        point=dict(joints_rad=q.tolist(),lift_rad=lift,sample_time=stamp)
        self.store.state['grasp_recovery']=dict(run_id=self.store.state['run_id'],point2=point,
            attempt=0,status='ready',outbound_trace=[q.tolist()])
        self.store.save();self.store.event('grasp_point2',dict(name='记录本轮抓取起始姿态，用于沿原路退出',**point))
        return copy.deepcopy(point)

    def record_approach(self,data):
        request=data.get('execution_id')
        recorded=self.state.setdefault('recorded_approaches',{})
        if request and request in recorded:return True
        trace=data.get('return_trace_rad')
        if trace:
            trace=simplify_trace(trace)
            self.state.update(outbound_trace=simplify_trace(self.state['outbound_trace']+trace),
                              last_approach_end=trace[-1])
        if request:recorded[request]=True
        self.store.save()
        self.store.event('grasp_arrival',dict(name='抓取轨迹结束，记录实际误差',
            **{k:data.get(k) for k in ('arrival_mode','pose_reached','reference_settled',
                                      'position_error_mm','orientation_error_deg','lift_offset_rad')}))
        return True

    def recover(self,reason,request_id=None):
        previous=self.state.get('last_recovery')
        if request_id and previous and previous.get('request_id')==request_id:return copy.deepcopy(previous)
        origin=np.asarray(self.state['point2']['joints_rad'],float)
        attempt=self.state['attempt']+1
        target=np.radians(POINT3_JOINTS_DEG)
        journal=self.directory/f'attempt-{attempt}.json'
        intent=dict(attempt=attempt,request_id=request_id,reason=str(reason)[:300],target_joints_rad=target.tolist())
        self.state.update(status='moving',attempt=attempt)
        self.store.save();atomic_json(journal,dict(status='unknown',**intent))
        self.store.event('grasp_pose_retry',dict(name=f'重试 {attempt}：返回预观察点 3',**intent))
        try:
            _,current,_,_=self._snapshot()
            reverse=simplify_trace([current.tolist()]+list(reversed(self.state['outbound_trace']))+[origin.tolist()])
            self._move(reverse)
            _,at_origin,_,_=self._snapshot()
            trace=self._move([at_origin.tolist(),target.tolist()])
            intent['completed_at']=time.time()
            self.state.update(status='ready',outbound_trace=simplify_trace([origin.tolist()]+trace),last_recovery=copy.deepcopy(intent),
                              last_approach_end=trace[-1])
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

    def _move(self,points):
        """Return using the existing driver trajectory completion receipt."""
        _,q,lift,_=self._snapshot()
        points=simplify_trace([q.tolist()]+points)
        path=np.asarray(points,float)
        model=self.controller.models['right']
        p=self.backend.policies['pick']
        tcp=np.asarray(self.backend.pick_setup['T_wrist_from_tool'],float)
        def guard(snapshot,now):
            self._snapshot()
            return True
        if len(path)>1:
            timed=TimedJointPath(path,model.reach+np.linalg.norm(tcp[:3,3]),
                tcp_position=lambda a:(model.wrist_transform(a,lift)@tcp)[:3,3],
                **self.backend.pick_setup['timing_limits'])
            plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=timed.times.tolist(),joint_waypoints_rad=path.tolist())
            self.controller.start_trajectory('right',plan,self.token,preflight=guard)
            deadline=time.monotonic()+float(timed.times[-1])+p['arm_policy']['timeout_s']
            trace=[q.tolist()]
            while True:
                guard(None,None)
                if time.monotonic()>=deadline:raise ValueError('Recovery trajectory timeout')
                r=self.controller.poll_trajectory(self.token,preflight=guard)
                _,actual,_,_=self._snapshot();trace.append(actual.tolist())
                if r['status']=='reference_complete':break
                if r['status'] not in ('running','awaiting_driver_status'):raise ValueError('Unknown recovery trajectory receipt')
                time.sleep(.02)
        else:trace=[q.tolist()]
        return simplify_trace(trace)
