"""Checked lift -> measured rest -> checked observer path -> measured rest.

No hardware setup or live registration. Caller supplies deterministic scene/path
preflights and owns polling, timeout supervision and verified stop cleanup.
"""
import copy
from threading import Lock
import numpy as np
from .motion_feedback import ArrivalMonitor


class CoarseAlignmentStep:
    def __init__(self, controller, token, *, lift_target_rad, observer_plan,
                 initial_joints, initial_lift_rad, fixed_joint_tolerance_rad,
                 lift_policy, arm_policy, max_pair_skew_s, lift_preflight, arm_preflight):
        if not controller.motion_valid(token) or not all(map(callable, (lift_preflight, arm_preflight))):
            raise ValueError('Reserved controller and deterministic preflights required')
        self.controller=controller;self.token=token;self.lock=Lock()
        self.plan=copy.deepcopy(observer_plan)
        self.initial={s:np.asarray(initial_joints[s],float).copy() for s in ('left','right')}
        self.initial_lift=float(initial_lift_rad);self.target=float(lift_target_rad)
        self.fixed_tolerance=float(fixed_joint_tolerance_rad);self.skew=float(max_pair_skew_s)
        if (not np.isfinite([self.initial_lift,self.target,self.fixed_tolerance,self.skew]).all()
                or not 0<=self.initial_lift<=18 or not 0<=self.target<=18
                or min(self.fixed_tolerance,self.skew)<=0
                or any(q.shape!=(6,) or not np.isfinite(q).all() for q in self.initial.values())):
            raise ValueError('Invalid coarse alignment feedback policy')
        self.stationary=self.plan.get('kind')=='stationary'
        if self.stationary:
            endpoint=np.asarray(self.plan.get('joint_position_rad'),float)
            if (set(self.plan)!={'version','kind','joint_position_rad'} or self.plan['version']!=1
                    or endpoint.shape!=(6,) or not np.isfinite(endpoint).all()
                    or not np.array_equal(endpoint,self.initial['right'])):
                raise ValueError('Stationary observer must retain the exact prepared joint pose')
            self.endpoint=endpoint.copy()
        else:
            points=np.asarray(self.plan['joint_waypoints_rad'],float)
            times=np.asarray(self.plan['time_from_start_s'],float)
            if (points.ndim!=2 or points.shape[1:]!=(6,) or len(points)<2
                    or times.shape!=(len(points),) or not np.isfinite(points).all()
                    or not np.isfinite(times).all() or times[0]!=0 or np.any(np.diff(times)<=0)
                    or np.any(np.abs(points[0]-self.initial['right'])>self.fixed_tolerance)
                    or self.plan.get('version')!=1 or self.plan.get('interpolation')!='segment_quintic_scalar'):
                raise ValueError('Observer plan must start at the fixed right-arm pose')
            self.endpoint=points[-1].copy()
        self.lift_policy=copy.deepcopy(lift_policy);self.arm_policy=copy.deepcopy(arm_policy)
        now=controller.clock()
        self.lift=ArrivalMonitor([self.target],started_at=now,**self.lift_policy)
        self.arm=ArrivalMonitor(self.endpoint,started_at=now,**self.arm_policy)
        if self.lift.tolerance.shape!=(1,) or self.arm.tolerance.shape!=(6,):
            raise ValueError('Lift and arm policies require one and six axes')
        self.lift_preflight=lift_preflight;self.arm_preflight=arm_preflight
        self.stage='prepared';self.failure=None;self.stop_requests={}
        self.achieved_lift=None;self.last_now=now;self.last_stamps=None

    def _receipt(self):
        return dict(status=self.failure or self.stage,stop_required=self.failure is not None,
            stopped_verified=False,grasp_confirmed=False,
            new_observation_required=self.stage=='ready_to_observe' and self.failure is None,
            achieved_lift_rad=self.achieved_lift,stop_requests=dict(self.stop_requests))

    def _abort(self, reason):
        if self.failure is None:
            self.failure=reason
            gravity_timeout=reason=='timeout' and self.stage in ('observing_motion','settling_observer')
            if gravity_timeout:
                self.stop_requests['right']='failed'
                try:
                    r=self.controller.submit({'type':'arm.mode','arm':'right','mode':3},self.token)
                    if r.get('ok') is True:self.stop_requests['right']='gravity_comp_sent_unverified'
                except Exception as error:
                    self.controller.log('观察姿态超时切换重力补偿失败：'+str(error),'warning')
            self.controller.interrupt_motion(self.token)
            commands=[('lift',{'type':'lift.stop'}),('left',{'type':'arm.stop','arm':'left'})]
            if not gravity_timeout or self.stop_requests.get('right')=='failed':
                commands.insert(1,('right',{'type':'arm.stop','arm':'right'}))
            for name, command in commands:
                self.stop_requests[name]='failed'
                try:
                    r=self.controller.submit(command,self.token)
                    if r.get('ok') is True:self.stop_requests[name]='sent_unverified'
                except Exception as error:
                    self.controller.log('粗对位停止请求失败：'+str(error),'warning')
        return self._receipt()

    def _check(self, snapshot, now, *, lifting):
        if (not self.controller.motion_valid(self.token) or snapshot.get('chassis_active') is not False
                or not np.isfinite(now) or now<self.last_now):
            raise ValueError('Ownership, chassis or clock changed')
        states=[snapshot['arms'][s] for s in ('left','right')]+[snapshot['body']]
        stamps=np.asarray([s['received'] for s in states],float)
        max_age=min(self.lift.max_age,self.arm.max_age)
        if (not all(s.get('fresh') is True for s in states) or not np.isfinite(stamps).all()
                or np.any(stamps>now) or np.any(now-stamps>max_age) or np.ptp(stamps)>self.skew
                or (self.last_stamps is not None and np.any(stamps<self.last_stamps))):
            raise ValueError('Fresh paired feedback required')
        q=[np.asarray(s['joints'],float) for s in states[:2]]
        height=float(states[2]['height'])
        if (any(v.shape!=(6,) or not np.isfinite(v).all() for v in q)
                or not np.isfinite(height) or not 0<=height<=18):
            raise ValueError('Invalid joint or lift feedback')
        fixed=('left','right') if lifting else ('left',)
        for i,side in enumerate(('left','right')):
            if side in fixed and np.any(np.abs(q[i]-self.initial[side])>self.fixed_tolerance):
                raise ValueError('Fixed arm changed during coarse alignment')
        expected=self.initial_lift if self.stage=='prepared' else self.achieved_lift
        if expected is not None and abs(height-expected)>self.lift.tolerance[0]:
            raise ValueError('Lift changed outside the planned phase')
        caps=snapshot.get('capabilities',{})
        if (caps.get('lift')!=''
                or any(caps.get(side+':arm.joints')!='' for side in ('left','right'))
                or (not lifting and caps.get('right:arm.wrist')!='')):
            raise ValueError('Controller not ready')
        check=self.lift_preflight if lifting else self.arm_preflight
        if check(snapshot,now) is not True:raise ValueError('Scene or path preflight failed')
        self.last_now=now;self.last_stamps=stamps.copy()
        return q[1],height,float(stamps.min())

    def _guard(self, snapshot, now):
        self._check(snapshot,now,lifting=self.stage in ('prepared','lifting'))
        return True

    def start(self):
        with self.lock:
            if self.stage!='prepared' or self.failure:raise ValueError('Coarse step cannot be replayed or resumed')
            try:
                self.lift=ArrivalMonitor([self.target],started_at=self.controller.clock(),**self.lift_policy)
                r=self.controller.submit({'type':'lift','target':self.target},self.token,preflight=self._guard)
                if r.get('ok') is not True:raise ValueError('Lift submission not confirmed')
                self.stage='lifting'
                if not self.controller.motion_valid(self.token):raise ValueError('Interrupted')
            except Exception as error:
                self.controller.log('粗对位未启动：'+str(error),'warning')
                return self._abort('coarse_start_failed')
            return self._receipt()

    def cancel(self):
        with self.lock:return self._abort('cancelled')

    def timeout(self):
        with self.lock:return self._abort('timeout')

    def poll(self, *, cancelled=False):
        with self.lock:
            if self.failure:return self._receipt()
            if cancelled:return self._abort('cancelled')
            if self.stage=='prepared':raise ValueError('Coarse step has not started')
            try:
                now=self.controller.clock()
                q,height,stamp=self._check(self.controller.snapshot(),now,lifting=self.stage=='lifting')
                if self.stage=='lifting':
                    if now-self.lift.start>=self.lift.timeout:return self._abort('timeout')
                    r=self.lift.update([height],stamp,now)
                    if r['stop_required']:return self._abort(r['status'])
                    if r['arrived']:
                        if self.achieved_lift is None:self.achieved_lift=height
                        # Controller's lift-rest history can exceed the local arrival window.
                        if self.controller.snapshot()['capabilities'].get('right:arm.wrist')!='':
                            return self._receipt()
                        self.stage='observing_motion'
                        def observer_start(snapshot, time):
                            actual,_,_=self._check(snapshot,time,lifting=False)
                            if np.any(np.abs(actual-self.initial['right'])>self.fixed_tolerance):
                                raise ValueError('Observer start no longer matches the checked path')
                            return True
                        if self.stationary:
                            observer_start(self.controller.snapshot(),self.controller.clock())
                            self.arm=ArrivalMonitor(self.endpoint,started_at=self.controller.clock(),**self.arm_policy)
                            self.stage='settling_observer'
                        else:
                            trajectory={k:self.plan[k] for k in ('interpolation','time_from_start_s','joint_waypoints_rad')}
                            self.controller.start_trajectory('right',trajectory,self.token,preflight=observer_start)
                elif self.stage=='observing_motion':
                    r=self.controller.poll_trajectory(self.token,preflight=self._guard)
                    if r['status']=='reference_complete':
                        self.arm=ArrivalMonitor(self.endpoint,started_at=now,**self.arm_policy)
                        self.stage='settling_observer'
                elif self.stage=='settling_observer':
                    r=self.arm.update(q,stamp,now)
                    if r['stop_required']:return self._abort(r['status'])
                    if r['arrived']:self.stage='ready_to_observe'
                elif np.any(np.abs(q-self.endpoint)>self.arm.tolerance):
                    raise ValueError('Observer moved after settling')
            except Exception as error:
                self.controller.log('粗对位中止：'+str(error),'warning')
                return self._abort('coarse_feedback_failed')
            return self._receipt()
