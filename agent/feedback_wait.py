"""Pause one owned grasp operation on delayed feedback; never reuse a stopped path."""
import numpy as np
from .motion_feedback import ArrivalMonitor


def feedback_pending(controller, policy, *, paired=True):
    snapshot=controller.snapshot();now=controller.clock()
    arm=snapshot['arms']['right'];body=snapshot['body']
    q=np.asarray(arm['joints'],float)
    stamps=np.asarray([arm['received'],body['received']],float)
    if (q.shape!=(6,) or not np.isfinite(q).all() or not np.isfinite(body['height'])
            or not np.isfinite(arm['gripper']) or not np.isfinite(stamps).all()
            or np.any(stamps>now)):
        raise ValueError('Invalid robot feedback')
    return (arm.get('fresh') is not True or body.get('fresh') is not True
            or np.any(now-stamps>policy['arm_policy']['max_age_s'])
            or paired and np.ptp(stamps)>policy['max_pair_skew_s'])


class FeedbackWaitStep:
    def __init__(self,step,controller,token,policy,operation,restart):
        self.step=step;self.controller=controller;self.token=token
        self.policy=policy;self.operation=operation;self.restart=restart
        self.started=False;self.waiting=False;self.stopped_path=False;self.trace=[]

    def _tick(self):
        if not self.controller.motion_valid(self.token):
            return self.step.cancel()
        paired=self.operation in ('execute_battery_approach','execute_pick')
        if feedback_pending(self.controller,self.policy,paired=paired):
            if not self.waiting:
                self.stopped_path=self.controller.pause_trajectory_for_feedback(self.token)
                self.waiting=True
                self.controller.log('夹取反馈过期或不同步：保留任务，停车等待新反馈')
            retreat=getattr(self.step,'retreat',None)
            if retreat is not None:
                retreat.pose_wait.begin(retreat.moved);retreat.pose_wait.hold()
            return dict(status='waiting_for_feedback',waiting_for_feedback=True,stop_required=False)
        q=list(self.controller.snapshot()['arms']['right']['joints'])
        if not self.trace or q!=self.trace[-1]:self.trace.append(q)
        if self.waiting:
            self.controller.log('夹取反馈恢复：重新执行原有校验')
            self.waiting=False
            if self.stopped_path and paired:
                return dict(status='feedback_replan_required',stop_required=False,
                    feedback_replan_required=True,return_trace_rad=self.trace)
            if self.stopped_path:
                self.step=self.restart();self.started=False;self.stopped_path=False
            monitor=getattr(self.step,'monitor',None)
            if monitor is not None:
                now=self.controller.clock();monitor.last_sample=None
                monitor.arm=ArrivalMonitor(np.zeros(6),started_at=now,**monitor.arm_policy)
                monitor.gripper=ArrivalMonitor([monitor.gripper_target],started_at=now,**monitor.gripper_policy)
                if hasattr(self.step,'arrival') and not self.step.approaching:
                    self.step.arrival=ArrivalMonitor([2.9],minimum=[2.9],started_at=now,**monitor.gripper_policy)
            if hasattr(self.step,'return_monitor') and self.step.return_monitor is not None:
                previous=self.step.return_monitor
                self.step.return_monitor=ArrivalMonitor(previous.target,started_at=self.controller.clock(),
                                                       **self.step.arm_policy)
        if not self.started:
            self.started=True
            return self.step.start()
        return self.step.poll()

    def start(self):return self._tick()
    def poll(self):return self._tick()
    def cancel(self):return self.step.cancel()
    def timeout(self):return self.step.timeout()
