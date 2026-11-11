"""Feedback-only arrival checks. This module cannot submit or stop hardware commands."""
import numpy as np
from scipy.spatial.transform import Rotation
from .grasp_geometry import rigid


class ArrivalMonitor:
    """One command's post-submit samples, using the controller's monotonic clock.

    Tolerances and speed limits are supplied by a validated execution policy, never
    inferred from a model response. A terminal receipt cannot restart this monitor.
    stop_required requests a future executor stop; it does not mean hardware stopped.
    """
    def __init__(self,target,tolerance,*,started_at,settle_s,timeout_s,stall_s,max_age_s,max_speed):
        self.target=np.asarray(target,float).copy()
        self.tolerance=np.asarray(tolerance,float).copy()
        self.max_speed=np.asarray(max_speed,float).copy()
        if (self.target.ndim!=1 or not self.target.size
                or self.tolerance.shape!=self.target.shape or self.max_speed.shape!=self.target.shape
                or not np.isfinite(np.r_[self.target,self.tolerance,self.max_speed]).all()
                or np.any(self.tolerance<=0) or np.any(self.max_speed<=0)
                or not np.isfinite([started_at,settle_s,timeout_s,stall_s,max_age_s]).all()
                or min(settle_s,timeout_s,stall_s,max_age_s)<=0
                or settle_s>=timeout_s or stall_s>=timeout_s):
            raise ValueError('Invalid arrival policy; explicit finite tolerances and timings required')
        self.start=float(started_at);self.settle=float(settle_s);self.timeout=float(timeout_s)
        self.stall=float(stall_s);self.max_age=float(max_age_s)
        self.previous=None;self.previous_time=None;self.last_poll=self.start
        self.stable_since=None;self.progress_at=self.start;self.best_error=float('inf')
        self.terminal=None

    def _result(self,status,*,sample_time=None,error=None):
        r=dict(status=status,arrived=status=='arrived',
            stop_required=status not in ('waiting','arrived'),sample_time=sample_time,
            error=None if error is None else error.tolist())
        if status!='waiting':self.terminal=r
        return r.copy()

    def update(self,value,sample_time,now,*,ready=True,cancelled=False):
        if self.terminal is not None:return self.terminal.copy()
        if cancelled:return self._result('cancelled')
        if ready is not True:return self._result('driver_fault')
        try:
            value=np.asarray(value,float)
            if (value.shape!=self.target.shape or not np.isfinite(value).all()
                    or not np.isfinite([sample_time,now]).all() or now<self.last_poll
                    or sample_time>now or (self.previous_time is not None and sample_time<self.previous_time)):
                return self._result('invalid_feedback')
        except (ValueError,TypeError):return self._result('invalid_feedback')
        self.last_poll=float(now)
        if now-sample_time>self.max_age:return self._result('feedback_lost')
        if now-self.start>=self.timeout:return self._result('timeout')
        error=value-self.target
        if sample_time<=self.start:return self._result('waiting',error=error)
        if sample_time==self.previous_time:return self._result('waiting',error=error)
        normalized=float(np.max(np.abs(error)/self.tolerance))
        if normalized<self.best_error-.1:
            self.best_error=normalized;self.progress_at=float(sample_time)
        inside=normalized<=1
        continuous=self.previous_time is not None and sample_time-self.previous_time<=self.max_age
        stationary=continuous and np.all(np.abs(value-self.previous)/(sample_time-self.previous_time)<=self.max_speed)
        if inside and stationary and np.all(np.abs(self.previous-self.target)<=self.tolerance):
            if self.stable_since is None:self.stable_since=self.previous_time
        else:self.stable_since=None
        self.previous=value.copy();self.previous_time=float(sample_time)
        if self.stable_since is not None and sample_time-self.stable_since>=self.settle:
            return self._result('arrived',sample_time=float(sample_time),error=error)
        if not inside and sample_time-self.progress_at>=self.stall:
            return self._result('stalled',sample_time=float(sample_time),error=error)
        return self._result('waiting',sample_time=float(sample_time),error=error)


def tool_feedback(snapshot,model,wrist_from_tool,target_base_tool,*,now,max_age_s,
                  max_pair_skew_s,expected_lift_rad,lift_tolerance_rad):
    """Compute measured TCP error from the existing console snapshot, not SDK-zero XYZ."""
    if wrist_from_tool is None:raise ValueError('TCP pending')
    tcp,target=rigid(wrist_from_tool),rigid(target_base_tool)
    if (not np.isfinite([now,max_age_s,max_pair_skew_s,expected_lift_rad,lift_tolerance_rad]).all()
            or min(max_age_s,max_pair_skew_s,lift_tolerance_rad)<=0
            or not 0<=expected_lift_rad<=18):raise ValueError('Invalid feedback policy')
    if snapshot.get('chassis_active') is not False:raise ValueError('Chassis command is active or unknown')
    try:
        arm=snapshot['arms']['right'];body=snapshot['body']
        q=np.asarray(arm['joints'],float);height=float(body['height'])
        stamps=np.array([arm['received'],body['received']],float)
        if (q.shape!=(6,) or not np.isfinite(q).all() or not np.isfinite(height)
                or not 0<=height<=18 or not np.isfinite(stamps).all()
                or arm.get('fresh') is not True or body.get('fresh') is not True
                or np.any(stamps>now) or np.any(now-stamps>max_age_s)
                or np.ptp(stamps)>max_pair_skew_s):raise ValueError('Missing fresh paired arm and lift feedback')
        if abs(height-expected_lift_rad)>lift_tolerance_rad:raise ValueError('Lift changed during arm motion')
    except (KeyError,TypeError) as error:raise ValueError('Invalid robot feedback') from error
    actual=rigid(model.wrist_transform(q,height)@tcp)
    angle=np.degrees(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude())
    return dict(base_from_tool=actual.tolist(),position_error_mm=((actual[:3,3]-target[:3,3])*1000).tolist(),
                orientation_error_deg=float(angle),joints_rad=q.tolist(),sample_time=float(stamps.min()))
