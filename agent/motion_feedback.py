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
    def __init__(self,target,tolerance,*,started_at,settle_s,timeout_s,stall_s,max_age_s,max_speed,minimum=None):
        self.target=np.asarray(target,float).copy()
        self.tolerance=np.asarray(tolerance,float).copy()
        self.max_speed=np.asarray(max_speed,float).copy()
        self.minimum=None if minimum is None else np.asarray(minimum,float).copy()
        if (self.target.ndim!=1 or not self.target.size
                or self.tolerance.shape!=self.target.shape or self.max_speed.shape!=self.target.shape
                or (self.minimum is not None and (self.minimum.shape!=self.target.shape
                                                  or not np.isfinite(self.minimum).all()))
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
        def accepted(value):
            return np.all(value>=self.minimum) if self.minimum is not None else np.all(np.abs(value-self.target)<=self.tolerance)
        inside=bool(accepted(value))
        continuous=self.previous_time is not None and sample_time-self.previous_time<=self.max_age
        stationary=continuous and np.all(np.abs(value-self.previous)/(sample_time-self.previous_time)<=self.max_speed)
        if inside and stationary and accepted(self.previous):
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
    rotation=Rotation.from_matrix(target[:3,:3].T@actual[:3,:3])
    angle=np.degrees(rotation.magnitude())
    return dict(base_from_tool=actual.tolist(),position_error_mm=((actual[:3,3]-target[:3,3])*1000).tolist(),
                orientation_error_deg=float(angle),orientation_error_rotvec_deg=np.degrees(rotation.as_rotvec()).tolist(),
                joints_rad=q.tolist(),sample_time=float(stamps.min()))


class PickMonitor:
    """Insertion/closure feedback sequence for the executor, with no command transport.

    start_closing grants one closure only on a new, still-settled sample. The caller
    must submit immediately under its control lock, and abort on an uncertain reply.
    This monitor neither owns the controller nor proves collision clearance or hold.
    """
    def __init__(self,model,wrist_from_tool,target_base_tool,*,expected_lift_rad,
                 lift_tolerance_rad,max_pair_skew_s,started_at,arm_policy,
                 gripper_target,gripper_policy,hold_gripper_range):
        if wrist_from_tool is None:raise ValueError('TCP pending')
        if (not np.isfinite([expected_lift_rad,lift_tolerance_rad,max_pair_skew_s]).all()
                or not 0<=expected_lift_rad<=18 or min(lift_tolerance_rad,max_pair_skew_s)<=0):
            raise ValueError('Invalid lift or pairing policy')
        self.model=model;self.tcp=rigid(wrist_from_tool).copy()
        self.target=rigid(target_base_tool).copy()
        self.feedback_policy=dict(expected_lift_rad=expected_lift_rad,
            lift_tolerance_rad=lift_tolerance_rad,max_pair_skew_s=max_pair_skew_s,
            max_age_s=arm_policy['max_age_s'])
        self.arm_policy=dict(arm_policy)
        self.arm=ArrivalMonitor(np.zeros(6),started_at=started_at,**self.arm_policy)
        self.settling_started=False
        if self.arm.tolerance.shape!=(6,):raise ValueError('Arm policy needs XYZ and rotation-vector tolerances')
        self.gripper_target=float(gripper_target);self.gripper_policy=dict(gripper_policy)
        self.hold_range=np.asarray(hold_gripper_range,float)
        if (self.hold_range.shape!=(2,) or not np.isfinite(self.hold_range).all()
                or self.hold_range[0]>self.hold_range[1]):
            raise ValueError('Hold range must be [minimum, maximum]')
        # Validate both policies before any caller can start a physical operation.
        self.gripper=ArrivalMonitor([self.gripper_target],started_at=started_at,**self.gripper_policy)
        self.stage='inserting';self.last_poll=started_at
        self.last_value=None;self.last_sample=None;self.arrival_sample=None
        self.failure=None

    def begin_settling(self,now):
        """Start the arrival window once the reference trajectory has ended."""
        if (self.failure or self.stage!='inserting' or self.last_sample is not None
                or self.settling_started or not np.isfinite(now) or now<self.last_poll):
            raise ValueError('Cannot restart an active or interrupted arrival monitor')
        self.arm=ArrivalMonitor(np.zeros(6),started_at=now,**self.arm_policy)
        self.last_poll=now
        self.settling_started=True

    def _receipt(self,close=False):
        return dict(status=self.failure or self.stage,close_permitted=close,
                    stop_required=self.failure is not None,grasp_confirmed=False)

    def abort(self,reason):
        if self.failure is None:self.failure=str(reason) if reason else 'execution_unknown'
        return self._receipt()

    def _read(self,snapshot,now,ready,cancelled):
        if self.failure:return None
        if cancelled:self.abort('cancelled');return None
        if ready is not True:self.abort('driver_fault');return None
        if not np.isfinite(now) or now<self.last_poll:
            self.abort('invalid_clock');return None
        self.last_poll=now
        try:
            measured=tool_feedback(snapshot,self.model,self.tcp,self.target,now=now,**self.feedback_policy)
            value=np.r_[measured['position_error_mm'],measured['orientation_error_rotvec_deg']]
            stamp=measured['sample_time']
            if self.last_sample is not None and stamp<self.last_sample:raise ValueError('Out-of-order feedback')
            if self.stage!='inserting':
                if np.any(np.abs(value)>self.arm.tolerance):raise ValueError('Tool drifted after arrival')
                if self.last_sample is not None and stamp-self.last_sample>self.arm.max_age:
                    raise ValueError('Feedback gap after arrival')
            return value,stamp
        except (ValueError,TypeError,KeyError):
            self.abort('feedback_invalid');return None

    def update(self,snapshot,now,*,ready,cancelled=False):
        sample=self._read(snapshot,now,ready,cancelled)
        if sample is None:return self._receipt()
        value,stamp=sample
        self.last_value=value.copy();self.last_sample=stamp
        if self.stage=='inserting':
            r=self.arm.update(value,stamp,now,ready=ready,cancelled=cancelled)
            if r['stop_required']:return self.abort(r['status'])
            if r['arrived']:self.stage='ready_to_close';self.arrival_sample=stamp
        elif self.stage=='closing':
            try:
                arm=snapshot['arms']['right']
                value=float(arm['gripper'])
                r=self.gripper.update([value],arm['received'],now,ready=ready,cancelled=cancelled)
            except (KeyError,TypeError,ValueError):return self.abort('gripper_feedback_invalid')
            # A held object prevents full closure; a stable stop in the taught range is expected.
            if r['status']=='stalled' and self.hold_range[0]<=value<=self.hold_range[1]:
                self.stage='awaiting_hold_verification'
            elif r['stop_required']:return self.abort(r['status'])
            if r['arrived']:self.stage='awaiting_hold_verification'
        return self._receipt()

    def start_closing(self,snapshot,now,*,ready,cancelled=False):
        if self.failure:return self._receipt()
        if self.stage!='ready_to_close':return self.update(snapshot,now,ready=ready,cancelled=cancelled)
        sample=self._read(snapshot,now,ready,cancelled)
        if sample is None:return self._receipt()
        value,stamp=sample
        if stamp<=self.arrival_sample:return self._receipt()
        self.last_value=value.copy();self.last_sample=stamp
        self.gripper=ArrivalMonitor([self.gripper_target],started_at=now,**self.gripper_policy)
        self.stage='closing'
        return self._receipt(close=True)
