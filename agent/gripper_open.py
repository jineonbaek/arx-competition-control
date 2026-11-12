"""Open through the existing Controller before a checked approach; no device setup."""
from threading import Lock
import numpy as np
from .motion_feedback import ArrivalMonitor, tool_feedback


class OpenThenApproachStep:
    def __init__(self, approach, start_joints):
        self.approach=approach;self.controller=approach.controller
        self.token=approach.token;self.monitor=approach.monitor
        self.preflight=approach.preflight
        q=np.asarray(start_joints,float)
        config=self.controller.config['arms']['right']
        self.target=float(config['gripper_open_raw'])
        low,high=config['gripper_limits_raw']
        if q.shape!=(6,) or not np.isfinite(q).all() or not np.isfinite(self.target) or not low<=self.target<=high:
            raise ValueError('Finite planned start and configured open target required')
        self.start_pose=self.monitor.model.wrist_transform(q,self.monitor.feedback_policy['expected_lift_rad'])@self.monitor.tcp
        self.arrival=ArrivalMonitor([self.target],started_at=self.controller.clock(),**self.monitor.gripper_policy)
        self.lock=Lock();self.started=False;self.approaching=False;self.failure=None
        # The driver submission and subsequent approach polls must also keep the jaws open.
        self.approach.preflight=self._approach_ready

    def _open(self,snapshot,now):
        arm=snapshot['arms']['right']
        return (arm.get('fresh') is True and 0<=now-arm['received']<=self.arrival.max_age
                and np.isfinite(arm['gripper']) and abs(arm['gripper']-self.target)<=self.arrival.tolerance[0])

    def _approach_ready(self,snapshot,now):
        return self._open(snapshot,now) and self.preflight(snapshot,now) is True

    def _guard(self,snapshot,now):
        if (not self.controller.motion_valid(self.token)
                or snapshot.get('capabilities',{}).get('right:gripper')!=''
                or snapshot.get('capabilities',{}).get('right:arm.wrist')!=''
                or self.preflight(snapshot,now) is not True):
            return False
        measured=tool_feedback(snapshot,self.monitor.model,self.monitor.tcp,self.start_pose,
                               now=now,**self.monitor.feedback_policy)
        error=np.r_[measured['position_error_mm'],measured['orientation_error_rotvec_deg']]
        return bool(np.all(np.abs(error)<=self.monitor.arm.tolerance))

    def _abort(self,reason):
        if self.failure is None:
            self.failure=dict(status=reason,stop_required=True,grasp_confirmed=False,close_permitted=False)
            try:self.failure={**self.approach.cancel(),**self.failure}
            except Exception as error:
                self.failure['stop_request']='failed'
                self.controller.log('Opening stop request failed: '+str(error),'warning')
        return self.failure.copy()

    def start(self):
        with self.lock:
            if self.started or self.failure:raise ValueError('Opening cannot be replayed or resumed')
            self.started=True
            try:
                self.arrival=ArrivalMonitor([self.target],started_at=self.controller.clock(),**self.monitor.gripper_policy)
                result=self.controller.submit({'type':'gripper','arm':'right','position':self.target},
                                              self.token,preflight=self._guard)
                if result.get('ok') is not True:return self._abort('opening_not_confirmed')
                if not self.controller.motion_valid(self.token):return self._abort('interrupted')
                return dict(status='opening_gripper',stop_required=False,close_permitted=False,grasp_confirmed=False)
            except Exception as error:
                self.controller.log('Gripper opening failed: '+str(error),'warning')
                return self._abort('execution_unknown')

    def cancel(self):
        with self.lock:return self._abort('cancelled')

    def poll(self,*,cancelled=False):
        with self.lock:
            if self.failure:return self.failure.copy()
            if cancelled:return self._abort('cancelled')
            if not self.started:raise ValueError('Opening has not started')
            if self.approaching:return self.approach.poll()
            try:
                snapshot=self.controller.snapshot();now=self.controller.clock()
                if not self._guard(snapshot,now):return self._abort('opening_preflight_failed')
                arm=snapshot['arms']['right']
                receipt=self.arrival.update([arm['gripper']],arm['received'],now)
                if receipt['stop_required']:return self._abort(receipt['status'])
                if receipt['arrived']:
                    self.approaching=True
                    return self.approach.start()
                return dict(status='opening_gripper',stop_required=False,close_permitted=False,grasp_confirmed=False)
            except Exception as error:
                self.controller.log('Gripper opening feedback failed: '+str(error),'warning')
                return self._abort('feedback_invalid')
