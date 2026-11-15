"""Extract by chassis retreat, then return joints without opening the gripper."""
import numpy as np
from .motion_feedback import ArrivalMonitor
from .chassis_retreat import ChassisRetreatStep
from .grasp_path import TimedJointPath


class WristExtractionStep:
    def __init__(self,controller,token,*,refinement,arm_policy,timing_limits,wrist_from_tool):
        self.controller=controller;self.token=token
        self.arm_policy=dict(arm_policy)
        self.timing_limits=dict(timing_limits);self.tcp=np.asarray(wrist_from_tool,float).copy()
        self.returning=False;self.reference_complete=False
        self.carry_joints_deg=[0.,0.,0.,0.,0.,-90.]
        self.return_monitor=None
        self.retreat=None
        self.started=False;self.failure=None;self.failure_detail=None;self.stop_request=None

    def _receipt(self,status):
        return dict(status=self.failure or status,stop_required=self.failure is not None,
            stop_request=self.stop_request,failure_detail=self.failure_detail,grasp_confirmed=False,
            target_transport_joints_deg=self.carry_joints_deg[:],
            target_wrist_base_mm=None)

    def _snapshot(self):
        if not self.controller.motion_valid(self.token):raise ValueError('Motion interrupted')
        snapshot=self.controller.snapshot();arm=snapshot['arms']['right'];body=snapshot['body']
        if arm.get('fresh') is not True or body.get('fresh') is not True:
            raise ValueError('Fresh arm and lift feedback required')
        return snapshot

    def _return_preflight(self,snapshot,now):
        return (self.controller.motion_valid(self.token)
                and snapshot['arms']['right'].get('fresh') is True
                and snapshot['body'].get('fresh') is True)

    def start(self):
        if self.started or self.failure:raise ValueError('Extraction cannot be replayed')
        self.started=True
        try:
            self._snapshot()
            self.retreat=ChassisRetreatStep(self.controller,self.token)
            self.retreat.start()
            return self._receipt('backing_chassis')
        except Exception:return self.cancel('extraction_start_failed')

    def poll(self):
        if not self.started:raise ValueError('Extraction not started')
        if self.failure:return self._receipt(self.failure)
        try:
            if self.returning:
                arm=self._snapshot()['arms']['right']
                if not self.reference_complete:
                    result=self.controller.poll_trajectory(self.token,preflight=self._return_preflight)
                    if result['status']!='reference_complete':return self._receipt('returning')
                    self.reference_complete=True
                    self.return_monitor=ArrivalMonitor(np.radians(self.carry_joints_deg),
                        started_at=self.controller.clock(),**self.arm_policy)
                result=self.return_monitor.update(arm['joints'],arm['received'],
                    self.controller.clock(),ready=True)
                if result['stop_required']:return self.cancel(result['status'])
                return self._receipt('carry_ready' if result['arrived'] else 'returning')
            self._snapshot()
            if self.retreat.poll()['status']!='chassis_stopped':return self._receipt('backing_chassis')
            snapshot=self._snapshot()
            start=np.asarray(snapshot['arms']['right']['joints'],float)
            target=np.radians(self.carry_joints_deg)
            self.returning=True
            if np.array_equal(start,target):
                self.reference_complete=True
                self.return_monitor=ArrivalMonitor(target,started_at=self.controller.clock(),**self.arm_policy)
            else:
                model=self.controller.models['right'];height=snapshot['body']['height']
                path=TimedJointPath([start,target],model.reach+np.linalg.norm(self.tcp[:3,3]),
                    tcp_position=lambda q:(model.wrist_transform(q,height)@self.tcp)[:3,3],**self.timing_limits)
                plan=dict(interpolation='segment_quintic_scalar',
                    time_from_start_s=path.times.tolist(),joint_waypoints_rad=path.q.tolist())
                self.controller.start_trajectory('right',plan,self.token,
                    preflight=self._return_preflight,require_lift_settle=False)
            return self._receipt('returning')
        except Exception as error:return self.cancel('extraction_feedback_failed',error)

    def cancel(self,reason='cancelled',error=None):
        if self.failure is None:
            self.failure=reason
            if error is not None:
                self.failure_detail=f'{type(error).__name__}: {error}'[:240]
                try:self.controller.log('stow_for_carry failed: '+self.failure_detail)
                except Exception:pass

            if self.retreat is not None:
                try:self.retreat.cancel()
                except Exception:self.stop_request='failed'
            if self.started:
                self.controller.interrupt_motion(self.token)
                self.stop_request='failed'
                try:
                    if self.controller.submit({'type':'arm.stop','arm':'right'},self.token).get('ok') is True:
                        self.stop_request='sent_unverified'
                except Exception:pass
        return self._receipt(self.failure)

    def timeout(self):return self.cancel('timeout')
