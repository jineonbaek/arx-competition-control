"""Extract through the wrist interface, then return joints without opening the gripper."""
import numpy as np
from scipy.spatial.transform import Rotation
from .motion_feedback import ArrivalMonitor
from .chassis_retreat import ChassisRetreatStep


class WristExtractionStep:
    def __init__(self,controller,token,*,refinement,arm_policy):
        self.controller=controller;self.token=token
        self.position_tolerance=np.asarray(refinement['position_tolerance_mm'],float)
        self.orientation_tolerance=float(refinement['orientation_tolerance_deg'])
        self.arm_policy=dict(arm_policy)
        self.carry_joints_deg=[0.,0.,0.,0.,0.,-90.]
        self.return_monitor=None
        self.retreat=None
        self.target=None;self.started=False;self.failure=None;self.stop_request=None

    def _receipt(self,status):
        return dict(status=self.failure or status,stop_required=self.failure is not None,
            stop_request=self.stop_request,grasp_confirmed=False,
            target_transport_joints_deg=self.carry_joints_deg[:],
            target_wrist_base_mm=None if self.target is None else (self.target[:3,3]*1000).tolist())

    def _snapshot(self):
        if not self.controller.motion_valid(self.token):raise ValueError('Motion interrupted')
        snapshot=self.controller.snapshot();arm=snapshot['arms']['right'];body=snapshot['body']
        if arm.get('fresh') is not True or body.get('fresh') is not True:
            raise ValueError('Fresh arm and lift feedback required')
        return snapshot

    def _pose(self):
        snapshot=self._snapshot()
        return self.controller.models['right'].wrist_transform(
            snapshot['arms']['right']['joints'],snapshot['body']['height'])

    def start(self):
        if self.started or self.failure:raise ValueError('Extraction cannot be replayed')
        self.started=True
        try:
            self.target=self._pose().copy();self.target[0,3]-=.155
            command=dict(type='arm.wrist',arm='right',frame='base_link',
                position_mm=(self.target[:3,3]*1000).tolist(),
                orientation_deg=Rotation.from_matrix(self.target[:3,:3]).as_euler('xyz',degrees=True).tolist())
            if self.controller.submit(command,self.token).get('ok') is not True:
                raise ValueError('Extraction submission unconfirmed')
            return self._receipt('withdrawing')
        except Exception:return self.cancel('extraction_start_failed')

    def poll(self):
        if not self.started:raise ValueError('Extraction not started')
        if self.failure:return self._receipt(self.failure)
        try:
            if self.return_monitor is not None:
                arm=self._snapshot()['arms']['right']
                result=self.return_monitor.update(arm['joints'],arm['received'],
                    self.controller.clock(),ready=True)
                if result['stop_required']:return self.cancel(result['status'])
                return self._receipt('carry_ready' if result['arrived'] else 'returning')
            if self.retreat is None:
                actual=self._pose()
                position=np.abs(actual[:3,3]-self.target[:3,3])*1000
                angle=np.degrees(Rotation.from_matrix(self.target[:3,:3].T@actual[:3,:3]).magnitude())
                arrived=np.all(position<=self.position_tolerance) and angle<=self.orientation_tolerance
                if not arrived:return self._receipt('withdrawing')
                self.retreat=ChassisRetreatStep(self.controller,self.token)
                self.retreat.start()
                return self._receipt('backing_chassis')
            self._snapshot()
            if self.retreat.poll()['status']!='chassis_stopped':return self._receipt('backing_chassis')
            self.return_monitor=ArrivalMonitor(np.radians(self.carry_joints_deg),
                started_at=self.controller.clock(),**self.arm_policy)
            command=dict(type='arm.joints',arm='right',joints=self.carry_joints_deg[:])
            if self.controller.submit(command,self.token).get('ok') is not True:
                raise ValueError('Transport return submission unconfirmed')
            return self._receipt('returning')
        except Exception:return self.cancel('extraction_feedback_failed')

    def cancel(self,reason='cancelled'):
        if self.failure is None:
            self.failure=reason
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
