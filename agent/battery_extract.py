"""One manual-interface wrist move after grasping; never changes the gripper."""
import numpy as np
from scipy.spatial.transform import Rotation


class WristExtractionStep:
    def __init__(self,controller,token,*,refinement):
        self.controller=controller;self.token=token
        self.position_tolerance=np.asarray(refinement['position_tolerance_mm'],float)
        self.orientation_tolerance=float(refinement['orientation_tolerance_deg'])
        self.target=None;self.started=False;self.failure=None;self.stop_request=None

    def _receipt(self,status):
        return dict(status=self.failure or status,stop_required=self.failure is not None,
            stop_request=self.stop_request,grasp_confirmed=False,
            target_wrist_base_mm=None if self.target is None else (self.target[:3,3]*1000).tolist())

    def _pose(self):
        if not self.controller.motion_valid(self.token):raise ValueError('Motion interrupted')
        snapshot=self.controller.snapshot();arm=snapshot['arms']['right'];body=snapshot['body']
        if arm.get('fresh') is not True or body.get('fresh') is not True:
            raise ValueError('Fresh arm and lift feedback required')
        return self.controller.models['right'].wrist_transform(arm['joints'],body['height'])

    def start(self):
        if self.started or self.failure:raise ValueError('Extraction cannot be replayed')
        self.started=True
        try:
            self.target=self._pose().copy();self.target[0,3]-=.200
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
            actual=self._pose()
            position=np.abs(actual[:3,3]-self.target[:3,3])*1000
            angle=np.degrees(Rotation.from_matrix(self.target[:3,:3].T@actual[:3,:3]).magnitude())
            arrived=np.all(position<=self.position_tolerance) and angle<=self.orientation_tolerance
            return self._receipt('withdrawn' if arrived else 'withdrawing')
        except Exception:return self.cancel('extraction_feedback_failed')

    def cancel(self,reason='cancelled'):
        if self.failure is None:
            self.failure=reason
            if self.started:
                self.controller.interrupt_motion(self.token)
                self.stop_request='failed'
                try:
                    if self.controller.submit({'type':'arm.stop','arm':'right'},self.token).get('ok') is True:
                        self.stop_request='sent_unverified'
                except Exception:pass
        return self._receipt(self.failure)

    def timeout(self):return self.cancel('timeout')
