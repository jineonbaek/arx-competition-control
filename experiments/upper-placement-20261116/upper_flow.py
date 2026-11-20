"""Offline stage coordination for the unchanged driver. No ROS or HTTP writes."""
import copy
import numpy as np
from fine_bridge import fine_target

class OfflineFlow:
    def __init__(self):
        self.phase='new';self.coarse=None

    @staticmethod
    def ready(d):
        if (not isinstance(d.get('driver_instance'),str) or not d['driver_instance']
            or type(d.get('command_epoch')) is not int or d['command_epoch']<0
            or d.get('enabled') is not True or d.get('ready') is not True
            or d.get('manual_joint_active') is not False):
            raise ValueError('Original driver unavailable')

    def bind(self,driver,q,height,stamp):
        if self.phase!='new':raise ValueError('Session already bound')
        self.ready(driver)
        if driver.get('status') not in ('idle','reference_complete'):raise ValueError('Driver not idle')
        q=np.asarray(q,float)
        if q.shape!=(6,) or not np.isfinite(np.r_[q,height,stamp]).all():raise ValueError('Invalid measured state')
        self.driver=copy.deepcopy(driver);self.q=q.copy();self.height=float(height);self.stamp=float(stamp);self.phase='bound'

    def stage_coarse(self,proposal):
        if self.phase!='bound' or proposal.get('gripper_action')!='retain':raise ValueError('Wrong stage or clamp action')
        p=proposal['native_plan']
        if set(p)!={'interpolation','joint_waypoints_rad','time_from_start_s'} or p['interpolation']!='segment_quintic_scalar':raise ValueError('Invalid trajectory fields')
        q=np.asarray(p['joint_waypoints_rad'],float);t=np.asarray(p['time_from_start_s'],float)
        if (q.ndim!=2 or q.shape[1]!=6 or len(q)<2 or t.shape!=(len(q),)
            or not np.isfinite(q).all() or not np.isfinite(t).all() or t[0]!=0 or np.any(np.diff(t)<=0)
            or np.max(abs(q[0]-self.q))>np.radians(.1)):
            raise ValueError('Trajectory invalid or start changed')
        self.coarse=copy.deepcopy(proposal);self.phase='coarse_planned'

    def arrival(self,driver,q,height,stamp):
        if self.phase!='coarse_planned':raise ValueError('Coarse path missing')
        self.ready(driver);q=np.asarray(q,float)
        if (driver['driver_instance']!=self.driver['driver_instance']
            or driver['command_epoch']!=self.driver['command_epoch']+1
            or driver.get('status')!='reference_complete' or q.shape!=(6,)
            or not np.isfinite(np.r_[q,height,stamp]).all() or stamp<=self.stamp
            or abs(height-self.height)>.05
            or np.max(abs(q-np.asarray(self.coarse['native_plan']['joint_waypoints_rad'][-1])))>np.radians(.5)):
            raise ValueError('Reference complete without matching measured arrival')
        self.q=q.copy();self.height=float(height);self.arrived_at=float(stamp);self.phase='await_d405'

    def check_fine_frame(self,frame,now):
        if self.phase!='await_d405':raise ValueError('No measured outside arrival')
        # Reuse temporal/state validation with a zero observation; this does not
        # manufacture an image measurement or approve the resulting path.
        fine_target(frame,self.q,self.height,self.arrived_at,now,np.eye(4),lambda f:dict(step_mm=0.))

    def plan_fine(self,frame,now,model,tcp,limits,timing,estimate):
        from vision_path_bridge import fine
        self.check_fine_frame(frame,now)
        self.fine=fine(frame,model,tcp,self.q,self.height,self.arrived_at,now,limits,timing,estimate)
        self.phase='fine_planned';return copy.deepcopy(self.fine)

    def report(self):
        return dict(phase=self.phase,executable=False,robot_motion=False,gripper_action='retain',
                    driver_contract='original_status_and_measured_joints',
                    note='Offline only; no hardware executor, no insertion, no driver modification')
