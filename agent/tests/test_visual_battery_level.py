import unittest,time,tempfile,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from agent.visual_battery_level import motion_limits,entry_joints_deg,visual_level_step,d435_observation

class WristModel:
    def wrist_transform(self,q,h):
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',q[3:]).as_matrix();return T

def pose(tilt=5.):
    # CAD +Y points downward when the battery is level.
    R=Rotation.from_euler('x',tilt,degrees=True).as_matrix()@np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])
    T=np.eye(4);T[:3,:3]=R
    return dict(camera='d435',id='exposure',image_time=time.time(),quality=dict(status='consistent'),T_base_from_object=T.tolist())

class LevelTests(unittest.TestCase):
    def test_foundationpose_result_is_bound_to_actual_d435_exposure_and_cad(self):
        p=pose();frame=dict(camera_id='d435',id=p['id'],image_time=p['image_time'],arm_time=p['image_time'],body_time=p['image_time'],base_from_color=np.eye(4))
        report=dict(p,status='complete',T_camera_from_object=p['T_base_from_object'],mesh_sha256='a'*64)
        r=d435_observation(frame,report,'a'*64)
        self.assertEqual(r['camera'],'d435');self.assertEqual(r['image_time'],frame['image_time'])
        report['id']='other'
        with self.assertRaises(ValueError):d435_observation(frame,report,'a'*64)
    def test_visual_tilt_drives_wrist_even_when_robot_joints_report_level(self):
        r=visual_level_step(pose(),WristModel(),np.zeros(6),0.,np.eye(4),[[-180,180]]*6)
        self.assertEqual(r['action'],'correct');self.assertGreater(r['tilt_deg'],4.9)
        self.assertLess(r['predicted_tilt_deg'],r['tilt_deg'])
        np.testing.assert_allclose(r['joint_delta_rad'][:3],[0.,0.,0.])
        self.assertLessEqual(np.max(np.abs(np.degrees(r['joint_delta_rad']))),3.0001)
        self.assertEqual(r['gripper_action'],'retain')
    def test_camera_not_fk_decides_already_level(self):
        q=np.zeros(6);q[3]=.15
        self.assertEqual(visual_level_step(pose(.4),WristModel(),q,0.,np.eye(4),[[-180,180]]*6)['action'],'hold')
    def test_stale_bad_fit_wrong_camera_and_inverted_pose_rejected(self):
        for kind in ['stale','fit','camera','inverted']:
            p=pose()
            if kind=='stale':p['image_time']-=10
            elif kind=='fit':p['quality']['status']='inconsistent'
            elif kind=='camera':p['camera']='d405'
            else:p=pose(180)
            with self.subTest(kind=kind),self.assertRaises(ValueError):
                visual_level_step(p,WristModel(),np.zeros(6),0.,np.eye(4),[[-180,180]]*6)
    def test_latest_repo_speeds_are_limited_by_loaded_driver_acceleration(self):
        policy=dict(timing_limits=dict(joint_speed_deg_s=[15.,15.,15.,50.,50.,50.],joint_acceleration_deg_s2=[28.]*6,tcp_speed_mm_s=60.,tcp_acceleration_mm_s2=120.))
        control=dict(arms=dict(right=dict(trajectory_policy=dict(speed_deg_s=50.,acceleration_deg_s2=18.))))
        r=motion_limits(policy,control)
        self.assertEqual(r['joint_speed_deg_s'],[15.,15.,15.,50.,50.,50.]);self.assertEqual(r['joint_acceleration_deg_s2'],[18.]*6)
        self.assertEqual(entry_joints_deg(),[0.,0.,0.,0.,0.,-90.])
