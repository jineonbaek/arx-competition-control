import unittest
import numpy as np
from upper_flow import OfflineFlow

def driver(epoch=2,instance='original',status='idle'):
    return dict(driver_instance=instance,command_epoch=epoch,status=status,enabled=True,ready=True,manual_joint_active=False)

class UpperFlowTests(unittest.TestCase):
    def start(self):
        f=OfflineFlow();f.bind(driver(),np.zeros(6),10.,100.)
        f.stage_coarse(dict(executable=False,gripper_action='retain',native_plan=dict(interpolation='segment_quintic_scalar',joint_waypoints_rad=[[0.]*6,[.01]*6],time_from_start_s=[0.,1.])))
        return f
    def test_original_driver_needs_no_new_telemetry(self):
        f=self.start();self.assertEqual(f.phase,'coarse_planned')
        self.assertNotIn('gripper',f.coarse['native_plan'])
    def test_reference_complete_is_not_arrival(self):
        f=self.start()
        with self.assertRaises(ValueError):f.arrival(driver(3,status='reference_complete'),np.ones(6),10.,102.)
        self.assertEqual(f.phase,'coarse_planned')
    def test_driver_restart_rejected(self):
        f=self.start()
        with self.assertRaises(ValueError):f.arrival(driver(3,instance='new',status='reference_complete'),np.full(6,.01),10.,102.)
    def test_pre_arrival_frame_rejected(self):
        f=self.start();f.arrival(driver(3,status='reference_complete'),np.full(6,.01),10.,102.)
        with self.assertRaises(ValueError):f.check_fine_frame(dict(camera_id='d405',image_time=101.,arm_time=101.,body_time=101.,joints=np.full(6,.01),lift_rad=10.),102.)
    def test_matching_post_arrival_frame(self):
        f=self.start();f.arrival(driver(3,status='reference_complete'),np.full(6,.01),10.,102.)
        f.check_fine_frame(dict(camera_id='d405',image_time=103.,arm_time=103.,body_time=103.,joints=np.full(6,.01),lift_rad=10.),103.)
        self.assertFalse(f.report()['executable']);self.assertFalse(f.report()['robot_motion'])
    def test_wrong_epoch_rejected(self):
        f=self.start()
        with self.assertRaises(ValueError):f.arrival(driver(8,status='reference_complete'),np.full(6,.01),10.,102.)
    def test_gripper_command_rejected(self):
        f=OfflineFlow();f.bind(driver(),np.zeros(6),10.,100.)
        with self.assertRaises(ValueError):f.stage_coarse(dict(gripper_action='open',native_plan={}))

if __name__=='__main__':unittest.main()
