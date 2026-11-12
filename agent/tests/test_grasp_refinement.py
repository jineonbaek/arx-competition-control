"""Measured six-dimensional corrections with bounded steps and no observation budget."""
import copy
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from agent.grasp_refinement import assess


class RefinementTests(unittest.TestCase):
    def setUp(self):
        self.policy=dict(max_step_mm=10.,max_rotation_step_deg=5.,
                         position_tolerance_mm=[18.]*3,orientation_tolerance_deg=5.)
        self.old=dict(head_observation_id='head',observation_id='old',snapshot_id='old-frame',
            arm='right',frame='base_link',unit='mm',lift_rad=12.,
            contact_path=dict(kind='battery_approach_axis',direction_base=[1.,0.,0.],
                pre_mm=[400.,-250.,700.],end_mm=[450.,-250.,700.]),
            T_wrist_from_tool=np.eye(4).tolist(),R_base_from_tool=np.eye(3).tolist(),
            calibration_id='test',pick_setup_sha256='tcp',handeye_sha256='he',profile_sha256='cad',
            urdf_sha256='urdf',control_config_sha256='config')
        self.new=copy.deepcopy(self.old);self.new.update(observation_id='new',snapshot_id='new-frame')
        actual=np.eye(4);actual[:3,3]=[.4,-.25,.7]
        self.new['actual_base_from_tool']=actual.tolist()

    def decision(self):return assess(self.new,self.old,self.policy,corrections=50,total_mm=500.)

    def test_xyz_and_orientation_tolerance_are_inclusive_and_both_required(self):
        self.new['actual_base_from_tool'][0][3]-=.018
        rotation=Rotation.from_euler('x',5.,degrees=True).as_matrix()
        for i in range(3):self.new['actual_base_from_tool'][i][:3]=rotation[i].tolist()
        self.assertEqual(self.decision()['status'],'aligned')
        self.new['actual_base_from_tool'][0][3]-=.00001
        self.assertEqual(self.decision()['status'],'correction_required')

    def test_translation_is_clipped_by_three_dimensional_norm_not_each_axis(self):
        for i in range(3):self.new['actual_base_from_tool'][i][3]-=.03
        result=self.decision();step=np.array(result['correction_mm'])
        self.assertEqual(result['status'],'correction_required')
        self.assertAlmostEqual(np.linalg.norm(step),10.,places=8)
        np.testing.assert_allclose(step,np.ones(3)*10/np.sqrt(3))

    def test_pure_rotation_corrects_even_with_zero_translation(self):
        rotation=Rotation.from_euler('x',12.,degrees=True).as_matrix()
        for i in range(3):self.new['actual_base_from_tool'][i][:3]=rotation[i].tolist()
        result=self.decision()
        self.assertEqual(result['status'],'correction_required')
        self.assertAlmostEqual(np.linalg.norm(result['correction_rotation_deg']),5.)
        np.testing.assert_allclose(result['correction_mm'],[0,0,0])

    def test_simulated_joint_xyz_and_rotation_loop_converges_and_stops_within_tolerance(self):
        actual=np.eye(4);actual[:3,3]=[.35,-.28,.67]
        actual[:3,:3]=Rotation.from_euler('x',21.,degrees=True).as_matrix()
        steps=0
        while True:
            self.new['actual_base_from_tool']=actual.tolist()
            result=self.decision()
            if result['status']=='aligned':break
            self.assertLessEqual(np.linalg.norm(result['correction_mm']),10.+1e-9)
            self.assertLessEqual(np.linalg.norm(result['correction_rotation_deg']),5.+1e-9)
            actual=np.asarray(result['correction_target_base_from_tool'])
            steps+=1;self.assertLess(steps,10)
        self.assertGreater(steps,3)
        self.assertTrue(np.all(np.abs(result['measured_pregrasp_error_mm'])<=18))
        self.assertLessEqual(result['orientation_error_deg'],5.)

    def test_new_pose_and_actual_lift_are_used_without_previous_small_shift_gates(self):
        self.new['lift_rad']+=.025
        for key in ('pre_mm','end_mm'):self.new['contact_path'][key][0]+=30
        self.assertEqual(self.decision()['status'],'correction_required')

    def test_identity_tcp_and_fresh_evidence_still_required(self):
        for key in ('head_observation_id','calibration_id','pick_setup_sha256','handeye_sha256',
                    'profile_sha256','urdf_sha256','control_config_sha256','frame','unit','arm'):
            new=copy.deepcopy(self.new);new[key]='changed'
            with self.assertRaises(ValueError):assess(new,self.old,self.policy,corrections=0,total_mm=0.)
        for key in ('observation_id','snapshot_id'):
            new=copy.deepcopy(self.new);new[key]=self.old[key]
            with self.assertRaises(ValueError):assess(new,self.old,self.policy,corrections=0,total_mm=0.)
        self.new['T_wrist_from_tool'][0][3]=.001
        with self.assertRaises(ValueError):self.decision()

    def test_invalid_policy_and_broken_insertion_line_are_rejected(self):
        for key,value in [('max_step_mm',0),('max_step_mm',10.1),
                          ('max_rotation_step_deg',float('nan')),('position_tolerance_mm',[0,1,1])]:
            p=dict(self.policy,**{key:value})
            with self.assertRaises(ValueError):assess(self.new,self.old,p,corrections=0,total_mm=0.)
        self.new['contact_path']['pre_mm'][0]+=1
        with self.assertRaises(ValueError):self.decision()


if __name__=='__main__':unittest.main()
