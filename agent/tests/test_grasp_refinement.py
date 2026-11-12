"""Finite post-approach correction decisions from frozen, calibrated evidence."""
import copy
import unittest
import numpy as np
from agent.grasp_refinement import assess

class RefinementTests(unittest.TestCase):
    def setUp(self):
        self.policy=dict(max_corrections=2,max_target_shift_mm=[8.,8.,8.],
            max_step_mm=[6.,6.,6.],max_total_correction_mm=10.,
            position_tolerance_mm=[.5,.5,.5],orientation_tolerance_deg=.3,lift_tolerance_rad=.02)
        self.old=dict(head_observation_id='head',observation_id='old',snapshot_id='old-frame',
            arm='right',frame='base_link',unit='mm',lift_rad=12.,
            contact_path=dict(axis='+X',fixed_yz=True,pre_mm=[400.,-250.,700.],end_mm=[450.,-250.,700.]),
            T_wrist_from_tool=np.eye(4).tolist(),R_base_from_tool=np.eye(3).tolist(),
            calibration_id='test',pick_setup_sha256='tcp',handeye_sha256='he',profile_sha256='cad',
            urdf_sha256='urdf',control_config_sha256='config')
        self.new=copy.deepcopy(self.old);self.new.update(observation_id='new',snapshot_id='new-frame')
        actual=np.eye(4);actual[:3,3]=[.4,-.25,.7]
        self.new['actual_base_from_tool']=actual.tolist()

    def test_aligned_result_requires_no_motion_and_locks_new_observation(self):
        r=assess(self.new,self.old,self.policy,corrections=0,total_mm=0.)
        self.assertEqual(r['status'],'aligned');self.assertFalse(r['executable'])
        self.assertEqual(r['observation_id'],'new');self.assertEqual(r['correction_mm'],[0.,0.,0.])
        self.assertEqual(r['remaining_corrections'],2)

    def test_xyz_correction_uses_measured_tcp_and_keeps_endpoint_height(self):
        self.new['contact_path']['pre_mm'][2]+=3
        self.new['contact_path']['end_mm'][2]+=3
        r=assess(self.new,self.old,self.policy,corrections=0,total_mm=0.)
        self.assertEqual(r['status'],'correction_required')
        np.testing.assert_allclose(r['correction_mm'],[0,0,3],atol=1e-9)
        self.assertEqual(r['proposed_total_correction_mm'],3.)
        self.assertFalse(r['executable'])

    def test_budget_is_bounded_but_aligned_last_observation_can_finish(self):
        self.assertEqual(assess(self.new,self.old,self.policy,corrections=2,total_mm=10.)['status'],'aligned')
        self.new['actual_base_from_tool'][2][3]-=.003
        for count,total in [(2,3.),(1,9.)]:
            r=assess(self.new,self.old,self.policy,corrections=count,total_mm=total)
            self.assertEqual(r['status'],'reacquire_required');self.assertIsNone(r['correction_mm'])

    def test_target_jump_wrong_orientation_and_lift_change_never_correct(self):
        for key in ('jump','orientation','lift','step'):
            with self.subTest(key=key):
                new=copy.deepcopy(self.new)
                if key=='jump':
                    for field in ('pre_mm','end_mm'):new['contact_path'][field][0]+=9
                elif key=='orientation':new['actual_base_from_tool'][0][0]=-1;new['actual_base_from_tool'][1][1]=-1
                elif key=='lift':new['lift_rad']+=.1
                else:new['actual_base_from_tool'][1][3]-=.007
                r=assess(new,self.old,self.policy,corrections=0,total_mm=0.)
                self.assertEqual(r['status'],'reacquire_required');self.assertIsNone(r['correction_mm'])

    def test_identity_versions_and_path_cannot_change(self):
        for key in ('head_observation_id','calibration_id','pick_setup_sha256','handeye_sha256',
                    'profile_sha256','urdf_sha256','control_config_sha256','frame','unit','arm'):
            new=copy.deepcopy(self.new);new[key]='changed'
            with self.assertRaises(ValueError):assess(new,self.old,self.policy,corrections=0,total_mm=0.)
        for key in ('observation_id','snapshot_id'):
            new=copy.deepcopy(self.new);new[key]=self.old[key]
            with self.assertRaises(ValueError):assess(new,self.old,self.policy,corrections=0,total_mm=0.)
        new=copy.deepcopy(self.new);new['contact_path']['pre_mm'][0]+=1
        with self.assertRaises(ValueError):assess(new,self.old,self.policy,corrections=0,total_mm=0.)

    def test_explicit_finite_policy_no_defaults(self):
        for key,val in [('max_corrections',True),('max_corrections',4),
                        ('position_tolerance_mm',[0,1,1]),('max_target_shift_mm',[1,2,float('nan')]),
                        ('max_total_correction_mm',float('inf')),('orientation_tolerance_deg',0)]:
            p=copy.deepcopy(self.policy);p[key]=val
            with self.assertRaises(ValueError):assess(self.new,self.old,p,corrections=0,total_mm=0.)
        for count,total in [(True,0),(-1,0),(3,0),(1,float('nan'))]:
            with self.assertRaises(ValueError):assess(self.new,self.old,self.policy,corrections=count,total_mm=total)
        with self.assertRaises(ValueError):assess(self.new,self.old,None,corrections=0,total_mm=0.)

if __name__=='__main__':unittest.main()
