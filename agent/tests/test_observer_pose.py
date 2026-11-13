import json
import unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np
from agent.observer_pose import assess_observer, view_check
from web.src.handeye_column import ColumnModel

ROOT=Path(__file__).resolve().parents[2]

class ObserverTests(unittest.TestCase):
    def setUp(self):
        self.cal=json.loads((ROOT/'vision/handeye/results/d405-right-cc200-ee0ff2fa49e9e56285d271d6.json').read_text())
        self.limits=json.loads((ROOT/'web/config/control.json').read_text())['arms']['right']['joint_limits_deg']
        self.frame=dict(joints=np.zeros(6),image_time=10.,arm_time=10.)
        self.location=dict(surface_base_mm=[500,-250,750],
            surface_bounds_p90_base_mm=[[460,-290,690],[540,-210,810]])

    def test_real_urdf_observer_is_reachable_and_projects_observed_extent(self):
        result=assess_observer(self.frame,self.location,10.77963,self.cal,self.limits)
        self.assertEqual(result['status'],'observer_candidate',result)
        self.assertFalse(result['executable']);self.assertIsNone(result['tcp'])
        self.assertFalse(result['occlusion_checked'])
        q=np.asarray(result['joints_rad'])
        m=ColumnModel();w=m.wrist_transform(q,result['lift_rad'])
        C=np.asarray(self.cal['T_right_link26_from_D405_color'])
        np.testing.assert_allclose(w@C,result['base_from_camera'],atol=1e-8)
        self.assertTrue(m.check(self.frame['joints'],q,result['lift_rad'])['clear'])
        np.testing.assert_array_less(np.degrees(q),np.array(self.limits)[:,1]+1e-5)
        np.testing.assert_array_less(np.array(self.limits)[:,0]-1e-5,np.degrees(q))
        self.assertTrue(result['view']['in_view'])
        self.assertGreaterEqual(result['view']['edge_margin_px'],24)

    def test_already_visible_prefers_current_joints_without_an_extra_move(self):
        f=self.frame;f['joints']=np.radians([0,20,40,10,0,-90])
        T=ColumnModel().wrist_transform(f['joints'],10)@np.asarray(self.cal['T_right_link26_from_D405_color'])
        p=(T@np.array([0,0,.35,1]))[:3]*1000
        loc=dict(surface_base_mm=p,surface_bounds_p90_base_mm=[p-10,p+10])
        result=assess_observer(f,loc,10,self.cal,self.limits)
        self.assertEqual(result['status'],'observer_candidate')
        np.testing.assert_allclose(result['joints_rad'],f['joints'])
        self.assertFalse(result['arm_move_required'])
        self.assertEqual(result['stationary_plan'],dict(version=1,kind='stationary',joint_position_rad=f['joints'].tolist()))

    def test_missing_pair_bounds_calibration_or_unreachable_never_returns_targets(self):
        cases=[]
        f=dict(self.frame);f.pop('joints');cases.append((f,self.location,10,self.cal))
        f=dict(self.frame,arm_time=9);cases.append((f,self.location,10,self.cal))
        loc=dict(self.location);loc.pop('surface_bounds_p90_base_mm');cases.append((self.frame,loc,10,self.cal))
        c=dict(self.cal,parent_frame='base_link');cases.append((self.frame,self.location,10,c))
        loc=dict(surface_base_mm=[5000,0,0],surface_bounds_p90_base_mm=[[4990,-10,-10],[5010,10,10]])
        cases.append((self.frame,loc,10,self.cal))
        cases.append((self.frame,self.location,19,self.cal))
        for args in cases:
            result=assess_observer(*args,self.limits)
            self.assertEqual(result['status'],'observer_unavailable',result)
            self.assertIsNone(result['joints_rad']);self.assertFalse(result['executable'])

    def test_center_visible_does_not_accept_extent_outside_or_behind_camera(self):
        k=self.cal['camera']['intrinsics']
        ok=view_check(np.eye(4),[[ -.03,-.03,.3],[.03,.03,.4]],k)
        self.assertTrue(ok['in_view'])
        for bounds in ([[-.5,-.1,.3],[.5,.1,.4]],[[-.01,-.01,-.2],[.01,.01,-.1]],
                       [[-.01,-.01,1.6],[.01,.01,1.7]]):
            self.assertFalse(view_check(np.eye(4),bounds,k)['in_view'])

    def test_d435_directed_view_uses_current_absolute_position_and_mount_offset(self):
        preferences=dict(start_z_mm=700.5,rpy_deg=[-.3,-2.6,.1],acceptance_rad=5.)
        results=[]
        for y in (-200.,-133.811):
            p=np.array([632.144,y,700.361])
            loc=dict(surface_base_mm=p,surface_bounds_p90_base_mm=[p-[20,25,30],p+[20,25,30]])
            result=assess_observer(self.frame,loc,5.52,self.cal,self.limits,observer=preferences)
            self.assertEqual(result['status'],'observer_candidate',result)
            self.assertTrue(result['view']['in_view'])
            camera=np.array(result['base_from_camera'])[:3,3]*1000
            self.assertAlmostEqual(camera[1],y,places=3)
            self.assertTrue(249.5<=p[0]-camera[0]<=450.5)
            results.append(result)
        self.assertGreater(results[1]['wrist_base_mm'][1]-results[0]['wrist_base_mm'][1],60)

    def test_directed_observer_does_not_hide_missing_bounds_or_bad_preferences(self):
        preference=dict(start_z_mm=700.5,rpy_deg=[-.3,-2.6,.1],acceptance_rad=5.)
        for bad in (dict(preference,start_z_mm=float('nan')),
                    dict(preference,rpy_deg=[1,2])):
            result=assess_observer(self.frame,self.location,5.52,self.cal,self.limits,observer=bad)
            self.assertEqual(result['status'],'observer_unavailable')
            self.assertIsNone(result['joints_rad'])

    def test_directed_view_warning_does_not_block_coarse_motion(self):
        p=np.array([632.144,-133.811,700.361])
        loc=dict(surface_base_mm=p,surface_bounds_p90_base_mm=[p-20,p+20])
        with patch('agent.observer_pose.view_check',return_value={'in_view':False}):
            result=assess_observer(self.frame,loc,5.52,self.cal,self.limits,
                observer=dict(start_z_mm=700.5,rpy_deg=[-.3,-2.6,.1]))
        self.assertEqual(result['status'],'observer_candidate',result)
        self.assertFalse(result['view']['in_view'])

if __name__=='__main__':unittest.main()
