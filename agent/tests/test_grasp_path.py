import json,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from agent.grasp_path import plan_insert, TimedJointPath
from web.src.handeye_column import ColumnModel

class InsertTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=ColumnModel()
        root=Path(__file__).resolve().parents[2]
        cls.limits=json.loads((root/'web/config/control.json').read_text())['arms']['right']['joint_limits_deg']

    def test_missing_tcp_is_explicit_without_ik(self):
        r=plan_insert(None,None,None,[1,2,3],[51,2,3],None,None,None)
        self.assertEqual(r['status'],'tcp_pending')
        self.assertIsNone(r['joint_waypoints_rad']);self.assertFalse(r['executable'])

    def test_real_urdf_fixed_tool_orientation_and_horizontal_insert(self):
        q=np.radians([0,20,40,10,0,-90]);lift=12.
        wrist=self.model.wrist_transform(q,lift)
        tcp=np.eye(4);tcp[:3,3]=[.12,.007,-.004]
        # Nonidentity tool rotation checks that wrist and TCP frames are not conflated.
        tcp[:3,:3]=Rotation.from_euler('xyz',[10,20,30],degrees=True).as_matrix()
        tool=wrist@tcp;pre=tool[:3,3]*1000;end=pre+10*tool[:3,0]
        r=plan_insert(self.model,q,lift,pre,end,tool[:3,:3],tcp,self.limits)
        self.assertEqual(r['status'],'kinematic_preview')
        self.assertFalse(r['executable']);self.assertFalse(r['environment_collision_checked'])
        for angles,point in zip(r['joint_waypoints_rad'],r['tool_points_mm']):
            actual=self.model.wrist_transform(angles,lift)@tcp
            self.assertLess(np.linalg.norm(actual[:3,3]*1000-point),.5)
            offset=actual[:3,3]*1000-pre
            self.assertLess(np.linalg.norm(offset-np.dot(offset,tool[:3,0])*tool[:3,0]),.5)
            self.assertTrue(np.allclose(actual[:3,:3],tool[:3,:3],atol=.003))
        self.assertLessEqual(r['interpolation_position_bound_mm'],1.)
        self.assertTrue(r['column_checked'])

    def test_wrong_axis_invalid_tcp_and_unreachable_return_no_partial_plan(self):
        q=np.radians([0,20,40,10,0,-90]);T=self.model.wrist_transform(q,12)
        pre=T[:3,3]*1000
        axis=T[:3,0]
        for end in (pre+10*T[:3,1],pre-10*axis,pre+201*axis):
            with self.assertRaises(ValueError):plan_insert(self.model,q,12,pre,end,T[:3,:3],np.eye(4),self.limits)
        with self.assertRaises(ValueError):plan_insert(self.model,q,12,pre,pre+10*axis,T[:3,:3],np.zeros((4,4)),self.limits)
        with self.assertRaises(ValueError):plan_insert(self.model,q,12,pre+[10000,0,0],pre+[10000,0,0]+10*axis,T[:3,:3],np.eye(4),self.limits)

    def test_failed_middle_ik_and_column_check_do_not_return_partial_motion(self):
        from unittest.mock import patch
        q=np.radians([0,20,40,10,0,-90]);T=self.model.wrist_transform(q,12)
        pre=T[:3,3]*1000;end=pre+10*T[:3,0]
        real=self.model.inverse_wrist;calls=[]
        def fail_later(*args):
            calls.append(1)
            if len(calls)==3:raise ValueError('middle waypoint unreachable')
            return real(*args)
        with patch.object(self.model,'inverse_wrist',side_effect=fail_later):
            with self.assertRaisesRegex(ValueError,'middle'):plan_insert(self.model,q,12,pre,end,T[:3,:3],np.eye(4),self.limits)
        with patch.object(self.model,'check',return_value={'clear':False,'lower_mm':0}):
            with self.assertRaises(ValueError):plan_insert(self.model,q,12,pre,end,T[:3,:3],np.eye(4),self.limits)

    def test_joint_branch_jump_is_rejected_even_if_ik_claims_success(self):
        from unittest.mock import patch
        q=np.radians([0,20,40,10,0,-90]);T=self.model.wrist_transform(q,12);pre=T[:3,3]*1000
        with patch.object(self.model,'inverse_wrist',side_effect=[q,q+np.radians([0,0,0,0,0,10])]):
            with self.assertRaisesRegex(ValueError,'branch'):plan_insert(self.model,q,12,pre,pre+10*T[:3,0],T[:3,:3],np.eye(4),self.limits)



class TimingTests(unittest.TestCase):
    def policy(self):
        return dict(joint_speed_deg_s=[5.7]*6,joint_acceleration_deg_s2=[12.]*6,
                    tcp_speed_mm_s=20.,tcp_acceleration_mm_s2=40.)

    def test_synchronised_segments_are_bounded_and_stop_at_knots(self):
        q=np.radians([[0,0,0,0,0,0],[4,-3,2,1,0,5],[2,1,3,-1,0,4]])
        path=TimedJointPath(q,.7,**self.policy())
        receipt=path.receipt()
        self.assertEqual(receipt['interpolation'],'segment_quintic_scalar')
        self.assertFalse(receipt['executable'])
        for t,expected in zip(path.times,q):
            pos,vel,acc=path.sample(t)
            np.testing.assert_allclose(pos,expected,atol=1e-12)
            np.testing.assert_allclose(vel,0,atol=1e-12)
            np.testing.assert_allclose(acc,0,atol=1e-12)
        for t in np.linspace(0,path.times[-1],1001):
            pos,vel,acc=path.sample(t)
            self.assertTrue((np.abs(vel)<=np.radians(5.7)+1e-10).all())
            self.assertTrue((np.abs(acc)<=np.radians(12.)+1e-10).all())
            self.assertLessEqual(700*np.abs(vel).sum(),20.+1e-8)
            self.assertLessEqual(700*(np.abs(acc).sum()+np.abs(vel).sum()**2),40.+1e-8)
            index=min(np.searchsorted(path.times,t,side='right')-1,len(q)-2)
            dq=q[index+1]-q[index];moving=np.abs(dq)>1e-12
            fractions=(pos-q[index])[moving]/dq[moving]
            self.assertLess(np.ptp(fractions),1e-10)
            self.assertTrue((fractions>=-1e-10).all() and (fractions<=1+1e-10).all())

    def test_policy_and_sample_validation(self):
        q=np.array([[0]*6,[.01]*6])
        for bad in (0.,float('nan'),-1.):
            policy=self.policy();policy['tcp_speed_mm_s']=bad
            with self.assertRaises(ValueError):TimedJointPath(q,.7,**policy)
        for bad in (np.zeros((2,6)),np.zeros((1,6)),np.full((2,6),np.nan)):
            with self.assertRaises(ValueError):TimedJointPath(bad,.7,**self.policy())
        for reach in (0.,-1.,float('nan')):
            with self.assertRaises(ValueError):TimedJointPath(q,reach,**self.policy())
        policy=self.policy();policy['joint_speed_deg_s']=[6.]*6
        with self.assertRaises(ValueError):TimedJointPath(q,.7,**policy)
        path=TimedJointPath(q,.7,**self.policy())
        for t in (-1.,float('nan'),path.times[-1]+1):
            with self.assertRaises(ValueError):path.sample(t)

    def test_real_model_timed_insert_retains_tcp_line_and_pose(self):
        model=ColumnModel();root=Path(__file__).resolve().parents[2]
        limits=json.loads((root/'web/config/control.json').read_text())['arms']['right']['joint_limits_deg']
        q=np.radians([0,20,40,10,0,-90]);lift=12.
        tcp=np.eye(4);tcp[:3,3]=[.12,.007,-.004]
        T=model.wrist_transform(q,lift)@tcp;pre=T[:3,3]*1000
        result=plan_insert(model,q,lift,pre,pre+10*T[:3,0],T[:3,:3],tcp,limits,timing_limits=self.policy())
        self.assertEqual(result['timing']['status'],'timed_preview')
        self.assertFalse(result['executable'])
        path=TimedJointPath(result['joint_waypoints_rad'],model.reach+np.linalg.norm(tcp[:3,3]),**self.policy())
        for t in np.linspace(0,path.times[-1],101):
            angles,_,_=path.sample(t);actual=model.wrist_transform(angles,lift)@tcp
            offset=actual[:3,3]*1000-pre
            self.assertLess(np.linalg.norm(offset-np.dot(offset,T[:3,0])*T[:3,0]),1.)
            self.assertLess(np.degrees(Rotation.from_matrix(T[:3,:3].T@actual[:3,:3]).magnitude()),.3)
        self.assertTrue(any('驱动' in text for text in result['blockers']))

if __name__=='__main__':unittest.main()
