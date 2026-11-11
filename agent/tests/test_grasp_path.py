import json,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from agent.grasp_path import plan_insert
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
        tool=wrist@tcp;pre=tool[:3,3]*1000;end=pre+[10,0,0]
        r=plan_insert(self.model,q,lift,pre,end,tool[:3,:3],tcp,self.limits)
        self.assertEqual(r['status'],'kinematic_preview')
        self.assertFalse(r['executable']);self.assertFalse(r['environment_collision_checked'])
        for angles,point in zip(r['joint_waypoints_rad'],r['tool_points_mm']):
            actual=self.model.wrist_transform(angles,lift)@tcp
            self.assertLess(np.linalg.norm(actual[:3,3]*1000-point),.5)
            self.assertLess(np.linalg.norm(actual[:3,3][1:]*1000-pre[1:]),.5)
            self.assertTrue(np.allclose(actual[:3,:3],tool[:3,:3],atol=.003))
        self.assertLessEqual(r['interpolation_position_bound_mm'],1.)
        self.assertTrue(r['column_checked'])

    def test_wrong_axis_invalid_tcp_and_unreachable_return_no_partial_plan(self):
        q=np.radians([0,20,40,10,0,-90]);T=self.model.wrist_transform(q,12)
        pre=T[:3,3]*1000
        for end in (pre+[10,1,0],pre+[-10,0,0],pre+[201,0,0]):
            with self.assertRaises(ValueError):plan_insert(self.model,q,12,pre,end,T[:3,:3],np.eye(4),self.limits)
        with self.assertRaises(ValueError):plan_insert(self.model,q,12,pre,pre+[10,0,0],T[:3,:3],np.zeros((4,4)),self.limits)
        with self.assertRaises(ValueError):plan_insert(self.model,q,12,pre+[10000,0,0],pre+[10010,0,0],T[:3,:3],np.eye(4),self.limits)

    def test_failed_middle_ik_and_column_check_do_not_return_partial_motion(self):
        from unittest.mock import patch
        q=np.radians([0,20,40,10,0,-90]);T=self.model.wrist_transform(q,12)
        pre=T[:3,3]*1000
        real=self.model.inverse_wrist;calls=[]
        def fail_later(*args):
            calls.append(1)
            if len(calls)==3:raise ValueError('middle waypoint unreachable')
            return real(*args)
        with patch.object(self.model,'inverse_wrist',side_effect=fail_later):
            with self.assertRaisesRegex(ValueError,'middle'):plan_insert(self.model,q,12,pre,pre+[10,0,0],T[:3,:3],np.eye(4),self.limits)
        with patch.object(self.model,'check',return_value={'clear':False,'lower_mm':0}):
            with self.assertRaises(ValueError):plan_insert(self.model,q,12,pre,pre+[10,0,0],T[:3,:3],np.eye(4),self.limits)

    def test_joint_branch_jump_is_rejected_even_if_ik_claims_success(self):
        from unittest.mock import patch
        q=np.radians([0,20,40,10,0,-90]);T=self.model.wrist_transform(q,12);pre=T[:3,3]*1000
        with patch.object(self.model,'inverse_wrist',side_effect=[q,q+np.radians([0,0,0,0,0,10])]):
            with self.assertRaisesRegex(ValueError,'branch'):plan_insert(self.model,q,12,pre,pre+[10,0,0],T[:3,:3],np.eye(4),self.limits)

if __name__=='__main__':unittest.main()
