import unittest
from unittest.mock import patch
import numpy as np
import vision_path_bridge as b

class CoarseRetry(unittest.TestCase):
    def test_small_orientation_retry_is_explicit(self):
        attempts=[]
        def plan(*args,**kw):
            attempts.append(args[4].copy())
            if len(attempts)<2:raise ValueError('当前姿态和关节限位内未找到足够精确的逆解')
            return dict(joint_waypoints_rad=[[0.]*6,[.01]*6],timing=dict(time_from_start_s=[0.,1.]))
        with patch.object(b,'localize',return_value={}),patch.object(b,'horizontal_plan',return_value=dict(R_base_from_tool=np.eye(3),tcp_pre_base_mm=[100,0,100])),patch.object(b,'plan_correction',side_effect=plan):
            r=b.coarse(dict(image_time=1.,joints=np.zeros(6),lift_rad=10.),None,np.eye(4),{}, {},[],[],{},now=1.,preclearance_mm=30.)
        self.assertLessEqual(abs(r['orientation_adjustment_deg']),2.)
        self.assertFalse(r['executable'])
        np.testing.assert_allclose(attempts[0][:3,3],attempts[1][:3,3])
