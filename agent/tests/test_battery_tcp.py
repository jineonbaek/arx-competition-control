import unittest
import numpy as np
from agent.battery_tcp import load_pick_setup
from web.src.handeye_column import ColumnModel

class TCPTests(unittest.TestCase):
    def test_measured_center_and_minus_ninety_convention(self):
        setup=load_pick_setup(timing_limits={},alignment_tolerance_deg=3,j6_tolerance_deg=5)
        T=np.array(setup['T_wrist_from_tool'])
        np.testing.assert_allclose(T[:3,3]*1000,[148.142,.181,-1.143],atol=.001)
        wrist=ColumnModel().wrist_transform(np.radians([0,0,0,0,0,-90]),0)
        np.testing.assert_allclose((wrist@T)[:3,:3],np.eye(3),atol=1e-12)
        # Fixed contact point: wrist offset rotates with the tool, not with world axes.
        target=np.eye(4);target[:3,3]=[.5,-.25,.7]
        np.testing.assert_allclose((target@np.linalg.inv(T))@T,target,atol=1e-12)
        self.assertEqual(setup['tool_frame'],'grip_reference')
    def test_each_load_is_independent(self):
        a=load_pick_setup(timing_limits={},alignment_tolerance_deg=3,j6_tolerance_deg=5)
        a['T_wrist_from_tool'][0][3]=999
        b=load_pick_setup(timing_limits={},alignment_tolerance_deg=3,j6_tolerance_deg=5)
        self.assertLess(b['T_wrist_from_tool'][0][3],.15)
if __name__=='__main__':unittest.main()
