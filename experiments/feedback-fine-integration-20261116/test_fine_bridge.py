import unittest
import numpy as np
from fine_bridge import fine_target

class FineTest(unittest.TestCase):
    def frame(self):
        return dict(camera_id='d405',image_time=100.,arm_time=100.,body_time=100.,joints=np.zeros(6),lift_rad=10.)
    def test_post_arrival_only(self):
        with self.assertRaises(ValueError):
            fine_target(self.frame(),np.zeros(6),10.,101.,101.,np.eye(4),lambda f: {'step_mm':1.})
    def test_vertical_target_keeps_rotation(self):
        r=fine_target(self.frame(),np.zeros(6),10.,99.,100.,np.eye(4),lambda f: {'step_mm':-2.})
        self.assertAlmostEqual(r['target_base_from_tool'][2][3],-.002)
        self.assertEqual(r['gripper_action'],'retain')
        self.assertFalse(r['executable'])
    def test_reject_unbounded_or_invalid(self):
        for value in (4.,float('nan')):
            with self.assertRaises(ValueError):
                fine_target(self.frame(),np.zeros(6),10.,99.,100.,np.eye(4),lambda f: {'step_mm':value})
