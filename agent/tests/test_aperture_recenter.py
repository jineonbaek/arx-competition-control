import unittest
import numpy as np
from agent.held_placement_geometry import aperture_recenter

class RecenterTest(unittest.TestCase):
    def test_recenter_with_bounded_motion_and_positive_clearance(self):
        fit={'footprint_uv_mm': [[2,38.3],[3,-36.7],[84.4,39.59],[85.5,-35.4]],
             'aperture_polygon_mm': [[-84.8,-39.5],[84.8,-39.5],[84.8,39.5],[-84.8,39.5]]}
        delta, clearance=aperture_recenter(fit)
        self.assertLessEqual(np.linalg.norm(delta),5.)
        self.assertGreater(clearance,.5)
        self.assertLess(delta[0],0)

    def test_unresolvable_fit_fails(self):
        with self.assertRaises(ValueError):
            aperture_recenter({'footprint_uv_mm': [[0,50],[0,-50],[100,0]],
                               'aperture_polygon_mm': [[-84,-39],[84,-39],[84,39],[-84,39]]})

    def test_large_offset_uses_original_coarse_goal(self):
        from fast_visual_level import alignment_goal
        fit={'qualified_fit':False,'footprint_uv_mm': [[0,50],[0,-50],[100,0]],
             'aperture_polygon_mm': [[-84,-39],[84,-39],[84,39],[-84,39]]}
        plan={'R_base_from_tool':np.eye(3).tolist(),'tcp_pre_base_mm':[100,200,300]}
        goal,delta,_=alignment_goal(fit,np.eye(4),{'left_axis_base':[0,1,0],'up_axis_base':[0,0,1]},plan)
        self.assertIsNone(delta)
        np.testing.assert_allclose(goal[:3,3],[.1,.2,.3])
