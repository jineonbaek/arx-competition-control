import unittest,numpy as np
from scipy.spatial.transform import Rotation
from agent.docking_fit import evaluate_fit

class ShallowApertureTests(unittest.TestCase):
    def test_outside_transient_is_checked_at_the_approach_buffer_not_30mm_away(self):
        from agent.shallow_aperture import aperture_relevant
        self.assertFalse(aperture_relevant(30.))
        self.assertTrue(aperture_relevant(5.))
        self.assertTrue(aperture_relevant(-5.))
    def scene(self):
        tool=np.eye(4);tool[:3,:3]=Rotation.from_euler('y',2.,degrees=True).as_matrix()
        tool[:3,3]=[.37,.0424,.6+.195*np.sin(np.radians(2.))]
        window=dict(center_base_m=[.6,0.,.6],outward_normal_base=[-1.,0.,0.],left_axis_base=[0.,1.,0.],up_axis_base=[0.,0.,1.],measured_size_mm=[169.6,79.])
        slot=dict(size_mm=[169.6,79.],center_mm=[0.,0.],outline_mm=[[-84.8,-39.5],[84.8,-39.5],[84.8,39.5],[-84.8,39.5]])
        battery=dict(bounds_mm=[[-41.175,-37.5,0.],[41.175,37.5,195.]])
        return tool,window,slot,battery,np.array([0.,0.,195.])
    def test_unentered_rear_corners_do_not_block_a_clear_5mm_front_cap(self):
        from agent.shallow_aperture import evaluate_shallow_fit
        args=self.scene();full=evaluate_fit(*args);shallow=evaluate_shallow_fit(*args,depth_mm=5.)
        self.assertFalse(full['qualified_fit']);self.assertTrue(shallow['qualified_fit'],shallow)
        self.assertEqual(shallow['depth_mm'],5.)
        self.assertEqual(shallow['entering_vertices'],8)
    def test_actual_entering_front_collision_still_blocks(self):
        from agent.shallow_aperture import evaluate_shallow_fit
        args=list(self.scene());args[0][2,3]+=.01
        self.assertFalse(evaluate_shallow_fit(*args,depth_mm=5.)['qualified_fit'])
