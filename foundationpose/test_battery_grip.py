import unittest
import numpy as np
from analyze_grip import analyze
from battery_grip import transform_reference,draw_reference


class BatteryGripTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile,_,_=analyze()

    def test_actual_mesh_dimensions_and_cutout_fit(self):
        p=self.profile
        np.testing.assert_allclose(p["hole_size_mm"],[40,14],atol=.02)
        self.assertAlmostEqual(p["handle_thickness_mm"],24.4,places=2)
        self.assertAlmostEqual(p["notch_depth_mm"],5,places=2)
        self.assertAlmostEqual(p["bump_height_mm"],3.2,places=2)
        self.assertAlmostEqual(p["bump_relief_clearance_mm"],1.8,places=2)
        self.assertEqual(p["insertion_mm"],26)
        self.assertLess(p["tip_alignment_error_mm"],.001)
        coverage=p["recess_coverage"]
        self.assertEqual(coverage["inside_recess_samples"],coverage["protrusion_vertex_samples"])
        self.assertAlmostEqual(coverage["minimum_clearance_mm"],1.8,places=2)
        self.assertAlmostEqual(p["insertion_exact_mm"],25.5345764,places=4)
        self.assertTrue(all(c["penetrating_samples"]==0 for c in p["ray_check"]["cases"]["candidate"]))
        self.assertTrue(any(c["penetrating_samples"]>0 for c in p["ray_check"]["cases"]["wrong_j6_sign"]))
        self.assertTrue(any(c["penetrating_samples"]>0 for c in p["ray_check"]["cases"]["too_deep_42mm"]))

    def test_rotated_translated_pose_preserves_horizontal_26mm_and_bump_down(self):
        T=np.eye(4)
        T[:3,:3]=[[0,0,-1],[1,0,0],[0,-1,0]]
        T[:3,3]=[.8,-.25,.7]
        result=transform_reference(self.profile,T,np.eye(4))
        p=result["points_base_mm"]
        np.testing.assert_allclose(np.subtract(p["tip"],p["front"]),[26,0,0],atol=1e-6)
        expected=T[:3,:3]@self.profile["points_object_mm"]["grip"]+T[:3,3]*1000
        np.testing.assert_allclose(p["grip"],expected)
        self.assertAlmostEqual(result["bump_down_error_deg"],0)
        self.assertAlmostEqual(result["approach_x_error_deg"],0)
        self.assertEqual(result["required_j6_deg"],-90)

    def test_invalid_transform_rejected(self):
        for bad in [np.zeros((4,4)),np.eye(3),np.diag([1,1,-1,1])]:
            with self.assertRaises(ValueError):transform_reference(self.profile,bad,np.eye(4))

    def test_overlay_points_match_transformed_projection(self):
        T=np.eye(4);T[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]];T[:3,3]=[-.18,0,.6]
        def project(points):
            points=np.asarray(points)@T[:3,:3].T+T[:3,3]
            return np.rint(points[:,:2]/points[:,2:]*500+[320,240]).astype(int)
        image=np.zeros((480,640,3),np.uint8)
        result=draw_reference(self.profile,np.eye(4),T,project,image)
        g=result["pixels"]["grip"]
        np.testing.assert_allclose(g,project([np.array(self.profile["points_object_mm"]["grip"])*.001])[0])
        self.assertGreater(image.sum(),0)
        self.assertEqual(image[g[1],g[0]].tolist(),[255,146,86])


if __name__=="__main__":unittest.main()
