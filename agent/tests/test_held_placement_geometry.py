import unittest,numpy as np
from scipy.spatial.transform import Rotation

class HeldPlacementGeometryTests(unittest.TestCase):
    def test_prepared_height_is_near_three_rad(self):
        from agent.held_placement_geometry import prepared_height
        self.assertTrue(prepared_height(3.0))
        self.assertTrue(prepared_height(2.65))
        self.assertTrue(prepared_height(3.35))
        self.assertFalse(prepared_height(2.64))
        self.assertFalse(prepared_height(3.36))
    def test_capture_binding_accepts_small_settling_but_rejects_large_motion_or_new_command(self):
        from agent.held_placement_geometry import capture_motion_bound
        a=np.eye(4);b=np.eye(4);b[0,3]=.002;b[:3,:3]=Rotation.from_euler('z',.2,degrees=True).as_matrix()
        self.assertLess(capture_motion_bound(a,b,7,7),5.)
        b[0,3]=.006
        with self.assertRaises(ValueError):capture_motion_bound(a,b,7,7)
        with self.assertRaises(ValueError):capture_motion_bound(a,a,7,8)
    def test_loaded_reference_is_preserved_at_start_and_tracks_the_nominal_curve(self):
        from agent.held_placement_geometry import retained_reference_path
        actual=np.zeros(6);gravity=np.radians([0.,0.,0.,.65,0.,.5]);nominal=np.array([actual,actual+[.01]*6])
        commands=retained_reference_path(nominal,actual+gravity,actual)
        # Static position-control model: measured q = reference - load offset.
        np.testing.assert_allclose(commands-gravity,nominal,atol=1e-12)
        np.testing.assert_allclose(commands[0],actual+gravity)
        with self.assertRaises(ValueError):retained_reference_path(nominal,actual+.1,actual)
    def test_endpoint_compensation_keeps_the_real_start_and_obeys_joint_limits(self):
        from agent.held_placement_geometry import compensate_path,updated_bias
        points=np.array([[0.]*6,[.1]*6]);bias=updated_bias(np.zeros(6),np.ones(6)*.1,np.ones(6)*.08)
        out=compensate_path(points,bias,[[-180,180]]*6)
        np.testing.assert_allclose(out[0],points[0]);np.testing.assert_allclose(out[-1],points[-1]+.02)
        self.assertTrue(np.max(abs(updated_bias(np.zeros(6),np.ones(6),np.zeros(6))))<=np.radians(2.))
    def test_scene_change_cannot_exceed_remaining_checked_clearance(self):
        from agent.held_placement_geometry import scene_allowance
        proof=dict(lower_mm=dict(aperture=1.1,table=13.),required_mm=dict(aperture=.5,table=10.))
        self.assertAlmostEqual(scene_allowance(proof,inside=True),.5)
        proof['lower_mm']['aperture']=.6
        self.assertAlmostEqual(scene_allowance(proof,inside=True),.1)
    def test_visual_attachment_preserves_world_battery_corners(self):
        from agent.held_placement_geometry import effective_tcp,nominal_attachment
        tcp=np.eye(4);tcp[:3,3]=[.18,0,0]
        attachment=np.eye(4);attachment[:3,:3]=Rotation.from_euler('xyz',[.1,.02,-.03]).as_matrix();attachment[:3,3]=[.03,.01,-.04]
        contact=np.array([2.,4.,160.]);effective=effective_tcp(tcp,attachment,contact)
        np.testing.assert_allclose(effective@nominal_attachment(contact),tcp@attachment,atol=1e-12)
    def test_partial_height_success_uses_measured_three_rad_height_without_reissuing_lift(self):
        from agent.held_placement_geometry import prepared_height
        self.assertTrue(prepared_height(2.70));self.assertTrue(prepared_height(3.1));self.assertFalse(prepared_height(2.4))
