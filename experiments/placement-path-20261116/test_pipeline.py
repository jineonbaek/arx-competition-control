import unittest
import numpy as np
from unittest.mock import patch
import offline_pipeline as p

class FakeModel:
    def wrist_transform(self,q,h):return np.eye(4)
    def check(self,a,b,h):return {"clear":True,"lower_mm":100.}

class PipelineTest(unittest.TestCase):
    def inputs(self):
        return dict(frame={"joints":np.zeros(6),"lift_rad":3.,"image_time":10.},pose_report={},mesh_sha="x",model=FakeModel(),tcp=np.eye(4),slot={},battery={},contact=[0,0,0],limits=[[-180,180]]*6,timing_limits={})
    def test_replays_level_align_then_fixed_z_without_execution(self):
        q1=[0.,0.,0.,.01,0.,0.]; q2=[0.,.01,0.,.01,0.,0.]
        window={"left_axis_base":[0,1,0]}; preview={"tcp_pre_base_mm":[0,0,0],"tcp_end_base_mm":[5,0,0],"R_base_from_tool":np.eye(3).tolist(),"insertion_axis_base":[1,0,0],"nominal_cad_clearance_mm":2.}
        level={"action":"correct","tilt_deg":3.,"nominal_target_rad":q1}
        with patch.object(p,"localize",return_value=window),patch.object(p,"d435_observation",return_value={"T_base_from_object":np.eye(4).tolist()}),patch.object(p,"visual_level_step",return_value=level),patch.object(p,"effective_tcp",return_value=np.eye(4)),patch.object(p,"preview_shallow",return_value=preview),patch.object(p,"plan_correction",return_value={"joint_waypoints_rad":[q1,q2]}),patch.object(p,"plan_insert",return_value={"joint_waypoints_rad":[q2,q2],"status":"kinematic_preview"}):
            result=p.plan_offline(**self.inputs())
        self.assertEqual(result["stage_order"],["cabinet","level","align","insert"])
        self.assertEqual(result["level"]["nominal_target_rad"],q1)
        self.assertEqual(result["align"]["joint_waypoints_rad"][-1],q2)
        self.assertEqual(result["insert"]["status"],"kinematic_preview")
        self.assertFalse(result["executable"])
    def test_d405_vertical_offset_changes_target_without_mutating_input(self):
        window={"left_axis_base":[0,1,0],"up_axis_base":[0,0,1],"outward_normal_base":[-1,0,0]}
        slot={"outline_mm":[[-50,-50],[50,-50],[50,50],[-50,50]],"center_mm":[0,0],"size_mm":[100,100]}
        preview={"tcp_points_base_mm":[[0,0,0],[5,0,0]],"tcp_pre_base_mm":[0,0,0],"tcp_end_base_mm":[5,0,0],"leading_points_base_mm":[[0,0,0],[5,0,0]],"aperture_envelope_samples_mm":[[-10,-10],[10,10]],"effective_opening_size_mm":[100,100],"insertion_axis_base":[1,0,0]}
        out=p.apply_d405_vertical(preview,window,slot,{"status":"visual_guidance_only","step_mm":3.})
        self.assertEqual(out["tcp_pre_base_mm"][2],3.)
        self.assertEqual(out["tcp_end_base_mm"][2],3.)
        self.assertEqual(preview["tcp_pre_base_mm"][2],0.)
        self.assertEqual(out["d405_vertical_step_mm"],3.)
    def test_d405_step_does_not_cross_aperture_or_exceed_limit(self):
        window={"left_axis_base":[0,1,0],"up_axis_base":[0,0,1],"outward_normal_base":[-1,0,0]};slot={"outline_mm":[[-50,-50],[50,-50],[50,50],[-50,50]],"center_mm":[0,0],"size_mm":[100,100]};preview={"tcp_points_base_mm":[[0,0,0],[5,0,0]],"tcp_pre_base_mm":[0,0,0],"tcp_end_base_mm":[5,0,0],"leading_points_base_mm":[[0,0,0],[5,0,0]],"aperture_envelope_samples_mm":[[-10,-10],[10,49]],"effective_opening_size_mm":[100,100],"insertion_axis_base":[1,0,0]}
        with self.assertRaises(ValueError):p.apply_d405_vertical(preview,window,slot,{"status":"visual_guidance_only","step_mm":3.})
        with self.assertRaises(ValueError):p.apply_d405_vertical(preview,window,slot,{"status":"visual_guidance_only","step_mm":3.1})
    def test_d405_needs_paired_time_and_feedback(self):
        ref={"image_time":10.}
        good={"camera_id":"d405","image_time":10.1,"arm_time":10.12,"body_time":10.11}
        self.assertTrue(p.d405_paired(ref,good))
        self.assertFalse(p.d405_paired(ref,good|{"image_time":10.5}))
        self.assertFalse(p.d405_paired(ref,good|{"arm_time":10.5}))
        self.assertFalse(p.d405_paired(ref,{"camera_id":"d405"}))
    def test_no_kinematic_result_is_invented_after_ik_failure(self):
        window={"left_axis_base":[0,1,0]};preview={"tcp_pre_base_mm":[0,0,0],"tcp_end_base_mm":[5,0,0],"R_base_from_tool":np.eye(3).tolist(),"insertion_axis_base":[1,0,0],"nominal_cad_clearance_mm":2.}
        with patch.object(p,"localize",return_value=window),patch.object(p,"d435_observation",return_value={"T_base_from_object":np.eye(4).tolist()}),patch.object(p,"visual_level_step",return_value={"action":"hold","tilt_deg":0.}),patch.object(p,"effective_tcp",return_value=np.eye(4)),patch.object(p,"preview_shallow",return_value=preview),patch.object(p,"plan_correction",side_effect=ValueError("IK failed")),patch.object(p,"plan_insert") as insert:
            result=p.plan_offline(**self.inputs())
        self.assertEqual(result["align"]["status"],"unavailable")
        self.assertIn("IK failed",result["align"]["reason"])
        self.assertIsNone(result["insert"]);insert.assert_not_called();self.assertFalse(result["executable"])
if __name__=="__main__":unittest.main()
