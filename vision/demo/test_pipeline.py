import tempfile
from pathlib import Path
import unittest
import numpy as np
from PIL import Image
from pipeline import (QwenJudge, checked_box, validate_choice, mask_geometry, run_frame,
                      filter_fixed_gripper_candidates, choice_in_fixed_gripper_zone)

class Models:
    def __init__(self, candidates):
        self.candidates=candidates
    def detect(self,*args):
        return self.candidates
    def segment(self,image,**kwargs):
        self.prompt=kwargs
        mask=np.zeros((image.height,image.width),bool)
        mask[10:30,10:30]=True
        return mask,.9

class Judge:
    def __init__(self, selection, coarse=None, review="accepted"):
        self.selection,self.coarse,self.review_status=selection,coarse,review
        self.modes=[]
    def select(self,image,reference,candidates,task,coarse=False):
        self.modes.append(coarse)
        return self.coarse if coarse else self.selection
    def review(self,*args):
        return {"status":self.review_status,"reason":"fixture"}

class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output=Path(self.temp.name)
        self.image=Image.new("RGB",(80,60))
        self.candidates=[{"candidate_id":1,"bbox_xyxy":[8,8,32,32],"score":.8,"label":"object"}]

    def run_frame(self,models,judge,**kwargs):
        return run_frame(models,judge,self.image,self.image,"test",self.output,**kwargs)

    def test_candidate_selection_segments_then_reviews(self):
        models=Models(self.candidates)
        judge=Judge({"status":"selected","candidate_id":1})
        result=self.run_frame(models,judge)
        self.assertEqual(result["status"],"perception_accepted")
        self.assertEqual(models.prompt,{"box":[8,8,32,32]})
        self.assertEqual(judge.modes,[False])
        self.assertTrue((self.output/"mask.png").exists())
        self.assertFalse(result["geometry"]["robot_geometry_ready"])

    def test_shape_evidence_keeps_color_only_in_display_overlay(self):
        from unittest.mock import Mock
        from target import TASK, DINO_PROMPT, load_reference, shape_view
        self.image=Image.new("RGB",(80,60),(180,45,16))
        models=Mock(wraps=Models(self.candidates))
        judge=Mock(wraps=Judge({"status":"selected","candidate_id":1}))
        reference=load_reference()
        report=run_frame(models,judge,self.image,reference,TASK,self.output)
        self.assertEqual(report["status"],"perception_accepted")
        self.assertEqual(models.detect.call_args.args[1],DINO_PROMPT)
        for im in (models.detect.call_args.args[0],models.segment.call_args.args[0],
                   judge.select.call_args.args[0],judge.review.call_args.args[0]):
            self.assertEqual(im.tobytes(),shape_view(self.image).tobytes())
        self.assertEqual(judge.select.call_args.args[1].tobytes(),shape_view(reference).tobytes())
        self.assertEqual(judge.review.call_args.args[2].getpixel((0,0)),shape_view(self.image).getpixel((0,0)))
        with Image.open(self.output/"overlay.png") as annotated:
            self.assertEqual(annotated.getpixel((0,0)),self.image.getpixel((0,0)))

    def test_zero_candidates_use_coarse_point(self):
        models=Models([])
        judge=Judge(None,{"status":"selected","point_xy":[20,20]})
        result=self.run_frame(models,judge)
        self.assertEqual(judge.modes,[True])
        self.assertEqual(models.prompt,{"point":[20,20]})
        self.assertEqual(result["path"],"qwen_coarse")

    def test_rejected_candidates_enter_coarse_recovery(self):
        judge=Judge({"status":"ambiguous"},{"status":"selected","bbox_xyxy":[8,8,32,32]})
        result=self.run_frame(Models(self.candidates),judge)
        self.assertEqual(judge.modes,[False,True])
        self.assertEqual(result["status"],"perception_accepted")

    def test_d435_fixed_bottom_grippers_are_removed_before_qwen(self):
        candidates=[
            {"candidate_id":1,"bbox_xyxy":[80,370,184,477],"score":.49,"label":"case"},
            {"candidate_id":2,"bbox_xyxy":[495,0,636,112],"score":.37,"label":"case"},
            {"candidate_id":3,"bbox_xyxy":[535,358,634,475],"score":.29,"label":"case"}]
        kept,excluded=filter_fixed_gripper_candidates(candidates,(640,480),"d435")
        self.assertEqual([row["candidate_id"] for row in kept],[2])
        self.assertEqual([row["candidate_id"] for row in excluded],[1,3])
        same,_=filter_fixed_gripper_candidates(candidates,(640,480),"d405")
        self.assertEqual(same,candidates)

    def test_qwen_coarse_choice_cannot_reintroduce_fixed_gripper(self):
        choice={"box":[80,370,184,477]}
        self.assertTrue(choice_in_fixed_gripper_zone(choice,(640,480),"d435"))
        self.assertFalse(choice_in_fixed_gripper_zone(choice,(640,480),"d405"))
        self.image=Image.new("RGB",(640,480))
        judge=Judge(None,{"status":"selected","coordinate_space":"normalized_1000",
                          "bbox_xyxy":[125,771,288,994]})
        result=self.run_frame(Models([]),judge,camera="d435")
        self.assertEqual(result["status"],"needs_new_observation")
        self.assertIn("excluded_fixed_gripper_choice",result)

    def test_real_frame_keeps_top_right_battery_and_drops_both_grippers(self):
        candidates=[
            {"candidate_id":1,"bbox_xyxy":[81,370,184,477],"score":.44,"label":"case"},
            {"candidate_id":2,"bbox_xyxy":[577,386,633,464],"score":.44,"label":"case"},
            {"candidate_id":3,"bbox_xyxy":[494,0,636,112],"score":.37,"label":"case"},
            {"candidate_id":4,"bbox_xyxy":[496,0,613,111],"score":.32,"label":"case"}]
        kept,excluded=filter_fixed_gripper_candidates(candidates,(640,480),"d435")
        self.assertEqual([row["candidate_id"] for row in kept],[3,4])
        self.assertEqual([row["candidate_id"] for row in excluded],[1,2])

    def test_no_target_does_not_segment(self):
        result=self.run_frame(Models([]),Judge(None,{"status":"not_found"}))
        self.assertEqual(result["status"],"needs_new_observation")
        self.assertFalse((self.output/"mask.png").exists())

    def test_review_rejection_prevents_acceptance(self):
        result=self.run_frame(Models(self.candidates),
                              Judge({"status":"selected","candidate_id":1},review="rejected"))
        self.assertEqual(result["status"],"needs_new_observation")

    def test_unknown_candidate_cannot_be_selected(self):
        with self.assertRaises(ValueError):
            validate_choice({"status":"selected","candidate_id":9},self.candidates,(80,60),False)

    def test_coordinate_rejection(self):
        for box in [[0,0,90,20],[0,0,float("nan"),20],[1,1,1,5],[True,0,4,5],[1.1,1.1,1.2,2.0]]:
            with self.subTest(box=box),self.assertRaises(ValueError):
                checked_box(box,(80,60))
        for point in [[80,3],[-1,2],[float("inf"),3]]:
            with self.subTest(point=point),self.assertRaises(ValueError):
                validate_choice({"status":"selected","point_xy":point},[],(80,60),True)

    def test_coarse_requires_one_prompt(self):
        for data in [{},{"point_xy":[5,5],"bbox_xyxy":[1,1,10,10]}]:
            with self.assertRaises(ValueError):
                validate_choice({"status":"selected",**data},[],(80,60),True)

    def test_depth_is_optional_and_unaligned_depth_is_rejected(self):
        mask=np.zeros((60,80),bool);mask[10:30,10:30]=True
        self.assertFalse(mask_geometry(mask)["metric_valid"])
        result=mask_geometry(mask,np.ones((60,80),np.uint16),{"aligned_to_color":False})
        self.assertEqual(result["reason"],"depth_not_aligned")

    def test_metric_surface_geometry_and_empty_depth(self):
        mask=np.zeros((60,80),bool);mask[10:30,10:30]=True
        meta={"aligned_to_color":True,"scale_m_per_unit":.001,
              "intrinsics":{"fx":100.,"fy":100.,"cx":40.,"cy":30.}}
        result=mask_geometry(mask,np.full((60,80),500,np.uint16),meta)
        self.assertTrue(result["metric_valid"])
        self.assertAlmostEqual(result["median_surface_xyz_m"][2],.5)
        self.assertFalse(result["robot_geometry_ready"])
        self.assertFalse(mask_geometry(mask,np.zeros((60,80),np.uint16),meta)["metric_valid"])

    def test_plain_visual_reply_gets_one_bounded_repair(self):
        from unittest.mock import Mock
        judge=QwenJudge.__new__(QwenJudge)
        judge.records=[]
        judge.client=Mock()
        report={'role':'assistant','content':'','tool_calls':[{'function':{
            'name':'report_visual_result','arguments':'{"status":"not_found","reason":"empty","coordinate_space":"normalized_1000"}'}}]}
        plain={'role':'assistant','content':'No object'}
        judge.client.complete.side_effect=[plain,report]
        result=judge.select(self.image,self.image,[],"test",coarse=True)
        self.assertEqual(result["status"],"not_found")
        self.assertEqual(judge.client.complete.call_count,2)
        judge.client.complete.side_effect=[plain,plain]
        with self.assertRaises(ValueError):
            judge.select(self.image,self.image,[],"test",coarse=True)


    def test_explicit_normalized_coordinates_map_to_original_pixels(self):
        choice={"status":"selected","coordinate_space":"normalized_1000",
                "bbox_xyxy":[600,100,800,500]}
        self.assertEqual(validate_choice(choice,[],(80,60),True),{"box":[48,6,64,30]})
        point={"status":"selected","coordinate_space":"normalized_1000","point_xy":[500,500]}
        self.assertEqual(validate_choice(point,[],(80,60),True),{"point":[40,30]})
        choice["bbox_xyxy"]=[600,100,1001,500]
        with self.assertRaises(ValueError):
            validate_choice(choice,[],(80,60),True)


if __name__=="__main__":
    unittest.main()
