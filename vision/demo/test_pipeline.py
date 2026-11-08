import tempfile
from pathlib import Path
import unittest
import numpy as np
from PIL import Image
from pipeline import checked_box, validate_choice, mask_geometry, run_frame

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
        for box in [[0,0,90,20],[0,0,float("nan"),20],[1,1,1,5],[True,0,4,5]]:
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

if __name__=="__main__":
    unittest.main()
