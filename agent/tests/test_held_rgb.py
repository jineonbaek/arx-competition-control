import unittest
import cv2
import numpy as np
from agent.held_rgb import ReferenceMatcher
from agent.tests.test_held_battery import IDENTITY


def fixture():
    rng=np.random.default_rng(64);empty=np.full((240,320,3),75,np.uint8)
    jaw=np.zeros((240,320),np.uint8);jaw[25:70,20:85]=255;jaw[25:70,235:300]=255
    texture=rng.integers(30,245,(240,320,3),dtype=np.uint8);empty[jaw>0]=texture[jaw>0]
    good=empty.copy();mask=np.zeros((240,320),np.uint8);mask[95:205,90:230]=255
    good[mask>0]=texture[mask>0];cv2.rectangle(good,(105,110),(215,180),(230,230,230),3)
    return good,empty,mask,jaw


class RGBTests(unittest.TestCase):
    def make(self):
        good,empty,mask,jaw=fixture();return ReferenceMatcher(good,mask,jaw,identity=IDENTITY,reference_kind='synthetic',empty_rgb=empty)
    def test_local_reference_tracks_without_depth_or_whole_battery(self):
        r=self.make().analyze(fixture()[0],frame_id='1',captured_at=10.)
        self.assertEqual(r['presence'],'present');self.assertEqual(r['engagement'],'visually_consistent');self.assertIsNone(r['metric_pose'])
        self.assertEqual(r['pose'],'matched');self.assertFalse(r['executable'])
    def test_visible_empty_can_be_distinguished_from_matching_battery(self):
        r=self.make().analyze(fixture()[1],frame_id='1',captured_at=10.)
        self.assertEqual(r['presence'],'absent')
    def test_battery_shift_relative_to_jaws_is_misgrasp(self):
        good,empty,mask,_=fixture();shifted=cv2.warpAffine(good,np.float32([[1,0,12],[0,1,0]]),(320,240));out=empty.copy();region=cv2.warpAffine(mask,np.float32([[1,0,12],[0,1,0]]),(320,240))>0;out[region]=shifted[region]
        r=self.make().analyze(out,frame_id='1',captured_at=10.)
        self.assertEqual(r['presence'],'present');self.assertEqual(r['engagement'],'visibly_incorrect');self.assertGreater(r['image_delta'][0],9.)
    def test_shared_image_motion_does_not_fake_relative_slip(self):
        good=fixture()[0];m=np.float32([[1,0,4],[0,1,2]]);moved=cv2.warpAffine(good,m,(320,240))
        r=self.make().analyze(moved,frame_id='1',captured_at=10.)
        self.assertEqual(r['engagement'],'visually_consistent');self.assertLess(np.linalg.norm(r['image_delta'][:2]),1.)
    def test_missing_jaw_anchors_or_occluded_battery_is_unknown(self):
        good,_,mask,jaw=fixture()
        for m in [mask,jaw]:
            image=good.copy();image[m>0]=140;r=self.make().analyze(image,frame_id='1',captured_at=10.)
            self.assertEqual(r['presence'],'unknown');self.assertFalse(r['critical_visible'])
    def test_arbitrary_first_frame_is_not_a_qualified_reference(self):
        good,empty,mask,jaw=fixture()
        with self.assertRaises(ValueError):ReferenceMatcher(good,mask,jaw,identity=IDENTITY,reference_kind='first_frame',empty_rgb=empty)
