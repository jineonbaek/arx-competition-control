import unittest
from pathlib import Path
import cv2
import numpy as np
import offline
from d405_partial import estimate_vertical_alignment

ROOT=Path(__file__).resolve().parent

class OfflineTests(unittest.TestCase):
    def test_coarse_frame_detected(self):
        image=np.full((240,320,3),230,np.uint8)
        cv2.rectangle(image,(60,50),(260,160),(0,0,255),10)
        result=offline.detect_cabinet(image)
        self.assertEqual(result['status'],'detected')
        self.assertLess(abs(result['center_px'][0]-160),3)
        self.assertFalse(result['executable'])
    def test_blank_image_does_not_invent_cabinet(self):
        with self.assertRaises(ValueError):offline.detect_cabinet(np.full((240,320,3),200,np.uint8))
    def test_point_cloud_reprojection_obeys_transform_and_image_bounds(self):
        view={'width':100,'height':100,'base_from_color':np.eye(4).tolist(),
              'points':[[0.,0.,1.,1,2,3],[10.,0.,1.,1,2,3]]}
        result=offline.project_points(view,{'fx':50.,'fy':50.,'cx':50.,'cy':50.})
        self.assertEqual(len(result['camera_xyz']),1)
        np.testing.assert_array_equal(result['pixel_xy'][0],[50,50])
    def test_multiple_red_boxes_are_not_arbitrarily_selected(self):
        image=np.full((240,400,3),230,np.uint8)
        cv2.rectangle(image,(20,30),(160,180),(0,0,255),10)
        cv2.rectangle(image,(230,30),(370,180),(0,0,255),10)
        with self.assertRaises(ValueError):offline.detect_cabinet(image)
    def test_d405_partial_upper_rim_and_missing_side(self):
        h,w=480,640;rgb=np.full((h,w,3),230,np.uint8)
        cv2.rectangle(rgb,(200,0),(220,49),(255,0,0),-1)
        cv2.rectangle(rgb,(420,0),(440,49),(255,0,0),-1)
        cv2.rectangle(rgb,(200,35),(440,52),(255,0,0),-1)
        cv2.rectangle(rgb,(190,85),(370,460),(25,25,25),-1)
        xx,yy=np.meshgrid(np.arange(w),np.arange(h));z=np.full_like(xx,.2,dtype=float)
        xyz=np.stack(((xx-320)*z/400,(yy-240)*z/400,z),axis=2)
        T=np.eye(4);T[:3,:3]=[[0,0,1],[0,-1,0],[1,0,0]]
        frame={'camera_id':'d405','rgb':rgb,'xyz':xyz,'base_from_color':T}
        result=estimate_vertical_alignment(frame)
        self.assertTrue(result['rim_clipped_top']);self.assertLessEqual(abs(result['step_mm']),3.)
        self.assertFalse(result['executable'])
        frame['rgb'][:60,400:]=230
        with self.assertRaises(ValueError):estimate_vertical_alignment(frame)

    def test_d405_no_depth_cannot_produce_adjustment(self):
        h,w=480,640;rgb=np.full((h,w,3),230,np.uint8)
        cv2.rectangle(rgb,(200,0),(220,49),(255,0,0),-1)
        cv2.rectangle(rgb,(420,0),(440,49),(255,0,0),-1)
        cv2.rectangle(rgb,(200,35),(440,52),(255,0,0),-1)
        cv2.rectangle(rgb,(190,85),(370,460),(25,25,25),-1)
        T=np.eye(4);T[:3,:3]=[[0,0,1],[0,-1,0],[1,0,0]]
        frame={'camera_id':'d405','rgb':rgb,'xyz':np.zeros((h,w,3)),'base_from_color':T}
        with self.assertRaises(ValueError):estimate_vertical_alignment(frame)

    def test_invalid_transform_is_not_used(self):
        view={'width':100,'height':100,'base_from_color':np.zeros((4,4)).tolist(),'points':[[0,0,1,1,2,3]]}
        with self.assertRaises(ValueError):offline.project_points(view,{'fx':50.,'fy':50.,'cx':50.,'cy':50.})

    def test_invalid_intrinsics_are_rejected(self):
        view={'width':100,'height':100,'base_from_color':np.eye(4).tolist(),'points':[[0,0,1,1,2,3]]}
        for fx in (0.,float('nan'),-1.):
            with self.assertRaises(ValueError):offline.project_points(view,{'fx':fx,'fy':50.,'cx':50.,'cy':50.})

if __name__=='__main__':unittest.main()
