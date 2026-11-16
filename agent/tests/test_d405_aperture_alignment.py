import unittest
import cv2
import numpy as np
from agent.d405_aperture_alignment import estimate_vertical_alignment

class D405AlignmentTest(unittest.TestCase):
    def frame(self):
        rgb=np.full((480,640,3),230,np.uint8)
        cv2.rectangle(rgb,(200,60),(440,150),(255,0,0),-1)
        cv2.rectangle(rgb,(220,80),(420,145),(230,230,230),-1)
        cv2.rectangle(rgb,(190,100),(370,460),(25,25,25),-1)
        xx,yy=np.meshgrid(np.arange(640),np.arange(480));z=np.full_like(xx,.2,dtype=float)
        xyz=np.stack(((xx-320)*z/400,(yy-240)*z/400,z),axis=2)
        T=np.eye(4);T[:3,:3]=[[0,0,1],[0,-1,0],[1,0,0]]
        return {'camera_id':np.array('d405'),'rgb':rgb,'xyz':xyz,'base_from_color':T}

    def test_reports_bounded_upward_guidance_without_motor_command(self):
        result=estimate_vertical_alignment(self.frame())
        self.assertEqual(result['direction'],'up')
        self.assertGreater(result['raw_error_mm'],0)
        self.assertLessEqual(result['step_mm'],3.)
        self.assertFalse(result['executable'])

    def test_rejects_camera_orientation_not_aligned_to_vertical(self):
        f=self.frame();f['base_from_color']=np.eye(4)
        with self.assertRaises(ValueError):estimate_vertical_alignment(f)

    def test_uses_tip_depth_instead_of_assuming_rim_depth(self):
        f=self.frame();black=f['rgb'][:,:,0]<50
        f['xyz'][black]*=.75
        result=estimate_vertical_alignment(f)
        self.assertAlmostEqual(result['raw_error_mm'],15.,delta=1.)

    def test_rejects_hidden_opening(self):
        f=self.frame();f['rgb'][:,:,:]=230
        with self.assertRaises(ValueError):estimate_vertical_alignment(f)
