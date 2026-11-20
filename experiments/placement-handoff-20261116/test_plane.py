import unittest
import numpy as np
import cv2
from d405_plane import estimate_battery_plane

class PlaneTests(unittest.TestCase):
    def frame(self,angle_deg=0.):
        h,w=120,160;rgb=np.full((h,w,3),230,np.uint8)
        cv2.rectangle(rgb,(25,15),(135,110),(20,20,20),-1)
        yy,xx=np.mgrid[:h,:w];x=(xx-80)*.001;y=(yy-60)*.001
        z=.4+np.tan(np.radians(angle_deg))*x
        xyz=np.stack((x,y,z),-1)
        return {'camera_id':'d405','rgb':rgb,'xyz':xyz,'base_from_color':np.eye(4)}
    def test_synthetic_horizontal_plane(self):
        result=estimate_battery_plane(self.frame())
        self.assertAlmostEqual(result['tilt_deg'],0.,delta=.1)
        self.assertFalse(result['executable'])
    def test_synthetic_tilt(self):
        result=estimate_battery_plane(self.frame(5.))
        self.assertAlmostEqual(result['tilt_deg'],5.,delta=.3)
    def test_missing_depth_and_wrong_camera_fail(self):
        f=self.frame();f['xyz'][:]=0
        with self.assertRaises(ValueError):estimate_battery_plane(f)
        f=self.frame();f['camera_id']='d435'
        with self.assertRaises(ValueError):estimate_battery_plane(f)
if __name__=='__main__':unittest.main()
