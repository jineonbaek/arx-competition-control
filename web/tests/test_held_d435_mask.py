import unittest
import cv2
import numpy as np

from agent.held_d435_mask import held_battery_mask


class HeldD435MaskTests(unittest.TestCase):
    def test_depth_volume_and_low_saturation_exclude_colored_gripper(self):
        rgb=np.full((60,80,3),220,np.uint8)
        xyz=np.zeros((60,80,3),float)
        rgb[18:42,24:52]=[45,45,45]
        xyz[18:42,24:52]=[.02,.02,.08]
        rgb[36:52,36:64]=[20,130,145]
        xyz[36:52,36:64]=[.03,.03,.09]
        mask,roi=held_battery_mask(
            rgb,xyz,np.eye(4),np.eye(4),
            np.array([[0.,0.,0.],[100.,100.,200.]]),
            projected_center_px=[38.,30.],min_area=100)
        self.assertGreater(np.count_nonzero(mask[20:34,26:50]),250)
        self.assertEqual(np.count_nonzero(mask[44:50,40:60]),0)
        self.assertEqual(roi,[24,18,52,42])

    def test_dark_background_outside_object_volume_is_rejected(self):
        rgb=np.full((40,50,3),20,np.uint8)
        xyz=np.full((40,50,3),[.3,.3,.3],float)
        rgb[12:28,18:34]=[55,55,55]
        xyz[12:28,18:34]=[.02,.02,.08]
        mask,_=held_battery_mask(
            rgb,xyz,np.eye(4),np.eye(4),
            np.array([[0.,0.,0.],[100.,100.,200.]]),
            projected_center_px=[26.,20.],min_area=80)
        self.assertEqual(np.count_nonzero(mask[:10]),0)
        self.assertGreater(np.count_nonzero(mask[12:28,18:34]),200)


if __name__=='__main__':unittest.main()
