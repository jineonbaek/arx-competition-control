import unittest
import numpy as np
from infer_snapshot import input_mask

class MaskInputTests(unittest.TestCase):
    def test_mask_matches_raw_frame(self):
        rgb=np.zeros((30,40,3),np.uint8)
        mask=np.zeros((30,40),np.uint8);mask[2:20,3:25]=255
        self.assertEqual(int(input_mask(rgb,[3,2,25,20],mask).sum()),396)
    def test_reject_mismatch_empty_and_roi_leak(self):
        rgb=np.zeros((30,40,3),np.uint8)
        for mask in (np.zeros((15,20),np.uint8),np.zeros((30,40),np.uint8),np.ones((30,40),np.uint8)):
            with self.assertRaises(ValueError):input_mask(rgb,[3,2,25,20],mask)
if __name__=="__main__":unittest.main()
