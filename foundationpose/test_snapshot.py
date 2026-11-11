import unittest
import numpy as np
from infer_snapshot import segment


class SnapshotMaskTests(unittest.TestCase):
    def test_box_initialization_extracts_shape_instead_of_filling_background(self):
        rgb=np.full((100,120,3),220,np.uint8)
        rgb[30:75,40:80]=30
        mask=segment(rgb,[25,15,95,90])
        self.assertGreater(mask[35:70,45:75].mean(),.95)
        self.assertFalse(mask[:10].any())
        self.assertLess(mask.sum(),70*75*.7)

    def test_tiny_or_outside_box_is_rejected(self):
        rgb=np.zeros((100,120,3),np.uint8)
        for roi in [[0,0,10,10],[0,0,999,100]]:
            with self.assertRaises(ValueError):segment(rgb,roi)


if __name__=="__main__":unittest.main()
