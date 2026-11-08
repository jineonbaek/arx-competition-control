import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from handeye_steps import capture_position

class CapturePositionTests(unittest.TestCase):
    def position(self,start,target,actual):
        return capture_position(*(np.radians(v) for v in [start,target,actual]),motion=True)

    def test_approximate_destination_measured_at_7_3_is_eligible(self):
        start=np.zeros(6);target=start.copy();target[5]=8
        actual=start.copy();actual[5]=7.3
        ok,joint,error=self.position(start,target,actual)
        self.assertTrue(ok)
        self.assertEqual(joint,6)
        self.assertAlmostEqual(error,.7)

    def test_unmoved_halfway_reversed_and_far_off_axis_are_ineligible(self):
        start=np.zeros(6);target=start.copy();target[5]=8
        for j6,j2 in [(0,0),(4,0),(-8,0),(8,4)]:
            actual=start.copy();actual[5]=j6;actual[1]=j2
            self.assertFalse(self.position(start,target,actual)[0])
        self.assertFalse(self.position(start,start,start)[0])

    def test_smaller_candidate_requires_proportionate_progress(self):
        start=np.zeros(6);target=start.copy();target[4]=2
        actual=start.copy();actual[4]=.8
        self.assertFalse(self.position(start,target,actual)[0])
