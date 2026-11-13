import unittest
import numpy as np
from chassis_filter import bottom_right_weak_returns


class CloseReturnTest(unittest.TestCase):
    def test_only_measured_return_and_volume(self):
        inside = [.08, -.35, .40]
        points = np.array([inside]*4 + [[.12,-.35,.40],[.08,-.40,.40],
                                        [.08,-.35,.54],[.08,-.35,.30],[.08,.35,.40]])
        strength = np.array([1., 0., 2., np.nan] + [1.]*5)
        self.assertEqual(bottom_right_weak_returns(points,strength).tolist(),
                         [True] + [False]*8)

    def test_no_usable_reflectivity(self):
        points = np.array([[.08,-.35,.40]])
        self.assertFalse(bottom_right_weak_returns(points,None).any())
