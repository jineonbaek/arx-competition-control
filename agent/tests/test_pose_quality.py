import unittest
import numpy as np
from foundationpose.pose_quality import assess_depth_fit

class PoseQualityTests(unittest.TestCase):
    def setUp(self):
        self.mask=np.zeros((40,60),bool);self.mask[5:35,10:50]=True
        self.render=np.where(self.mask,.5,0.)
        self.depth=np.where(self.mask,.502,1.)

    def test_matching_surface_reports_mm_without_motion_approval(self):
        r=assess_depth_fit(self.mask,self.depth,self.render)
        self.assertEqual(r['status'],'consistent')
        self.assertAlmostEqual(r['median_abs_depth_mm'],2)
        self.assertEqual(r['mask_coverage'],1)
        self.assertFalse(r['physical_accuracy_validated'])

    def test_wrong_range_or_silhouette_rejected(self):
        r=assess_depth_fit(self.mask,self.depth+.04,self.render)
        self.assertEqual(r['status'],'rejected')
        render=np.roll(self.render,20,axis=1)
        self.assertEqual(assess_depth_fit(self.mask,self.depth,render)['status'],'rejected')

    def test_holes_are_missing_evidence_not_good_fit(self):
        d=self.depth.copy();d[self.mask]=0;d[5:7,10:50]=.5
        r=assess_depth_fit(self.mask,d,self.render)
        self.assertEqual(r['status'],'insufficient_depth')
        self.assertLess(r['valid_depth_ratio'],.1)

    def test_nonfinite_and_dimensions_fail(self):
        for render in (np.full_like(self.render,np.nan),np.zeros((4,4))):
            with self.assertRaises(ValueError):assess_depth_fit(self.mask,self.depth,render)
        with self.assertRaises(ValueError):assess_depth_fit(np.full(self.mask.shape,np.nan),self.depth,self.render)

    def test_small_outliers_report_tail_and_do_not_hide_mismatch(self):
        d=self.depth.copy();d[5:12,10:50]+=.06
        r=assess_depth_fit(self.mask,d,self.render)
        self.assertEqual(r['status'],'rejected')
        self.assertGreater(r['p90_abs_depth_mm'],50)

if __name__=='__main__':unittest.main()
