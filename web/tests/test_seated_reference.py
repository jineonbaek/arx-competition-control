import unittest
from seated_reference import compare_seat

class SeatedStandardTests(unittest.TestCase):
    def setUp(self):
        self.sample=dict(shoulders_mm=[-9.,-9.5],handle_outside_mm=13.,source_frame='standard',
                         surface_z_mm=[158.,162.],calibration='cal',camera_serial='405',
                         mouth=dict(inside_axis_base=[1.,0.,0.],mouth_anchor_base_m=[.5,0.,0.]))
        self.standard=dict(reference_id='confirmed',signature=self.sample)
    def observed(self,offsets,handle=13.):
        return dict(self.sample,source_frame='fresh',shoulders_mm=[a+b for a,b in zip(self.sample['shoulders_mm'],offsets)],handle_outside_mm=handle)
    def test_requires_both_shoulders_and_a_handle_outside(self):
        self.assertTrue(compare_seat(self.standard,self.observed([1.,-1.]))['reference_match'])
        for gaps,handle in [([2.1,1.],14.),([1.,1.],25.),([0.,0.],0.)]:
            self.assertFalse(compare_seat(self.standard,self.observed(gaps,handle))['reference_match'])
    def test_only_corroborated_shortfall_can_request_more_travel(self):
        result=compare_seat(self.standard,self.observed([14.,15.],27.5))
        self.assertFalse(result['reference_match'])
        self.assertAlmostEqual(result['remaining_mm'],14.)
        for gaps,handle in [([12.,4.],23.),([8.,8.],14.),([-3.,-4.],10.)]:
            self.assertFalse(compare_seat(self.standard,self.observed(gaps,handle))['allow_compensation'])
    def test_reused_exposure_other_calibration_or_invalid_surface_cannot_pass(self):
        for updates in [dict(source_frame='standard'),dict(calibration='other'),dict(camera_serial='other'),
                        dict(surface_z_mm=[170.,162.]),dict(shoulders_mm=[float('nan'),0.])]:
            with self.assertRaises(ValueError):compare_seat(self.standard,dict(self.observed([0.,0.]),**updates))

    def test_visual_disagreement_at_nominal_depth_blocks_release(self):
        import time
        from mouth_push import verify_fixed_depth
        def observe():
            r=compare_seat(self.standard,self.observed([0.,1.],26.))
            return dict(r,source='d405_operator_seated_reference',body_depth_mm=150.,
                        cameras={'d405':dict(frame_id='fresh',image_time=time.time())})
        result=verify_fixed_depth({'rail_guided':True,'axis_base':[1.,0.,0.]},observe,lambda:True)
        self.assertFalse(result['insertion_depth_verified'])
        self.assertFalse(result['allow_compensation'])

    def test_disagreement_with_positive_distance_stops_after_initial_stroke(self):
        import time
        import numpy as np
        from mouth_push import run_fixed_stroke
        tool=np.eye(4);steps=[]
        def move(phase,length,rotation):
            steps.append(length);tool[0,3]+=length/1000
        def verify(observation,guard):
            return dict(insertion_depth_verified=False,allow_compensation=False,verified_depth_mm=139.,
                        handle_outside_mm=12.,verification_frame_ids=['after'],verification_image_time=time.time())
        result=run_fixed_stroke(tool.copy(),lambda:tool.copy(),move,lambda:True,verify)
        self.assertFalse(result['release_ready'])
        self.assertEqual(steps,[150.])

    def test_partial_depth_can_only_authorize_compensation_never_release(self):
        import time
        from mouth_push import verify_fixed_depth
        def check(depth):
            def observe():
                return dict(source='d405_partial_body',mouth=self.sample['mouth'],body_depth_mm=depth,
                            handle_outside_mm=None,allow_compensation=depth<138.,compensation_limit_mm=max(0.,140.-depth),
                            cameras={'d405':dict(image_time=time.time(),frame_id='fresh')})
            return verify_fixed_depth({'rail_guided':True,'axis_base':[1.,0.,0.]},observe,lambda:True)
        far=check(70.)
        self.assertTrue(far['allow_compensation'])
        self.assertFalse(far['insertion_depth_verified'])
        self.assertIsNone(far['handle_outside_mm'])
        self.assertFalse(check(150.)['insertion_depth_verified'])
