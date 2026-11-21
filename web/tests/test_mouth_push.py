import unittest
import time
from unittest.mock import patch
import numpy as np

from mouth_push import ROOT, measured_end, axial_gap, run_mouth_push, end_surfaces, measure_approach
from agent.docking_fit import OBJECT_TO_TOOL


class FixedStrokeTests(unittest.TestCase):
    def run_case(self,depth,finals,short=False):
        tool=np.eye(4);steps=[];seen=iter(finals)
        def move(phase,length,rotation):
            np.testing.assert_array_equal(rotation,np.eye(3))
            steps.append(length);tool[0,3]+=(length-(5 if short else 0))/1000.
        initial=dict(gap_mm=-depth,body_depth_mm=depth,travel_cosine=1.,axis_base=[1.,0.,0.],start_tool=tool.tolist())
        def verify(obs,guard):
            d=next(seen)
            return dict(insertion_depth_verified=d is not None and 148<=d<=152,
                        verified_depth_mm=d,handle_outside_mm=None if d is None else 195.-d,
                        verification_reason='depth evidence',travel_cosine=1.,verification_frame_ids=[str(time.monotonic_ns())])
        return run_mouth_push(initial,lambda:tool.copy(),move,lambda:True,verify),steps

    def test_continuous_crossing_partial_entry_and_already_seated(self):
        for initial,expected in [(-23.,[173.]),(0.,[150.]),(80.,[70.]),(150.,[])]:
            result,steps=self.run_case(initial,[150.])
            self.assertEqual(steps,expected)
            self.assertTrue(result['insertion_depth_verified'])

    def test_positive_and_negative_eight_degree_strokes_follow_locked_axis(self):
        for degrees in (8.,-8.):
            theta=np.radians(degrees);axis=np.array([np.cos(theta),np.sin(theta),0.])
            tool=np.eye(4)
            initial=dict(gap_mm=0.,body_depth_mm=0.,axis_base=axis.tolist(),travel_cosine=1.,start_tool=tool.tolist())
            def move(phase,length,rotation):tool[:3,3]+=axis*length/1000.
            def verify(obs,guard):return dict(insertion_depth_verified=True,verification_frame_ids=['new'])
            result=run_mouth_push(initial,lambda:tool.copy(),move,lambda:True,verify)
            self.assertTrue(result['insertion_depth_verified'])
            np.testing.assert_allclose(tool[:3,3]*1000.,150.*axis,atol=1e-8)

    def test_only_measured_shortfall_can_trigger_bounded_compensation(self):
        for results,steps,verified in [([144.,150.],[150.,6.],True),
                                       ([142.,142.],[150.,8.],False),
                                       ([None],[150.],False),([160.],[150.],False)]:
            r,moves=self.run_case(0.,results)
            self.assertEqual(steps,moves);self.assertEqual(verified,r['insertion_depth_verified'])

    def test_fixed_stroke_checks_axial_arrival_after_lateral_drift(self):
        from mouth_push import run_fixed_stroke
        tool=np.eye(4);moves=[]
        def move(phase,length,rotation):
            tool[0,3]+=length/1000.
            tool[1,3]=.00267
            moves.append((phase,length))
        feedback=iter([dict(insertion_depth_verified=False,verified_depth_mm=144.,handle_outside_mm=45.,
                            travel_cosine=1.,verification_image_time=time.time(),verification_frame_ids=['one']),
                       dict(insertion_depth_verified=True,verified_depth_mm=150.,handle_outside_mm=45.,verification_image_time=time.time(),verification_frame_ids=['two'])])
        result=run_fixed_stroke(tool.copy(),lambda:tool.copy(),move,lambda:True,lambda obs,guard:next(feedback))
        self.assertEqual(moves,[('seat_push',150.),('compensate',6.)])
        self.assertTrue(result['insertion_depth_verified'])
        self.assertAlmostEqual(result['measured_travel_mm'],156.)

    def test_short_arm_feedback_does_not_authorize_more_travel(self):
        with self.assertRaisesRegex(ValueError,'终点'):
            self.run_case(20.,[],short=True)

    def test_tilted_plane_projects_gap_onto_fixed_x(self):
        n=np.array([.98,.14,.06]);n/=np.linalg.norm(n)
        p=np.array([.56,-.18,.51]);front=p-[.023,0.,0.]
        self.assertAlmostEqual(axial_gap(dict(inside_axis_base=n,mouth_anchor_base_m=p),front),23.)

    def test_depth_of_observed_rear_face_supplies_hidden_nose(self):
        bounds,contact,origin,surfaces=end_surfaces(ROOT)
        sign,z,mask=surfaces[1]
        yy,xx=np.nonzero(mask[::3,::3]);xy=np.c_[xx*1.5,yy*1.5]+origin
        for extra in (0.,6.):
            obj=np.c_[xy,np.full(len(xy),z+extra)]
            tool=np.eye(4);tool[:3,3]=[.48,0.,.5]
            points=(obj-contact)@OBJECT_TO_TOOL.T/1000+tool[:3,3]
            t=np.eye(4)
            rgb=np.full((len(points),1,3),65,np.uint8)
            f=dict(rgb=rgb,xyz=points[:,None,:],base_from_color=t)
            front=measured_end(f,tool,ROOT)
            self.assertEqual(front['method'],'measured_rear_face_and_CAD_length')
            self.assertAlmostEqual(front['point_base_m'][0],tool[0,3]+(contact[2]-bounds[0,2]-extra)/1000,places=5)

    def test_background_plane_extending_beyond_battery_is_rejected(self):
        _,contact,_,surfaces=end_surfaces(ROOT)
        y,x=np.indices((90,104));obj=np.stack([x-55.,y-45.,np.full(x.shape,surfaces[1][1])],axis=-1)
        tool=np.eye(4);tool[:3,3]=[.48,0.,.5]
        frame=dict(xyz=(obj-contact)@OBJECT_TO_TOOL.T/1000+tool[:3,3],rgb=np.full(obj.shape,75,np.uint8),base_from_color=np.eye(4))
        with self.assertRaises(ValueError):measured_end(frame,tool,ROOT)

    def test_post_measurement_uses_fresh_depth_same_mouth_and_external_handle(self):
        from mouth_push import verify_fixed_depth
        plane=dict(inside_axis_base=[1.,0.,0.],mouth_anchor_base_m=[.5,0.,0.])
        now=time.time()
        def measurement(stamp,ident):
            return dict(cameras={'d405':dict(image_time=stamp,frame_id=ident)},mouth=plane,
                        body_depth_mm=150.,handle_outside_mm=45.,axis_base=[1.,0.,0.],travel_cosine=1.)
        initial=measurement(now-.5,'before');post=measurement(now,'after')
        for change,expected in [({},True),({'body_depth_mm':144.},False),
                                ({'handle_outside_mm':-1.},False),
                                ({'mouth':dict(plane,mouth_anchor_base_m=[.52,0.,0.])},False),
                                ({'cameras':initial['cameras']},False)]:
            with patch('mouth_push.time.time',return_value=now):
                r=verify_fixed_depth(initial,lambda:dict(post,**change),lambda:True)
            self.assertEqual(r['insertion_depth_verified'],expected)

    def test_end_only_verification_requires_fresh_post_stroke_depth(self):
        from mouth_push import verify_fixed_depth
        plane=dict(inside_axis_base=[1.,0.,0.],mouth_anchor_base_m=[.5,0.,0.])
        now=time.time()
        for stamp,depth,outside,verified in [(now,150.,40.,True),(now-.2,150.,40.,False),
                                              (now,135.,40.,False),(now,150.,-2.,False)]:
            post=dict(cameras={'d405':dict(image_time=stamp,frame_id='after')},mouth=plane,
                      body_depth_mm=depth,handle_outside_mm=outside)
            with patch('mouth_push.time.time',return_value=now):
                result=verify_fixed_depth({'axis_base':[1.,0.,0.]},lambda:post,lambda:True)
            self.assertEqual(result['insertion_depth_verified'],verified)

    def test_visual_gate_accepts_exactly_ten_and_checks_travel_axis(self):
        from mouth_push import seating_geometry
        from scipy.spatial.transform import Rotation
        tool=np.eye(4)
        for angle in (8.,10.,10.01):
            n=Rotation.from_euler('z',angle,degrees=True).apply([1.,0.,0.])
            plane=dict(inside_axis_base=n,mouth_anchor_base_m=[.5,0.,0.])
            if angle>10.:
                with self.assertRaisesRegex(ValueError,'10°'):
                    seating_geometry(plane,[.65,0.,0.],tool,169.28)
            else:
                r=seating_geometry(plane,[.65,0.,0.],tool,169.28)
                self.assertAlmostEqual(r['body_depth_mm'],150*n[0])
                self.assertAlmostEqual(r['handle_outside_mm'],19.28*n[0])

class RailGuidedTests(unittest.TestCase):
    def stroke(self, depths, *, slip=0., repeated=False, stale=False):
        from mouth_push import run_fixed_stroke
        from scipy.spatial.transform import Rotation
        start=np.eye(4);tool=start.copy();path=[];samples=iter(depths)
        def move(phase,length,rotation):
            np.testing.assert_allclose(rotation,tool[:3,:3])
            tool[0,3]+=(length-(slip if not path else 0.))/1000
            tool[:3,:3]=Rotation.from_euler('z',135,degrees=True).as_matrix()
            path.append(length)
        def verify(obs,guard):
            depth=next(samples)
            return dict(insertion_depth_verified=depth==150.,verified_depth_mm=depth,
                        handle_outside_mm=40.,verification_reason='camera',
                        verification_image_time=time.time()-(20 if stale else 0),
                        verification_frame_ids=['one' if repeated else str(len(path))])
        return run_fixed_stroke(start,lambda:tool.copy(),move,lambda:True,verify),path

    def test_recorded_147mm_stroke_verifies_and_compensates_only_measured_gap(self):
        result,commands=self.stroke([143.,150.],slip=2.23)
        self.assertEqual(commands,[150.,7.])
        self.assertAlmostEqual(result['measured_travel_mm'],154.77)
        self.assertTrue(result['release_ready'])
        result,commands=self.stroke([None],slip=2.23)
        self.assertEqual(commands,[150.])
        self.assertFalse(result['release_ready'])

    def test_more_than_two_corrections_after_large_rotation(self):
        result,path=self.stroke([116.,126.,136.,146.,150.])
        self.assertEqual(path,[150.,10.,10.,10.,4.])
        self.assertTrue(result['release_ready'])
        self.assertAlmostEqual(result['measured_travel_mm'],184.)

    def test_seat_image_below_measured_minimum_holds(self):
        result,path=self.stroke([150.],slip=.5)
        self.assertEqual(path,[150.])
        self.assertFalse(result['release_ready'])
        self.assertTrue(result['insertion_depth_verified'])

    def test_stale_repeated_missing_or_stalled_depth_holds(self):
        for depths,options,steps in [([150.],{'stale':True},1),([140.,150.],{'repeated':True},2),
                                     ([None],{},1),([130.,130.],{},2)]:
            result,path=self.stroke(depths,**options)
            self.assertFalse(result['release_ready'])
            self.assertEqual(len(path),steps)

    def test_release_rechecks_live_position_and_image_age(self):
        from mouth_push import require_seated_release
        result,_=self.stroke([150.])
        initial=np.eye(4);tool=initial.copy();tool[0,3]=.150
        require_seated_release(initial,tool,initial[:3,0],result)
        tool[0,3]=.14999
        with self.assertRaises(ValueError):require_seated_release(initial,tool,initial[:3,0],result)
        tool[0,3]=.150
        with self.assertRaises(ValueError):require_seated_release(initial,tool,initial[:3,0],dict(result,verification_image_time=time.time()-6))

    def test_seat_geometry_at_oblique_angle_has_no_entry_gate(self):
        from mouth_push import seating_geometry,verify_fixed_depth
        from scipy.spatial.transform import Rotation
        tool=np.eye(4);tool[:3,:3]=Rotation.from_euler('z',20,degrees=True).as_matrix()
        plane=dict(inside_axis_base=[1.,0.,0.],mouth_anchor_base_m=[0.,0.,0.])
        geo=seating_geometry(plane,[.15,0.,0.],tool,195.,seat_only=True)
        self.assertAlmostEqual(geo['body_depth_mm'],150.);self.assertIsNone(geo['gap_mm'])
        def capture():return dict(geo,cameras={'d405':dict(image_time=time.time(),frame_id='after')})
        result=verify_fixed_depth(dict(axis_base=tool[:3,0],rail_guided=True),capture,lambda:True)
        self.assertTrue(result['insertion_depth_verified'])
