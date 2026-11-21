import time
import unittest
from unittest.mock import patch

import numpy as np
from agent.docking_fit import OBJECT_TO_TOOL
from insertion_observation import measure_insertion, partial_front


class MeasurementTests(unittest.TestCase):
    def setUp(self):
        self.tool = np.eye(4)
        self.tool[:3,3] = [.3,0.,.5]
        self.cab = dict(inside_axis_base=[1.,0.,0.], up_base=[0.,0.,1.], mouth_anchor_base_m=[.5,.12,.48])
        self.frame = dict(camera_id='d405', id='a', image_time=time.time(), calibration_sha256='a'*64)
        self.front = dict(point_base_m=[.475,0.,.5], support_pixels=80, quad_px=[[0,0],[1,0],[1,1],[0,1]])
        self.bounds = np.array([[-45.,-37.5,0.],[36.,37.5,195.]])
        self.contact = np.array([0.,0.,169.])
        coords = np.c_[np.full(43,36.), np.linspace(-21.,21.,43), np.full(43,1.)]
        self.points = (coords-self.contact) @ OBJECT_TO_TOOL.T / 1000 + self.tool[:3,3]
        self.pixels = np.c_[np.arange(43),np.zeros(43)]
        self.mouth = patch('insertion_observation.insertion_mouth',return_value=self.cab).start()
        patch('insertion_observation.paired_geometry',return_value=(None,)*5).start()
        self.side = patch('insertion_observation._left_battery',return_value=(self.points,self.pixels)).start()
        self.addCleanup(patch.stopall)

    def test_signed_depth_without_upper_or_lower_rim_and_tracking_after_occlusion(self):
        with patch('insertion_observation.battery_front',return_value=self.front) as detect:
            first=measure_insertion(self.frame,self.tool)
            self.assertAlmostEqual(first['depth_mm'],-25.)
            self.assertIsNone(first['initial_left_clearance_mm'])
            self.assertEqual(detect.call_args.kwargs['outside_range_mm'],(-210.,300.))
            self.tool[0,3]+=.175
            self.front['point_base_m'][0]+=.175
            self.frame['id']='b'
            with patch('insertion_observation.paired_geometry',return_value=(None,)*5):
                after=measure_insertion(self.frame,self.tool,first)
                self.assertAlmostEqual(after['depth_mm'],150.)
                self.front['point_base_m'][1]=.04
                with self.assertRaisesRegex(ValueError,'不一致'):measure_insertion(self.frame,self.tool,first)
                self.frame['calibration_sha256']='b'*64
                with self.assertRaisesRegex(ValueError,'标定'):measure_insertion(self.frame,self.tool,first)
            self.mouth.assert_called_once()

    def test_distal_edge_without_four_corners_tracks_same_edge_after_push(self):
        with patch('insertion_observation.battery_front',side_effect=ValueError('occluded')) as four, \
             patch('insertion_observation.battery_geometry',return_value=(self.bounds,self.contact)):
            first=measure_insertion(self.frame,self.tool)
            self.assertEqual(first['front_method'],'partial_distal_edge')
            self.assertAlmostEqual(first['depth_mm'],-32.)
            self.tool[0,3]+=.182
            with patch('insertion_observation.paired_geometry',return_value=(None,)*5), \
                 patch('insertion_observation._left_battery',return_value=(self.points+[.182,0.,0.],self.pixels)):
                self.assertAlmostEqual(measure_insertion(self.frame,self.tool,first)['depth_mm'],150.)
            four.assert_called_once()

    def test_false_four_corner_detection_on_shoulder_never_generates_travel(self):
        self.front['point_base_m'][0] = .445
        self.side.return_value=(self.points-[.025,0.,0.],self.pixels)
        with patch('insertion_observation.battery_front',return_value=self.front), \
             patch('insertion_observation.battery_geometry',return_value=(self.bounds,self.contact)):
            with self.assertRaisesRegex(ValueError,'前端'):
                measure_insertion(self.frame,self.tool)

    def test_side_shoulder_cannot_be_mistaken_for_leading_edge(self):
        with self.assertRaisesRegex(ValueError,'前端'):
            partial_front(self.points-[.025,0.,0.],self.pixels,self.tool,self.bounds,self.contact)
        with self.assertRaisesRegex(ValueError,'横棱'):
            partial_front(self.points[:15],self.pixels[:15],self.tool,self.bounds,self.contact)

    def test_push_depth_is_independent_of_lateral_gap(self):
        for offset in (-.022, .022):
            self.tool[1,3]=offset
            self.front['point_base_m'][1]=offset
            with patch('insertion_observation.battery_front',return_value=self.front):
                measured=measure_insertion(self.frame,self.tool)
            self.assertAlmostEqual(measured['depth_mm'],-25.)
        self.side.assert_not_called()

    def test_heading_is_checked_before_attempting_depth(self):
        self.cab['inside_axis_base']=[np.cos(.2),np.sin(.2),0.]
        with self.assertRaisesRegex(ValueError,'11.46°'):measure_insertion(self.frame,self.tool)

    def test_rounding_above_angle_limit_is_unambiguous(self):
        angle=np.deg2rad(10.021)
        self.cab['inside_axis_base']=[np.cos(angle),np.sin(angle),0.]
        with self.assertRaisesRegex(ValueError,'10.02°'):
            measure_insertion(self.frame,self.tool)

    def test_missing_mouth_does_not_become_a_success(self):
        self.mouth.side_effect=ValueError('未看到仓口红边')
        with self.assertRaisesRegex(ValueError,'未看到'):measure_insertion(self.frame,self.tool)

class MouthTests(unittest.TestCase):
    def test_mouth_and_heading_from_right_rim_alone(self):
        import json
        from camera_geometry import camera_intrinsics
        from d405_left_edges import insertion_mouth
        from test_placement_stage_geometry import frame
        f=frame()
        rgb=f['rgb']
        rgb[:]=0
        rgb[325:340,270:370]=[230,15,15]
        xyz=f['xyz']
        xyz[280:325,270:370]*=1.25  # Depth recessed on aperture side only.
        rs,intr=camera_intrinsics(json.loads(f['intrinsics']))
        mouth=insertion_mouth(rgb,xyz,f['base_from_color'],rs,intr)
        self.assertEqual(mouth['method'],'depth_supported_right_inner_lip')
        np.testing.assert_allclose(mouth['inside_axis_base'],[1.,0.,0.],atol=1e-5)
        self.assertAlmostEqual(mouth['mouth_anchor_base_m'][0],.5,places=5)
        with self.assertRaisesRegex(ValueError,'入口内沿'):
            insertion_mouth(rgb,frame()['xyz'],f['base_from_color'],rs,intr)
