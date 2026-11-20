import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

import d435_live_overlay as overlay_module
from d435_live_overlay import LiveOverlay, battery_box_edges, battery_top_face, cabinet_axis_from_quad, project_base, snap_wireframe_to_top_face, tracked_object_pose, visible_face_overlay, write_depth_map


class Model:
    def wrist_transform(self, joints, lift):
        out = np.eye(4)
        out[:3, 3] = [float(joints[0]), float(joints[1]), float(lift)]
        return out


class OverlayTests(unittest.TestCase):
    @staticmethod
    def depth_plane(box, *, shape=(100, 120), depth=.7):
        x1, y1, x2, y2 = box
        rgb = np.zeros((*shape, 3), np.uint8)
        xyz = np.zeros((*shape, 3), np.float32)
        yy, xx = np.mgrid[y1:y2, x1:x2]
        xyz[y1:y2, x1:x2, 0] = (xx - shape[1] / 2) * .001
        xyz[y1:y2, x1:x2, 1] = (yy - shape[0] / 2) * .001
        xyz[y1:y2, x1:x2, 2] = depth + (xx - x1) * .0001
        return rgb, xyz

    def depth_detector(self):
        detector = getattr(overlay_module, 'battery_depth_face', None)
        self.assertIsNotNone(detector, 'battery_depth_face must be implemented')
        return detector

    def test_object_pose_tracks_tool_without_rerunning_foundationpose(self):
        model = Model()
        tcp = np.eye(4)
        reference = np.eye(4)
        reference[:3, 3] = [1.1, 2.2, 3.3]
        current = tracked_object_pose(model, tcp, [1, 2, 0, 0, 0, 0], 3,
                                      reference, [1, 2, 0, 0, 0, 0], 3)
        np.testing.assert_allclose(current, reference)
        moved = tracked_object_pose(model, tcp, [1.07, 2, 0, 0, 0, 0], 3.09,
                                    reference, [1, 2, 0, 0, 0, 0], 3)
        np.testing.assert_allclose(moved[:3, 3], reference[:3, 3] + [.07, 0, .09])

    def test_projection_rejects_points_behind_camera(self):
        intrinsics = dict(fx=100, fy=100, ppx=50, ppy=40)
        self.assertEqual(project_base([0, 0, 1], np.eye(4), intrinsics), (50, 40))
        self.assertEqual(project_base([0, 0, 1], np.eye(4), dict(fx=100, fy=100, cx=51, cy=41)), (51, 41))
        self.assertIsNone(project_base([0, 0, -1], np.eye(4), intrinsics))

    def test_cabinet_axis_can_fall_back_to_calibrated_red_quad(self):
        intrinsics = dict(fx=500, fy=500, cx=320, cy=240)
        axis = cabinet_axis_from_quad([[270, 210], [370, 210], [370, 270], [270, 270]], intrinsics)
        self.assertIsNotNone(axis)
        self.assertEqual(axis[0], (320, 240))

    def test_battery_is_drawn_as_twelve_cad_edges(self):
        intrinsics = dict(fx=100, fy=100, cx=50, cy=40)
        transform = np.eye(4); transform[2, 3] = 1
        edges = battery_box_edges(transform, [[-10, -20, 0], [10, 20, 100]], np.eye(4), intrinsics)
        self.assertEqual(len(edges), 12)

    def test_rgbd_top_face_selects_gray_component_with_valid_depth(self):
        rgb = np.zeros((100, 120, 3), np.uint8); rgb[20:70, 40:90] = [125, 140, 150]
        xyz = np.zeros((100, 120, 3), np.float32)
        yy,xx=np.mgrid[20:70,40:90];xyz[20:70,40:90,0]=(xx-65)*.001;xyz[20:70,40:90,1]=(yy-45)*.001;xyz[20:70,40:90,2]=.7
        quad = battery_top_face(rgb, xyz, (65, 45), .7)
        self.assertEqual(quad.shape, (4, 2))

    def test_partial_black_battery_at_right_boundary_is_depth_detected(self):
        fixture = Path(__file__).parent / 'fixtures/d435-partial-black-battery-frame.npz'
        with np.load(fixture, allow_pickle=False) as frame:
            quad = battery_top_face(frame['rgb'], frame['xyz'], tuple(frame['expected_px']),
                                    float(frame['expected_depth_m']))
        self.assertIsNotNone(quad)
        self.assertEqual(quad.shape, (4, 2))
        self.assertGreaterEqual(float(np.max(quad[:, 0])), 638.)

    def test_depth_face_accepts_right_edge_crop(self):
        rgb, xyz = self.depth_plane((68, 15, 120, 90))
        observation = self.depth_detector()(rgb, xyz, (100, 50), .705)
        self.assertIsNotNone(observation)
        self.assertIn('right', observation['boundary_contacts'])
        self.assertEqual(observation['method'], 'depth_partial_plane')

    def test_depth_face_accepts_top_right_crop(self):
        rgb, xyz = self.depth_plane((62, 0, 120, 78))
        observation = self.depth_detector()(rgb, xyz, (96, 35), .705)
        self.assertIsNotNone(observation)
        self.assertEqual(set(observation['boundary_contacts']), {'top', 'right'})

    def test_depth_face_rejects_plane_at_wrong_depth(self):
        rgb, xyz = self.depth_plane((35, 20, 105, 85), depth=.25)
        self.assertIsNone(self.depth_detector()(rgb, xyz, (70, 50), .7))

    def test_depth_face_rejects_thin_arm_like_plane(self):
        rgb, xyz = self.depth_plane((8, 45, 115, 55))
        self.assertIsNone(self.depth_detector()(rgb, xyz, (65, 50), .705))

    def test_depth_face_rejects_insufficient_area(self):
        rgb, xyz = self.depth_plane((55, 42, 70, 57))
        self.assertIsNone(self.depth_detector()(rgb, xyz, (62, 50), .705))

    def test_depth_face_rejects_missing_depth(self):
        rgb = np.zeros((100, 120, 3), np.uint8)
        xyz = np.zeros((100, 120, 3), np.float32)
        self.assertIsNone(self.depth_detector()(rgb, xyz, (60, 50), .7))

    def test_observed_top_face_warps_all_twelve_cad_edges(self):
        pixels = [(10, 10), (10, 50), (30, 10), (30, 50), (50, 10), (50, 50), (70, 10), (70, 50)]
        edges, axis, aligned = snap_wireframe_to_top_face(pixels, ((40, 30), (40, 50)),
                                                          [[280, 210], [360, 210], [360, 270], [280, 270]],
                                                          dict(fx=500,fy=500,cx=320,cy=240),
                                                          [[-40,-30,0],[40,30,190]], .5)
        self.assertTrue(aligned); self.assertEqual(len(edges), 12); self.assertIsNotNone(axis)

    def test_visible_face_overlay_never_extrapolates_into_air(self):
        quad = np.array([[40,20],[90,20],[80,80],[50,80]],np.float32)
        edges, axis = visible_face_overlay(quad)
        self.assertEqual(len(edges), 4)
        self.assertEqual(axis, ((65,80),(65,20)))

    def test_depth_map_uses_fixed_scale_and_black_for_invalid_pixels(self):
        with tempfile.TemporaryDirectory() as directory:
            xyz = np.zeros((100,100,3),np.float32); xyz[40:60,40:60,2] = .7
            path = Path(directory)/'depth.jpg'; write_depth_map(xyz,path)
            image = cv2.imread(str(path))
            self.assertIsNotNone(image); self.assertGreater(image[50,50].sum(),0)
            self.assertLess(image[90,90].sum(),20)

    def test_render_draws_contour_rim_and_both_orientation_arrows(self):
        image = np.zeros((100, 140, 3), np.uint8)
        edges = [((12, 30), (45, 30)), ((45, 30), (45, 72)), ((45, 72), (12, 72)), ((12, 72), (12, 30))]
        detection = dict(status='complete', outer_contour_px=[[75, 25], [125, 25], [125, 80], [75, 80]],
                         inner_contour_px=[[82, 32], [118, 32], [118, 73], [82, 73]],
                         inner_quad_px=[[82, 32], [118, 32], [118, 73], [82, 73]])
        with tempfile.TemporaryDirectory() as directory:
            overlay = LiveOverlay.__new__(LiveOverlay)
            overlay.output = Path(directory)
            result = overlay.render(image, edges, detection, ((28, 52), (40, 52)), ((100, 52), (90, 52)), 'frame-1')
            self.assertTrue((Path(directory) / 'd435.jpg').is_file())
            metadata = json.loads((Path(directory) / 'd435.json').read_text())
            self.assertEqual(metadata['sequence'], 'frame-1')
            self.assertEqual(metadata['battery_axis']['status'], 'visible')
            self.assertEqual(metadata['cabinet_axis']['status'], 'visible')
            self.assertEqual(metadata['battery_contour']['method'], 'current depth-supported visible face only')
            self.assertFalse(metadata['stale'])
            self.assertEqual(result['status'], 'ready')

    def test_render_labels_measured_partial_contour_and_inferred_completion(self):
        image = np.zeros((100, 140, 3), np.uint8)
        observation = dict(method='depth_partial_plane', visible_fraction=.61,
                           boundary_contacts=['top', 'right'], confidence=.82)
        measured = [[78, 0], [139, 0], [139, 70], [82, 72]]
        inferred = [((78, 0), (139, 0)), ((82, 72), (139, 70)),
                    ((78, 0), (82, 72)), ((139, 0), (139, 70))]
        with tempfile.TemporaryDirectory() as directory:
            overlay = LiveOverlay.__new__(LiveOverlay)
            overlay.output = Path(directory)
            overlay.render(image, [], {'status': 'not_found'}, ((110, 55), (100, 35)), None, 'partial-1',
                           measured_contour=measured, inferred_edges=inferred,
                           battery_observation=observation)
            metadata = json.loads((Path(directory) / 'd435.json').read_text())
        self.assertEqual(metadata['battery_contour']['status'], 'visible')
        self.assertEqual(metadata['battery_contour']['method'], 'depth_partial_plane')
        self.assertEqual(metadata['battery_contour']['boundary_contacts'], ['top', 'right'])
        self.assertAlmostEqual(metadata['battery_contour']['visible_fraction'], .61)
        self.assertAlmostEqual(metadata['battery_contour']['confidence'], .82)
        self.assertTrue(metadata['battery_contour']['inferred_completion'])
        self.assertFalse(metadata['battery_contour']['hidden_space_clearance_proven'])


if __name__ == '__main__':
    unittest.main()
