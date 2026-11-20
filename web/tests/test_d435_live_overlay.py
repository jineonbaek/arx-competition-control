import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from d435_live_overlay import LiveOverlay, battery_box_edges, battery_top_face, cabinet_axis_from_quad, project_base, snap_wireframe_to_top_face, tracked_object_pose, visible_face_overlay, write_depth_map


class Model:
    def wrist_transform(self, joints, lift):
        out = np.eye(4)
        out[:3, 3] = [float(joints[0]), float(joints[1]), float(lift)]
        return out


class OverlayTests(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
