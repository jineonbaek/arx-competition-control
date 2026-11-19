import copy
import unittest

import cv2
import numpy as np

from agent.d405_aperture_alignment import (
    estimate_horizontal_alignment,
    estimate_vertical_alignment,
)


CALIBRATION = "a" * 64


def frame(*, axis_source="camera_y"):
    height, width = 120, 160
    rgb = np.full((height, width, 3), 255, np.uint8)
    cv2.rectangle(rgb, (30, 20), (130, 80), (255, 0, 0), 6)
    cv2.rectangle(rgb, (55, 72), (105, 108), (0, 0, 0), -1)
    xyz = np.zeros((height, width, 3), float)
    xyz[:, :, 2] = .5
    red = (rgb[:, :, 0] > 200) & (rgb[:, :, 1] < 40)
    battery = np.all(rgb < 20, axis=2)
    transform = np.eye(4)
    if axis_source == "camera_y":
        xyz[red, 1] = 0.
        xyz[battery, 1] = .040
    elif axis_source == "camera_x":
        # base Y is camera X, while the observed edge pixels stay unchanged.
        transform[:3, :3] = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        xyz[red, 0] = 0.
        xyz[battery, 0] = .040
    else:
        raise AssertionError(axis_source)
    return dict(
        id=np.array("frame-1"),
        camera_id=np.array("d405"),
        calibration_sha256=np.array(CALIBRATION),
        rgb=rgb,
        xyz=xyz,
        base_from_color=transform,
        image_time=np.array(10.),
        arm_time=np.array(10.02),
        body_time=np.array(10.01),
    )


class D405HorizontalAlignmentTests(unittest.TestCase):
    def estimate(self, value=None, **kwargs):
        after = kwargs.pop("after_image_time", 9.)
        return estimate_horizontal_alignment(
            frame() if value is None else value,
            target_offset_mm=42.4,
            correction_axis_base=(0., 1., 0.),
            max_step_mm=3.,
            expected_calibration_sha256=CALIBRATION,
            after_image_time=after,
            **kwargs,
        )

    def test_horizontal_alignment_returns_base_y_residual_to_left_quarter_target(self):
        result = self.estimate()
        self.assertEqual(result["status"], "visual_guidance_only")
        self.assertAlmostEqual(result["rim_center_axis_m"], 0., places=6)
        self.assertAlmostEqual(result["battery_front_axis_m"], .040, places=6)
        self.assertEqual(result["target_offset_mm"], 42.4)
        self.assertAlmostEqual(result["observed_offset_mm"], 40., places=6)
        self.assertAlmostEqual(result["raw_error_mm"], 2.4, places=6)
        self.assertAlmostEqual(result["step_mm"], 2.4, places=6)
        self.assertEqual(result["axis_base"], [0., 1., 0.])
        self.assertEqual(result["frame_id"], "frame-1")

    def test_horizontal_alignment_is_invariant_when_base_y_maps_to_image_rows(self):
        result = self.estimate(frame(axis_source="camera_x"))
        self.assertAlmostEqual(result["raw_error_mm"], 2.4, places=6)
        self.assertAlmostEqual(result["step_mm"], 2.4, places=6)

    def test_horizontal_alignment_clamps_step_to_three_mm(self):
        value = frame()
        value["xyz"][np.all(value["rgb"] < 20, axis=2), 1] = .020
        result = self.estimate(value)
        self.assertAlmostEqual(result["raw_error_mm"], 22.4, places=6)
        self.assertEqual(result["step_mm"], 3.)

    def test_horizontal_alignment_rejects_stale_frame(self):
        with self.assertRaisesRegex(ValueError, "newer D405 frame"):
            self.estimate(after_image_time=10.)

    def test_horizontal_alignment_rejects_wrong_camera_or_calibration(self):
        wrong = frame()
        wrong["camera_id"] = np.array("d435")
        with self.assertRaisesRegex(ValueError, "identity"):
            self.estimate(wrong)
        wrong = frame()
        wrong["calibration_sha256"] = np.array("b" * 64)
        with self.assertRaisesRegex(ValueError, "identity"):
            self.estimate(wrong)

    def test_horizontal_alignment_rejects_clipped_rim_missing_battery_edge_and_invalid_depth(self):
        clipped = frame()
        clipped["rgb"] = np.roll(clipped["rgb"], -30, axis=1)
        with self.assertRaisesRegex(ValueError, "clipped"):
            self.estimate(clipped)
        missing = frame()
        missing["rgb"][np.all(missing["rgb"] < 20, axis=2)] = 255
        with self.assertRaisesRegex(ValueError, "battery front edge"):
            self.estimate(missing)
        invalid = frame()
        invalid["xyz"][:] = np.nan
        with self.assertRaisesRegex(ValueError, "depth"):
            self.estimate(invalid)

    def test_existing_vertical_alignment_contract_remains_available(self):
        self.assertTrue(callable(estimate_vertical_alignment))
        bad = copy.deepcopy(frame())
        with self.assertRaisesRegex(ValueError, "image horizontal axis"):
            estimate_vertical_alignment(bad)


if __name__ == "__main__":
    unittest.main()
