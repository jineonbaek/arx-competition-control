import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "web" / "src")]

from agent.d405_aperture_alignment import estimate_horizontal_alignment
from agent.tests.test_d405_aperture_alignment import CALIBRATION, frame
from d405_alignment_overlay import write_horizontal_overlay


class D405AlignmentOverlayTests(unittest.TestCase):
    def measurement(self, value=None):
        return estimate_horizontal_alignment(
            frame() if value is None else value,
            target_offset_mm=42.4,
            correction_axis_base=(0., 1., 0.),
            max_step_mm=3.,
            expected_calibration_sha256=CALIBRATION,
            after_image_time=9.,
        ) | {"attempt": 2, "cumulative_correction_mm": 4.1, "converged": False}

    def test_writes_atomic_overlay_with_observed_edges_and_correction_arrow(self):
        with tempfile.TemporaryDirectory() as folder:
            result = write_horizontal_overlay(frame(), self.measurement(), Path(folder))
            image_path = Path(folder) / "d405-horizontal.jpg"
            meta_path = Path(folder) / "d405-horizontal.json"
            self.assertEqual(result["status"], "ready")
            self.assertTrue(image_path.is_file())
            meta = json.loads(meta_path.read_text())
            self.assertEqual(meta["frame_id"], "frame-1")
            self.assertEqual(meta["calibration_sha256"], CALIBRATION)
            self.assertEqual(meta["rim"]["status"], "visible")
            self.assertEqual(meta["battery_front"]["status"], "visible")
            self.assertAlmostEqual(meta["raw_error_mm"], 2.4)
            self.assertAlmostEqual(meta["step_mm"], 2.4)
            self.assertEqual(meta["attempt"], 2)
            self.assertAlmostEqual(meta["cumulative_correction_mm"], 4.1)
            image = cv2.imread(str(image_path))
            blue, green, red = cv2.split(image)
            self.assertTrue(np.any((red > 220) & (red > green + 80) & (red > blue + 80)))
            self.assertTrue(np.any((red > 180) & (blue > 180) & (green < 100)))
            self.assertTrue(np.any((red > 180) & (green > 180) & (blue < 100)))
            self.assertFalse(list(Path(folder).glob("*.tmp*")))

    def test_new_generation_atomically_replaces_prior_identity(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            write_horizontal_overlay(frame(), self.measurement(), output)
            newer = frame(); newer["id"] = np.array("frame-2"); newer["image_time"] = np.array(11.); newer["arm_time"] = np.array(11.01); newer["body_time"] = np.array(11.02)
            write_horizontal_overlay(newer, self.measurement(newer), output)
            meta = json.loads((output / "d405-horizontal.json").read_text())
            self.assertEqual(meta["frame_id"], "frame-2")
            self.assertEqual(meta["image_time"], 11.)
            self.assertFalse(list(output.glob("*.tmp*")))

    def test_unavailable_overlay_does_not_invent_hidden_edges(self):
        with tempfile.TemporaryDirectory() as folder:
            unavailable = {"status": "unavailable", "reason": "rim clipped", "frame_id": "frame-1", "image_time": 10., "calibration_sha256": CALIBRATION}
            write_horizontal_overlay(frame(), unavailable, Path(folder))
            meta = json.loads((Path(folder) / "d405-horizontal.json").read_text())
            self.assertEqual(meta["status"], "unavailable")
            self.assertEqual(meta["rim"]["status"], "unavailable")
            self.assertEqual(meta["battery_front"]["status"], "unavailable")
            self.assertNotIn("points", meta["rim"])
            self.assertNotIn("points", meta["battery_front"])


if __name__ == "__main__":
    unittest.main()
