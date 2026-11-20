from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "web" / "src"))

from placement_reference import PlacementReferenceController


class PlacementReferenceControllerTests(unittest.TestCase):
    def test_record_partial_capture_becomes_active_and_returns_comparison(self):
        with tempfile.TemporaryDirectory() as folder:
            calls = []

            def capture(include_artifacts):
                calls.append(include_artifacts)
                value = {
                    "navigation_pose": {"x": 1.0},
                    "right_joints_rad": [0.1] * 6,
                    "d435": {"valid": True, "image_file": "d435.jpg"},
                    "residual": {"forward_m": 0.01},
                }
                return value, ({"d435.jpg": b"jpeg"} if include_artifacts else {})

            controller = PlacementReferenceController(Path(folder), capture, clock=lambda: 100.0)
            saved = controller.action({"op": "record_current"})
            self.assertEqual(saved["status"], "partial")
            self.assertEqual(controller.snapshot()["reference_id"], saved["reference_id"])
            self.assertEqual(calls, [True, False])

    def test_select_switches_pointer_without_overwriting_records(self):
        with tempfile.TemporaryDirectory() as folder:
            counter = iter((100.0, 101.0, 102.0, 103.0, 104.0))

            def capture(include_artifacts):
                return {"navigation_pose": {}, "right_joints_rad": [0.1] * 6, "d435": {}}, {}

            controller = PlacementReferenceController(Path(folder), capture, clock=lambda: next(counter))
            first = controller.action({"op": "record_current"})
            second = controller.action({"op": "record_current"})
            controller.action({"op": "select", "reference_id": first["reference_id"]})
            self.assertEqual(controller.snapshot()["reference_id"], first["reference_id"])
            self.assertNotEqual(first["reference_id"], second["reference_id"])


if __name__ == "__main__":
    unittest.main()
