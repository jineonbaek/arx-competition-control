import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "web" / "src"))

from placement_reference_store import (
    comparison_payload,
    load_active_reference,
    save_reference_record,
    select_active_reference,
)


class PlacementReferenceStoreTests(unittest.TestCase):
    def test_partial_capture_is_saved_with_missing_fields(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            record = save_reference_record(
                root,
                {
                    "navigation_pose": {"x": 1.0, "y": 2.0, "yaw": 0.1},
                    "right_joints_rad": [0.1] * 6,
                    "d435": {"valid": True, "image_file": "d435.jpg"},
                },
                now=100.0,
                reference_id="ref-100",
                artifacts={"d435.jpg": b"jpeg"},
            )
            self.assertEqual(record["status"], "partial")
            self.assertNotIn("astra", record["missing"])
            self.assertTrue((root / "records" / "ref-100" / "d435.jpg").is_file())

    def test_reference_records_are_immutable(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            capture = {"navigation_pose": {}, "right_joints_rad": [0.1] * 6}
            save_reference_record(root, capture, now=100.0, reference_id="same")
            with self.assertRaisesRegex(ValueError, "already exists"):
                save_reference_record(root, capture, now=101.0, reference_id="same")

    def test_active_pointer_selects_without_changing_record(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            capture = {"navigation_pose": {}, "right_joints_rad": [0.1] * 6, "d435": {}}
            original = save_reference_record(root, capture, now=100.0, reference_id="ref-a")
            select_active_reference(root, "ref-a", now=101.0)
            loaded = load_active_reference(root)
            self.assertEqual(loaded["reference_id"], "ref-a")
            self.assertEqual(json.loads((root / "records" / "ref-a" / "record.json").read_text()), original)

    def test_comparison_payload_marks_unavailable_images(self):
        reference = {
            "reference_id": "ref-a",
            "capture": {"d435": {"image_file": "d435.jpg"}},
        }
        current = {"d435": None, "residual": {"forward_m": 0.01}}
        result = comparison_payload(reference, current)
        self.assertEqual(result["reference_id"], "ref-a")
        self.assertEqual(set(result["images"]), {"reference_d435", "current_d435"})
        self.assertEqual(result["images"]["current_d435"]["status"], "unavailable")
        self.assertEqual(result["residual"]["forward_m"], 0.01)


if __name__ == "__main__":
    unittest.main()
