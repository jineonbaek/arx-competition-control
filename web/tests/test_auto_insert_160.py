import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "web" / "src")]

from fast_visual_level import load_fixed_entry, required_cameras, requires_three_rad


class AutoInsert160Tests(unittest.TestCase):
    def test_mode_uses_no_mandatory_camera_or_three_rad_lift(self):
        self.assertEqual(required_cameras("auto_insert_160"), ())
        self.assertFalse(requires_three_rad("auto_insert_160", 0.34))

    def test_fixed_entry_requires_right_arm_and_six_finite_joints(self):
        payload = {
            "arm": "right",
            "joints_rad": [0.1] * 6,
            "wrist_base_mm_deg": [1.0] * 6,
            "gripper_command_target": 0,
            "lift_rad": 0.34,
        }
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "entry.json"
            path.write_text(json.dumps(payload))
            loaded = load_fixed_entry(path, 0.35)
            self.assertEqual(loaded["joints_rad"], payload["joints_rad"])
            payload["arm"] = "left"
            path.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "fixed entry"):
                load_fixed_entry(path, 0.35)

    def test_taught_route_is_160_mm_and_points_to_fixed_entry(self):
        route = json.loads((ROOT / "config" / "taught-placement.json").read_text())
        self.assertEqual(route["version"], 2)
        self.assertEqual(route["full_depth_mm"], 160.0)
        self.assertEqual(
            route["fixed_entry_waypoint"],
            "reports/manual-waypoints/right-return-20261119-154755.json",
        )
        self.assertEqual(
            route["d405_preobservation_waypoint"],
            "reports/manual-waypoints/right-d405-preobserve-current.json",
        )
        self.assertEqual(route["d405_target_offset_mm"], 42.4)
        self.assertEqual(route["d405_convergence_mm"], 0.75)
        self.assertEqual(route["d405_step_limit_mm"], 3.0)
        self.assertEqual(route["d405_total_limit_mm"], 15.0)
        self.assertEqual(route["d405_attempts"], 4)
        self.assertEqual(route["d405_capture_timeout_s"], 8.0)


if __name__ == "__main__":
    unittest.main()
