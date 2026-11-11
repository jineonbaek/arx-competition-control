import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from control import Controller
from tcp_calibration import TcpCalibration, solve_pivot
from test_control import FakeAdapter, configured


class PivotSolverTests(unittest.TestCase):
    def test_recovers_known_wrist_to_tip_translation(self):
        tip_wrist_m = np.array([0.031, -0.014, 0.182])
        pivot_base_m = np.array([0.42, -0.18, 0.73])
        rotations = Rotation.from_euler(
            "xyz",
            [[0, 0, 0], [25, 0, 0], [0, 30, 0], [0, 0, 40],
             [-20, 15, 10], [18, -25, 35], [35, 20, -15], [-30, -18, 22]],
            degrees=True,
        ).as_matrix()
        transforms = []
        for matrix in rotations:
            transform = np.eye(4)
            transform[:3, :3] = matrix
            transform[:3, 3] = pivot_base_m - matrix @ tip_wrist_m
            transforms.append(transform)

        result = solve_pivot(transforms)

        np.testing.assert_allclose(result["tcp_offset_mm"], [31, -14, 182], atol=1e-8)
        np.testing.assert_allclose(result["pivot_base_mm"], [420, -180, 730], atol=1e-8)
        self.assertEqual(result["rank"], 6)
        self.assertLess(result["rms_mm"], 1e-8)
        self.assertLess(result["max_error_mm"], 1e-8)
        self.assertEqual(len(result["errors_mm"]), 8)

    def test_rejects_too_few_or_unexcited_or_non_rigid_samples(self):
        with self.assertRaisesRegex(ValueError, "至少记录 6 个"):
            solve_pivot([np.eye(4)] * 5)
        with self.assertRaisesRegex(ValueError, "姿态变化不足"):
            solve_pivot([np.eye(4)] * 6)
        malformed = [np.eye(4) for _ in range(6)]
        malformed[-1][3, 3] = 2
        with self.assertRaisesRegex(ValueError, "齐次变换"):
            solve_pivot(malformed)


class TcpCalibrationSessionTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.adapter = FakeAdapter()
        self.controller = Controller(configured(), self.adapter, lambda: self.now)
        class DeterministicWristModel:
            def wrist_transform(self, joints, lift):
                transform = np.eye(4)
                transform[:3, :3] = Rotation.from_rotvec(np.asarray(joints[:3])).as_matrix()
                transform[:3, 3] = [0.4, 0.1, 0.5 + lift / 100.0]
                return transform
        self.controller.models = {
            "left": DeterministicWristModel(), "right": DeterministicWristModel()
        }
        self.controller.update_body(6.0, [0.0] * 4, 0.0, 0.0, 0.0)
        self.controller.update_arm("left", [0.0] * 6, -1.0, [0.0] * 6)
        self.controller.update_arm("right", [0.0] * 6, -1.0, [0.0] * 6)
        self.temp = tempfile.TemporaryDirectory()
        self.calibration = TcpCalibration(
            self.controller, out=Path(self.temp.name), clock=lambda: 1234.5
        )

    def tearDown(self):
        self.temp.cleanup()

    def test_capture_is_read_only_and_sessions_are_isolated(self):
        with self.assertRaisesRegex(ValueError, "确认工具尖端"):
            self.calibration.action("browser-a", {"op": "capture", "arm": "left"})

        state = self.calibration.action(
            "browser-a", {"op": "capture", "arm": "left", "confirmed": True}
        )

        self.assertEqual(state["arms"]["left"]["sample_count"], 1)
        self.assertEqual(state["arms"]["right"]["sample_count"], 0)
        self.assertEqual(self.calibration.state("browser-b")["arms"]["left"]["sample_count"], 0)
        self.assertEqual(self.adapter.calls, [])
        sample = state["arms"]["left"]["samples"][0]
        self.assertEqual(len(sample["joints_rad"]), 6)
        self.assertEqual(np.asarray(sample["wrist_matrix"]).shape, (4, 4))

    def test_stale_feedback_and_motion_are_not_recorded(self):
        self.now += 2.0
        with self.assertRaisesRegex(ValueError, "反馈缺失或已过期"):
            self.calibration.action(
                "browser-a", {"op": "capture", "arm": "right", "confirmed": True}
            )
        self.assertEqual(self.calibration.state("browser-a")["arms"]["right"]["sample_count"], 0)

    def test_candidate_contains_raw_samples_and_solution(self):
        session = self.calibration._session("browser-a")
        tip = np.array([0.02, 0.01, 0.16])
        pivot = np.array([0.45, 0.08, 0.65])
        for index, matrix in enumerate(Rotation.from_euler(
            "xyz", [[0, 0, 0], [20, 0, 0], [0, 25, 0], [0, 0, 30],
                    [-20, 10, 15], [15, -25, 35]], degrees=True
        ).as_matrix()):
            transform = np.eye(4)
            transform[:3, :3] = matrix
            transform[:3, 3] = pivot - matrix @ tip
            session["left"]["samples"].append({
                "index": index + 1,
                "captured_at": 1000 + index,
                "joints_rad": [0.0] * 6,
                "lift_raw": 6.0,
                "wrist_matrix": transform.tolist(),
                "wrist_pose": [0.0] * 6,
            })

        solved = self.calibration.action("browser-a", {"op": "solve", "arm": "left"})
        saved = self.calibration.action(
            "browser-a", {"op": "save", "arm": "left", "label": "left pen"}
        )

        self.assertEqual(solved["arms"]["left"]["solution"]["tcp_offset_mm"], [20.0, 10.0, 160.0])
        path = Path(saved["arms"]["left"]["candidate_path"])
        self.assertTrue(path.is_file())
        payload = json.loads(path.read_text())
        self.assertEqual(payload["arm"], "left")
        self.assertEqual(payload["label"], "left pen")
        self.assertEqual(len(payload["samples"]), 6)
        self.assertEqual(payload["solution"]["tcp_offset_mm"], [20.0, 10.0, 160.0])


if __name__ == "__main__":
    unittest.main()
