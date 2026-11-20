import json
import math
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from agent.visual_chassis_correction import (
    compare_observations,
    finish_visual_chassis_workflow,
    load_chassis_policy,
    load_locked_handoff,
    plan_chassis_step,
    run_visual_chassis_workflow,
)


def policy():
    return {
        "version": 1,
        "observation_max_age_s": 1.0,
        "admission_forward_m": 0.020,
        "admission_lateral_m": 0.020,
        "admission_yaw_deg": 2.0,
        "max_translation_step_m": 0.050,
        "max_yaw_step_deg": 2.0,
        "max_total_translation_m": 0.30,
        "max_total_yaw_deg": 15.0,
        "handoff_max_age_s": 2.0,
        "max_correction_cycles": 8,
    }


def observation(captured_at=99.5):
    return {
        "d435": {
            "valid": True,
            "captured_at": captured_at,
            "center_base_m": [0.65, -0.19, 0.61],
            "inside_axis_base": [1.0, 0.0, 0.0],
        },
    }


class VisualChassisCorrectionTests(unittest.TestCase):
    def test_d435_metric_difference_drives_correction(self):
        reference = observation()
        current = observation()
        current["d435"] = current["d435"] | {
            "center_base_m": [0.70, -0.16, 0.61],
            "inside_axis_base": [math.cos(math.radians(3)), math.sin(math.radians(3)), 0.0],
        }
        result = compare_observations(reference, current, policy(), now=100.0)
        self.assertEqual(result["source"], "d435")
        self.assertAlmostEqual(result["forward_m"], 0.05)
        self.assertAlmostEqual(result["lateral_m"], 0.03)
        self.assertAlmostEqual(result["yaw_deg"], 3.0, places=5)

    def test_invalid_d435_never_falls_back_to_astra(self):
        reference = observation()
        reference["astra"] = {
            "valid": True, "captured_at": 99.5, "center_px": [320.0, 240.0],
            "yaw_deg": 0.0, "meters_per_pixel": [0.0005, 0.0005],
        }
        current = observation()
        current["d435"]["valid"] = False
        current["astra"] = reference["astra"] | {"center_px": [340.0, 230.0], "yaw_deg": -2.0}
        with self.assertRaisesRegex(ValueError, "D435"):
            compare_observations(reference, current, policy(), now=100.0)

    def test_step_is_bounded_and_admission_locks_chassis(self):
        step = plan_chassis_step(
            {"forward_m": 0.08, "lateral_m": 0.06, "yaw_deg": 4.0, "source": "d435", "coarse_only": False},
            policy(),
        )
        self.assertAlmostEqual(math.hypot(step["forward_m"], step["lateral_m"]), 0.05)
        self.assertEqual(step["yaw_deg"], 2.0)
        self.assertEqual(step["action"], "correct_chassis")
        admitted = plan_chassis_step(
            {"forward_m": 0.01, "lateral_m": -0.01, "yaw_deg": 1.0, "source": "d435", "coarse_only": False},
            policy(),
        )
        self.assertEqual(admitted["action"], "lock_admitted")
        self.assertTrue(admitted["admitted"])

    def test_non_d435_residual_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "D435"):
            plan_chassis_step(
                {"forward_m": 0.0, "lateral_m": 0.0, "yaw_deg": 0.0, "source": "astra", "coarse_only": True},
                policy(),
            )

    def test_critical_failure_finishes_degraded_instead_of_blocking(self):
        receipt = finish_visual_chassis_workflow(
            admitted=False,
            reason="no valid stopped-pose observation",
            chassis_pose={"x": 1.0, "y": 2.0, "yaw": 0.1},
            now=100.0,
        )
        self.assertEqual(receipt["status"], "complete_degraded")
        self.assertEqual(receipt["next_phase"], "complete")
        self.assertTrue(receipt["insertion_skipped"])

    def test_locked_handoff_must_be_fresh_and_admitted(self):
        ready = finish_visual_chassis_workflow(
            admitted=True,
            reason="visual chassis correction admitted",
            chassis_pose={"x": 1.0, "y": 2.0, "yaw": 0.1},
            now=100.0,
        )
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "handoff.json"
            path.write_text(json.dumps(ready))
            self.assertTrue(load_locked_handoff(path, now=101.0, max_age_s=2.0)["admission_valid"])
            with self.assertRaisesRegex(ValueError, "handoff"):
                load_locked_handoff(path, now=103.0, max_age_s=2.0)
            ready["admission_valid"] = False
            path.write_text(json.dumps(ready))
            with self.assertRaisesRegex(ValueError, "handoff"):
                load_locked_handoff(path, now=101.0, max_age_s=2.0)

    def test_saved_policy_loads(self):
        loaded = load_chassis_policy(ROOT / "config" / "visual-chassis-placement.json")
        self.assertEqual(loaded["admission_forward_m"], 0.020)
        self.assertEqual(loaded["handoff_max_age_s"], 2.0)

    def test_executor_corrects_stops_and_writes_admitted_handoff(self):
        reference = observation()
        observations = iter((
            observation() | {"d435": observation()["d435"] | {"center_base_m": [0.70, -0.19, 0.61]}},
            observation(),
        ))
        commands = []
        receipts = []
        result = run_visual_chassis_workflow(
            reference,
            policy(),
            observe=lambda: next(observations),
            command_chassis=commands.append,
            stop_chassis=lambda: {"stopped": True, "pose": {"x": 1.0, "y": 2.0, "yaw": 0.1}},
            write_handoff=receipts.append,
            clock=lambda: 100.0,
        )
        self.assertEqual(len(commands), 1)
        self.assertTrue(result["admission_valid"])
        self.assertEqual(receipts, [result])

    def test_executor_converts_observation_failure_to_terminal_degraded(self):
        receipts = []
        result = run_visual_chassis_workflow(
            observation(),
            policy(),
            observe=lambda: (_ for _ in ()).throw(ValueError("camera unavailable")),
            command_chassis=lambda command: self.fail("must not move"),
            stop_chassis=lambda: {"stopped": True, "pose": {}},
            write_handoff=receipts.append,
            clock=lambda: 100.0,
        )
        self.assertEqual(result["status"], "complete_degraded")
        self.assertIn("camera unavailable", result["reason"])
        self.assertEqual(receipts, [result])


if __name__ == "__main__":
    unittest.main()
