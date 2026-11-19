from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "web" / "src")]

from agent.taught_placement import load_taught_route
from fast_visual_level import (
    FastVisualLevel,
    d405_horizontal_correction_target,
    validate_lateral_endpoint,
)


class D405HorizontalPureTests(unittest.TestCase):
    def setUp(self):
        self.route = load_taught_route(ROOT / "config" / "taught-placement.json")
        self.tool = np.eye(4); self.tool[:3, 3] = [.60, -.20, .53]
        self.q = np.radians([5., 38., 37., 0., 2., -90.])

    def test_target_translates_only_base_y(self):
        measurement = dict(status="visual_guidance_only", step_mm=2.5,
                           axis_base=[0., 1., 0.])
        target = d405_horizontal_correction_target(self.tool, measurement, limit_mm=3.)
        expected = self.tool.copy(); expected[1, 3] += .0025
        np.testing.assert_allclose(target, expected)
        for bad in (dict(measurement, step_mm=3.1),
                    dict(measurement, axis_base=[1., 0., 0.])):
            with self.assertRaisesRegex(ValueError, "horizontal correction"):
                d405_horizontal_correction_target(self.tool, bad, limit_mm=3.)

    def test_endpoint_rejects_x_z_rotation_axis_and_j6_drift(self):
        good = self.tool.copy(); good[1, 3] += .003
        result = validate_lateral_endpoint(self.tool, good, self.q, self.route)
        self.assertAlmostEqual(result["delta_y_mm"], 3.)
        cases = []
        x = good.copy(); x[0, 3] += .0006; cases.append(x)
        z = good.copy(); z[2, 3] += .0006; cases.append(z)
        r = good.copy(); angle = np.radians(.3); r[:3, :3] = [[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]; cases.append(r)
        for end in cases:
            with self.assertRaisesRegex(ValueError, "lateral endpoint"):
                validate_lateral_endpoint(self.tool, end, self.q, self.route)
        bad_q = self.q.copy(); bad_q[5] = np.radians(-86.)
        with self.assertRaisesRegex(ValueError, "J6"):
            validate_lateral_endpoint(self.tool, good, bad_q, self.route)


class FakeModel:
    def __init__(self, state): self.state = state
    def wrist_transform(self, q, height):
        tool = np.eye(4); tool[1, 3] = float(q[0]); tool[2, 3] = .53
        return tool


class FakeController:
    def __init__(self):
        self.q = np.radians([0., 38., 37., 0., 2., -90.])
        self.models = {"right": FakeModel(self)}
        self.config = {"arms": {"right": {"joint_limits_deg": [[-100., 100.]] * 6}}}
    def snapshot(self): return {"arms": {"right": {"joints": self.q.tolist()}}}


class D405HorizontalFlowTests(unittest.TestCase):
    def make_level(self):
        level = object.__new__(FastVisualLevel)
        level.route = load_taught_route(ROOT / "config" / "taught-placement.json")
        level.ctrl = FakeController(); level.height = .34; level.limits = {}
        level.tcp = np.eye(4); level.state = {}; level.run = Path(tempfile.mkdtemp())
        level.save = lambda **kw: level.state.update(kw)
        level.arm_to = lambda target, **kw: setattr(level.ctrl, "q", np.asarray(target, float))
        return level

    def test_fresh_reobservation_converges_and_records_each_step(self):
        level = self.make_level(); frames = []
        for stamp in (10., 11.):
            frames.append({"id": np.array(f"f-{stamp}"), "image_time": np.array(stamp),
                           "calibration_sha256": np.array("a" * 64)})
        level.capture_pair = lambda timeout_s: (level.run, frames.pop(0))
        observed_after = []
        measurements = iter([
            dict(status="visual_guidance_only", frame_id="f-10", image_time=10., calibration_sha256="a" * 64,
                 axis_base=[0., 1., 0.], target_offset_mm=42.4, raw_error_mm=2., step_mm=2.),
            dict(status="visual_guidance_only", frame_id="f-11", image_time=11., calibration_sha256="a" * 64,
                 axis_base=[0., 1., 0.], target_offset_mm=42.4, raw_error_mm=.2, step_mm=.2),
        ])
        def estimator(frame, **kw):
            observed_after.append(kw["after_image_time"])
            return next(measurements)
        def planner(_planner, model, q, height, tcp, target, limits, **kw):
            end = q.copy(); end[0] = target[1, 3]
            return {"joint_waypoints_rad": [q.tolist(), end.tolist()]}
        start = level.ctrl.models["right"].wrist_transform(level.ctrl.q, level.height)
        with patch("fast_visual_level.estimate_horizontal_alignment", side_effect=estimator), \
             patch("fast_visual_level.continuous_correction", side_effect=planner), \
             patch("fast_visual_level.write_horizontal_overlay") as overlay:
            result = level.auto_d405_horizontal_trim(level.ctrl.models["right"], np.eye(4), start)
        self.assertEqual(observed_after, [None, 10.])
        self.assertTrue(result["converged"]); self.assertFalse(result["degraded"])
        self.assertEqual(result["attempt"], 2); self.assertAlmostEqual(result["cumulative_correction_mm"], 2.)
        self.assertAlmostEqual(level.ctrl.q[0], .002)
        self.assertEqual(overlay.call_count, 2)
        for key in ("d405_attempt", "d405_raw_error_mm", "d405_step_mm", "d405_cumulative_correction_mm", "d405_converged", "d405_degraded"):
            self.assertIn(key, level.state)

    def test_unavailable_degrades_once_and_does_not_move(self):
        level = self.make_level(); start_q = level.ctrl.q.copy(); calls = []
        def unavailable(timeout_s): calls.append(timeout_s); raise ValueError("D405 offline")
        level.capture_pair = unavailable
        start = level.ctrl.models["right"].wrist_transform(level.ctrl.q, level.height)
        result = level.auto_d405_horizontal_trim(level.ctrl.models["right"], np.eye(4), start)
        self.assertEqual(calls, [8.])
        self.assertTrue(result["degraded"]); self.assertFalse(result["converged"])
        self.assertIn("offline", result["reason"])
        np.testing.assert_allclose(level.ctrl.q, start_q)

    def test_cumulative_limit_degrades_before_dispatch(self):
        level = self.make_level(); level.route = types.SimpleNamespace(**(level.route.__dict__ | {"d405_total_limit_mm": 1.}))
        level.capture_pair = lambda timeout_s: (level.run, {"id": np.array("f"), "image_time": np.array(10.), "calibration_sha256": np.array("a" * 64)})
        measurement = dict(status="visual_guidance_only", frame_id="f", image_time=10., calibration_sha256="a" * 64,
                           axis_base=[0., 1., 0.], target_offset_mm=42.4, raw_error_mm=2., step_mm=2.)
        start = level.ctrl.models["right"].wrist_transform(level.ctrl.q, level.height)
        with patch("fast_visual_level.estimate_horizontal_alignment", return_value=measurement), \
             patch("fast_visual_level.write_horizontal_overlay"):
            result = level.auto_d405_horizontal_trim(level.ctrl.models["right"], np.eye(4), start)
        self.assertTrue(result["degraded"])
        self.assertIn("cumulative", result["reason"])
        self.assertAlmostEqual(level.ctrl.q[0], 0.)


if __name__ == "__main__": unittest.main()
