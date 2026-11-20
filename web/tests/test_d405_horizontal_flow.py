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
    require_loaded_gripper,
    require_gripper_retained,
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
        y_over = self.tool.copy(); y_over[1, 3] += .004
        with self.assertRaisesRegex(ValueError, "authorized Y"):
            validate_lateral_endpoint(self.tool, y_over, self.q, self.route,
                                      y_bounds_mm=(0., 3.))

    def test_gripper_feedback_is_locked_until_explicit_release(self):
        self.assertTrue(require_loaded_gripper(-1.30))
        with self.assertRaisesRegex(ValueError, "Loaded battery clamp"):
            require_loaded_gripper(0.)
        self.assertTrue(require_gripper_retained(-1.12, -1.30, tolerance=.25))
        with self.assertRaisesRegex(ValueError, "Gripper feedback changed"):
            require_gripper_retained(-.9, -1.30, tolerance=.25)
        with self.assertRaisesRegex(ValueError, "Loaded battery clamp"):
            require_gripper_retained(-.05, -.26, tolerance=.25)


class FakeModel:
    def __init__(self, state): self.state = state; self.reach = .6
    def wrist_transform(self, q, height):
        tool = np.eye(4); tool[0, 3] = float(q[1]); tool[1, 3] = float(q[0]); tool[2, 3] = .53
        return tool
    def check(self, a, b, height): return {"clear": True}


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
        level.battery = {"bounds_mm":[[-46., -37.5, 0.], [36., 37.5, 195.]]}
        level.contact = np.array([0., 0., 169.])
        level.save = lambda **kw: level.state.update(kw)
        level.arm_to = lambda target, **kw: setattr(level.ctrl, "q", np.asarray(target, float))
        def execute(path, model, tcp, preobservation, **kwargs):
            level.ctrl.q = np.asarray(path["joint_waypoints_rad"][-1], float)
        level.execute_d405_lateral_path = execute
        return level

    def test_fresh_reobservation_converges_and_records_each_step(self):
        level = self.make_level(); frames = []
        for stamp in (10., 11.):
            frames.append({"id": np.array(f"f-{stamp}"), "image_time": np.array(stamp),
                           "calibration_sha256": np.array("a" * 64),
                           "joints":level.ctrl.q.copy(),"lift_rad":np.array(level.height)})
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
            return {"joint_waypoints_rad": [q.tolist(), end.tolist()],
                    "timing":{"time_from_start_s":[0., 1.]}}
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
        level.capture_pair = lambda timeout_s: (level.run, {"id": np.array("f"), "image_time": np.array(10.), "calibration_sha256": np.array("a" * 64),
            "joints":level.ctrl.q.copy(),"lift_rad":np.array(level.height)})
        measurement = dict(status="visual_guidance_only", frame_id="f", image_time=10., calibration_sha256="a" * 64,
                           axis_base=[0., 1., 0.], target_offset_mm=42.4, raw_error_mm=2., step_mm=2.)
        start = level.ctrl.models["right"].wrist_transform(level.ctrl.q, level.height)
        with patch("fast_visual_level.estimate_horizontal_alignment", return_value=measurement), \
             patch("fast_visual_level.write_horizontal_overlay"):
            result = level.auto_d405_horizontal_trim(level.ctrl.models["right"], np.eye(4), start)
        self.assertTrue(result["degraded"])
        self.assertIn("cumulative", result["reason"])
        self.assertAlmostEqual(level.ctrl.q[0], 0.)

    def test_planned_path_is_prevalidated_before_any_dispatch(self):
        level = self.make_level(); dispatched = []
        level.motion = types.SimpleNamespace(arm=lambda *args, **kwargs: dispatched.append(args))
        level.ctrl.adapter = types.SimpleNamespace(trajectory_status=lambda side: {
            "driver_instance":"driver", "command_epoch":1})
        level.driver = "driver"; level.guard = lambda: True; level.reached_route = []
        start = level.ctrl.models["right"].wrist_transform(level.ctrl.q, level.height)
        points = [level.ctrl.q.copy(), level.ctrl.q.copy(), level.ctrl.q.copy()]
        points[1][1] = .001
        path = {"joint_waypoints_rad":[p.tolist() for p in points],
                "timing":{"time_from_start_s":[0., .5, 1.]}}
        with self.assertRaisesRegex(ValueError, "lateral endpoint"):
            FastVisualLevel.execute_d405_lateral_path(
                level, path, level.ctrl.models["right"], np.eye(4), start)
        self.assertEqual(dispatched, [])

    def test_joint_interpolation_is_sampled_not_only_its_endpoints(self):
        level = self.make_level(); dispatched = []
        class CurvedModel(FakeModel):
            def wrist_transform(self, q, height):
                tool = super().wrist_transform(q, height)
                tool[0, 3] += .001 * np.sin(np.pi * float(q[0]) / .002)
                return tool
        model = CurvedModel(level.ctrl)
        level.motion = types.SimpleNamespace(arm=lambda *args, **kwargs: dispatched.append(args))
        level.ctrl.adapter = types.SimpleNamespace(trajectory_status=lambda side: {
            "driver_instance":"driver", "command_epoch":1})
        level.driver = "driver"; level.guard = lambda: True; level.reached_route = []
        start_q = level.ctrl.q.copy(); start_q[0] = 0.; level.ctrl.q = start_q
        end_q = start_q.copy(); end_q[0] = .002
        start = model.wrist_transform(start_q, level.height)
        path = {"joint_waypoints_rad":[start_q.tolist(), end_q.tolist()],
                "timing":{"time_from_start_s":[0., 1.]}}
        with self.assertRaisesRegex(ValueError, "lateral endpoint"):
            FastVisualLevel.execute_d405_lateral_path(
                level, path, model, np.eye(4), start)
        self.assertEqual(dispatched, [])

    def test_joint_interpolation_cannot_overshoot_authorized_y(self):
        level = self.make_level(); dispatched = []
        class YCurvedModel(FakeModel):
            def wrist_transform(self, q, height):
                tool = super().wrist_transform(q, height)
                tool[1, 3] += .002 * np.sin(np.pi * float(q[0]) / .002)
                return tool
        model = YCurvedModel(level.ctrl)
        level.motion = types.SimpleNamespace(arm=lambda *args, **kwargs: dispatched.append(args))
        level.ctrl.adapter = types.SimpleNamespace(trajectory_status=lambda side: {
            "driver_instance":"driver", "command_epoch":1})
        level.driver = "driver"; level.guard = lambda: True; level.reached_route = []
        start_q = level.ctrl.q.copy(); start_q[0] = 0.; level.ctrl.q = start_q
        end_q = start_q.copy(); end_q[0] = .002
        start = model.wrist_transform(start_q, level.height)
        path = {"joint_waypoints_rad":[start_q.tolist(), end_q.tolist()],
                "timing":{"time_from_start_s":[0., 1.]}}
        with self.assertRaisesRegex(ValueError, "authorized Y"):
            FastVisualLevel.execute_d405_lateral_path(
                level, path, model, np.eye(4), start, y_start_mm=0., y_end_mm=2.)
        self.assertEqual(dispatched, [])

    def test_segment_endpoints_also_reserve_analytic_gap_margin(self):
        level = self.make_level(); dispatched = []
        base_q = level.ctrl.q.copy()
        class NearLimitModel(FakeModel):
            def wrist_transform(self, q, height):
                tool = super().wrist_transform(q, height)
                tool[0, 3] += .49 * (float(q[2]) - float(base_q[2]))
                return tool
        model = NearLimitModel(level.ctrl)
        level.motion = types.SimpleNamespace(arm=lambda *args, **kwargs: dispatched.append(args))
        level.ctrl.adapter = types.SimpleNamespace(trajectory_status=lambda side: {
            "driver_instance":"driver", "command_epoch":1})
        level.driver = "driver"; level.guard = lambda: True; level.reached_route = []
        end_q = base_q.copy(); end_q[2] += .001
        start = model.wrist_transform(base_q, level.height)
        path = {"joint_waypoints_rad":[base_q.tolist(), end_q.tolist()],
                "timing":{"time_from_start_s":[0., 1.]}}
        with self.assertRaisesRegex(ValueError, "lateral endpoint"):
            FastVisualLevel.execute_d405_lateral_path(
                level, path, model, np.eye(4), start)
        self.assertEqual(dispatched, [])


if __name__ == "__main__": unittest.main()
