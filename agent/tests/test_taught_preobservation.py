import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from agent.taught_placement import (
    load_preobservation_waypoint,
    load_taught_route,
    validate_preobservation_waypoint,
    validate_preobservation_pose,
)


ROOT = Path(__file__).resolve().parents[2]


class TaughtPreobservationTests(unittest.TestCase):
    def setUp(self):
        self.route = load_taught_route(ROOT / "config" / "taught-placement.json")
        self.limits = np.array([
            [-89.9, 89.9], [-5.7, 206.2], [-5.7, 171.8],
            [-73.9, 73.9], [-84.7, 84.7], [-99.6, 99.6],
        ])
        self.joints = np.radians([5., 38., 37., 0., 2., -90.])
        self.tool = np.eye(4)
        self.tool[:3, 3] = [.61, -.21, .53]

    def test_route_v2_has_dedicated_waypoint_and_literal_d405_limits(self):
        route = self.route
        self.assertEqual(route.version, 2)
        self.assertEqual(route.d405_preobservation_waypoint,
                         "reports/manual-waypoints/right-d405-preobserve-current.json")
        self.assertEqual(route.d405_target_offset_mm, 42.4)
        self.assertEqual(route.d405_convergence_mm, .75)
        self.assertEqual(route.d405_step_limit_mm, 3.)
        self.assertEqual(route.d405_total_limit_mm, 15.)
        self.assertEqual(route.d405_attempts, 4)
        self.assertEqual(route.d405_capture_timeout_s, 8.)

    def test_zero_lift_route_and_waypoint_share_the_same_reference_height(self):
        self.assertEqual(self.route.lift_target_rad, 0.)
        waypoint = load_preobservation_waypoint(ROOT/self.route.d405_preobservation_waypoint, 0.)
        self.assertEqual(waypoint['lift_rad'], 0.)
        self.assertEqual(waypoint['source'], 'offline_fk_rebased_to_zero_lift_no_motion')
        with self.assertRaises(ValueError):
            load_preobservation_waypoint(ROOT/self.route.d405_preobservation_waypoint, 7.)

    def test_route_still_rejects_negative_lift_target(self):
        payload = json.loads((ROOT/'config/taught-placement.json').read_text())
        payload['lift_target_rad'] = -1.
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'route.json';path.write_text(json.dumps(payload))
            with self.assertRaises(ValueError):load_taught_route(path)

    def test_load_waypoint_requires_right_arm_closed_gripper_and_current_lift(self):
        payload = dict(version=1, arm="right", joints_rad=self.joints.tolist(),
                       wrist_base_mm_deg=[610., -210., 530., 0., 0., 0.],
                       lift_rad=.34, gripper_command_target=0,
                       captured_at=10., source="stationary-feedback",
                       tool_base_matrix=self.tool.tolist(),
                       source_hashes={"config/right-grip-tcp.json":"a" * 64})
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "waypoint.json"
            path.write_text(json.dumps(payload))
            self.assertEqual(load_preobservation_waypoint(path, .35)["arm"], "right")
            for key, value in (("arm", "left"), ("gripper_command_target", -3.14),
                               ("lift_rad", .8)):
                bad = dict(payload, **{key: value})
                path.write_text(json.dumps(bad))
                with self.assertRaisesRegex(ValueError, "pre-observation waypoint"):
                    load_preobservation_waypoint(path, .35)

    def test_pose_locks_true_x_z_rotation_and_joint_margins(self):
        locked = validate_preobservation_pose(self.tool, self.joints, self.limits, self.route)
        np.testing.assert_allclose(locked["insertion_axis_base"], [1., 0., 0.])
        self.assertEqual(locked["tcp_z_m"], .53)
        np.testing.assert_allclose(locked["rotation_base_from_tool"], np.eye(3))
        self.assertEqual(locked["z_tolerance_mm"], .5)
        self.assertEqual(locked["rotation_tolerance_deg"], .2)
        self.assertGreaterEqual(locked["j5_margin_deg"], 5.)
        self.assertGreaterEqual(locked["elbow_margin_deg"], 10.)

    def test_pose_rejects_axis_rotation_j6_j5_and_elbow_violations(self):
        tilted = self.tool.copy()
        tilted[:3, :3] = Rotation.from_euler("z", .3, degrees=True).as_matrix()
        cases = [
            (tilted, self.joints, "insertion axis"),
            (self.tool, np.r_[self.joints[:5], np.radians(-86.)], "J6"),
            (self.tool, np.r_[self.joints[:4], np.radians(81.), self.joints[5]], "J5"),
            (self.tool, np.r_[self.joints[:2], np.radians(4.), self.joints[3:]], "elbow"),
        ]
        for tool, joints, reason in cases:
            with self.subTest(reason=reason), self.assertRaisesRegex(ValueError, reason):
                validate_preobservation_pose(tool, joints, self.limits, self.route)

    def test_pose_rejects_invalid_rigid_transform_and_joint_limits(self):
        bad = self.tool.copy(); bad[2, 3] = np.nan
        with self.assertRaisesRegex(ValueError, "pre-observation pose"):
            validate_preobservation_pose(bad, self.joints, self.limits, self.route)
        narrow = self.limits.copy(); narrow[0] = [6., 10.]
        with self.assertRaisesRegex(ValueError, "joint limits"):
            validate_preobservation_pose(self.tool, self.joints, narrow, self.route)

    def test_waypoint_fk_must_match_recorded_wrist_before_motion(self):
        wrist = np.eye(4); wrist[:3, 3] = [.61, -.21, .53]
        waypoint = dict(joints_rad=self.joints.tolist(),
                        wrist_base_mm_deg=[610., -210., 530., 0., 0., 0.],
                        tool_base_matrix=wrist.tolist(),
                        source_hashes={"config/right-grip-tcp.json":"a" * 64})
        checked = validate_preobservation_waypoint(
            waypoint, wrist, wrist, self.joints, self.limits, self.route, "a" * 64)
        self.assertEqual(checked["tcp_z_m"], .53)
        stale = dict(waypoint, wrist_base_mm_deg=[611., -210., 530., 0., 0., 0.])
        with self.assertRaisesRegex(ValueError, "recorded wrist"):
            validate_preobservation_waypoint(
                stale, wrist, wrist, self.joints, self.limits, self.route, "a" * 64)
        wrong_tool = dict(waypoint, tool_base_matrix=(wrist + np.diag([0, 0, 0, .001])).tolist())
        with self.assertRaisesRegex(ValueError, "recorded tool"):
            validate_preobservation_waypoint(
                wrong_tool, wrist, wrist, self.joints, self.limits, self.route, "a" * 64)
        with self.assertRaisesRegex(ValueError, "TCP hash"):
            validate_preobservation_waypoint(
                waypoint, wrist, wrist, self.joints, self.limits, self.route, "b" * 64)


if __name__ == "__main__":
    unittest.main()
