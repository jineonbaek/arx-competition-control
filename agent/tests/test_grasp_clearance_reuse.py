"""The real IK already certifies its exact start/end segment; do not repeat it."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np

from agent.grasp_path import plan_insert, plan_correction
from web.src.handeye_column import ColumnModel


class LegacyAdapter:
    """Existing adapters exposing only IK and a separate geometry check."""
    def __init__(self, model):
        self.reach = model.reach
        self.inverse_wrist = model.inverse_wrist
        self.check = model.check
        self.wrist_transform = model.wrist_transform


class ClearanceReuseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ColumnModel()
        root = Path(__file__).resolve().parents[2]
        cls.limits = json.loads((root/'web/config/control.json').read_text())['arms']['right']['joint_limits_deg']
        cls.q = np.radians([0, 20, 40, 10, 0, -90])
        cls.tcp = np.eye(4)
        cls.tcp[:3, 3] = [.12, .007, -.004]
        cls.tool = cls.model.wrist_transform(cls.q, 12.) @ cls.tcp
        cls.timing = dict(joint_speed_deg_s=[5.7]*6, joint_acceleration_deg_s2=[12.]*6,
                          tcp_speed_mm_s=20., tcp_acceleration_mm_s2=40.)

    def test_insert_checks_each_exact_segment_once_and_preserves_path(self):
        pre = self.tool[:3, 3]*1000
        args = (self.q, 12., pre, pre+10*self.tool[:3, 0], self.tool[:3, :3], self.tcp, self.limits)
        old = plan_insert(LegacyAdapter(self.model), *args, timing_limits=self.timing)
        with patch.object(self.model, 'check', wraps=self.model.check) as checks:
            new = plan_insert(self.model, *args, timing_limits=self.timing)
        self.assertEqual(new, old)
        self.assertEqual(checks.call_count, len(new['joint_waypoints_rad']))
        points = [self.q.tolist()] + new['joint_waypoints_rad']
        for call, a, b in zip(checks.call_args_list, points, points[1:]):
            np.testing.assert_array_equal(call.args[0], a)
            np.testing.assert_array_equal(call.args[1], b)
            self.assertEqual(call.args[2], 12.)

    def test_correction_checks_each_exact_segment_once_and_preserves_timing(self):
        target = self.tool.copy()
        target[:3, 3] += .006*self.tool[:3, 0]
        args = (self.q, 12., self.tcp, target, self.limits)
        old = plan_correction(LegacyAdapter(self.model), *args, timing_limits=self.timing)
        with patch.object(self.model, 'check', wraps=self.model.check) as checks:
            new = plan_correction(self.model, *args, timing_limits=self.timing)
        self.assertEqual(new, old)
        self.assertEqual(checks.call_count, len(new['joint_waypoints_rad'])-1)


if __name__ == '__main__':
    unittest.main()
