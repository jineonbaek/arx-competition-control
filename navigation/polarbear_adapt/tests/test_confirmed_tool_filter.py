"""User-confirmed 2026-11-14 right claw/wiring returns, with bounded height."""
import json
import os
import tempfile
import unittest
from pathlib import Path
import numpy as np
import chassis_filter


class ConfirmedToolFilter(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.model = root/'mesh.npz'
        np.savez(self.model, rigid=np.array([[0., 0., .1], [.005, 0., .1]]), pitch=.005)
        cfg = root/'disabled.json'
        cfg.write_text(json.dumps({'enabled': False}))
        self.before = chassis_filter.BASE_ENVELOPE_CONFIG
        self.before_flag = os.environ.get('ARX_CONFIRMED_RIGHT_TOOL_RETURNS')
        os.environ['ARX_CONFIRMED_RIGHT_TOOL_RETURNS'] = '1'
        chassis_filter.BASE_ENVELOPE_CONFIG = cfg
        self.filter = chassis_filter.ChassisFilter(self.model)

    def tearDown(self):
        chassis_filter.BASE_ENVELOPE_CONFIG = self.before
        if self.before_flag is None:
            os.environ.pop('ARX_CONFIRMED_RIGHT_TOOL_RETURNS', None)
        else:
            os.environ['ARX_CONFIRMED_RIGHT_TOOL_RETURNS'] = self.before_flag
        self.tmp.cleanup()

    def test_confirmed_measured_right_tool_points_are_filtered(self):
        observed = np.array([[.334351, -.374758, .488014],
                             [.391970, -.382364, .612684],
                             [.321074, -.451419, .531041]])
        self.assertTrue(self.filter.contains(observed).all())

    def test_nearby_foreign_points_outside_bounded_signature_are_retained(self):
        points = np.array([[.42, -.40, .55], [.30, -.40, .55], [.36, -.475, .55],
                           [.36, -.40, .72], [.36, -.40, .34]])
        self.assertFalse(self.filter.contains(points).any())

    def test_nominal_gripper_column_still_applies_at_other_heights(self):
        points = np.array([[.374010, -.2472173, z] for z in (.05, .8, 1.5)])
        self.assertTrue(self.filter.contains(points).all())

    def test_nonfinite_points_are_not_classified_as_confirmed_tool_material(self):
        points = np.array([[.36, -.40, float('nan')], [float('inf'), -.40, .55]])
        self.assertFalse(self.filter.contains(points).any())

    def test_unconfirmed_future_session_does_not_enable_the_measured_band(self):
        os.environ.pop('ARX_CONFIRMED_RIGHT_TOOL_RETURNS', None)
        future = chassis_filter.ChassisFilter(self.model)
        self.assertFalse(future.contains(np.array([[.334351, -.374758, .488014]])).any())


if __name__ == '__main__':
    unittest.main()
