"""Saved right-arm point shared by the console and fast competition."""
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

from agent import fast_pick
from battery_task import BatteryTask
import test_competition_task as fixtures


class FastPregraspPointTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'point.json'
        p=patch.object(fast_pick,'FAST_PREGRASP_PATH',self.path);p.start();self.addCleanup(p.stop)
        self.f=fixtures.CompetitionTests();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.battery=BatteryTask(Path(self.temp.name)/'runs',self.f.controller,lambda *a:None)
        self.addCleanup(self.battery.close)

    def test_record_actual_feedback_persists_and_fast_competition_uses_it(self):
        self.assertEqual(fast_pick.load_pregrasp(),list(fast_pick.FAST_PREGRASP_JOINTS_DEG))
        actual=[15.,32.,45.,-13.,16.,-90.]
        self.f.controller.update_arm('right',list(np.radians(actual)),-1.,[0.]*6)
        request=dict(action='record_fast_pregrasp',request_id='record')
        result=self.battery.action('console',request)
        saved=result['settings']['fast_pregrasp_joints_deg']
        np.testing.assert_allclose(saved,actual)
        self.assertEqual(self.f.adapter.calls,[])
        restored=BatteryTask(Path(self.temp.name)/'runs',self.f.controller,lambda *a:None)
        self.addCleanup(restored.close)
        self.assertEqual(restored.snapshot('other')['settings']['fast_pregrasp_joints_deg'],saved)
        self.battery.action('console',request)  # Idempotent record retry.
        self.f.task.action('browser',dict(action='start',request_id='fast',mode='fast',
            expected_epoch=self.f.controller.command_epoch,environment_clear=True))
        self.f.wait('completed')
        commands=[c for c in self.f.adapter.calls if c[0]=='arm']
        np.testing.assert_allclose(commands[1][1][1],np.radians(saved))
        self.assertEqual(self.f.controller.snapshot()['right_avoidance_preset']['tolerance_deg'],2.)

    def test_record_does_not_accept_browser_angles_and_invalid_feedback_preserves_file(self):
        self.path.write_text('[15,32,45,-13,16,-90]')
        before=self.path.read_bytes()
        with self.assertRaises(ValueError):
            self.battery.action('console',dict(action='record_fast_pregrasp',request_id='bad',joints=[0]*6))
        with patch.object(self.f.controller,'snapshot',return_value={'arms':{'right':{'fresh':False}}}):
            with self.assertRaises(ValueError):
                self.battery.action('console',dict(action='record_fast_pregrasp',request_id='stale'))
        for values in ([0]*5,[False]*6,[float('nan')]*6):
            with self.assertRaises(ValueError):fast_pick.pregrasp_joints(values)
        self.assertEqual(self.path.read_bytes(),before)

    def test_saved_fast_point_does_not_change_stable_competition(self):
        self.path.write_text('[15,32,45,-13,16,-90]')
        self.f.test_fixed_sequence_keeps_one_owner_and_normal_path_never_calls_model()
