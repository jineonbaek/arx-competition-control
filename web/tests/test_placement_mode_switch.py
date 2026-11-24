"""Switch the real task wiring without issuing a robot command."""
import json
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from http.server import ThreadingHTTPServer

import server
from competition_task import CompetitionTask
from t2_test_task import T2TestTask
from fast_visual_level import FastVisualLevel
from camera_depth_control import DepthControl
from navigation_placement import NavigationPlacement, CurrentPositionPlacement
from t2_timed_placement import TimedT2Placement
from test_control import Controller, FakeAdapter, configured
import test_battery_task_http as helper


class PlacementModeSwitchTests(unittest.TestCase):
    request = staticmethod(helper.BatteryTaskHTTPTests.request)
    session = helper.BatteryTaskHTTPTests.session

    def setUp(self):
        self.assertTrue(hasattr(server, 'PlacementModeSwitch'), 'Runtime placement mode selection is missing')
        temp = tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        (self.root/'config').mkdir()
        self.path = self.root/'config/t2-placement.json'
        self.path.write_text(json.dumps(dict(version=1, mode='lidar_special_point')))
        self.adapter = FakeAdapter()
        self.ctrl = Controller(configured(), self.adapter)
        self.fast = FastVisualLevel(self.ctrl, DepthControl())
        self.fast.placement_mode = 'lidar_special_point'
        self.task = CompetitionTask(self.ctrl, self.root/'runs')
        self.single = T2TestTask(self.ctrl, self.root/'single')
        self.nav = SimpleNamespace()
        self.switch = server.PlacementModeSwitch(self.ctrl, None, self.fast, self.nav,
                                                self.root, self.task, self.single)
        self.http = ThreadingHTTPServer(('127.0.0.1', 0), server.make_handler(
            self.ctrl, fast_level=self.fast, competition_task=self.task,
            t2_test_task=self.single, placement_modes=self.switch))
        self.http.daemon_threads=True; self.http.stopping=threading.Event()
        worker=threading.Thread(target=self.http.serve_forever,daemon=True); worker.start()
        self.addCleanup(worker.join); self.addCleanup(self.http.server_close)
        self.addCleanup(self.http.shutdown); self.addCleanup(self.http.stopping.set)
        self.headers=self.session(self.http)

    def choose(self, mode, headers=None):
        return self.request(self.http, 'POST', '/api/competition-task', json.dumps(dict(
            action='set_placement_mode', mode=mode, request_id='choose-'+mode)),
            self.headers if headers is None else headers)

    def test_chassis_skips_visual_timer_and_selecting_visual_restores_old_chain(self):
        for mode, backend, visual in (
            ('lidar_special_point','lidar_special_point_chassis',False),
            ('visual_seated','lidar_locked_d405_seat_body',True),
            ('lidar_special_point','lidar_special_point_chassis',False)):
            code,_,body=self.choose(mode)
            self.assertEqual(code,202,body)
            view=json.loads(body)
            self.assertEqual(view['placement_mode']['selected'],mode)
            self.assertEqual(view['placement_backend'],backend)
            self.assertEqual(view['insertion_policy']['visual_verification'],visual)
            self.assertEqual('release_after_chassis' in self.task.modules,not visual)
            self.assertEqual('release_after_chassis' in self.single.modules,not visual)
            if visual:
                self.assertIsInstance(self.task.modules['place'],TimedT2Placement)
                self.assertIsInstance(self.single.modules['place'].placement_module,CurrentPositionPlacement)
            else:
                self.assertIsInstance(self.task.modules['place'],NavigationPlacement)
                self.assertNotIsInstance(self.task.modules['place'],TimedT2Placement)
                self.assertFalse(view['insertion_policy']['d405_horizontal_enabled'])
                self.assertFalse(view['insertion_policy']['chassis_recheck_enabled'])
                self.assertEqual(view['insertion_policy']['arm_push_mm'],0.)
                self.assertEqual(view['insertion_policy']['terminal_forward_s'],6.)
                self.assertEqual(view['insertion_policy']['terminal_forward_sdk'],.3)
                self.assertEqual(view['insertion_policy']['completion_basis'],'saved_special_point_plus_6s_chassis')
            self.assertEqual(json.loads(self.path.read_text())['mode'],mode)
            self.assertEqual(self.fast.placement_mode,mode)
            code,_,body=self.request(self.http,'GET','/api/t2-test',headers=self.headers)
            self.assertEqual(json.loads(body)['placement_mode']['selected'],mode)
        self.assertEqual(self.adapter.calls,[])

    def test_active_ownership_rejects_switch_without_changing_file_or_modules(self):
        before=self.task.modules['place']; saved=self.path.read_bytes()
        lease=self.ctrl.reserve_competition(self.ctrl.command_epoch)
        try:
            code,_,body=self.choose('visual_seated')
            self.assertEqual(code,400,body)
            self.assertIs(self.task.modules['place'],before)
            self.assertEqual(self.path.read_bytes(),saved)
            self.assertEqual(self.fast.placement_mode,'lidar_special_point')
        finally:self.ctrl.release_competition(lease)

    def test_worker_cleanup_also_blocks_switch_after_ownership_release(self):
        stop=threading.Event(); worker=threading.Thread(target=stop.wait); worker.start()
        self.task.active=dict(status='needs_attention',worker=worker,competition_token=None)
        try:
            with self.assertRaisesRegex(ValueError,'运行|结束'):
                self.switch.select('visual_seated')
            self.assertEqual(self.fast.placement_mode,'lidar_special_point')
        finally:
            stop.set();worker.join();self.task.active=None

    def test_invalid_or_cross_origin_selection_never_changes_configuration(self):
        saved=self.path.read_bytes()
        self.assertEqual(self.choose('anything')[0],400)
        self.assertEqual(self.choose('visual_seated',dict(self.headers,Origin='http://other'))[0],403)
        self.assertEqual(self.path.read_bytes(),saved)
        self.assertEqual(self.adapter.calls,[])

    def test_failed_persistence_leaves_runtime_selection_intact(self):
        before=self.task.modules['place']
        with patch('placement_mode_switch.atomic_json',side_effect=OSError('disk unavailable')):
            with self.assertRaises(OSError):self.switch.select('visual_seated')
        self.assertIs(self.task.modules['place'],before)
        self.assertEqual(self.fast.placement_mode,'lidar_special_point')

    def test_selection_clears_resumable_old_run_instead_of_mixing_modes(self):
        self.task.active=dict(status='paused',worker=threading.Thread(),competition_token=None)
        self.switch.select('visual_seated')
        self.assertIsNone(self.task.active)
        self.assertFalse(self.task.snapshot()['can_continue'])


if __name__=='__main__': unittest.main()
