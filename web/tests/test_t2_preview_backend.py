import importlib
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest


class T2OnlinePreviewTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.candidate = self.root / 'candidate'
        self.candidate.mkdir()
        self.support = self.root / 'support'
        self.support.mkdir()
        for name in ('frame.npz', 'pose.json', 'control.json'):
            (self.root / name).write_text('{}')
        self.config = self.root / 'preview.json'
        self.settings = dict(mode='recorded_preview', candidate_dir=str(self.candidate),
            workspace=str(self.root), support_dirs=[str(self.support)],
            frame=str(self.root/'frame.npz'), pose=str(self.root/'pose.json'),
            control_config=str(self.root/'control.json'), timeout_s=3.)
        self.write_planner("return {'executable': True, 'insert': {'status': 'kinematic_preview'}}")
        self.task = type('ForbiddenHardware', (), {
            'start_owned': lambda self, token: (_ for _ in ()).throw(AssertionError('hardware'))})()

    def backend(self):
        self.assertIsNotNone(importlib.util.find_spec('t2_preview_backend'),
                             'T2 preview factory is not integrated')
        return importlib.import_module('t2_preview_backend')

    def write_planner(self, body):
        (self.candidate/'recorded_t2_planner.py').write_text(
            "class RecordedT2Planner:\n"
            "    def __init__(self, **kwargs): self.kwargs=kwargs\n"
            "    def __call__(self, context):\n        "+body+"\n")

    def make(self):
        self.config.write_text(json.dumps(self.settings))
        module = self.backend().make_t2_placement(self.task, self.config, self.root/'results')
        self.addCleanup(module.close)
        return module

    def context(self):
        return dict(run_id='competition-test', request_id='competition-test:place',
                    stage='place', token=object(), cancelled=threading.Event())

    def test_missing_config_keeps_existing_placement_module(self):
        module = self.backend().make_t2_placement(self.task, self.config, self.root/'results')
        self.assertIs(module.task, self.task)
        self.assertFalse(getattr(module, 'preview_only', False))

    def test_disabled_config_keeps_existing_placement_module(self):
        self.config.write_text('{"mode":"disabled"}')
        module = self.backend().make_t2_placement(self.task, self.config, self.root/'results')
        self.assertIs(module.task, self.task)

    def test_invalid_enabled_config_cannot_fall_back_to_real_placement(self):
        self.config.write_text('{"mode":"recorded_preview"}')
        with self.assertRaises(ValueError):
            self.backend().make_t2_placement(self.task, self.config, self.root/'results')
        self.config.write_text('{"mode":"typo"}')
        with self.assertRaises(ValueError):
            self.backend().make_t2_placement(self.task, self.config, self.root/'results')

    def test_preview_never_completes_and_records_historical_input_and_identity(self):
        module = self.make()
        self.assertEqual(module.snapshot()['status'], 'idle')
        context = self.context()
        module.start(context)
        self.assertTrue(module.wait(context, 3.))
        result = module.inspect(context)
        self.assertEqual(result['status'], 'failed')
        self.assertFalse(result['battery_placed'])
        self.assertTrue(result['preview_only'])
        self.assertEqual(result['input_source'], 'recorded')
        receipt = json.loads(Path(result['result_path']).read_text())
        self.assertEqual(receipt['competition_run_id'], 'competition-test')
        self.assertEqual(receipt['request_id'], 'competition-test:place')
        self.assertEqual(receipt['inputs']['frame'], str(self.root/'frame.npz'))
        self.assertFalse(receipt['battery_placed'])
        self.assertFalse(receipt['executable'])
        self.assertEqual(module.snapshot()['status'], 'failed')
        self.assertEqual(module.inspect(context | {'token': object()})['status'], 'unknown')
        with self.assertRaises(ValueError): module.start(context)

    def test_child_failure_is_incomplete_and_has_diagnostic_receipt(self):
        self.write_planner("raise ValueError('fixture perception unavailable')")
        module = self.make(); context = self.context()
        module.start(context); self.assertTrue(module.wait(context, 3.))
        result = module.inspect(context)
        self.assertEqual(result['status'], 'failed')
        self.assertIn('fixture perception unavailable', result['reason'])
        receipt = json.loads(Path(result['result_path']).read_text())
        self.assertEqual(receipt['status'], 'failed')
        self.assertFalse(receipt['battery_placed'])

    def test_timeout_does_not_leave_child_running_or_report_completion(self):
        self.write_planner("__import__('time').sleep(20); return {}")
        self.settings['timeout_s'] = .2
        module = self.make(); context = self.context()
        started = time.monotonic(); module.start(context)
        self.assertTrue(module.wait(context, 3.))
        self.assertLess(time.monotonic()-started, 2.)
        self.assertEqual(module.inspect(context)['status'], 'failed')
        self.assertIn('超时', module.inspect(context)['reason'])

    def test_stop_cancels_child_without_waiting_for_full_planning_timeout(self):
        self.write_planner("__import__('time').sleep(20); return {}")
        module = self.make(); context = self.context()
        module.start(context)
        time.sleep(.1)
        self.assertTrue(module.stop(context)['stopped'])
        self.assertTrue(module.wait(context, .5))
        self.assertEqual(module.inspect(context)['status'], 'stopped')


if __name__ == '__main__':
    unittest.main()
