"""Competition scheduling and ownership with inert modules and fake robot IO."""
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock

from test_control import Controller, FakeAdapter, configured, Rejected
from competition_task import CompetitionTask, STAGES
from competition_modules import GraspModule, QwenRecovery
from battery_task import BatteryTask


class Module:
    def __init__(self, trace, name, status='completed'):
        self.trace, self.name, self.status = trace, name, status
        self.recoveries = {}
        self.can_resume = True
        self.stop_fails = False
        self.on_start = lambda context: None

    def start(self, context):
        self.trace.append(('start', self.name, context['request_id'], context['token']))
        self.on_start(context)
        return dict(status=self.status, reason='fixture failure')

    def inspect(self, context):
        self.trace.append(('inspect', self.name, context['request_id'], context['token']))
        return dict(status=self.status, can_resume=self.can_resume, reason='fixture state')

    def stop(self, context):
        if self.stop_fails: raise ValueError('stop fixture failed')
        self.status = 'stopped'
        return dict(stopped=True)

    def resume(self, context):
        self.trace.append(('resume', self.name, context['request_id'], context['token']))
        self.status = 'completed'
        return dict(status='completed')


class CompetitionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.adapter = FakeAdapter()
        self.controller = Controller(configured(), self.adapter)
        self.controller.update_body(5., [0.]*4, 0., 0., 0.)
        for side in ('left', 'right'):
            self.controller.update_arm(side, [0.]*6, -1., [0.]*6)
        self.trace = []
        self.modules = {key:Module(self.trace, key) for key, _ in STAGES}
        self.recovery = Mock()
        self.recovery.decide.return_value = dict(name='stop', reason='不能确定恢复方式')
        self.task = CompetitionTask(self.controller, self.temp.name, modules=self.modules, recovery=self.recovery)
        self.addCleanup(self.task.close)

    def action(self, action, request=None):
        body = dict(action=action, request_id=request or action+str(time.monotonic_ns()))
        if action != 'stop':
            body.update(expected_epoch=self.controller.command_epoch, environment_clear=True)
        return self.task.action('browser', body)

    def wait(self, status):
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            view = self.task.snapshot()
            if view['status'] == status and (status in ('running','recovering') or not self.task.active['worker'].is_alive()):
                return view
            time.sleep(.01)
        self.fail(str(self.task.snapshot()))

    def test_fixed_sequence_keeps_one_owner_and_normal_path_never_calls_model(self):
        self.assertEqual(self.adapter.calls, [])
        self.action('start')
        view = self.wait('completed')
        starts = [row for row in self.trace if row[0] == 'start']
        self.assertEqual([r[1] for r in starts], [k for k, _ in STAGES])
        self.assertEqual(len({id(r[3]) for r in starts}), 1)
        self.assertTrue(all(t['status']=='completed' for t in view['tasks']))
        self.assertFalse(self.controller.snapshot()['motion_reserved'])
        self.assertFalse(view['control_owned'])
        self.recovery.decide.assert_not_called()
        self.assertEqual(self.adapter.calls, [])

    def test_missing_navigation_is_not_faked_and_releases_control(self):
        self.task.modules.pop('navigate_pick')
        self.action('start')
        view = self.wait('needs_attention')
        self.assertIn('能力未接入', view['reason'])
        self.assertFalse(view['control_owned'])
        self.assertEqual(self.trace, [])
        self.assertEqual(self.adapter.calls, [])

    def test_other_task_ownership_is_not_reported_as_manual_available(self):
        token=self.controller.reserve_motion(self.controller.command_epoch)
        try:
            view=self.task.snapshot()
            self.assertFalse(view['manual_available'])
            self.assertFalse(view['can_start'])
            self.assertFalse(view['control_owned'])
        finally:self.controller.release_motion(token)

    def test_stop_resume_checks_same_request_and_never_restarts_completed_actions(self):
        module = self.modules['navigate_place']
        module.status = 'running'
        self.action('start')
        deadline = time.monotonic()+2
        # Phase selection precedes module dispatch; this test pauses an already started module.
        while not any(row[:2]==('start','navigate_place') for row in self.trace) and time.monotonic()<deadline: time.sleep(.01)
        self.assertTrue(any(row[:2]==('start','navigate_place') for row in self.trace))
        self.action('stop')
        paused = self.wait('paused')
        self.assertEqual([t['status'] for t in paused['tasks'][:3]], ['completed']*3)
        self.assertFalse(paused['control_owned'])
        self.assertFalse(self.controller.snapshot()['motion_reserved'])
        self.action('continue')
        self.wait('completed')
        self.assertEqual([r[1] for r in self.trace if r[0]=='start'], [k for k, _ in STAGES])
        resume = next(r for r in self.trace if r[0]=='resume')
        start = next(r for r in self.trace if r[:2]==('start','navigate_place'))
        self.assertEqual(resume[2], start[2])
        self.assertIsNot(resume[3], start[3])
        self.assertTrue(any(r[:2]==('inspect','navigate_place') for r in self.trace))

    def test_completed_receipt_racing_stop_is_preserved(self):
        self.modules['navigate_pick'].on_start = lambda c:c['cancelled'].set()
        self.action('start')
        view = self.wait('paused')
        self.assertEqual(view['tasks'][0]['status'], 'completed')
        self.action('continue')
        self.wait('completed')
        self.assertEqual(sum(r[:2]==('start','navigate_pick') for r in self.trace), 1)

    def test_unknown_state_after_manual_changes_does_not_replay(self):
        module = self.modules['navigate_pick'];module.status='running'
        self.action('start');self.wait('running');self.action('stop');self.wait('paused')
        module.status='unknown';module.can_resume=False
        self.action('continue')
        self.wait('needs_attention')
        self.assertEqual(sum(r[:2]==('start','navigate_pick') for r in self.trace), 1)
        self.assertFalse(any(r[0]=='resume' for r in self.trace))

    def test_model_selects_only_registered_recovery_then_checks_original_receipt(self):
        original = self.modules['navigate_pick'];original.status='failed'
        recovery = Module(self.trace, 'recover_navigation')
        recovery.on_start=lambda c:setattr(original, 'status', 'completed')
        original.recoveries={'recover_navigation':recovery}
        self.recovery.decide.return_value=dict(name='recover_navigation',reason='已有恢复能力')
        self.action('start');self.wait('completed')
        self.recovery.decide.assert_called_once()
        self.assertIn('recover_navigation', [r[1] for r in self.trace if r[0]=='start'])
        self.assertEqual(sum(r[:2]==('start','navigate_pick') for r in self.trace), 1)
        self.assertFalse(self.controller.snapshot()['motion_reserved'])

    def test_unregistered_model_choice_cannot_dispatch_any_recovery(self):
        self.modules['navigate_pick'].status='failed'
        self.recovery.decide.return_value=dict(name='invented_motion',reason='invalid')
        self.action('start');self.wait('needs_attention')
        self.assertEqual([r[1] for r in self.trace if r[0]=='start'], ['navigate_pick'])
        self.assertFalse(self.controller.snapshot()['motion_reserved'])

    def test_controller_fault_still_allows_read_only_model_analysis(self):
        module=self.modules['navigate_pick'];module.status='failed'
        module.on_start=lambda c:self.controller.interrupt_motion(c['token'])
        self.action('start');self.wait('needs_attention')
        self.recovery.decide.assert_called_once()
        self.assertTrue(self.recovery.decide.call_args.args[0]['robot']['motion_interrupted'])
        self.assertFalse(self.controller.snapshot()['motion_reserved'])

    def test_stop_during_model_call_discards_late_answer(self):
        self.modules['navigate_pick'].status='failed'
        recovery = Module(self.trace,'recover_late')
        self.modules['navigate_pick'].recoveries={'recover_late':recovery}
        entered, release = threading.Event(), threading.Event()
        def decide(*args):
            entered.set();release.wait(3)
            return dict(name='recover_late',reason='late')
        self.recovery.decide.side_effect=decide
        self.action('start');self.assertTrue(entered.wait(2))
        self.action('stop');self.wait('paused')
        self.assertFalse(self.controller.snapshot()['motion_reserved'])
        release.set();time.sleep(.05)
        self.assertFalse(any(r[:2]==('start','recover_late') for r in self.trace))

    def test_failed_stop_is_explicit_and_retry_stop_returns_manual_control(self):
        module=self.modules['navigate_pick'];module.status='failed';module.stop_fails=True
        self.action('start');view=self.wait('needs_attention')
        self.assertTrue(view['control_owned'])
        self.assertIn('控制权待清理',view['reason'])
        module.stop_fails=False
        self.action('stop');view=self.wait('paused')
        self.assertFalse(view['control_owned'])
        self.assertFalse(self.controller.snapshot()['motion_reserved'])

    def test_duplicate_start_single_dispatch_and_shared_browser_stop(self):
        module=self.modules['navigate_pick'];module.status='running'
        body=dict(action='start',request_id='one',expected_epoch=0,environment_clear=True)
        first=self.task.action('A',body);again=self.task.action('A',body)
        self.assertEqual(first['run_id'],again['run_id'])
        with self.assertRaises(Rejected):self.task.action('A',dict(body,action='continue'))
        self.task.action('B',dict(action='stop',request_id='cross-session'))
        self.wait('paused')
        self.assertLessEqual(sum(r[0]=='start' for r in self.trace),1)

    def test_new_controller_instance_does_not_resume_saved_run(self):
        self.action('start');self.wait('completed')
        restored=CompetitionTask(self.controller,self.temp.name)
        self.assertEqual(restored.snapshot()['status'],'idle')
        self.assertEqual(self.adapter.calls,[])

    def test_borrowed_grasp_keeps_parent_token_and_details_until_parent_finishes(self):
        class Flow:
            def run(self):return dict(status='completed')
        task=BatteryTask(Path(self.temp.name)/'grasp',self.controller,lambda *args:Flow())
        self.addCleanup(task.close)
        carry=self.modules['carry'];carry.status='running'
        self.task.register('pick',GraspModule(task))
        self.action('start')
        deadline=time.monotonic()+2
        while self.task.snapshot()['phase']!='carry' and time.monotonic()<deadline:time.sleep(.01)
        self.assertEqual(task.snapshot('viewer')['status'],'completed')
        self.assertIsNotNone(task.result()['run_id'])
        self.assertTrue(self.controller.snapshot()['motion_reserved'])
        self.action('stop');self.wait('paused')
        self.assertFalse(self.controller.snapshot()['motion_reserved'])

    def test_grasp_stop_preserves_gripper_and_never_homes_or_powers_off(self):
        class Flow:
            def __init__(self,store):self.store=store
            def run(self):
                while not self.store.cancelled:time.sleep(.01)
                return dict(status='cancelled')
        battery=BatteryTask(Path(self.temp.name)/'grasp',self.controller,lambda store,*args:Flow(store))
        self.addCleanup(battery.close)
        self.task.register('pick',GraspModule(battery))
        self.action('start')
        deadline=time.monotonic()+2
        while not battery.active and time.monotonic()<deadline:time.sleep(.01)
        self.action('stop');self.wait('paused')
        self.assertFalse(self.controller.snapshot()['motion_reserved'])
        for name, args in self.adapter.calls:
            if name=='arm':self.assertEqual(args[2],-1.)
            self.assertNotEqual(name,'mode')


class RecoveryModelTests(unittest.TestCase):
    def test_qwen_is_only_a_registered_task_selector(self):
        advisor=QwenRecovery(Mock())
        advisor.client=Mock()
        advisor.client.complete.return_value=dict(tool_calls=[dict(function=dict(
            name='select_recovery', arguments='{"name":"resume_extract","reason":"原目标明确"}'))])
        self.assertEqual(advisor.decide({'stage':'pick'},['resume_extract'])['name'],'resume_extract')
        self.assertFalse(advisor.client.complete.call_args.kwargs['parallel_tool_calls'])
        advisor.client.complete.return_value['tool_calls'][0]['function']['arguments']='{"name":"invented","reason":"bad"}'
        with self.assertRaises(Exception):advisor.decide({},['resume_extract'])
