import json
import os
from pathlib import Path
import subprocess
import sys
import socket
import threading
import unittest
from unittest.mock import patch

from agent.runtime import AgentRuntime
from agent.storage import RunStore
from agent.tools import ToolRegistry
from agent.tests.test_runtime import ScriptModel, call
from agent.tests import test_system as fixtures


class RegressionTests(unittest.TestCase):
    setUp = fixtures.SystemTests.setUp
    call = fixtures.SystemTests.call
    prepare_pick = fixtures.SystemTests.prepare_pick

    def arrive(self):
        old = self.prepare_pick()
        self.assertTrue(self.call('execute_pick', plan_id='pick_1')['ok'])
        obs = self.call('observe_scene', cameras=['d435'])['data']['observation_id']
        self.assertTrue(self.call('verify_hold', observation_id=obs)['ok'])
        self.assertTrue(self.call('stow_for_carry')['ok'])
        self.assertTrue(self.call('navigate_to_area')['ok'])
        return old

    def select_slot(self):
        obs = self.call('observe_scene', cameras=['d435'])['data']['observation_id']
        self.assertTrue(self.call('select_slot', observation_id=obs, camera='d435', bbox_xyxy=[1, 1, 10, 10], slot_id='red_slot')['ok'])
        return obs

    def test_wrong_receipt_identity_never_advances_or_clears_pending(self):
        self.prepare_pick()
        real_invoke = self.backend.invoke
        self.backend.invoke = lambda op, args, identity: dict(real_invoke(op, args, identity), execution_id='other_request')
        self.assertFalse(self.call('execute_pick', plan_id='pick_1')['ok'])
        self.assertEqual(self.store.state['mission']['stage'], 'PICK_PLANNED')
        self.assertIsNotNone(self.store.state['pending'])
        self.backend.status = lambda op, identity: {'status': 'rejected', 'execution_id': 'other_request'}
        self.assertFalse(self.call('get_execution_status')['ok'])
        self.assertIsNotNone(self.store.state['pending'])

    def test_conflicting_planner_candidate_or_arm_is_rejected(self):
        self.prepare_pick()
        real_payload = self.backend.payload
        for conflict in ({'candidate_id': 'another_candidate'}, {'arm': 'right'}):
            self.backend.payload = lambda op, args: dict(real_payload(op, args), **conflict)
            self.assertFalse(self.call('plan_pick', candidate_id='grasp_1', arm='left')['ok'])

    def test_pre_action_observations_cannot_verify_hold_select_slot_or_placement(self):
        old = self.prepare_pick()
        self.call('execute_pick', plan_id='pick_1')
        before = len(self.backend.calls)
        self.assertFalse(self.call('verify_hold', observation_id=old)['ok'])
        self.assertEqual(len(self.backend.calls), before)
        obs = self.call('observe_scene', cameras=['d435'])['data']['observation_id']
        self.call('verify_hold', observation_id=obs)
        self.call('stow_for_carry')
        self.call('navigate_to_area')
        self.assertFalse(self.call('select_slot', observation_id=old, camera='d435', bbox_xyxy=[1, 1, 10, 10], slot_id='red_slot')['ok'])
        slot = self.select_slot()
        self.call('align_battery_to_slot', observation_id=slot)
        self.call('insert_battery')
        self.call('release_and_withdraw')
        self.assertFalse(self.call('verify_placement', observation_id=slot)['ok'])
        self.assertEqual(self.store.state['mission']['stage'], 'RELEASED')

    def test_alignment_requires_slot_bound_camera_geometry(self):
        self.arrive()
        slot = self.select_slot()
        unrelated = self.call('observe_scene', cameras=['astra'])['data']['observation_id']
        before = len(self.backend.calls)
        self.assertFalse(self.call('align_battery_to_slot', observation_id=unrelated)['ok'])
        self.assertEqual(len(self.backend.calls), before)
        self.assertTrue(self.call('align_battery_to_slot', observation_id=slot)['ok'])

    def test_crash_after_alignment_dispatch_recovers_original_call_without_replay(self):
        self.arrive()
        slot = self.select_slot()
        arguments = {'observation_id': slot}
        self.backend.next_status = 'unknown'
        self.tools.execute('align_battery_to_slot', arguments, 'crash_alignment')
        del self.store.state['calls']['crash_alignment']
        self.store.state['messages'] = [{'role': 'user', 'content': 'continue'},
                                        {'role': 'assistant', 'content': '', 'tool_calls': [call('align_battery_to_slot', arguments, 'crash_alignment')]}]
        self.store.save()
        before = len(self.backend.calls)
        model = ScriptModel([{'role': 'assistant', 'content': 'recovered'}])
        result = AgentRuntime(self.config, self.store, self.tools, model).run()
        self.assertEqual(result['reason'], 'model_finished')
        self.assertEqual(len(self.backend.calls), before)
        sent = model.requests[0]
        index = next(i for i, message in enumerate(sent) if message.get('tool_calls'))
        self.assertEqual(sent[index + 1]['role'], 'tool')
        self.assertTrue(json.loads(sent[index + 1]['content'])['ok'])

    def test_read_only_policy_is_enforced_and_persists_on_reopen(self):
        self.tools = ToolRegistry(self.config, self.store, self.observations, self.backend,
                                  allowed_tools={'get_system_status', 'observe_scene', 'read_observation'})
        self.assertFalse(self.call('set_task', target_color='red', destination='red_slot')['ok'])
        self.assertEqual(self.store.state['mission']['stage'], 'IDLE')
        path = self.store.path
        self.store.close()
        self.store = RunStore.open(path)
        self.addCleanup(self.store.close)
        self.tools = ToolRegistry(self.config, self.store, self.observations, self.backend)
        self.assertFalse(self.call('set_task', target_color='red', destination='red_slot')['ok'])

    def test_rejected_recovery_receipt_also_caches_original_call_before_clearing_pending(self):
        self.arrive()
        slot = self.select_slot()
        arguments = {'observation_id': slot}
        self.backend.next_status = 'unknown'
        self.tools.execute('align_battery_to_slot', arguments, 'rejected_alignment')
        del self.store.state['calls']['rejected_alignment']
        self.store.state['messages'] = [{'role': 'user', 'content': 'continue'},
                                        {'role': 'assistant', 'content': '', 'tool_calls': [call('align_battery_to_slot', arguments, 'rejected_alignment')]}]
        self.backend.status = lambda op, identity: {'status': 'rejected', 'execution_id': identity}
        before = len(self.backend.calls)
        first = AgentRuntime(self.config, self.store, self.tools, ScriptModel([])).run()
        self.assertEqual(first['reason'], 'backend_rejected')
        AgentRuntime(self.config, self.store, self.tools, ScriptModel([{'role': 'assistant', 'content': 'rejected'}])).run()
        self.assertEqual(len(self.backend.calls), before)

    def test_crashed_writer_releases_os_lock_without_unlinking_another_writer(self):
        path = self.store.path
        self.store.close()
        script = 'from agent.storage import RunStore; import sys; s=RunStore.open(sys.argv[1]); print("locked",flush=True); sys.stdin.read()'
        child = subprocess.Popen([sys.executable, '-c', script, str(path)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), 'locked')
            with self.assertRaises(RuntimeError):
                RunStore.open(path)
            self.assertTrue((path / 'writer.lock').exists())
        finally:
            child.kill()
            child.communicate(timeout=10)
        self.assertTrue((path / 'writer.lock').exists())
        recovered = RunStore.open(path)
        recovered.close()

    def test_two_stale_lock_recoveries_never_create_two_writers(self):
        path = self.store.path
        self.store.close()
        (path / 'writer.lock').write_text(json.dumps({'pid': 999999999, 'host': socket.gethostname(), 'token': 'stale'}))
        second_checked, first_acquired = threading.Event(), threading.Event()
        counter_lock, writers, errors = threading.Lock(), [], []
        checks = [0]

        def old_pid_probe(pid):
            with counter_lock:
                checks[0] += 1
                order = checks[0]
            if order == 1:
                second_checked.wait(3)
            else:
                second_checked.set()
                first_acquired.wait(3)
            return False

        def open_writer():
            try:
                writers.append(RunStore.open(path))
                first_acquired.set()
            except RuntimeError as error:
                errors.append(error)

        with patch('agent.storage.pid_alive', side_effect=old_pid_probe, create=True):
            threads = [threading.Thread(target=open_writer) for _ in range(2)]
            try:
                for thread in threads:
                    thread.start()
                for thread in threads:
                    thread.join(5)
                self.assertEqual(len(writers), 1)
                self.assertEqual(len(errors), 1)
            finally:
                for writer in writers:
                    writer.close()


if __name__ == '__main__':
    unittest.main()
