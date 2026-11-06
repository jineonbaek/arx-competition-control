import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agent.config import Config
from agent.runtime import AgentRuntime
from agent.storage import RunStore
from agent.tools import ToolRegistry
from agent.tests.test_runtime import ScriptModel
from agent.tests.test_system import FixtureObservations
from agent.observations import ConsoleObservations
from agent.tests.test_transport import Server, response


class ChatTests(unittest.TestCase):
    def setUp(self):
        from arxgent import ArxgentChat, ChatError
        self.ChatError = ChatError
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.release, self.entered = threading.Event(), threading.Event()
        self.runtimes = []
        owner = self

        class Runtime:
            def __init__(self, config, store, tools, **kwargs):
                self.store, self.tools = store, tools
                self.progress = kwargs['progress']
                owner.runtimes.append(self)

            def run(self, instruction=None):
                owner.entered.set()
                owner.release.wait(3)
                self.progress({'event': 'tool', 'tool': 'get_system_status'})
                self.progress({'event': 'tool_result', 'tool': 'get_system_status', 'ok': True})
                self.store.state['messages'].append({'role': 'user', 'content': instruction})
                self.store.save()
                return {'reason': 'cancelled' if self.store.cancelled else 'model_finished', 'answer': '相机读取测试回答', 'mission_success': False}

            def continue_turn(self, instruction):
                return self.run(instruction)

        cfg = Config(profiles={'cloud': {'model': 'fixture', 'base_url': 'http://127.0.0.1:1', 'stream': False}})
        self.chat = ArxgentChat(cfg, Path(self.temp.name), runtime_factory=Runtime)
        def finish():
            self.release.set()
            self.chat.close()
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and any(s['job'] and s['job']['status'] in ('running', 'cancelling') for s in self.chat.sessions.values()):
                time.sleep(.01)
        self.addCleanup(finish)
        self.client = str(uuid.uuid4())

    def send(self, text='看一下相机', identity=None, client=None):
        client = client or self.client
        session = self.chat.snapshot(client)
        return self.chat.handle(client, {'action': 'send', 'conversation_id': session['conversation_id'],
                                          'request_id': identity or str(uuid.uuid4()), 'message': text})

    def wait_done(self, client=None):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            result = self.chat.snapshot(client or self.client)
            if not result['busy']:
                return result
            time.sleep(.01)
        self.fail('Chat worker did not complete')

    def test_send_is_async_deduplicated_and_result_has_no_secret_or_private_history(self):
        identity = str(uuid.uuid4())
        started = time.monotonic()
        first = self.send(identity=identity)
        self.assertLess(time.monotonic() - started, .5)
        self.assertTrue(first['busy'])
        self.assertTrue(self.entered.wait(1))
        self.assertEqual(self.send(identity=identity)['job']['id'], first['job']['id'])
        with self.assertRaises(self.ChatError):
            self.send(text='different', identity=identity)
        self.release.set()
        result = self.wait_done()
        self.assertEqual(len(self.runtimes), 1)
        self.assertEqual([m['role'] for m in result['messages']], ['user', 'assistant'])
        self.assertEqual(result['messages'][-1]['content'], '相机读取测试回答')
        self.assertIn('get_system_status', [a['tool'] for a in result['activity']])
        self.assertNotIn('run_dir', json.dumps(result))
        self.assertNotIn('reasoning_content', json.dumps(result))
        self.assertEqual(self.runtimes[0].tools.allowed_tools, {'get_system_status', 'observe_scene', 'read_observation'})

    def test_conversation_isolation_busy_bound_and_followup_reuses_run(self):
        self.send('记住我说的红色电池')
        other = str(uuid.uuid4())
        self.assertEqual(self.chat.snapshot(other)['messages'], [])
        with self.assertRaises(self.ChatError):
            self.send(client=other)
        self.release.set()
        self.wait_done()
        path = self.runtimes[0].store.path
        self.send('我刚才说什么颜色')
        self.wait_done()
        self.assertEqual(self.runtimes[-1].store.path, path)
        self.assertEqual([m['content'] for m in self.runtimes[-1].store.state['messages']], ['记住我说的红色电池', '我刚才说什么颜色'])

    def test_cancel_new_conversation_and_stale_submission(self):
        first = self.send()
        self.assertTrue(self.entered.wait(1))
        self.chat.handle(self.client, {'action': 'cancel', 'conversation_id': first['conversation_id'], 'request_id': first['job']['id']})
        self.release.set()
        result = self.wait_done()
        self.assertEqual(result['job']['status'], 'cancelled')
        new = self.chat.handle(self.client, {'action': 'new', 'conversation_id': first['conversation_id']})
        self.assertNotEqual(new['conversation_id'], first['conversation_id'])
        self.assertEqual(new['messages'], [])
        with self.assertRaises(self.ChatError):
            self.chat.handle(self.client, {'action': 'send', 'conversation_id': first['conversation_id'], 'request_id': str(uuid.uuid4()), 'message': 'old'})

    def test_invalid_input_and_unknown_actions_do_not_start_workers(self):
        for data in [{}, {'action': 'shell'}, {'action': 'send', 'message': 'x'},
                     {'action': 'send', 'conversation_id': self.chat.snapshot(self.client)['conversation_id'], 'request_id': str(uuid.uuid4()), 'message': 'x' * 2001}]:
            with self.subTest(data=str(data)[:80]), self.assertRaises(self.ChatError):
                self.chat.handle(self.client, data)
        self.assertEqual(self.runtimes, [])

    def test_agent_continuation_preserves_role_context_and_resets_only_turn_budget(self):
        store = RunStore.create(Path(self.temp.name) / 'continuation')
        self.addCleanup(store.close)
        cfg = Config(max_rounds=1)
        tools = ToolRegistry(cfg, store, allowed_tools={'get_system_status'})
        first = AgentRuntime(cfg, store, tools, ScriptModel([{'role': 'assistant', 'content': '记住红色'}]))
        self.assertEqual(first.run('记住红色')['reason'], 'model_finished')
        model = ScriptModel([{'role': 'assistant', 'content': '红色'}])
        second = AgentRuntime(cfg, store, tools, model)
        self.assertEqual(second.continue_turn('我说的是什么颜色')['answer'], '红色')
        self.assertEqual([m['role'] for m in model.requests[0]], ['system', 'user', 'assistant', 'user'])
        self.assertEqual(store.state['usage']['rounds'], 1)

    def test_same_provider_call_id_in_next_turn_takes_a_new_observation(self):
        store = RunStore.create(Path(self.temp.name) / 'new-frame')
        self.addCleanup(store.close)
        observations = FixtureObservations(store)
        tools = ToolRegistry(Config(), store, observations=observations, allowed_tools={'observe_scene'})
        call = {'role': 'assistant', 'content': '', 'tool_calls': [{'id': 'reused', 'type': 'function', 'function': {'name': 'observe_scene', 'arguments': '{"cameras":["d435"]}'}}]}
        first = AgentRuntime(Config(), store, tools, ScriptModel([call, {'role': 'assistant', 'content': 'first'}]))
        first.run('take a frame')
        second = AgentRuntime(Config(), store, tools, ScriptModel([call, {'role': 'assistant', 'content': 'new frame'}]))
        second.continue_turn('take a fresh frame')
        self.assertEqual(len(store.state['observations']), 2)
        replies = [json.loads(message['content']) for message in store.state['messages'] if message['role'] == 'tool']
        self.assertNotEqual(replies[0]['data']['observation_id'], replies[1]['data']['observation_id'])

    def test_status_tool_reads_selected_live_feedback_and_reports_read_failure(self):
        def respond(path, body):
            self.assertEqual(path, '/api/state')
            return response({'time': 123, 'arms': {'left': {'fresh': False, 'joints': [.1] * 6, 'secret': 'exclude'}},
                             'body': {'height': .14, 'fresh': True}, 'ros': {'connected': True, 'domain_id': '191', 'secret': 'exclude'},
                             'chassis_active': False, 'private_field': 'exclude'})
        store = RunStore.create(Path(self.temp.name) / 'live-status')
        self.addCleanup(store.close)
        with Server(respond) as server:
            cfg = Config(console_url=server.url)
            tools = ToolRegistry(cfg, store, allowed_tools={'get_system_status'})
            result = tools.execute('get_system_status', {}, 'read_live')
            robot = result['data']['robot_state']
            self.assertTrue(robot['available'])
            self.assertFalse(robot['snapshot']['arms']['left']['fresh'])
            self.assertTrue(robot['snapshot']['ros']['connected'])
            self.assertNotIn('exclude', json.dumps(robot))
            self.assertEqual([method for method, _, _ in server.requests], ['GET'])
        result = tools.execute('get_system_status', {}, 'read_failed')
        self.assertFalse(result['data']['robot_state']['available'])

    def test_spectator_churn_preserves_recent_chat_and_active_job(self):
        self.send('保留我的对话')
        self.assertTrue(self.entered.wait(1))
        for _ in range(40):
            self.assertTrue(self.chat.snapshot(str(uuid.uuid4()))['ok'])
        self.assertTrue(self.chat.snapshot(self.client)['busy'])
        self.release.set()
        previous = self.wait_done()
        for _ in range(40):
            self.chat.snapshot(str(uuid.uuid4()))
        self.assertEqual(self.chat.snapshot(self.client)['messages'], previous['messages'])
        self.assertLessEqual(len(self.chat.sessions), 32)

    def test_expired_idle_history_releases_a_session(self):
        self.release.set()
        self.send()
        self.wait_done()
        self.chat.sessions[self.client]['last_seen'] -= 3601
        self.chat.snapshot(str(uuid.uuid4()))
        self.assertNotIn(self.client, self.chat.sessions)


if __name__ == '__main__':
    unittest.main()
