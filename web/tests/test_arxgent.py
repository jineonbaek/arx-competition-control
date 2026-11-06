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
        self.addCleanup(self.chat.close)
        self.addCleanup(self.release.set)
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


if __name__ == '__main__':
    unittest.main()
