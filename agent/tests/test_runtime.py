import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from agent.config import Config
from agent.storage import RunStore
from agent.tools import ToolRegistry
from agent.tests.test_system import FixtureBackend, FixtureObservations


def call(name, arguments, identity):
    return {'id': identity, 'type': 'function', 'function': {'name': name, 'arguments': json.dumps(arguments)}}


class ScriptModel:
    def __init__(self, replies):
        self.replies, self.requests = iter(replies), []

    def complete(self, messages, tools):
        self.requests.append(copy.deepcopy(messages))
        reply = next(self.replies)
        return reply() if callable(reply) else reply


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = RunStore.create(self.temp.name)
        self.addCleanup(self.store.close)
        self.config = Config()
        self.tools = ToolRegistry(self.config, self.store, FixtureObservations(self.store), FixtureBackend())

    def run_model(self, replies, instruction='test'):
        from agent.runtime import AgentRuntime
        model = ScriptModel(replies)
        result = AgentRuntime(self.config, self.store, self.tools, model).run(instruction)
        return result, model

    def test_multiple_calls_have_all_replies_and_model_words_cannot_complete_mission(self):
        result, model = self.run_model([
            {'role': 'assistant', 'content': '', 'tool_calls': [call('set_task', {'target_color': 'red', 'destination': 'red_slot'}, 'a'), call('get_system_status', {}, 'b')]},
            {'role': 'assistant', 'content': 'Task finished successfully'},
        ])
        self.assertEqual([m['tool_call_id'] for m in model.requests[1] if m['role'] == 'tool'], ['a', 'b'])
        self.assertFalse(result['mission_success'])
        self.assertEqual(result['stage'], 'TASK_ACCEPTED')

    def test_tool_budget_blocks_remaining_calls_and_survives_resume(self):
        self.config.max_tool_calls = 1
        result, _ = self.run_model([{'role': 'assistant', 'content': '', 'tool_calls': [call('get_system_status', {}, 'a'), call('observe_scene', {'cameras': ['d435']}, 'b')]}])
        self.assertEqual(result['reason'], 'tool_budget_exhausted')
        self.assertFalse(self.store.state['observations'])
        result, model = self.run_model([], None)
        self.assertEqual(result['reason'], 'tool_budget_exhausted')
        self.assertFalse(model.requests)

    def test_cancel_during_model_reply_prevents_tool_effect(self):
        def cancelled_reply():
            (self.store.path / 'cancel.flag').touch()
            return {'role': 'assistant', 'content': '', 'tool_calls': [call('set_task', {'target_color': 'red', 'destination': 'red_slot'}, 'a')]}
        result, _ = self.run_model([cancelled_reply])
        self.assertEqual(result['reason'], 'cancelled')
        self.assertEqual(self.store.state['mission']['stage'], 'IDLE')

    def test_resume_drains_persisted_unanswered_call_once(self):
        self.store.state['messages'] = [{'role': 'user', 'content': 'test'}, {'role': 'assistant', 'content': '', 'tool_calls': [call('set_task', {'target_color': 'red', 'destination': 'red_slot'}, 'original')]}]
        self.store.save()
        result, model = self.run_model([{'role': 'assistant', 'content': 'accepted'}], None)
        self.assertEqual(result['stage'], 'TASK_ACCEPTED')
        self.assertEqual(len(self.store.state['calls']), 1)
        self.assertEqual(model.requests[0][-1]['role'], 'tool')

    def test_observation_images_sent_to_model_but_not_base64_persisted(self):
        obs = self.store.save_observation({'cameras': {'d435': {'image_size_px': [16, 12], 'geometry_ready': False}}}, {'d435': b'fixture-jpeg'})
        result, model = self.run_model([
            {'role': 'assistant', 'content': '', 'tool_calls': [call('read_observation', {'observation_id': obs['observation_id']}, 'read')]},
            {'role': 'assistant', 'content': 'observed'},
        ])
        image_message = model.requests[1][-1]['content']
        images = [part for part in image_message if part['type'] == 'image_url']
        self.assertEqual(len(images), 1)
        self.assertTrue(images[0]['image_url']['url'].startswith('data:image/jpeg;base64,'))
        self.assertNotIn('base64,', (self.store.path / 'state.json').read_text(encoding='utf-8'))

    def test_full_workspace_style_key_loaded_without_truncation_or_shell_execution(self):
        from agent.config import load_private_env
        path = Path(self.temp.name) / 'private.env'
        path.write_text('DASHSCOPE_API_KEY=sk-ws-v1.abc.def.ghi\nQWEN_BASE_URL=https://maas.qianwenaiapi.com/compatible-mode/v1\n')
        path.chmod(0o600)
        with patch.dict('os.environ', {}, clear=True):
            load_private_env(path)
            self.assertEqual(Config().profile('cloud')['api_key'], 'sk-ws-v1.abc.def.ghi')


if __name__ == '__main__':
    unittest.main()
