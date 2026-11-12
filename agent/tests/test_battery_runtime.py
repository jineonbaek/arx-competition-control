"""Existing model/tool runtime with battery stages, no robot or model network."""
import json
import tempfile
import unittest
from agent.config import Config
from agent.storage import RunStore
from agent.battery_tools import BatteryExecutionTools
from agent.battery_runtime import BatteryRuntime
from agent.tests.test_runtime import ScriptModel,call

class BatteryRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.store=RunStore.create(self.temp.name);self.addCleanup(self.store.close)
        self.config=Config()
    def test_requires_explicit_matching_battery_registry(self):
        with self.assertRaises(ValueError):BatteryRuntime(self.config,self.store)
        tools=BatteryExecutionTools(self.config,self.store,observations=object())
        with self.assertRaises(ValueError):BatteryRuntime(Config(),self.store,tools=tools)
    def test_model_cannot_skip_observation_or_grant_motion(self):
        tools=BatteryExecutionTools(self.config,self.store,observations=object())
        model=ScriptModel([
            dict(role='assistant',content='',tool_calls=[call('plan_battery_pick',{'observation_id':'guessed'},'skip')]),
            dict(role='assistant',content='',tool_calls=[call('execute_battery_approach',{'plan_id':'guessed'},'move')]),
            dict(role='assistant',content='Cannot execute without prerequisites.')])
        result=BatteryRuntime(self.config,self.store,tools=tools,model=model).run('抓取电池')
        self.assertEqual(result['stage'],'IDLE');self.assertFalse(result['mission_success'])
        self.assertIsNone(result['pending'])
        replies=[json.loads(m['content']) for m in self.store.state['messages'] if m['role']=='tool']
        self.assertTrue(all(not r['ok'] for r in replies))
        system=self.store.state['messages'][0]['content']
        self.assertIn('execute_battery_approach',system)
        self.assertIn('TCP',system)
    def test_exposes_planning_but_not_unconfigured_execution(self):
        tools=BatteryExecutionTools(self.config,self.store,observations=object())
        visible={d['function']['name'] for d in tools.definitions(available_only=True)}
        self.assertIn('plan_battery_pick',visible)
        self.assertIn('observe_battery',visible)
        self.assertNotIn('execute_battery_approach',visible)
        self.assertNotIn('execute_pick',visible)

if __name__=='__main__':unittest.main()
