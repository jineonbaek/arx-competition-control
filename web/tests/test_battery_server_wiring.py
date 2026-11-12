"""The web execution backend is opt-in and construction is inert."""
import json
import tempfile
import unittest
from pathlib import Path

import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from server import make_battery_task
from test_control import Controller,FakeAdapter,configured
from agent.tests.test_execution_policy import policy


class BatteryServerWiringTests(unittest.TestCase):
    def test_absent_policy_keeps_backend_disabled(self):
        controller=Controller(configured(),FakeAdapter())
        self.assertIsNone(make_battery_task(controller,9983,None))

    def test_explicit_policy_constructs_without_hardware_calls(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'policy.json';path.write_text(json.dumps(policy()))
            adapter=FakeAdapter();controller=Controller(configured(),adapter)
            task=make_battery_task(controller,9983,path)
            try:
                self.assertEqual(task.snapshot('owner')['status'],'idle')
                self.assertEqual(adapter.calls,[])
            finally:task.close()


if __name__=='__main__':unittest.main()
