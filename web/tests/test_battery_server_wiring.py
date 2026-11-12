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


ROOT=Path(__file__).resolve().parents[2]


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


    def test_site_start_enables_valid_policy_and_arm_trajectories(self):
        policy_path=ROOT/'config/battery-execution.json'
        controller=Controller(configured(),FakeAdapter())
        task=make_battery_task(controller,9983,policy_path)
        try:self.assertEqual(task.snapshot('owner')['status'],'idle')
        finally:task.close()
        start=(ROOT/'web/start.sh').read_text()
        self.assertIn('--battery-policy "$CONTROL_ROOT/config/battery-execution.json"',start)
        expected={'speed_deg_s':5.7,'acceleration_deg_s2':12.0,
                  'start_tolerance_deg':0.5,'tracking_tolerance_deg':3.0,
                  'max_tick_gap_s':0.05,'lease_s':0.5,
                  'max_duration_s':60.0,'start_speed_deg_s':0.2}
        control=json.loads((ROOT/'web/config/control.json').read_text())
        self.assertEqual(control['arms']['left']['trajectory_policy'],expected)
        self.assertEqual(control['arms']['right']['trajectory_policy'],expected)


if __name__=='__main__':unittest.main()
