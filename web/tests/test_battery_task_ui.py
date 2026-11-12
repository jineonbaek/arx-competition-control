"""Static contract for the one-confirmation fixed-flow controls."""
import unittest
from pathlib import Path


PUBLIC=Path(__file__).resolve().parents[1]/'public'


class BatteryTaskUITests(unittest.TestCase):
    def test_start_requires_one_clear_area_confirmation_and_current_epoch(self):
        html=(PUBLIC/'grasp-integration.html').read_text()
        script=(PUBLIC/'js/grasp-integration.js').read_text()
        self.assertEqual(html.count('id="environment-clear"'),1)
        self.assertIn("expected_epoch:execution.command_epoch",script)
        self.assertIn("environment_clear:true",script)
        self.assertEqual(html.count('id="lift-target-rad"'),1)
        self.assertEqual(html.count('id="observer-z-mm"'),1)
        self.assertIn('lift_target_rad:lift.valueAsNumber',script)
        self.assertIn('observer_z_mm:observer.valueAsNumber',script)
        self.assertIn("action:'cancel'",script)
        self.assertEqual(html.count('id="execution-log"'),1)
        self.assertIn("execution?.logs",script)
        self.assertIn("error.code",script)
        self.assertNotIn("'/api/grasp-integration'",script)
        self.assertIn('/api/battery-task/result',script)
        self.assertNotIn('id="start"',html)
        self.assertNotIn('TCP 未接入',html)
        self.assertIn('id="d435-live"',html)
        self.assertIn('id="d405-live"',html)


if __name__=='__main__':unittest.main()
