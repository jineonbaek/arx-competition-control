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
        self.assertIn("action:'cancel'",script)


if __name__=='__main__':unittest.main()
