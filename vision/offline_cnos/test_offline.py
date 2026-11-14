"""Offline-only checks: dataset labeling and no actuator dependencies."""
import ast
import unittest
from pathlib import Path
from evaluate import CASES, overlap_label


class OfflineTests(unittest.TestCase):
    def test_annotation_keeps_battery_parts_but_not_drone_or_background(self):
        truth = [500, 0, 620, 104]
        self.assertEqual(overlap_label(truth, truth), 'battery_visible_part')
        self.assertEqual(overlap_label([530, 20, 600, 70], truth), 'battery_visible_part')
        self.assertEqual(overlap_label([50, 127, 155, 224], truth), 'non_battery_or_mixed')
        self.assertEqual(overlap_label([149, 0, 637, 469], truth), 'non_battery_or_mixed')
        self.assertEqual(overlap_label(truth, None), 'non_battery')

    def test_cases_are_distinct_and_contain_absent_target_control(self):
        self.assertEqual(len({case[0] for case in CASES}), len(CASES))
        self.assertTrue(any(case[3] is None for case in CASES))
        self.assertTrue(all(case[1].startswith('run-20261114') for case in CASES))

    def test_no_live_controller_or_transport_imports(self):
        tree = ast.parse(Path(__file__).with_name('evaluate.py').read_text())
        imports = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imports.add(node.module or '')
        for name in imports:
            self.assertFalse(name.startswith(('agent', 'rclpy', 'control', 'ros_adapter', 'requests', 'urllib')))


if __name__ == '__main__':
    unittest.main()
