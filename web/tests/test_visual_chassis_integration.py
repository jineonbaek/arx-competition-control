import ast
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


class VisualChassisIntegrationTests(unittest.TestCase):
    def test_route_points_to_visual_chassis_policy_and_handoff(self):
        route = json.loads((ROOT / "config" / "taught-placement.json").read_text())
        self.assertEqual(route["navigation_handoff"], "visual_chassis_locked_admitted")
        self.assertEqual(route["visual_chassis_policy"], "config/visual-chassis-placement.json")
        self.assertEqual(route["visual_chassis_handoff"], "reports/visual-chassis-active/handoff.json")

    def test_arm_route_uses_locked_handoff_then_d405_preobservation_and_true_x_insert(self):
        source = (ROOT / "web" / "src" / "fast_visual_level.py").read_text()
        tree = ast.parse(source)
        method = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "auto_insert_160_place")
        text = ast.get_source_segment(source, method)
        self.assertIn("load_locked_handoff", text)
        self.assertIn("d405_preobservation", text)
        self.assertIn("auto_d405_horizontal_trim", text)
        self.assertNotIn("d435_reference_align", text)
        self.assertLess(text.index("load_locked_handoff"), text.index("d405_preobservation"))
        self.assertLess(text.index("validate_preobservation_waypoint"), text.index("self._timed('d405_preobservation'"))
        self.assertLess(text.index("d405_preobservation"), text.index("auto_d405_horizontal_trim"))
        self.assertLess(text.index("auto_d405_horizontal_trim"), text.index("full_insert"))
        self.assertIn("np.array([1.,0.,0.])", text)
        self.assertIn("fixed_z=True", text)
        trim = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "auto_d405_horizontal_trim")
        trim_text = ast.get_source_segment(source, trim)
        self.assertIn("execute_d405_lateral_path(path", trim_text)
        self.assertNotIn("self.arm_to(", trim_text)
        self.assertIn("dict(op='auto_insert_160',visual_chassis_handoff='locked_admitted')", source)

    def test_page_records_reference_and_displays_side_by_side_comparison(self):
        html = (ROOT / "web" / "public" / "visual-docking.html").read_text()
        script = (ROOT / "web" / "public" / "visual-docking.js").read_text()
        for marker in ("record-reference", "reference-d435", "current-d435", "comparison-residual"):
            self.assertIn(marker, html)
        self.assertNotIn("astra", html.lower())
        self.assertNotIn("astra", script.lower())
        self.assertIn("/api/placement-reference", script)
        self.assertIn("complete_degraded", script)
        self.assertNotIn("onclick=()=>action('auto_insert_160')", script)


if __name__ == "__main__":
    unittest.main()
