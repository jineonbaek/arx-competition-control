import unittest
from pathlib import Path


class TaughtPlacementUiTests(unittest.TestCase):
    def test_one_click_full_flow_preserves_overlays_and_has_no_release_confirm(self):
        root=Path(__file__).resolve().parents[2]
        html=(root/'web/public/visual-docking.html').read_text()
        js=(root/'web/public/visual-docking.js').read_text()
        self.assertIn('完整放置并自动返回',html)
        self.assertIn("action('auto_full')",js)
        self.assertIn('/camera-overlay/',js)
        self.assertIn('total_duration_s',js)
        self.assertIn('红框',html)
        self.assertIn('电池轮廓',html)
        self.assertIn('D435 仓外视觉闭环',html)
        self.assertNotIn('复用首次成功示教路线',html)
        for text in ('计划耗时','实际耗时','累计耗时'):
            self.assertIn(text,html)
        self.assertIn('planned_duration_s',js)
        self.assertIn('cumulative_duration_s',js)
        self.assertIn('timeline',js)
        self.assertNotIn('确认松爪',html+js)

    def test_server_wires_only_the_arm_visual_placement_endpoint(self):
        root=Path(__file__).resolve().parents[2]
        source=(root/'web/src/server.py').read_text()
        self.assertIn("'/api/fast-level'",source)
        self.assertIn('FastVisualLevel',source)
        self.assertNotIn("'/api/visual-docking'",source)
        self.assertNotIn('LiveDockTask',source)


if __name__=='__main__': unittest.main()
