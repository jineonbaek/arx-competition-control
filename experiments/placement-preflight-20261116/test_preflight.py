import tempfile, unittest
from pathlib import Path
import fast_visual_level as mod

class Adapter:
    def trajectory_status(self, side): return {"driver_instance":"test", "ready":True, "enabled":True}
class Control:
    def __init__(self, left=True, right=True, body=True):
        self.adapter=Adapter(); self.reserved=False
        self.state={"motion_reserved":False,"competition_reserved":False,"chassis_active":False,"arms":{"left":{"fresh":left},"right":{"fresh":right}},"body":{"fresh":body},"command_epoch":1}
    def snapshot(self): return self.state
    def reserve_motion(self, epoch): self.reserved=True; raise ValueError("test prevents any motion")
class PreflightTests(unittest.TestCase):
    def test_missing_feedback_rejected_before_reserving(self):
        for field in ("left","right","body"):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as temp:
                ctrl=Control(**{field:False}); old=mod.WORK; mod.WORK=Path(temp); (mod.WORK / "reports").mkdir()
                try:
                    with self.assertRaisesRegex(ValueError, "feedback"):
                        mod.FastVisualLevel(ctrl,object()).action({"op":"start"})
                    self.assertFalse(ctrl.reserved)
                finally: mod.WORK=old

    def test_driver_not_ready_rejected_before_reserving(self):
        for value in (None, {"ready":False,"enabled":True,"driver_instance":"x"}, {"ready":True,"enabled":False,"driver_instance":"x"}):
            with self.subTest(driver=value), tempfile.TemporaryDirectory() as temp:
                ctrl=Control(); ctrl.adapter.trajectory_status=lambda side:value
                old=mod.WORK; mod.WORK=Path(temp); (mod.WORK / "reports").mkdir()
                try:
                    with self.assertRaisesRegex(ValueError, "driver not ready"):
                        mod.FastVisualLevel(ctrl,object()).action({"op":"start"})
                    self.assertFalse(ctrl.reserved)
                finally: mod.WORK=old
    def test_existing_ownership_is_not_overridden(self):
        ctrl=Control(); ctrl.state["motion_reserved"]=True
        with self.assertRaisesRegex(ValueError, "Other control active"):
            mod.FastVisualLevel(ctrl,object()).action({"op":"start"})
        self.assertFalse(ctrl.reserved)

    def test_ready_feedback_passes_preflight_without_hardware(self):
        from unittest.mock import patch
        class FakeThread:
            def __init__(self, **kwargs): self.started=False
            def start(self): self.started=True
            def is_alive(self): return self.started
        with tempfile.TemporaryDirectory() as temp:
            ctrl=Control(); old=mod.WORK; mod.WORK=Path(temp); (mod.WORK / "reports").mkdir()
            try:
                with patch.object(mod.threading, "Thread", FakeThread):
                    task=mod.FastVisualLevel(ctrl,object())
                    self.assertEqual(task.action({"op":"start"})["status"], "running")
                    self.assertTrue(task.thread.started)
                    self.assertFalse(ctrl.reserved)
            finally: mod.WORK=old
if __name__ == "__main__": unittest.main()
