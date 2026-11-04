import time
import unittest
from test_control import configured, FakeAdapter
from control import Controller, Rejected
from ros_adapter import RosAdapter
class ArmReadyTests(unittest.TestCase):
    def test_readiness_must_be_true_and_fresh(self):
        adapter = RosAdapter(configured())
        self.assertFalse(adapter.arm_ready("left"))
        adapter.arm_readiness["left"] = (True, time.monotonic())
        self.assertTrue(adapter.arm_ready("left"))
        adapter.arm_readiness["left"] = (True, time.monotonic()-2)
        self.assertFalse(adapter.arm_ready("left"))
        adapter.arm_readiness["left"] = (False, time.monotonic())
        self.assertFalse(adapter.arm_ready("left"))
    def test_arm_stop_uses_selected_driver_service_and_readiness_blocks_motion(self):
        cfg = configured()
        cfg["arms"]["right"].update(ready_topic="/web/right/ready", stop_service="/web/right/stop")
        adapter = FakeAdapter()
        controller = Controller(cfg,adapter,lambda:100)
        controller.update_arm("right",[.1]*6,-1,[0.]*6)
        controller.submit({"type":"arm.stop","arm":"right"},"A")
        self.assertEqual(adapter.calls,[("arm_stop",("right",))])
        adapter.arm_ready = lambda side:False
        with self.assertRaises(Rejected):
            controller.submit({"type":"arm.joints","arm":"right","joints":[5.7]*6},"A")
        self.assertEqual(len(adapter.calls),1)
