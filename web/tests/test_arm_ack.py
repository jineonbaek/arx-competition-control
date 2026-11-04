import unittest
from types import SimpleNamespace
from test_control import configured
from ros_adapter import RosAdapter
from control import Rejected
class ArmAckTests(unittest.TestCase):
    def adapter(self, ok):
        a=RosAdapter(configured());a.RobotCmd=lambda:SimpleNamespace(header=SimpleNamespace())
        def publish(msg):
            self.assertEqual(msg.gripper,0.0)
            self.assertTrue(msg.header.frame_id.startswith("gripper|"))
            event,result=a.pending_arm_commands[msg.header.frame_id.split("|")[1]]
            result.update(ok=ok,message="driver rejected test" if not ok else "accepted")
            event.set()
        a._publisher=lambda *args:SimpleNamespace(publish=publish)
        return a
    def test_zero_close_target_waits_for_driver_acceptance(self):
        a=self.adapter(True);a.arm("left",[0.]*6,0,[0.]*6,5,kind="gripper")
        self.assertEqual(a.pending_arm_commands,{})
    def test_driver_rejection_is_not_reported_as_sent(self):
        a=self.adapter(False)
        with self.assertRaisesRegex(Rejected,"driver rejected test"):
            a.arm("left",[0.]*6,0,[0.]*6,5,kind="gripper")
        self.assertEqual(a.pending_arm_commands,{})

    def test_confirmation_wait_releases_controller_lock_for_feedback(self):
        import threading
        from control import Controller
        from test_control import FakeAdapter
        cfg=configured();cfg["arms"]["left"]["command_result_topic"]="/ack"
        adapter=FakeAdapter();controller=Controller(cfg,adapter,lambda:100)
        controller.update_arm("left",[0.]*6,-1,[0.]*6)
        def arm(*args, **kwargs):
            def confirm():
                done=threading.Event()
                def feedback():
                    controller.update_arm("left",[.01]*6,-1,[0.]*6)
                    done.set()
                thread=threading.Thread(target=feedback,daemon=True);thread.start()
                self.assertTrue(done.wait(.5),"Feedback blocked while waiting for ack")
                thread.join()
            return confirm
        adapter.arm=arm
        controller.submit({"type":"gripper","arm":"left","position":0},"A")
        self.assertEqual(controller.arms["left"]["joints"],[.01]*6)
