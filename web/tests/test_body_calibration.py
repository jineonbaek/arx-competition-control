import time
import unittest
from test_control import FakeAdapter, configured
from control import Controller, Rejected
from ros_adapter import RosAdapter

class BodyCalibrationTests(unittest.TestCase):
    def test_missing_false_or_expired_calibration_is_not_ready(self):
        adapter = RosAdapter(configured())
        for value in [None, (False,time.monotonic()), (True,time.monotonic()-2)]:
            adapter.body_calibration = value
            self.assertFalse(adapter.body_calibrated())
        adapter.body_calibration = (True,time.monotonic())
        self.assertTrue(adapter.body_calibrated())

    def test_uncalibrated_body_cannot_start_chassis_or_lift(self):
        config = configured()
        config["body"]["calibrated_topic"] = "/web/body/calibrated"
        adapter = FakeAdapter()
        adapter.body_calibrated = lambda: False
        controller = Controller(config,adapter)
        controller.update_body(.1,[0.]*4,0.,0.,0.)
        for data in [{"type":"lift","target":1.}, {"type":"chassis","direction":"forward","speed":.03,"turn_speed":.05,"sequence":1}]:
            with self.assertRaises(Rejected):controller.submit(data,"test")
        self.assertEqual(adapter.calls,[])

    def test_dedicated_lift_stop_does_not_retarget_measured_position(self):
        config = configured()
        config["body"]["lift_stop_service"] = "/web/body/stop_lift"
        adapter = FakeAdapter()
        adapter.lift_stop_ready = lambda: True
        calls=[]
        adapter.lift_stop = lambda: calls.append("stop")
        controller=Controller(config,adapter)
        controller.update_body(.5,[0.]*4,0.,0.,0.)
        controller.submit({"type":"lift.stop"},"test")
        self.assertEqual(calls,["stop"])
        self.assertEqual(adapter.calls,[])
