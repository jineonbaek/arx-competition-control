
from pathlib import Path
from types import SimpleNamespace
import json
import threading
import unittest
from test_control import configured, FakeAdapter
from control import Controller
from ros_adapter import RosAdapter


class FakeNode:
    def __init__(self):
        self.endpoints = {}
        self.fail = False
    def get_publishers_info_by_topic(self, topic):
        if self.fail:
            raise RuntimeError("graph temporarily unavailable")
        return [SimpleNamespace(topic_type=value) for value in self.endpoints.get(topic, [])]


class FeedbackLogTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.config = configured()
        self.adapter = RosAdapter(self.config)
        self.controller = Controller(self.config, FakeAdapter(), lambda: self.now)
        self.adapter.controller = self.controller
        self.adapter.node = FakeNode()
        self.adapter.body_calibration = (True, __import__("time").monotonic())

    def test_missing_publishers_reported_once_without_motion_calls(self):
        self.adapter.check_feedback()
        self.assertEqual(len(self.controller.logs), 3)
        left = self.adapter.status()["feedback_sources"]["left"]
        self.assertEqual(left["publishers"], 0)
        self.assertEqual(left["status"], "no_publisher")
        self.assertIn(self.config["arms"]["left"]["state_topic"], left["message"])
        self.adapter.check_feedback()
        self.assertEqual(len(self.controller.logs), 3)
        self.assertEqual(self.controller.adapter.calls, [])

    def test_waiting_recovery_and_expiry_are_logged_as_transitions(self):
        topic = self.config["arms"]["left"]["state_topic"]
        self.adapter.node.endpoints[topic] = ["arx5_arm_msg/msg/RobotStatus"]
        self.adapter.check_feedback()
        self.assertEqual(self.adapter.feedback_sources["left"]["status"], "waiting")
        self.controller.update_arm("left", [.1]*6, -1., [.1]*6)
        count = len(self.controller.logs)
        self.adapter.check_feedback()
        self.assertEqual(len(self.controller.logs), count+1)
        self.assertEqual(self.adapter.feedback_sources["left"]["status"], "fresh")
        self.assertEqual(self.controller.logs[-1]["level"], "info")
        self.now += 2
        self.adapter.check_feedback()
        self.assertEqual(self.adapter.feedback_sources["left"]["status"], "stale")
        self.assertEqual(self.controller.logs[-1]["level"], "warning")

    def test_wrong_message_type_and_graph_errors_are_not_misreported_as_no_publisher(self):
        topic = self.config["arms"]["right"]["state_topic"]
        self.adapter.node.endpoints[topic] = ["arm_control/msg/PosCmd"]
        self.adapter.check_feedback()
        item = self.adapter.feedback_sources["right"]
        self.assertEqual(item["status"], "type_mismatch")
        self.assertEqual(item["publishers"], 0)
        self.assertEqual(item["other_publishers"], 1)
        self.adapter.node.fail = True
        self.adapter.check_feedback()
        self.assertEqual(self.adapter.feedback_sources["right"]["status"], "graph_error")
        self.assertIsNone(self.adapter.feedback_sources["right"]["publishers"])
        self.assertIsNotNone(self.adapter.node)

    def test_concurrent_logging_preserves_unique_ids_and_bounded_history(self):
        def writer():
            for i in range(40):
                self.controller.log("test-%d" % i)
        threads = [threading.Thread(target=writer) for _ in range(4)]
        for thread in threads:thread.start()
        for thread in threads:thread.join()
        ids = [item["id"] for item in self.controller.logs]
        self.assertEqual(len(ids), 100)
        self.assertEqual(ids, list(range(61,161)))
        self.assertEqual(self.controller.adapter.calls, [])

    def test_diagnostics_are_returned_by_existing_http_state_shape(self):
        self.adapter.check_feedback()
        self.controller.adapter = self.adapter
        data = json.loads(json.dumps(self.controller.snapshot()))
        self.assertIn("domain_id", data["ros"])
        self.assertEqual(data["ros"]["feedback_sources"]["left"]["status"], "no_publisher")
        self.assertFalse(data["arms"]["left"]["fresh"])


if __name__ == "__main__":
    unittest.main()
