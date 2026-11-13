import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

class MissionTest(unittest.TestCase):
    def test_only_confirmed_arrival_advances(self):
        from waypoint_queue import Route
        route=Route()
        route.add("left"); route.add("turn")
        self.assertEqual(route.start(),"left")
        self.assertIsNone(route.start())
        self.assertEqual(route.finish("arrived_stopped"),"turn")
        self.assertIsNone(route.finish("goal_blocked"))
        self.assertFalse(route.running)
        self.assertEqual(route.state,"goal_blocked")
        self.assertIsNone(route.finish("arrived_stopped"))

    def test_cancel_and_new_route(self):
        from waypoint_queue import Route
        route=Route()
        route.add(1);route.add(2);route.start()
        self.assertFalse(route.add(3))
        route.reset("cancelled")
        self.assertIsNone(route.finish("arrived_stopped"))
        route.add(4);self.assertEqual(route.start(),4)
        self.assertIsNone(route.finish("arrived_stopped"))
        self.assertEqual(route.state,"completed")
