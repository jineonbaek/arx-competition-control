"""Inspection clicks must not cancel an active mission or replace its planner goal."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np,rclpy
from geometry_msgs.msg import PointStamped
import test_mission_ros as f
from plan_only import Planner

class ClickRetention(unittest.TestCase):
    setUpClass=classmethod(lambda cls:rclpy.init())
    tearDownClass=classmethod(lambda cls:rclpy.shutdown())
    setUp=f.MissionROS.setUp;tearDown=f.MissionROS.tearDown
    refresh=f.MissionROS.refresh;add=f.MissionROS.add
    def test_publish_point_does_not_cancel_running_goal(self):
        self.add('stop',2.);self.n.waypoints.start(None);target=self.n.target.copy();identity=self.n.transport.current
        self.n.changed(PointStamped())
        np.testing.assert_allclose(self.n.target,target)
        self.assertEqual(self.n.transport.current,identity);self.assertTrue(self.n.waypoints.route.running)
        self.assertEqual(self.n.transport.calls,[])
    def test_planner_keeps_active_goal_when_inspection_point_arrives(self):
        target=object();n=SimpleNamespace(target=target,report=lambda *a,**k:None,
            set_goal=lambda *a:(_ for _ in ()).throw(AssertionError('active planner goal was replaced')))
        Planner.point_goal(n,PointStamped());self.assertIs(n.target,target)
