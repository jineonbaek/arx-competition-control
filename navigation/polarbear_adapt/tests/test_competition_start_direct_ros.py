"""ROS197 integration: actual route/controller, inert chassis; no real robot."""
import os,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import rclpy
from geometry_msgs.msg import PoseArray
import test_special_ros as f

class CompetitionStart(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 refresh=f.SpecialROS.refresh;add=f.SpecialROS.add;worker=f.SpecialROS.worker
 def setUp(self):
  self.run=tempfile.TemporaryDirectory();self.env=patch.dict(os.environ,{'ARX_ADAPT_RUN':self.run.name,'ARX_DESKTOP_MODE':'mapping-navigation','ARX_DESKTOP_MANAGED':'0'})
  self.env.start();Path(self.run.name,'competition-map').touch();f.SpecialROS.setUp(self)
 def tearDown(self):
  f.SpecialROS.tearDown(self);self.env.stop();self.run.cleanup()
 def start(self,*xs):
  for x in xs:self.add('special',x)
  self.n.waypoints.start(None);self.n.seen=self.n.seq
 def test_far_first_point_is_visited_directly_with_occupied_maps(self):
  self.blocked=True;self.refresh();self.start(.18,.38)
  route=self.n.waypoints.route
  self.assertEqual(route.mode,'direct');self.assertEqual(route.index,0)
  self.assertEqual([p['pose'][0] for p in route.direct_points()],[.18,.38])
  self.assertEqual(route.direct_source['pose'],[0.,0.,0.])
  self.simulate=True;self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertEqual(route.index,1);self.assertGreater(self.position[0],.28)
  self.assertTrue(any(c[0]>0 for c in self.n.transport.calls));self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.))
 def test_single_far_first_special_finishes_without_normal_planning(self):
  self.missing=True;self.refresh();self.start(.18)
  self.assertEqual(self.n.waypoints.route.mode,'direct');self.simulate=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped');self.n.cancel('arrived_stopped')
  self.assertEqual(self.n.waypoints.route.state,'completed')
 def test_near_first_point_retains_existing_chain_entry(self):
  self.start(0.,.18);r=self.n.waypoints.route
  self.assertEqual(r.mode,'direct');self.assertEqual(r.index,1);self.assertEqual(len(r.direct_points()),1)
 def test_other_maps_keep_normal_start_and_occupied_map_rejection(self):
  Path(self.run.name,'competition-map').unlink();self.blocked=True;self.refresh();self.start(.18,.38)
  self.assertEqual(self.n.waypoints.route.mode,'navigation');self.assertFalse(self.n._special_execution)
  with self.assertRaisesRegex(ValueError,'blocked'):self.n.checked()
  self.assertEqual(self.n.transport.calls,[])
 def test_formal_task_first_keeps_task_direct_handoff_semantics(self):
  self.worker();self.add('task',.18,'point_1');self.n.waypoints.start(None)
  self.assertEqual(self.n.waypoints.route.mode,'task_direct');self.assertEqual(self.n.waypoints.route.points[0]['point_id'],'point_1')
  self.assertEqual(self.n.transport.calls,[])
 def test_withdrawn_pose_does_not_start_competition_route(self):
  self.add('special',.18);m=PoseArray();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg();self.n.receive_chassis_snapshot(m)
  self.n.waypoints.start(None)
  self.assertFalse(self.n.waypoints.route.running);self.assertIsNone(self.n.target);self.assertEqual(self.n.transport.calls,[])
if __name__=='__main__':unittest.main()
