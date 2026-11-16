import json,os,tempfile,unittest
assert os.environ.get('ROS_DOMAIN_ID') in ('197','198')
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool,String
from plan_only import Planner
class PlannerOrdering(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();os.environ['ARX_ADAPT_RUN']=self.tmp.name;self.n=Planner();self.n.tick=lambda:None
 def tearDown(self):self.n.destroy_node();self.tmp.cleanup();os.environ.pop('ARX_ADAPT_RUN',None)
 def goal(self):
  m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg();m.pose.orientation.w=1.;m.pose.position.x=1.;return m
 def constraints(self,m,fence=0):
  s=m.header.stamp;return String(data=json.dumps(dict(terminal_stamp_ns=s.sec*1000000000+s.nanosec,accept_after_ns=fence,vias=[])))
 def test_new_mission_goal_before_unlock_is_not_lost(self):
  n=self.n;n.handoff_changed(Bool(data=True));g=self.goal();n.set_goal(g);n.constraints_changed(self.constraints(g));n.handoff_changed(Bool(data=False))
  self.assertIsNotNone(n.target);self.assertEqual(n.target.header.stamp,g.header.stamp)
 def test_unlock_before_constraints_still_accepts_deferred_matching_goal(self):
  n=self.n;n.handoff_changed(Bool(data=True));g=self.goal();n.set_goal(g);n.handoff_changed(Bool(data=False));self.assertIsNone(n.target)
  n.constraints_changed(self.constraints(g));self.assertIsNotNone(n.target)
 def test_old_arrow_cannot_override_after_release_fence(self):
  n=self.n;old=self.goal();fence=n.get_clock().now().nanoseconds;fresh=self.goal();n.constraints_changed(self.constraints(fresh,fence));n.set_goal(old)
  self.assertIsNone(n.target)
if __name__=='__main__':unittest.main()
