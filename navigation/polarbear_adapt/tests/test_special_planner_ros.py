import os,json,tempfile,unittest
assert os.environ.get('ROS_DOMAIN_ID') in ('197','198')
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from plan_only import Planner

class SpecialPlanner(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();os.environ['ARX_ADAPT_RUN']=self.temp.name;self.p=Planner()
 def tearDown(self):self.p.destroy_node();self.temp.cleanup();os.environ.pop('ARX_ADAPT_RUN',None)
 def goal(self):
  m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.p.get_clock().now().to_msg();m.pose.orientation.w=1.;return m
 def constraint(self,mode,stamp,revision):
  self.p.constraints_changed(String(data=json.dumps(dict(terminal_stamp_ns=stamp,accept_after_ns=0,vias=[],execution_mode=mode,mode_generation_ns=revision))))
 def test_direct_mode_does_not_dispatch_native_planner(self):
  m=self.goal();t=m.header.stamp.sec*10**9+m.header.stamp.nanosec;self.constraint('direct',t,1);self.p.set_goal(m);self.p.tick()
  self.assertIsNone(self.p.target);self.assertFalse(self.p.pending)
 def test_reordered_return_to_normal_accepts_only_matching_owned_goal(self):
  self.constraint('direct',1,1);owned=self.goal();other=self.goal();other.header.stamp.nanosec+=1
  self.p.set_goal(owned);self.p.set_goal(other)
  t=owned.header.stamp.sec*10**9+owned.header.stamp.nanosec;self.constraint('navigation',t,2)
  self.assertEqual(self.p.target.header.stamp,owned.header.stamp)
  self.constraint('direct',1,1);self.assertEqual(self.p.target.header.stamp,owned.header.stamp)

if __name__=='__main__':unittest.main()
