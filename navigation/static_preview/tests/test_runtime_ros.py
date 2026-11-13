"""Run on NUC in ROS_DOMAIN_ID=199: cannot reach live robot graph."""
import os,tempfile,unittest,math
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from plan_only import Planner
from bridge import Bridge
class RuntimeTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  if os.getenv('ROS_DOMAIN_ID')!='199':raise RuntimeError('tests require isolated domain199')
  rclpy.init();cls.tmp=tempfile.TemporaryDirectory();os.environ['ARX_PREVIEW_RUN']=cls.tmp.name
 @classmethod
 def tearDownClass(cls):rclpy.shutdown();cls.tmp.cleanup()
 def test_stale_goal_never_submitted(self):
  n=Planner();m=PointStamped();m.header.frame_id='preview_map';n.goal(m)
  import json
  from pathlib import Path
  self.assertEqual(json.loads(Path(self.tmp.name,'plan-status.json').read_text())['status'],'rejected_stale_or_frame_or_nonfinite');self.assertFalse(n.busy);n.destroy_node()
 def test_downstream_stall_is_not_fresh(self):
  n=Planner();n.odom=Odometry();n.ot=n.ct=n.get_clock().now().nanoseconds/1e9
  self.assertFalse(n.fresh());n.destroy_node()
 def test_nodes_construct_and_finite_scan_policy(self):
  n=Bridge();m=LaserScan();m.range_min=.2;m.range_max=8.;m.ranges=[1.,float('inf'),9.,float('nan')];n.scan(m)
  self.assertEqual(m.ranges[0],1.);self.assertTrue(all(math.isnan(v) for v in m.ranges[1:]));n.destroy_node()
