"""Fresh real receipt may bridge callback work, never source/pose expiry."""
import copy,time,threading,unittest
from types import SimpleNamespace
from unittest.mock import patch
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import test_mission_ros as fixture
from executor_tf import TFReceiver

class ScanReceiptTests(unittest.TestCase):
 setUpClass=classmethod(lambda c:rclpy.init());tearDownClass=classmethod(lambda c:rclpy.shutdown())
 setUp=fixture.MissionROS.setUp;tearDown=fixture.MissionROS.tearDown;refresh=fixture.MissionROS.refresh
 def cached_old(self):self.n.sensor_received['/pb/scan']=time.monotonic()-.25
 def independent(self,age=0.,receipt_age=0.):
  self.n.tf_receiver.scan_sample=(self.n.get_clock().now().nanoseconds-int(age*1e9),time.monotonic()-receipt_age)
 def test_new_independent_frame_avoids_false_cache_timeout_without_mutating_pose(self):
  self.cached_old();stamp=self.n.pose_snapshot.header.stamp;cache=self.n.sensor_received['/pb/scan'];self.independent()
  self.assertNotIn('/pb/scan',self.n.stale_inputs())
  self.assertEqual(self.n.sensor_received['/pb/scan'],cache);self.assertEqual(self.n.pose_snapshot.header.stamp,stamp)
 def test_receipt_deadline_is_not_extended(self):
  self.cached_old();self.independent(receipt_age=.201);self.assertIn('/pb/scan',self.n.stale_inputs())
 def test_fresh_arrival_cannot_hide_expired_or_future_source(self):
  for age in (.581,-.01):
   self.cached_old();self.independent(age=age);self.assertIn('/pb/scan',self.n.stale_inputs())
 def test_pose_deadline_remains_independent(self):
  self.cached_old();self.independent();msg=copy.deepcopy(self.n.pose_snapshot);msg.header.stamp=Time(nanoseconds=self.n.get_clock().now().nanoseconds-401_000_000).to_msg();self.n.receive_chassis_snapshot(msg)
  with self.assertRaisesRegex(ValueError,'motion_pose_stale_or_invalid'):self.n.motion_poses()
 def test_same_or_older_than_main_stream_cannot_renew_receipt(self):
  self.cached_old();st=self.n.sensor['/pb/scan'];source=st.sec*1_000_000_000+st.nanosec
  for delta in (0,-1_000_000):
   self.n.tf_receiver.scan_sample=(source+delta,time.monotonic())
   self.assertIn('/pb/scan',self.n.stale_inputs())
 def test_duplicate_source_does_not_renew_independent_receipt(self):
  rx=SimpleNamespace(lock=threading.Lock(),scan_sample=None);m=LaserScan();m.header.stamp.sec=1
  with patch('executor_tf.time.monotonic',return_value=10.):TFReceiver.receive_scan(rx,m)
  with patch('executor_tf.time.monotonic',return_value=20.):TFReceiver.receive_scan(rx,m)
  self.assertEqual(rx.scan_sample,(1_000_000_000,10.))
if __name__=='__main__':unittest.main()
