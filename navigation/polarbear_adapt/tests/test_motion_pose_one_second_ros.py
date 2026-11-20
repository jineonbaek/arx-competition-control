"""Actual ROS197 driver checks, source timestamps retained, inert SDK output."""
import os,copy,time,unittest
assert os.environ.get('ROS_DOMAIN_ID')=='197'
from types import SimpleNamespace as NS
import numpy as np,rclpy
from rclpy.time import Time
import test_mission_ros as f
from agent.chassis_retreat import _PoseReader
from axis_trial import body_delta
class MotionPoseROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.MissionROS.setUp;tearDown=f.MissionROS.tearDown;refresh=f.MissionROS.refresh;add=f.MissionROS.add
 def aged(self,age):
  self.add('stop',1.);self.n.waypoints.start(None);self.n.seen=self.n.seq
  m=copy.deepcopy(self.n.pose_snapshot);m.header.stamp=Time(nanoseconds=self.n.get_clock().now().nanoseconds-round(age*1e9)).to_msg();self.n.receive_chassis_snapshot(m)
  self.n.sensor['/localization/odometry']=m.header.stamp;self.n.sensor_received['/localization/odometry']=time.monotonic()-age
  return m.header.stamp.sec*10**9+m.header.stamp.nanosec
 def test_valid_800ms_pose_and_receipt_do_not_interrupt_sdk_output(self):
  original=self.aged(.8);self.n.checked();self.n.transport.send((.08,0.,0.))
  self.assertEqual(self.n.transport.calls[-1],(.08,0.,0.));s=self.n.pose_snapshot.header.stamp;self.assertEqual(s.sec*10**9+s.nanosec,original)
 def test_grasp_reader_uses_the_same_one_second_motion_consumer(self):
  self.aged(.8);n=self.n;n.received_pose=True
  reader=_PoseReader.__new__(_PoseReader);reader.node=n;reader.executor=NS(spin_once=lambda **kw:None);reader.previous_pose=None;reader.measured=np.zeros(3);reader.body_delta=body_delta
  self.assertIsNotNone(reader.read());self.assertEqual(n.transport.calls,[])
 def test_pose_beyond_one_second_does_not_authorize_output(self):
  self.aged(1.001)
  with self.assertRaisesRegex(ValueError,'motion_pose_stale_or_invalid|sensor_or_costmap_stale /localization/odometry'):self.n.checked()
  self.assertEqual(self.n.transport.calls,[])
if __name__=='__main__':unittest.main()
