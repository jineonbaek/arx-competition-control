"""Real Bridge conversion with only TF/publishers replaced; no ROS node."""
import unittest
from types import SimpleNamespace,MethodType
import numpy as np
import bridge
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32,read_points_numpy

class Pub:
 def __init__(self):self.values=[]
 def publish(self,m):self.values.append(m)
class TF:
 def __init__(self):self.ready=False;self.calls=0
 def lookup_transform(self,*a):
  self.calls+=1
  if not self.ready:raise RuntimeError('TF delayed')
  t=TransformStamped();t.transform.translation.z=.23;t.transform.rotation.w=1.;return t

class BridgeSegments(unittest.TestCase):
 def fixture(self):
  n=SimpleNamespace(pairs={},rays={},buf=TF(),level='pb_lidar_level',global_filter=SimpleNamespace(keep=lambda x,*a:np.ones(len(x),bool)),clock_ns=1_200_000_000)
  for name in ('safeobs','safeground','navobs','shared_pub','global_pub'):setattr(n,name,Pub())
  n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=n.clock_ns))
  if hasattr(bridge,'SegmentFrames'):
   n.segment_frames=bridge.SegmentFrames()
   n.flush_segments=MethodType(bridge.Bridge.flush_segments,n)
   n.emit_segment=MethodType(bridge.Bridge.emit_segment,n)
  return n
 def cloud(self,sec=1):
  h=Header();h.stamp.sec=sec;h.frame_id='pb_lidar_level'
  return create_cloud_xyz32(h,np.array([[1.,0.,.10]],np.float32))
 def pair(self,n):
  bridge.Bridge.segment(n,self.cloud(),'ground');bridge.Bridge.segment(n,self.cloud(),'obstacle')
 def test_retry_complete_pair_after_tf_becomes_available_preserves_stamp(self):
  n=self.fixture();self.pair(n);self.assertEqual(len(n.navobs.values),0)
  n.buf.ready=True
  if hasattr(n,'flush_segments'):n.flush_segments()
  self.assertEqual(len(n.navobs.values),1,'Temporary TF absence must not permanently discard a complete pair')
  self.assertEqual(n.navobs.values[0].header.stamp.sec,1)
  self.assertEqual(n.navobs.values[0].header.stamp.nanosec,0)
 def test_cached_source_origin_avoids_tf_round_trip(self):
  n=self.fixture()
  if hasattr(n,'segment_frames'):n.segment_frames.remember(1_000_000_000,'pb_lidar_level',np.array([0.,0.,.23]))
  self.pair(n)
  self.assertEqual(len(n.navobs.values),1,'Matching source geometry is sufficient without a later TF callback')
  self.assertEqual(n.buf.calls,0)
  points=read_points_numpy(n.navobs.values[0])
  self.assertTrue(any(np.allclose(p,[1.,0.,.10]) for p in points))
 def test_expired_pair_never_reappears_when_tf_recovers(self):
  n=self.fixture();self.pair(n);n.clock_ns=1_600_000_001;n.buf.ready=True
  if hasattr(n,'flush_segments'):n.flush_segments()
  self.assertEqual(len(n.navobs.values),0)
if __name__=='__main__':unittest.main()
