"""Source-time boundaries with production policies; no ROS node or actuator."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from geometry_msgs.msg import Pose,PoseArray
from rclpy.time import Time
import pose_freshness as freshness
from chassis_pose_policy import PosePolicy,SnapshotLease
from goal_drive import Driver
from follow_preview import Follow
from special_segments import StraightChain
from single_bottom.odom_guard import OdomGuard

class MotionPose(unittest.TestCase):
 def test_authority_accepts_one_second_motion_pose_after_three_real_sources(self):
  p=PosePolicy();valid=[]
  for stamp in (10,10.1,10.2):valid.append(p.accept(round(stamp*1e9),round((stamp+.9)*1e9),(0,0,0)))
  self.assertEqual(valid,[False,False,True])
 def test_default_map_lease_still_expires_at_650ms(self):
  p=SnapshotLease();self.assertTrue(p.offer(10_000_000_000,10_400_000_000));self.assertFalse(p.usable(10_650_000_001))
 def test_motion_lease_boundary_and_old_sample_cannot_revive(self):
  p=SnapshotLease(max_age_ns=1_000_000_000);self.assertTrue(p.offer(10_000_000_000,10_900_000_000));self.assertTrue(p.usable(11_000_000_000));self.assertFalse(p.usable(11_000_000_001));self.assertFalse(p.offer(10_000_000_000,10_900_000_000))
 def node(self,age):
  now=20_000_000_000;stamp=now-round(age*1e9);state={'now':now};n=NS(pose_snapshot=None,pose_lease=SnapshotLease(max_age_ns=1_000_000_000),get_clock=lambda:NS(now=lambda:NS(nanoseconds=state['now'])))
  m=PoseArray();m.header.frame_id='map';m.header.stamp=Time(nanoseconds=stamp).to_msg()
  for _ in range(2):p=Pose();p.orientation.w=1.;m.poses.append(p)
  Driver.receive_chassis_snapshot(n,m);return n,m
 def test_motion_consumer_accepts_valid_400_to_1000ms_without_invalidating(self):
  for age in (.401,.65,.9,1.):
   n,m=self.node(age);result=Driver.motion_poses(n);self.assertIs(n.pose_snapshot,m);self.assertEqual(result[2],20.-age)
 def test_over_one_second_future_and_invalid_pose_still_rejected(self):
  for age in (1.000001,-.01):
   n,m=self.node(age)
   with self.assertRaisesRegex(ValueError,'motion_pose_stale_or_invalid'):Driver.motion_poses(n)
  n,m=self.node(.8);m.poses[0].orientation.w=0.;Driver.receive_chassis_snapshot(n,m)
  with self.assertRaisesRegex(ValueError,'motion_pose_stale_or_invalid'):Driver.motion_poses(n)
 def test_odom_age_and_receipt_both_allow_one_second_scan_bounds_unchanged(self):
  st=lambda v:Time(nanoseconds=round(v*1e9)).to_msg()
  n=NS(now=lambda:10.,sensor={'/pb/scan':st(9.9),'/localization/odometry':st(9.2)},sensor_received={'/pb/scan':9.9,'/localization/odometry':9.2},grids={f:(st(10.),None) for f in ('map','odom')})
  with patch('follow_preview.time.monotonic',return_value=10.):self.assertEqual(Follow.stale_inputs(n),[])
  n.sensor_received['/pb/scan']=9.7
  with patch('follow_preview.time.monotonic',return_value=10.):self.assertIn('/pb/scan',Follow.stale_inputs(n));self.assertNotIn('/localization/odometry',Follow.stale_inputs(n))
 def test_special_chain_allows_delayed_source_and_up_to_one_second_source_gap(self):
  c=StraightChain(np.zeros(3),np.zeros(3),[dict(id='goal',pose=[1.,0.,0.])]);c.sample([0,0,0],[0,0,0],10.,10.9);c.sample([0,0,0],[0,0,0],10.8,11.7)
  with self.assertRaisesRegex(ValueError,'special_pose_stale'):c.sample([0,0,0],[0,0,0],10.9,11.900001)
 def test_raw_odom_quality_gate_is_kept_independent_from_motion_consumers(self):
  guard=OdomGuard();self.assertEqual(guard.accept(10_000_000_000,10_900_000_000,[0,0,0],[0,0,0,1],[0]*6),'wait');self.assertEqual(guard.last,10_000_000_000)

class RemainingPoseConsumers(unittest.TestCase):
 def test_preview_gate_odometry_and_tf_accept_800ms(self):
  import sys
  from pathlib import Path
  sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'mppi_trial'))
  from preview_gate import PreviewGate
  stamp=lambda t:Time(nanoseconds=round(t*1e9)).to_msg()
  inputs={k:(10_000_000_000,10.,None) for k in ('front','rear','costmap')};inputs['odometry']=(9_200_000_000,9.2,None)
  n=NS(inputs=inputs,buf=NS(lookup_transform=lambda *a:NS(header=NS(stamp=stamp(9.2)))))
  with patch('preview_gate.time.monotonic',return_value=10.):self.assertIsNone(PreviewGate.blocked(n,10_000_000_000,require_velocity=False))
 def test_mapping_editor_capture_uses_same_one_second_pose_bound(self):
  from mapping_route_editor import MappingRouteEditor
  n,m=MotionPose().node(.9);editor=NS(snapshot=m,get_clock=n.get_clock)
  result=MappingRouteEditor.special_poses(editor);self.assertEqual(len(result),3)

class RouteSourceGap(unittest.TestCase):
 def test_via_crossing_is_not_lost_with_valid_800ms_source_gap(self):
  from mission_route import MissionRoute
  r=MissionRoute('room');r.add(dict(type='via',pose=[.1,0,0],pass_radius_m=.02));r.add(dict(type='stop',pose=[.4,0,0]));r.start()
  r.observe([0,0,0],[0,0,0],10.)
  self.assertTrue(r.observe([.2,0,0],[.2,0,0],10.8));self.assertEqual(r.index,1)
  r.observe([.21,0,0],[.21,0,0],10.9);r.finish('arrived_stopped');self.assertEqual(r.state,'completed')
if __name__=='__main__':unittest.main()
