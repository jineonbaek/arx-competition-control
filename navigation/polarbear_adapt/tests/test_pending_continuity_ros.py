import copy,unittest,time
import numpy as np,rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from async_path_audit import AuditPending
import test_mission_ros as f
from tracking_handoff import live_prefix
from path_audit import AuditGrid
class PendingGrid(AuditGrid):
 def __init__(self,base):self.__dict__.update(base.__dict__)
 def along(self,route):raise AuditPending()
 def inspect(self,*args):
  from path_audit import AuditGrid
  return AuditGrid.inspect(self,*args)
class Continuity(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init());tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.MissionROS.setUp;tearDown=f.MissionROS.tearDown;refresh=f.MissionROS.refresh;add=f.MissionROS.add
 def prepare(self):
  self.add('stop',2.);self.n.waypoints.start(None);self.n.seen=self.n.seq
  self.previous=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
  self.n.route=self.previous.copy();self.n.route_time=self.n.now();self.n.active=True
  self.n.grids={k:(st,PendingGrid(g)) for k,(st,g) in self.n.grids.items()}
 def message(self):
  m=Path();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg()
  for a in self.previous:
   p=PoseStamped();p.pose.position.x,p.pose.position.y=map(float,a[:2]);p.pose.orientation.w=1.;m.poses.append(p)
  return m
 def test_replacement_pending_does_not_brake_or_enter_recovery(self):
  self.prepare();self.n.receive(self.message());self.assertIsNotNone(self.n.route);np.testing.assert_allclose(self.n.route,self.previous)
  self.assertIsNone(self.n.recovery);self.assertEqual(self.n.transport.calls,[]);self.assertTrue(self.n.active)
 def test_initial_pending_never_creates_a_running_route(self):
  self.prepare();self.n.active=False;self.n.route=None;self.n.receive(self.message());self.assertIsNone(self.n.route);self.assertEqual(self.n.transport.calls,[])
 def test_prefix_checks_full_body_and_stops_future_obstacle(self):
  self.prepare();mp,op,_=self.n.motion_poses();g=self.n.grids['map'][1];l=self.n.grids['odom'][1]
  self.assertIsNotNone(live_prefix(self.previous,mp,op,g,l,[]))
  point=[self.n.poly[:,0].max()+.25,0.];col,row=np.floor((point-g.origin)/g.res).astype(int);g.grid[row,col]=100
  with self.assertRaisesRegex(ValueError,'map_sweep_blocked'):live_prefix(self.previous,mp,op,g,l,[])
 def test_unknown_is_not_turned_into_free_by_pending_policy(self):
  self.prepare();mp,op,_=self.n.motion_poses();g=self.n.grids['map'][1];l=self.n.grids['odom'][1]
  point=[self.n.poly[:,0].max()+.25,0.];col,row=np.floor((point-g.origin)/g.res).astype(int);g.grid[row,col]=-1
  with self.assertRaises(ValueError):live_prefix(self.previous,mp,op,g,l,[])
 def test_stale_stream_is_still_rejected_while_candidate_pending(self):
  self.prepare();self.n.receive(self.message());self.n.sensor_received['/localization/odometry']-=1.
  with self.assertRaisesRegex(ValueError,'sensor_or_costmap_stale'):self.n.checked()
 def test_same_goal_evidence_expires_and_foreign_target_does_not_refresh_it(self):
  self.prepare();m=self.message();self.n.receive(m);accepted=self.n.last_candidate_stamp
  m=self.message();m.poses[-1].pose.position.x=8.;self.n.receive(m);self.assertEqual(self.n.last_candidate_stamp,accepted)
 def test_departed_old_path_requests_replan_instead_of_losing_goal(self):
  self.prepare();mp=np.array([0.,.25,0.]);g=self.n.grids['map'][1];l=self.n.grids['odom'][1]
  with self.assertRaisesRegex(ValueError,'replan_blocked'):live_prefix(self.previous,mp,mp,g,l,[])
 def test_cancel_prevents_late_pending_candidate_from_restoring_route(self):
  self.prepare();self.n.receive(self.message());self.n.cancel('cancelled');self.n.retry_audit_path();self.assertIsNone(self.n.route);self.assertIsNone(self.n.target)
