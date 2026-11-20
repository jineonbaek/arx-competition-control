"""Recovery fault injection; private ROS197 and inert chassis only."""
import os,time,unittest
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import numpy as np,rclpy
import test_special_ros as f
from special_segments import StraightChain
from special_runtime import wait_special_ready
class SpecialAutoRecovery(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.SpecialROS.setUp;tearDown=f.SpecialROS.tearDown
 refresh=f.SpecialROS.refresh;add=f.SpecialROS.add;start_special=f.SpecialROS.start_special
 def chain(self,old_age=.3):
  self.start_special(.6);chain=StraightChain(self.position,self.position,self.n.waypoints.route.direct_points())
  chain.previous_sample=(self.n.now()-old_age,self.position.copy(),self.position.copy());return chain
 def test_matched_12cm_braking_motion_resumes_same_chain_without_advancing(self):
  chain=self.chain();ident=self.n.transport.current;index=self.n.waypoints.route.index
  self.position[0]=.12;self.refresh();mp,od,stamp=wait_special_ready(self.n,self.n.seen,chain)
  self.assertAlmostEqual(mp[0],.12);self.assertEqual(self.n.transport.current,ident);self.assertEqual(self.n.waypoints.route.index,index)
  self.assertEqual(chain.index,0);self.assertTrue(all(not any(c) for c in self.n.transport.calls))
 def test_map_only_jump_keeps_probing_until_alignment_returns_then_resumes(self):
  chain=self.chain();original=self.n.special_poses;began=time.monotonic();outputs=[];index=self.n.waypoints.route.index
  def poses():
   mp,od,st=original()
   if time.monotonic()-began<.4:mp=mp+np.array([.2,0,0])
   return mp,od,st
  self.n.special_poses=poses
  self.n.create_timer(.05,lambda:outputs.append(any(any(c) for c in self.n.transport.calls)))
  result=wait_special_ready(self.n,self.n.seen,chain)
  self.assertGreater(time.monotonic()-began,.65);self.assertAlmostEqual(result[0][0],0.)
  self.assertFalse(any(outputs));self.assertEqual(self.n.waypoints.route.index,index)
 def test_persistent_unproven_jump_never_moves_and_cancel_exits(self):
  chain=self.chain();original=self.n.special_poses;began=time.monotonic()
  def poses():
   mp,od,st=original();return mp+np.array([.2,0,0]),od,st
  self.n.special_poses=poses
  timer=self.n.create_timer(.7,lambda:self.n.cancel('cancelled'))
  with self.assertRaisesRegex(ValueError,'goal_changed'):wait_special_ready(self.n,self.n.seen,chain)
  self.n.destroy_timer(timer);self.assertTrue(all(not any(c) for c in self.n.transport.calls));self.assertIsNone(self.n.target)
 def test_joint_coordinate_jump_inside_stopped_proof_never_rebases_old_chain(self):
  chain=self.chain();self.n.create_timer(.3,lambda:self.position.__setitem__(0,1.))
  timer=self.n.create_timer(2.4,lambda:self.n.cancel('cancelled'))
  with self.assertRaisesRegex(ValueError,'goal_changed'):wait_special_ready(self.n,self.n.seen,chain)
  self.n.destroy_timer(timer);self.assertTrue(all(not any(c) for c in self.n.transport.calls))
  self.assertEqual(chain.goals[-1][0],.6)
 def test_recovery_phase_reports_reason_and_still_uses_new_source_proof(self):
  chain=self.chain();self.position[0]=.12;self.refresh();events=[];old=self.n.status
  def status(v):events.append(v);old(v)
  self.n.status=status;old_stamp=chain.previous_sample[0];_,_,stamp=wait_special_ready(self.n,self.n.seen,chain)
  self.assertGreater(stamp,old_stamp);self.assertTrue(any('special_inputs_restored' in s for s in events))
if __name__=='__main__':unittest.main()
