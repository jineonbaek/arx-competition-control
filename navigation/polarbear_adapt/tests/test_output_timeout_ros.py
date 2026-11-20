"""Real goal loop on ROS197; expired output injected into an inert transport."""
import os,time,unittest
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import numpy as np,rclpy
import test_retained_faults_ros as normal
import test_special_ros as special

class Fault:
 def inject(self,n,delay=0.):
  self.old=n.transport.current;self.expired=False;self.positive=[];submit=n.transport.submit
  def send(m):
   moving=m['type']!='chassis.end_output' and any(m[k] for k in ('x','y','omega'))
   old=m['generation']==self.old.generation
   if moving and not self.expired:
    self.expired=True
    if delay:time.sleep(delay)
    raise ValueError('底盘指令已过期或早于维护结束')
   if m['type']!='chassis.end_output' and old and self.expired:raise ValueError('底盘任务已结束或指令序号已过期')
   if moving:self.positive.append(dict(m))
   return submit(m)
  n.transport.submit=send

class NormalTimeout(unittest.TestCase,Fault):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=normal.RetainedFaults.setUp;tearDown=normal.RetainedFaults.tearDown
 refresh=normal.RetainedFaults.refresh;add=normal.RetainedFaults.add
 def test_over_one_second_timeout_recovers_same_goal_with_new_scoped_generation(self):
  normal.RetainedFaults.prepare(self,'chassis_timeout');self.inject(self.n,1.05);target=self.n.target.copy();epoch=self.n.seq
  with self.n.transport.task_scope(self.old):self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertTrue(self.expired);self.assertFalse(self.paused);self.assertGreater(self.n.transport.current.generation,self.old.generation)
  self.assertTrue(self.positive);self.assertTrue(all(p['generation']>self.old.generation for p in self.positive));np.testing.assert_allclose(self.n.target,target);self.assertEqual(self.n.seq,epoch);self.assertEqual(self.n.waypoints.route.index,0)
 def test_cancel_in_timeout_recovery_never_starts_new_output(self):
  normal.RetainedFaults.prepare(self,'chassis_timeout');self.inject(self.n)
  timer=self.n.create_timer(.4,lambda:self.n.cancel('cancelled'))
  try:
   with self.assertRaisesRegex(ValueError,'goal_changed'):
    with self.n.transport.task_scope(self.old):self.n.run_goal()
  finally:self.n.destroy_timer(timer)
  self.assertEqual(self.positive,[]);self.assertFalse(self.n.waypoints.route.running);self.assertIsNone(self.n.target)

class SpecialTimeout(unittest.TestCase,Fault):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=special.SpecialROS.setUp;tearDown=special.SpecialROS.tearDown
 refresh=special.SpecialROS.refresh;add=special.SpecialROS.add;start_special=special.SpecialROS.start_special
 def test_special_timeout_preserves_chain_and_completes_automatically(self):
  self.start_special(.35);self.simulate=True;self.inject(self.n);epoch=self.n.seq;target=self.n.target.copy()
  with self.n.transport.task_scope(self.old):self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertTrue(self.expired);self.assertTrue(self.positive);self.assertEqual(self.n.seq,epoch);np.testing.assert_allclose(self.n.target,target);self.assertLessEqual(abs(self.position[0]-.35),.10)
  self.assertEqual(self.n.special_session['epoch'],epoch);self.assertEqual(self.n.waypoints.route.index,1)

class CancelOnReload(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=normal.f.MissionROS.setUp;tearDown=normal.f.MissionROS.tearDown
 refresh=normal.f.MissionROS.refresh;add=normal.f.MissionROS.add;worker=normal.f.MissionROS.worker
 def test_relaying_existing_cancel_after_reload_proves_stop_without_costmap_or_motion(self):
  from task_handoff import HandoffHub
  h,t=normal.f.MissionROS.arrive_task(self);restarted=HandoffHub(h.path)
  self.assertEqual(restarted.active['status'],'needs_attention')
  self.n.waypoints.hub=restarted;self.n.waypoints.runtime.hub=restarted
  self.n.waypoints.clear(None);self.n.grids={}
  deadline=time.monotonic()+3.
  while restarted.active['status']!='acknowledged' and time.monotonic()<deadline:
   self.n.spin(.05);self.n.grids={}
  self.assertEqual(restarted.active['status'],'acknowledged');self.assertEqual(restarted.active['outcome'],'cancelled')
  self.assertFalse(self.n.waypoints.route.running);self.assertIsNone(self.n.target);self.assertTrue(all(not any(c) for c in self.n.transport.calls))
if __name__=='__main__':unittest.main()
