"""Injected time and real stopped-proof loops, all chassis submissions inert."""
import unittest,io,json
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import chassis_command as bridge
from arx_chassis.core import Core,Rejected
from test_recovery_source_cadence import VirtualDriver
from special_segments import StraightChain
try:import output_recovery as recovery
except ModuleNotFoundError:recovery=None

class Driver(VirtualDriver):
 def __init__(self,**kw):
  super().__init__(**kw);self.payloads=[];self.target=np.array([1.,0.,0.]);self.seen=1;self._special_execution=False;self.active=True;self.pending_fault=None;self.ready_file=None;self.end_after=10.;self.ends=[];self.replans=[]
  self.transport=bridge.WebChassis(submit=self.capture,wall=lambda:self.t);self.transport.sdk_units=self.transport.native_sdk=True;self.transport.client=NS(open=self.console)
  self.original=self.transport.current;self.waypoints.route=NS(index=2,mission_id='mission',route_id='route',running=True)
 def capture(self,m):self.payloads.append(dict(m));return dict(ok=True)
 def same_intent(self,e):
  self.current_intent(e)
  if self.target is None:raise ValueError('goal_changed')
 def end_output(self,handle=None):
  handle=self.transport.current if handle is None else handle;self.ends.append((self.t,handle))
  if self.t<self.end_after:return False
  self.transport.end_output(handle);self.active=False;return True
 def wait_controller_quiet(self,e):self.same_intent(e)
 def hold_for_replan(self,reason):self.replans.append(reason)
 def spin(self,dt):
  super().spin(dt)
  if getattr(self,'cancel_at',None) and self.t>=self.cancel_at:self.seq+=1;self.target=None

class Leases(unittest.TestCase):
 def test_navigation_default_one_second_and_contact_explicit_point_two(self):
  captured=[];c=bridge.WebChassis(submit=lambda p:captured.append(p) or dict(ok=True));c.native_sdk=c.sdk_units=True
  c.send((.1,0.,0.));c.send((.225,0.,0.),ttl_s=.2)
  self.assertEqual([p['ttl_s'] for p in captured],[1.,.2])
 def test_gamepad_moves_and_end_keep_original_short_lease(self):
  captured=[];c=bridge.WebChassis(submit=lambda p:captured.append(p) or dict(ok=True));c.native_sdk=c.sdk_units=True;c.command_kind='chassis.gamepad_sdk'
  c.begin_task();c.send((.1,0.,0.));self.assertEqual([p['ttl_s'] for p in captured],[.3,.3])
 def test_scope_rebinds_only_after_original_end_ack(self):
  n=Driver();c=n.transport;old=c.current
  self.assertTrue(hasattr(c,'renew_task'),'confirmed-end renewal is missing')
  with c.task_scope(old):
   with self.assertRaises(RuntimeError):c.renew_task(old)
   c.end_output(old);new=c.renew_task(old);c.send((.1,0.,0.))
   self.assertNotEqual(new,old);self.assertEqual(n.payloads[-1]['generation'],new.generation)
  self.assertFalse(any(p.get('x') for p in n.payloads[:-1]));self.assertEqual(c.current,new)
 def test_faults_classified_without_hiding_unrelated_rejections(self):
  self.assertTrue(hasattr(bridge,'output_unavailable'),'output failure classification missing')
  for text in ['底盘指令已过期或早于维护结束','底盘指令在输出前已过期','底盘任务已结束或指令序号已过期','ROS chassis command acknowledgement timeout','driver not ready or scheduler not bound']:
   self.assertTrue(bridge.output_unavailable(ValueError(text)),text)
  for text in ['SDK limit exceeded','source lease exceeded','goal_changed','execution_gate_closed','task_worker_identity_changed']:
   self.assertFalse(bridge.output_unavailable(ValueError(text)),text)

class SchedulerLease(unittest.TestCase):
 def setUp(self):
  self.t=100.;self.c=Core(lambda x:None,wall=lambda:self.t,clock=lambda:self.t);self.c.driver=('fake',True,self.t);self.c.bound_instance=self.c.instance
 def command(self,profile,ttl):
  client=self.c.register(profile);return self.c.command(dict(client_id=client,scheduler_instance=self.c.instance,operation='move',kind='chassis.nav_sdk' if profile=='navigation' else 'chassis.gamepad_sdk',source_id=profile,task_id='goal',generation=1,seq=1,issued_at=self.t,ttl_s=ttl,sdk=[.1,0.,0.]))
 def test_navigation_accepts_one_second(self):self.command('navigation',1.)
 def test_navigation_rejects_over_one_second(self):
  with self.assertRaises(Rejected):self.command('navigation',1.001)
 def test_gamepad_keeps_point_three_limit(self):
  with self.assertRaises(Rejected):self.command('gamepad',1.)

class Recovery(unittest.TestCase):
 def setUp(self):self.assertIsNotNone(recovery,'output recovery is missing')
 def run_recovery(self,n):
  fake=NS(monotonic=lambda:n.t)
  with patch('special_runtime.time',fake),patch('recovery_budget.time',fake),patch('output_recovery.time',fake),patch('special_runtime.rclpy.ok',return_value=True):
   return recovery.recover_output(n,1)
 def test_waits_for_new_stopped_samples_then_renews_same_goal_and_scope(self):
  n=Driver();target=n.target.copy();old=n.transport.current
  with n.transport.task_scope(old):
   self.run_recovery(n);n.transport.send((.1,0.,0.))
   self.assertEqual(n.payloads[-1]['generation'],n.transport.current.generation)
  self.assertNotEqual(n.transport.current,old);self.assertIn(old,n.transport.ended);self.assertGreaterEqual(n.t,11.35)
  np.testing.assert_array_equal(n.target,target);self.assertEqual(n.seq,1);self.assertEqual(n.waypoints.route.index,2)
  self.assertTrue(all(not p.get('x') and not p.get('y') and not p.get('omega') for p in n.payloads[:-1]));self.assertEqual(len(n.replans),1)
 def test_unconfirmed_end_cannot_allocate_a_new_generation(self):
  n=Driver();n.end_after=11.;old=n.transport.current;observed=[];spin=n.spin
  def watch(dt):
   observed.append((n.t,n.transport.current));spin(dt)
  n.spin=watch;self.run_recovery(n)
  self.assertTrue(all(h==old for t,h in observed if t<11.));self.assertGreaterEqual(n.t,12.35)
 def test_frozen_pose_never_renews_and_cancel_exits(self):
  n=Driver(freeze=.4);old=n.transport.current
  with self.assertRaisesRegex(ValueError,'goal_changed'):self.run_recovery(n)
  self.assertEqual(n.transport.current,old);self.assertTrue(all(not p.get('x') for p in n.payloads))
 def test_cancel_during_proof_never_revives_goal(self):
  n=Driver();n.cancel_at=10.4;old=n.transport.current
  with self.assertRaisesRegex(ValueError,'goal_changed'):self.run_recovery(n)
  self.assertEqual(n.transport.current,old);self.assertIsNone(n.target);self.assertEqual(n.replans,[])
 def test_special_segment_reuses_progress_chain(self):
  n=Driver();n._special_execution=True;chain=StraightChain(np.zeros(3),np.zeros(3),[dict(id='special',pose=[1.,0.,0.])]);chain.previous_sample=(n.t,np.zeros(3),np.zeros(3));n.special_session=dict(epoch=1,chain=chain,stage='backoff')
  self.run_recovery(n);self.assertIs(n.special_session['chain'],chain);self.assertEqual(n.special_session['stage'],'backoff');self.assertEqual(n.replans,[]);self.assertGreater(chain.previous_sample[0],10.)
 def test_attempted_t1_contact_is_never_renewed_or_replayed(self):
  n=Driver();old=n.transport.current;n.t1_contact_session=dict(epoch=1,attempted=True,chain=None)
  self.run_recovery(n);self.assertEqual(n.transport.current,old);self.assertIn(old,n.transport.ended);self.assertEqual(n.replans,[]);self.assertTrue(all(not p.get('x') for p in n.payloads))
if __name__=='__main__':unittest.main()
