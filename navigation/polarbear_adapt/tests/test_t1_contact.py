"""Virtual time, real short-TTL transport serialization; no hardware sessions."""
import io,json,math,unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from chassis_command import WebChassis
try:import t1_contact_runtime as contact
except ModuleNotFoundError:contact=None

class Clock:
 def __init__(self):self.t=100.
 def now(self):return self.t
class Driver:
 def __init__(self,kind='task',tag='point_1'):
  self.clock=Clock();self.seq=self.seen=1;self.target=np.zeros(3);self.events=[];self.active=False;self.cancel_at=None;self.stale_until=0.;self.gate=True;self.fail_send=False
  p=dict(id='t1',type=kind,point_id=tag,pose=[0.,0.,0.])
  self.waypoints=NS(route=NS(running=True,state='running',target_index=0,points=[p],route_id='route',mission_id='mission'),blocking=lambda:False,show=lambda s:self.events.append(('show',s)))
  self.payloads=[];self.transport=WebChassis(submit=self.capture,wall=self.clock.now);self.transport.native_sdk=self.transport.sdk_units=True
  self.transport.client=NS(open=self.console);self.intent=NS(publish=lambda m:None)
  self.pending_fault=None;self._special_execution=False;self.recovery=None;self.terminal_correction=False;self.mppi_requests=[];self.mppi_handle=None
  self.escape=NS(maybe_run=lambda e:False,clear=lambda:None)
 def capture(self,m):
  self.payloads.append(dict(m));self.events.append(('payload',dict(m)))
  if self.fail_send and m['type']!='chassis.end_output' and any(m[k] for k in ('x','y','omega')):raise TimeoutError('lost ack')
  return dict(ok=True)
 def console(self,*a,**kw):return io.BytesIO(json.dumps(dict(execution_enabled=True,nav_sdk_ready=True,body=dict(fresh=True),chassis_active=False,capabilities=dict(chassis=''))).encode())
 def now(self):return self.clock.t
 def special_poses(self):
  if self.clock.t<self.stale_until:raise ValueError('special_pose_stale')
  return np.zeros(3),np.zeros(3),self.clock.t
 def validate_motion_proof(self,stamp):
  if self.clock.t-stamp>.65:raise ValueError('motion_pose_stale_or_invalid')
 def current_intent(self,e):
  if e!=self.seq or self.target is None:raise ValueError('goal_changed')
  if not self.gate:raise ValueError('execution_gate_closed')
 def same_intent(self,e):self.current_intent(e)
 def wait_controller_quiet(self,e):self.current_intent(e)
 def recovery_console(self):pass
 def track_route(self):return 'arrived_stopped'
 def status(self,s):self.events.append(('status',s))
 def spin(self,dt):
  self.clock.t+=dt
  if self.cancel_at and self.clock.t>=self.cancel_at:self.seq+=1;self.target=None
 def brake(self):
  if self.active:self.transport.send((0.,0.,0.));self.active=False
 def soft_brake(self):self.brake();return True
 def end_output(self,handle=None):self.transport.end_output(handle);self.active=False;return True
 def pulses(self):return [m for m in self.payloads if m['type']!='chassis.end_output' and any(m[k] for k in ('x','y','omega'))]

class ContactTests(unittest.TestCase):
 def setUp(self):self.assertIsNotNone(contact,'T1 contact stage is missing')
 def run_contact(self,n):
  with patch('t1_contact_runtime.time.monotonic',n.clock.now),patch('t1_contact_runtime.rclpy.ok',return_value=True):
   return contact.finish_arrival(n,n.seen,'arrived_stopped')
 def test_one_forward_pulse_then_immediate_handoff_after_output_end(self):
  n=Driver();self.assertEqual(self.run_contact(n),'arrived_stopped');ps=n.pulses();self.assertGreater(len(ps),5)
  self.assertEqual(tuple(ps[0][k] for k in ('x','y','omega')),(.225,0.,0.));self.assertAlmostEqual(ps[0]['ttl_s'],.2)
  self.assertAlmostEqual(n.now(),102.);self.assertFalse(n.active);self.assertTrue(n.t1_contact_session['complete'])
  self.assertEqual(n.payloads[-1]['type'],'chassis.end_output')
  self.assertTrue(all(0<p['ttl_s']<=.2 for p in ps))
  self.assertTrue(all(p['issued_at']+p['ttl_s']<=102.+1e-9 for p in ps))
  self.assertTrue(all(tuple(p[k] for k in ('x','y','omega'))==(.225,0.,0.) for p in ps))
 def test_completed_stage_cannot_pulse_again(self):
  n=Driver();self.run_contact(n);count=len(n.pulses());self.run_contact(n);self.assertEqual(len(n.pulses()),count)
 def test_cancel_during_pulse_closes_output_without_handoff(self):
  n=Driver();n.cancel_at=100.08
  with self.assertRaisesRegex(ValueError,'goal_changed'):self.run_contact(n)
  self.assertFalse(n.t1_contact_session['complete']);self.assertFalse(n.active);self.assertEqual(n.payloads[-1]['type'],'chassis.end_output')
 def test_lost_ack_latches_attempt_before_send_and_never_replays(self):
  n=Driver();n.fail_send=True
  with self.assertRaises(TimeoutError):self.run_contact(n)
  self.assertFalse(n.active);n.fail_send=False
  self.run_contact(n);self.assertEqual(len(n.pulses()),1)
 def test_stale_post_pulse_inputs_delay_handoff_without_replay(self):
  n=Driver();old=n.end_output
  def end(handle=None):old(handle);n.stale_until=max(n.stale_until,102.8);return True
  n.end_output=end
  with self.assertRaisesRegex(ValueError,'special_pose_stale'):self.run_contact(n)
  count=len(n.pulses());n.clock.t=102.9;n.t1_contact_session['chain'].previous_sample=(n.now(),np.zeros(3),np.zeros(3));self.run_contact(n);self.assertEqual(len(n.pulses()),count);self.assertAlmostEqual(n.now(),102.9)
 def test_closed_gate_never_sends_or_hands_off(self):
  n=Driver();n.gate=False
  with self.assertRaisesRegex(ValueError,'execution_gate_closed'):self.run_contact(n)
  self.assertEqual(n.pulses(),[])
 def test_t2_test_marker_and_non_arrival_do_not_push(self):
  for kind,tag,result in [('task','point_2','arrived_stopped'),('special','point_1','arrived_stopped'),('stop','point_1','arrived_stopped'),('task','point_1','cancelled')]:
   n=Driver(kind,tag)
   with patch('t1_contact_runtime.time.monotonic',n.clock.now):self.assertEqual(contact.finish_arrival(n,1,result),result)
   self.assertEqual(n.pulses(),[])
 def test_no_second_stationary_window_after_forward_output_ends(self):
  n=Driver();original=n.special_poses
  def poses():
   mp,od,s=original();mp[0]=.03*min(s-100.,.7);return mp,od,s
  n.special_poses=poses;self.run_contact(n);self.assertAlmostEqual(n.now(),102.)
 def test_common_goal_exit_covers_normal_and_special_t1(self):
  from mppi_goal_drive import MppiDriver
  for special in (False,True):
   n=Driver();n._special_execution=special
   with patch('special_runtime.run_special',return_value='arrived_stopped'),patch('t1_contact_runtime.time.monotonic',n.clock.now),patch('t1_contact_runtime.rclpy.ok',return_value=True):
    self.assertEqual(MppiDriver.run_goal(n),'arrived_stopped')
   self.assertGreater(len(n.pulses()),5)
 def test_goal_retry_resumes_contact_instead_of_driving_back_to_goal(self):
  from mppi_goal_drive import MppiDriver
  n=Driver();n.fail_send=True
  with self.assertRaises(TimeoutError):self.run_contact(n)
  n.fail_send=False;n.track_route=lambda:(_ for _ in ()).throw(AssertionError('approach replayed'))
  with patch('t1_contact_runtime.time.monotonic',n.clock.now),patch('t1_contact_runtime.rclpy.ok',return_value=True):self.assertEqual(MppiDriver.run_goal(n),'arrived_stopped')
  self.assertEqual(len(n.pulses()),1)

 def test_contact_timeout_recovers_with_its_own_chain_without_ordinary_replanning(self):
  from mppi_goal_drive import MppiDriver
  n=Driver();n.fail_send=True
  n.recover_pending_fault=lambda e:(_ for _ in ()).throw(AssertionError('ordinary replan during contact'))
  waits=[]
  def recovered(driver,epoch,chain=None):
   waits.append(1)
   self.assertIs(chain,n.t1_contact_session['chain']);n.fail_send=False;n.clock.t+=1.1
   chain.previous_sample=(n.now(),np.zeros(3),np.zeros(3));return n.special_poses()
  with patch('special_runtime.wait_special_ready',side_effect=recovered),patch('t1_contact_runtime.time.monotonic',n.clock.now),patch('t1_contact_runtime.rclpy.ok',return_value=True):
   self.assertEqual(MppiDriver.run_goal(n),'arrived_stopped')
  self.assertEqual(len(n.pulses()),1);self.assertEqual(len(waits),1)

 def test_non_native_transport_never_pushes(self):
  n=Driver();n.transport.native_sdk=False
  with self.assertRaisesRegex(ValueError,'special_native_sdk_required'):self.run_contact(n)
  self.assertEqual(n.pulses(),[])
 def test_foreign_active_output_after_pulse_blocks_handoff(self):
  n=Driver();original=n.transport.client.open
  def busy(*a,**kw):
   s=json.loads(original().read());s['chassis_active']=bool(n.pulses());return io.BytesIO(json.dumps(s).encode())
  n.transport.client.open=busy
  with self.assertRaisesRegex(ValueError,'special_console_unready'):self.run_contact(n)
  self.assertFalse(n.t1_contact_session['complete']);self.assertGreater(len(n.pulses()),5)
 def test_unconfirmed_end_blocks_handoff_and_retry_does_not_push(self):
  n=Driver();original=n.end_output
  def uncertain(handle=None):original(handle);return False
  n.end_output=uncertain
  with self.assertRaisesRegex(ValueError,'previous_output_end_unconfirmed'):self.run_contact(n)
  self.assertFalse(n.t1_contact_session['complete']);count=len(n.pulses());n.end_output=original
  self.run_contact(n);self.assertEqual(len(n.pulses()),count)
 def test_cleanup_does_not_end_a_newer_goal_generation(self):
  n=Driver();original=n.spin;new=[]
  def spin(dt):
   original(dt)
   if n.now()>=100.08 and not new:
    n.seq+=1;n.target=np.ones(3);new.append(n.transport.begin_task('new-goal'))
  n.spin=spin
  with self.assertRaisesRegex(ValueError,'goal_changed'):self.run_contact(n)
  self.assertNotIn(new[0],n.transport.ended);self.assertLess(n.now(),100.1)

 def test_busy_before_contact_never_sends_a_pulse(self):
  n=Driver();original=n.transport.client.open
  def busy(*a,**kw):
   s=json.loads(original().read());s['chassis_active']=True;return io.BytesIO(json.dumps(s).encode())
  n.transport.client.open=busy
  with self.assertRaisesRegex(ValueError,'special_console_unready'):self.run_contact(n)
  self.assertEqual(n.pulses(),[])

 def test_actual_ros_timeout_exception_is_recoverable_without_replaying(self):
  n=Driver();original=n.transport.submit
  def lost(m):
   if m['type']!='chassis.end_output' and any(m[k] for k in ('x','y','omega')):
    n.payloads.append(dict(m));raise ValueError('ROS chassis command acknowledgement timeout')
   return original(m)
  n.transport.submit=lost
  with self.assertRaises(TimeoutError):self.run_contact(n)
  n.transport.submit=original;self.run_contact(n);self.assertEqual(len(n.pulses()),1)

 def test_midway_stale_input_stops_refresh_and_never_restarts(self):
  n=Driver();original=n.spin
  def spin(dt):
   original(dt)
   if n.now()>=100.5:n.stale_until=103.
  n.spin=spin
  with self.assertRaisesRegex(ValueError,'special_pose_stale'):self.run_contact(n)
  count=len(n.pulses());self.assertGreater(count,1);self.assertLess(n.now(),100.6)
  n.clock.t=103.1;n.t1_contact_session['chain'].previous_sample=(n.now(),np.zeros(3),np.zeros(3))
  self.run_contact(n);self.assertEqual(len(n.pulses()),count)

class TTLTests(unittest.TestCase):
 def test_short_ttl_is_original_not_renewed_after_transport_delay(self):
  clock=Clock();seen=[]
  def submit(m):seen.append(dict(m));clock.t+=.25;return dict(ok=True)
  bridge=WebChassis(submit=submit,wall=clock.now);bridge.sdk_units=bridge.native_sdk=True
  bridge.send((.225,0,0),ttl_s=.2)
  self.assertEqual(seen[0]['issued_at'],100.);self.assertEqual(seen[0]['ttl_s'],.2);self.assertLess(seen[0]['issued_at']+seen[0]['ttl_s'],clock.now())
 def test_invalid_ttl_never_reaches_transport(self):
  seen=[];bridge=WebChassis(submit=lambda m:seen.append(m) or dict(ok=True))
  for duration in [0.,-.1,.301,float('nan'),float('inf')]:
   with self.assertRaises(ValueError):bridge.send((.225,0,0),ttl_s=duration)
  self.assertEqual(seen,[])
 def test_ordinary_commands_keep_existing_ttl(self):
  seen=[];bridge=WebChassis(submit=lambda m:seen.append(m) or dict(ok=True));bridge.send((0.,0.,0.));self.assertEqual(seen[0]['ttl_s'],.3)
if __name__=='__main__':unittest.main()
