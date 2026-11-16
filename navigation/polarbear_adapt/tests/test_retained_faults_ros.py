"""Real MPPI goal loop; private ROS, inert transport and native action edge."""
import time,unittest
from concurrent.futures import Future
from types import SimpleNamespace
import numpy as np,rclpy
from std_msgs.msg import Bool,String
from geometry_msgs.msg import TwistStamped
import test_mission_ros as f
from unittest.mock import patch
from mppi_trial.session_policy import OwnedGoals

class Handle:
    accepted=True
    def __init__(self,code=0,complete=False):
        self.status=6 if code else (4 if complete else 2);self.result=Future()
        if code or complete:self.finish(code)
    def finish(self,code=0):
        self.status=6 if code else 4
        if not self.result.done():self.result.set_result(SimpleNamespace(status=self.status,result=SimpleNamespace(error_code=code,error_msg='fixture')))
    def get_result_async(self):return self.result
    def cancel_goal_async(self):self.status=5;return Future()

class RetainedFaults(unittest.TestCase):
    setUpClass=classmethod(lambda cls:rclpy.init())
    tearDownClass=classmethod(lambda cls:rclpy.shutdown())
    setUp=f.MissionROS.setUp;tearDown=f.MissionROS.tearDown
    refresh=f.MissionROS.refresh;add=f.MissionROS.add

    def prepare(self,fault,*,manual=False,late=False,worker=False,old_unknown=False):
        n=self.n
        if worker:
            self.hub,self.worker_token=f.MissionROS.worker(self)
            self.worker_registration={k:self.hub.worker[k] for k in ('worker_id','instance_id','point_ids')}
            self.add('stop',2.);self.add('task',3.,'point_1') # Native MPPI goal with a future task requiring the worker.
        else:self.add('stop',2.)
        n.waypoints.start(None);n.seen=n.seq
        self.identity=n.transport.current;self.target=n.target.copy();self.events=[];self.sends=[];self.paused=False;self.forbidden_worker_outputs=[]
        cap=n.transport.submit
        def capture(m):
            if worker and m['type']!='chassis.end_output' and any((m['x'],m['y'],m['omega'])) and (not self.hub.available_points() or self.hub.worker['instance_id']!=self.worker_registration['instance_id']):
                self.forbidden_worker_outputs.append(m)
                raise ValueError('unsafe_workerless_send')
            self.sends.append(dict(m));return cap(m)
        n.transport.submit=capture
        status=n.status
        def log(v):
            self.events.append(v);status(v)
            if v.startswith('paused_navigation'):
                self.paused=True;self.pause_at=time.monotonic();self.pause_count=len(self.sends)
        n.status=log;outer=self;self.started=time.monotonic();self.future=Future();self.late_handle=Handle()
        class Client:
            requests=0;ready_calls=0;callback=None;handle=None
            def wait_for_server(self,**kw):
                self.ready_calls+=1
                return not (fault=='mppi_unavailable' and self.ready_calls==1)
            def send_goal_async(self,request,feedback_callback):
                if old_unknown and not outer.future.done():raise ValueError('unknown_old_action_overlap')
                self.requests+=1;self.callback=feedback_callback
                if late and self.requests==1:return outer.future
                self.handle=Handle(code=106 if fault=='mppi_failed' and self.requests==1 else 0)
                if fault=='mppi_rejected_path' and self.requests==1:self.handle.accepted=False
                v=Future();v.set_result(self.handle);return v
        self.client=Client();n.mppi_client=self.client
        self.n.destroy_timer(self.timer);self.phase_sends=0;self.guard_once=False
        original_prefix=n.validate_active_prefix
        def prefix(*args):
            result=original_prefix(*args)
            if fault in ('control_cycle_overrun','invalid_sdk_effort','goal_timeout') and not self.guard_once:
                self.guard_once=True;raise ValueError(fault)
            return result
        n.validate_active_prefix=prefix
        def feed():
            now=time.monotonic()
            if now-self.started>14:raise AssertionError('retained fault fixture timed out')
            self.refresh()
            if worker and not manual and now-self.started>.5:
                self.hub.register(self.worker_registration);self.hub.heartbeat(self.worker_token)
            n.route=np.array([self.position.copy(),[1.,0.,0.],[2.,0.,0.]]) if self.position[0]<1. else np.array([self.position.copy(),[2.,0.,0.]])
            n.route_time=n.now()
            sdk=TwistStamped();sdk.header.stamp=n.get_clock().now().to_msg();sdk.twist.linear.x=.08
            n.mppi_received.update(inputs=(now,Bool(data=True)),sdk=(now,sdk),status=(now,String(data='preview_ready')))
            if self.client.callback:self.client.callback(None)
            moving=sum(m['type']!='chassis.end_output' and any((m['x'],m['y'],m['omega'])) for m in self.sends)
            if late and not self.future.done():
                self.assertLessEqual(self.client.requests,1,'new action before old request became terminal')
                if now-self.started>2.35:self.future.set_result(self.late_handle)
            if old_unknown and not self.future.done() and now-self.started>.35:self.future.set_result(self.late_handle)
            if moving>=2 and self.client.handle is not None and not manual:
                self.position[:]=[2.,0.,0.];self.refresh();self.client.handle.finish()
            if self.paused and now-self.pause_at>.25:
                self.assertEqual(len(self.sends),self.pause_count);self.assertEqual(n.transport.current,self.identity)
                np.testing.assert_allclose(n.target,self.target);self.assertTrue(n.waypoints.route.running)
                n.cancel('cancelled')
        self.timer=n.create_timer(.015,feed);feed()

    def recovered(self,fault,**kw):
        self.prepare(fault,**kw)
        with self.n.transport.task_scope(self.identity):self.assertEqual(self.n.run_goal(),'arrived_stopped')
        self.assertEqual(self.n.transport.current,self.identity);np.testing.assert_allclose(self.n.target,self.target)
        self.assertFalse(any(m['type']=='chassis.end_output' for m in self.sends))
    def test_server_unavailable_retries_same_goal(self):self.recovered('mppi_unavailable')
    def test_rejected_path_reaudits_before_new_action(self):self.recovered('mppi_rejected_path')
    def test_native_no_valid_control_replans_instead_of_failing(self):self.recovered('mppi_failed')
    def test_late_acceptance_is_quarantined_before_retry(self):
        self.recovered('mppi_accept_timeout',late=True);self.assertEqual(self.late_handle.status,5)
    def test_cycle_overrun_parks_then_reaudits(self):self.recovered('control_cycle_overrun')
    def test_invalid_effort_keeps_mission_parked_until_cancel(self):
        self.prepare('invalid_sdk_effort',manual=True)
        with self.assertRaisesRegex(ValueError,'goal_changed'):
            with self.n.transport.task_scope(self.identity):self.n.run_goal()
        self.assertTrue(self.paused)
    def test_global_timeout_keeps_mission_parked_until_cancel(self):
        self.prepare('goal_timeout',manual=True)
        with self.assertRaisesRegex(ValueError,'goal_changed'):
            with self.n.transport.task_scope(self.identity):self.n.run_goal()
        self.assertTrue(self.paused)
    def test_uncertain_async_path_withdrawal_keeps_task_for_manual_review(self):
        self.prepare('async_withdrawal',manual=True)
        self.n.retain_valid_route=lambda _:False
        self.n.retention_failure='unsafe_retention_fixture'
        seq=self.n.seq
        self.n.cancel('path_withdrawn')
        self.assertIsNotNone(self.n.target)
        self.assertEqual(self.n.seq,seq)
        with self.assertRaisesRegex(ValueError,'goal_changed'):
            with self.n.transport.task_scope(self.identity):self.n.run_goal()
        self.assertTrue(self.paused)
    def test_task_worker_lease_gap_brakes_then_resumes_same_task(self):
        self.prepare('worker_gap',worker=True);self.hub.worker['seen']-=3.
        with self.n.transport.task_scope(self.identity):self.assertEqual(self.n.run_goal(),'arrived_stopped')
        self.assertEqual(self.n.transport.current,self.identity);self.assertIsNone(self.hub.active)
    def test_initial_console_timeout_is_retried_inside_serve(self):
        self.prepare('console_start');original=self.n.transport.open;reads=[]
        def offline_once(*a,**kw):
            reads.append(1)
            if len(reads)==1:raise TimeoutError('startup console unavailable')
            return original(*a,**kw)
        self.n.transport.open=offline_once
        with patch('goal_drive.rclpy.ok',side_effect=lambda:self.n.target is not None):self.n.serve()
        self.assertTrue(any(m['type']!='chassis.end_output' and any((m['x'],m['y'],m['omega'])) for m in self.sends))
        self.assertGreater(len(reads),1)
    def test_new_semantic_goal_waits_for_old_unknown_request(self):
        self.prepare('old_request',old_unknown=True)
        owned=OwnedGoals();owned.close();self.future.add_done_callback(lambda f:owned.remember(f.result()))
        self.n.mppi_requests=[self.future]
        with self.n.transport.task_scope(self.identity):self.assertEqual(self.n.run_goal(),'arrived_stopped')
        self.assertEqual(self.late_handle.status,5)
    def test_unknown_native_status_is_not_a_quiescence_proof(self):
        self.prepare('unknown_status');handle=Handle();handle.status=0;future=Future();future.set_result(handle)
        self.n.mppi_requests=[future];began=time.monotonic()
        timer=self.n.create_timer(.3,lambda:setattr(handle,'status',4))
        try:self.n.wait_controller_quiet(self.n.seen)
        finally:self.n.destroy_timer(timer)
        self.assertGreaterEqual(time.monotonic()-began,.25)
    def test_different_worker_instance_cannot_resume_old_task(self):
        import uuid
        self.prepare('foreign_worker',worker=True,manual=True);self.hub.worker['seen']-=3.
        self.hub.register(dict(worker_id='battery',instance_id=str(uuid.uuid4()),point_ids=['point_1','point_2']))
        with self.assertRaisesRegex(ValueError,'goal_changed'):
            with self.n.transport.task_scope(self.identity):self.n.run_goal()
        self.assertTrue(self.paused)
        self.assertFalse(any(m['type']!='chassis.end_output' and any((m['x'],m['y'],m['omega'])) for m in self.sends))
        self.assertFalse(self.forbidden_worker_outputs)

if __name__=='__main__':unittest.main()
