"""Arrival drift recovery through real MPPI/stop-window code; no hardware."""
import math,time,unittest
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np,rclpy
from geometry_msgs.msg import TwistStamped,PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool,String
import test_mission_ros as f


class NativeHandle:
    accepted=True
    def __init__(self,done=False):
        self.status=4 if done else 2;self.result=Future()
        if done:self.complete()
    def complete(self):
        self.status=4
        if not self.result.done():self.result.set_result(SimpleNamespace(status=4,result=SimpleNamespace(error_code=0,error_msg='')))
    def get_result_async(self):return self.result
    def cancel_goal_async(self):self.status=5;return Future()


class TerminalCorrection(unittest.TestCase):
    setUpClass=classmethod(lambda cls:rclpy.init())
    tearDownClass=classmethod(lambda cls:rclpy.shutdown())
    setUp=f.MissionROS.setUp;tearDown=f.MissionROS.tearDown
    refresh=f.MissionROS.refresh;add=f.MissionROS.add

    def prepare(self,*,yaw=False,persistent=False,guard=None,unsettled=False,task_id=None):
        n=self.n;self.position[:]=[1.98 if yaw else 1.9467,0.,.12 if yaw else .0646]
        self.refresh()
        if task_id:
            self.task_hub,self.task_token=f.MissionROS.worker(self)
            n.create_timer(.2,lambda:self.task_hub.heartbeat(self.task_token))
            self.add('task',2.,task_id)
        else:self.add('stop',2.)
        n.waypoints.start(None);n.seen=n.seq
        # This fixture exercises the retained legacy MPPI terminal supervisor.
        # Production task_direct is covered by test_task_direct_ros instead.
        if task_id:n._special_execution=False;n.waypoints.route.mode='navigation'
        self.target=n.target.copy();self.identity=n.transport.current
        self.messages=[];self.states=[];self.outputs=0;self.correcting_at=None
        self.pause_observed=False;self.guarded_nonzero=[]
        self.paused_at=None;self.pause_samples=0;self.expect_clear_at_pause=False
        capture=n.transport.submit
        def output(m):
            self.messages.append(dict(m))
            if m['type']!='chassis.end_output' and any((m['x'],m['y'],m['omega'])):
                self.outputs+=1
                if self.guard_active:self.guarded_nonzero.append(m)
            return capture(m)
        n.transport.submit=output
        status=n.status
        def observe(value):
            self.states.append(value);status(value)
            if value.startswith('waiting_for_same_goal_replan') and self.correcting_at is None:self.correcting_at=time.monotonic()
            if value.startswith('paused_terminal_correction'):
                self.pause_observed=True
                self.assertEqual(n.transport.current,self.identity)
                self.assertIsNotNone(n.target);self.assertTrue(n.waypoints.route.running)
                self.assertFalse(any(m['type']=='chassis.end_output' for m in self.messages))
                if self.expect_clear_at_pause:self.assertEqual(n.escape.pins,[])
                self.paused_at=time.monotonic();self.pause_counts=(self.client.requests,self.outputs)
        n.status=observe
        outer=self
        class Client:
            callback=None;requests=0;handle=None
            def wait_for_server(self,**kwargs):return True
            def send_goal_async(self,request,feedback_callback):
                self.requests+=1;self.callback=feedback_callback
                self.handle=NativeHandle(done=self.requests==1 or persistent)
                future=Future();future.set_result(self.handle);return future
        self.client=Client();n.mppi_client=self.client
        self.n.destroy_timer(self.timer);began=time.monotonic();self.guard_active=False
        def feed():
            now=time.monotonic()
            if now-began>16:raise AssertionError('terminal recovery fixture timeout')
            self.guard_active=bool(guard and self.correcting_at and now-self.correcting_at<.7)
            if unsettled and self.correcting_at is None:self.position[0]=1.9467+.02*math.sin((now-began)*25.)
            if self.client.requests>1 and self.outputs>=2 and not persistent:
                self.position[:]=[2.,0.,0.];self.client.handle.complete()
            self.refresh()
            if self.guard_active:
                if guard=='scan':n.sensor['/pb/scan'].sec-=2;n.sensor_received['/pb/scan']-=2
                if guard=='obstacle':
                    for _,g in n.grids.values():
                        col,row=np.floor((n.target[:2]-g.origin)/g.res).astype(int);g.grid[row,col]=100
            if n.recovery is not None or n.route is None:
                n.route=np.array([self.position.copy(),[2.,0.,0.]])
            n.route_time=n.now()
            sdk=TwistStamped();sdk.header.stamp=n.get_clock().now().to_msg()
            sdk.twist.linear.x=.18;sdk.twist.angular.z=.02
            n.mppi_received.update(inputs=(now,Bool(data=True)),sdk=(now,sdk),status=(now,String(data='preview_ready')))
            if self.client.callback:self.client.callback(None)
            if self.paused_at is not None and n.target is not None:
                self.pause_samples+=1
                # Actual new candidate callback plus fresh preview/source frames
                # must not wake a parked executor.
                path=Path();path.header.frame_id='map';path.header.stamp=n.get_clock().now().to_msg()
                for x in (float(self.position[0]),2.):
                    p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.append(p)
                n.receive(path)
                self.assertEqual((self.client.requests,self.outputs),self.pause_counts)
                self.assertEqual(n.transport.current,self.identity)
                np.testing.assert_allclose(n.target,self.target)
                if now-self.paused_at>.35:n.cancel('cancelled')
        self.timer=n.create_timer(.015,feed);feed()

    def run_retained(self):
        with self.n.transport.task_scope(self.identity):return self.n.run_goal()

    def assert_recovered(self):
        self.assertEqual(self.run_retained(),'arrived_stopped')
        self.assertGreaterEqual(self.client.requests,2)
        self.assertGreater(self.outputs,0)
        self.assertEqual(self.n.transport.current,self.identity)
        np.testing.assert_allclose(self.n.target,self.target)
        self.assertFalse(any(m['type']=='chassis.end_output' for m in self.messages))
        moving=[m for m in self.messages if m['type']!='chassis.end_output' and any((m['x'],m['y'],m['omega']))]
        self.assertTrue(all(math.hypot(m['x'],m['y'])<=.100001 and abs(m['omega'])<=.100001 for m in moving))
        self.assertFalse(self.guarded_nonzero)

    def test_recorded_53mm_stationary_offset_retries_same_goal_slowly(self):
        self.prepare();self.assert_recovered()
    def test_final_yaw_offset_is_also_corrected_not_released(self):
        self.prepare(yaw=True);self.assert_recovered()
    def test_repeated_offset_parks_without_failing_or_finishing_mission(self):
        self.prepare(persistent=True)
        with self.assertRaisesRegex(ValueError,'goal_changed'):self.run_retained()
        self.assertTrue(self.pause_observed);self.assertLessEqual(self.client.requests,4)
        self.assertGreater(self.pause_samples,3)
        self.assertEqual(self.outputs,0)
    def test_unsettled_stop_window_retries_after_actual_stationary_proof(self):
        self.prepare(unsettled=True);self.assert_recovered()
        self.assertTrue(any('not_stationary' in s for s in self.states))
    def test_new_obstacle_during_correction_waits_then_reaudits(self):
        self.prepare(guard='obstacle');self.assert_recovered()
    def test_scan_outage_during_correction_never_sends_stale_effort(self):
        self.prepare(guard='scan');self.assert_recovered()
    def test_terminal_correction_never_starts_escape_motion_outside_its_budget(self):
        self.prepare()
        normal=self.n.escape.maybe_run
        def escape(epoch):
            if self.n.terminal_correction:
                self.n.transport.send((.14,0.,0.))  # Existing escape's separate output budget.
                return False
            return normal(epoch)
        self.n.escape.maybe_run=escape
        self.assert_recovered()
    def test_timeout_pause_withdraws_sparse_lease_before_callbacks_can_renew_it(self):
        self.prepare();self.n.escape.pins=[(.1,.1)];self.n.escape.owner=self.n.seq
        self.expect_clear_at_pause=True
        with self.assertRaisesRegex(ValueError,'goal_changed'):
            self.n.pause_terminal_correction(self.n.seen,'goal_timeout')
        self.assertEqual(self.n.escape.pins,[]);self.assertEqual(self.outputs,0)
    def test_next_goal_resets_terminal_retry_state(self):
        self.prepare();self.n.terminal_correction=True;self.n.terminal_checks=2
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg()
        m.pose.position.x=1.5;m.pose.orientation.w=1.
        self.n.accept_goal(m)
        self.assertFalse(self.n.terminal_correction);self.assertEqual(self.n.terminal_checks,0)
        self.assertNotEqual(self.n.transport.current,self.identity)
    def test_cancelled_correction_rejects_late_native_acceptance(self):
        self.prepare();pending=Future();original=self.client.send_goal_async
        def delayed(*args,**kwargs):
            future=original(*args,**kwargs)
            return pending if self.client.requests==2 else future
        self.client.send_goal_async=delayed
        def cancel_after_second_request():
            if self.client.requests==2 and self.n.target is not None:self.n.cancel('cancelled')
        timer=self.n.create_timer(.015,cancel_after_second_request)
        try:
            with self.assertRaisesRegex(ValueError,'goal_changed'):self.run_retained()
        finally:self.n.destroy_timer(timer)
        before=len(self.messages);pending.set_result(self.client.handle)
        self.assertEqual(self.client.handle.status,5)
        self.assertIsNone(self.n.target);self.assertEqual(len(self.messages),before)


if __name__=='__main__':unittest.main()
