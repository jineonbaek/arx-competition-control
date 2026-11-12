"""Lift then observer trajectory on an in-memory controller only."""
import copy
import unittest
from test_control import Controller,FakeAdapter,configured,Rejected
from agent.coarse_execution import CoarseAlignmentStep


class CoarseStepTests(unittest.TestCase):
    def setUp(self):
        self.now=10.;self.adapter=FakeAdapter()
        self.c=Controller(configured(),self.adapter,lambda:self.now)
        self.c.wrist_reason=lambda side:''
        self.q=[0.]*6;self.left=[0.]*6;self.height=5.
        self.feedback(10.)
        self.token=self.c.reserve_motion(self.c.command_epoch)
        self.state=dict(driver_instance='driver',command_epoch=3,request_id='',status='idle',enabled=True,ready=True)
        self.adapter.trajectory_status=lambda side:copy.deepcopy(self.state)
        def start(side,plan,expected):
            self.adapter.calls.append(('trajectory',copy.deepcopy(plan)))
            return dict(driver_instance='driver',command_epoch=3,request_id='observe'),lambda:None
        self.adapter.trajectory_start=start
        self.adapter.trajectory_keepalive=lambda *_:self.adapter.calls.append(('heartbeat',))
        self.plan=dict(version=1,interpolation='segment_quintic_scalar',time_from_start_s=[0.,3.],
            joint_waypoints_rad=[[0.]*6,[.01]*6])
        self.lift_policy=dict(tolerance=[.02],max_speed=[.1],settle_s=.2,timeout_s=5.,stall_s=1.,max_age_s=.2)
        self.arm_policy=dict(tolerance=[.002]*6,max_speed=[.01]*6,settle_s=.2,timeout_s=2.,stall_s=.6,max_age_s=.2)
        self.step=self.make()
    def make(self):
        return CoarseAlignmentStep(self.c,self.token,lift_target_rad=6.,observer_plan=self.plan,
            initial_joints={'left':[0.]*6,'right':[0.]*6},initial_lift_rad=5.,
            fixed_joint_tolerance_rad=.002,lift_policy=self.lift_policy,arm_policy=self.arm_policy,
            max_pair_skew_s=.05,lift_preflight=lambda *_:True,arm_preflight=lambda *_:True)
    def feedback(self,t):
        self.now=t
        self.c.update_arm('left',self.left,-1.5,[0.]*6)
        self.c.update_arm('right',self.q,-1.5,[0.]*6)
        self.c.update_body(self.height,[0.]*4,0.,0.,0.)
    def settle_lift(self):
        self.step.start();self.height=6.
        for t in (10.1,10.2,10.31):
            self.feedback(t);r=self.step.poll()
        return r
    def test_lift_then_observer_then_measured_rest(self):
        self.assertEqual(self.settle_lift()['status'],'observing_motion')
        self.assertEqual([c[0] for c in self.adapter.calls],['lift','trajectory'])
        self.state.update(command_epoch=4,request_id='observe',status='running')
        self.feedback(10.4);self.assertEqual(self.step.poll()['status'],'observing_motion')
        self.state['status']='reference_complete';self.q=[.01]*6
        self.feedback(13.4);self.assertEqual(self.step.poll()['status'],'settling_observer')
        for t in (13.41,13.51,13.62):
            self.feedback(t);r=self.step.poll()
        self.assertEqual(r['status'],'ready_to_observe')
        self.assertTrue(r['new_observation_required']);self.assertFalse(r['grasp_confirmed'])
        self.assertTrue(self.c.motion_valid(self.token))
        self.assertFalse(any(c[0]=='arm' for c in self.adapter.calls))
        with self.assertRaises(ValueError):self.step.start()
    def test_lift_failure_cannot_start_arm_and_stop_is_not_claimed_as_rest(self):
        for fault in ('left','right','stale','cancel','preflight'):
            with self.subTest(fault=fault):
                self.setUp();self.step.start();self.feedback(10.1)
                if fault=='left':self.left[0]=.02;self.feedback(10.1)
                if fault=='right':self.q[0]=.02;self.feedback(10.1)
                if fault=='stale':self.now=11.
                if fault=='preflight':self.step.lift_preflight=lambda *_:False
                r=self.step.poll(cancelled=fault=='cancel')
                self.assertTrue(r['stop_required']);self.assertFalse(r['stopped_verified'])
                self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))
                before=len(self.adapter.calls);self.step.poll();self.assertEqual(len(self.adapter.calls),before)
    def test_lift_drift_after_settling_cancels_observer(self):
        self.settle_lift();self.state.update(command_epoch=4,request_id='observe',status='running')
        self.height=6.1;self.feedback(10.4)
        self.assertTrue(self.step.poll()['stop_required'])
    def test_atomic_lift_guard_and_browser_denial(self):
        before=len(self.adapter.calls)
        self.step.lift_preflight=lambda *_:False
        self.assertTrue(self.step.start()['stop_required'])
        self.assertFalse(any(c[0]=='lift' and c[1]==(6.,) for c in self.adapter.calls[before:]))
        with self.assertRaises(Rejected):
            self.c.submit({'type':'lift','target':6.},'browser',preflight=lambda *_:True)
    def test_plan_start_must_match_arms_during_lift(self):
        self.plan['joint_waypoints_rad'][0][1]=.2
        with self.assertRaises(ValueError):self.make()
        self.assertEqual(self.adapter.calls,[])
    def test_reference_complete_does_not_mean_observer_reached(self):
        self.settle_lift();self.state.update(command_epoch=4,request_id='observe',status='reference_complete')
        for t in (11.,11.1,11.2,11.3,11.4,11.5,11.71):
            self.feedback(t);r=self.step.poll()
        self.assertTrue(r['stop_required']);self.assertNotEqual(r['status'],'ready_to_observe')

    def test_cancel_before_start_and_invalid_policy_cannot_send_motion(self):
        self.step.cancel()
        before=list(self.adapter.calls)
        with self.assertRaises(ValueError):self.step.start()
        self.assertEqual(self.adapter.calls,before)
        for field,value in (('tolerance',[float('nan')]),('max_speed',[0.]),('timeout_s',-.1)):
            self.setUp();self.lift_policy[field]=value
            with self.assertRaises(ValueError):self.make()
            self.assertEqual(self.adapter.calls,[])

    def test_feedback_clock_and_sample_regression_stop(self):
        for fault in ('clock','sample','pair','capability'):
            self.setUp();self.step.start();self.feedback(10.1);self.step.poll()
            self.feedback(10.2)
            if fault=='clock':self.now=10.05
            if fault=='sample':self.c.arms['left']['received']=10.09
            if fault=='pair':self.c.body['received']=10.1
            if fault=='capability':self.c.config['execution_enabled']=False
            self.assertTrue(self.step.poll()['stop_required'],fault)
            self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))

    def test_right_drift_at_atomic_observer_start_stops_without_trajectory(self):
        original=self.c.start_trajectory
        def changed(*args,**kwargs):
            self.q[0]=.02;self.feedback(self.now)
            return original(*args,**kwargs)
        self.c.start_trajectory=changed
        self.assertTrue(self.settle_lift()['stop_required'])
        self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))

    def test_observer_uses_real_ros_trajectory_schema_without_hardware(self):
        from test_trajectory_transport import TrajectoryTransportTests
        transport=TrajectoryTransportTests()
        transport.setUp()
        self.adapter.trajectory_status=transport.adapter.trajectory_status
        self.adapter.trajectory_start=transport.adapter.trajectory_start
        self.assertEqual(self.settle_lift()['status'],'observing_motion')
        self.assertEqual(len(transport.sent),1)
        self.assertEqual(transport.sent[0]['version'],1)
        self.assertEqual(transport.sent[0]['joint_waypoints_rad'],self.plan['joint_waypoints_rad'])

    def test_real_wrist_readiness_waits_for_lift_history_without_aborting(self):
        del self.c.wrist_reason
        self.c.lift_history.clear()
        for t in (9.7,9.85,10.):self.feedback(t)
        self.assertEqual(self.c.wrist_reason('right'),'')
        self.assertEqual(self.step.start()['status'],'lifting')
        self.height=6.
        for t in (10.1,10.2,10.31):
            self.feedback(t)
            self.assertEqual(self.step.poll()['status'],'lifting')
        self.assertNotEqual(self.c.wrist_reason('right'),'')
        self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))
        for t in (10.41,10.51):
            self.feedback(t);r=self.step.poll()
        self.assertEqual(r['status'],'observing_motion')
        self.assertEqual(sum(c[0]=='trajectory' for c in self.adapter.calls),1)

    def test_waiting_for_observer_readiness_is_bounded_and_retains_driver_checks(self):
        self.c.wrist_reason=lambda side:'not ready'
        self.assertEqual(self.settle_lift()['status'],'lifting')
        self.feedback(15.01)
        self.assertTrue(self.step.poll()['stop_required'])
        self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))
        self.setUp();self.step.start()
        self.c.config['arms']['right']['ready_topic']='/fake/right/ready'
        self.adapter.arm_ready=lambda side:False
        self.feedback(10.1)
        self.assertTrue(self.step.poll()['stop_required'])

    def stationary(self):
        self.plan=dict(version=1,kind='stationary',joint_position_rad=[0.]*6)
        self.step=self.make()

    def test_stationary_observer_waits_for_new_settled_feedback_without_arm_command(self):
        self.stationary()
        self.assertEqual(self.settle_lift()['status'],'settling_observer')
        self.assertEqual([c[0] for c in self.adapter.calls],['lift'])
        # Lift completion's last sample must not also establish arm settling.
        self.assertEqual(self.step.poll()['status'],'settling_observer')
        for t in (10.32,10.42,10.53):
            self.feedback(t);r=self.step.poll()
        self.assertEqual(r['status'],'ready_to_observe')
        self.assertTrue(r['new_observation_required'])
        self.assertEqual([c[0] for c in self.adapter.calls],['lift'])
        self.q[0]=.02;self.feedback(10.6)
        self.assertTrue(self.step.poll()['stop_required'])

    def test_stationary_observer_cannot_hide_a_target_or_bypass_arm_preflight(self):
        self.plan=dict(version=1,kind='stationary',joint_position_rad=[.001]*6)
        with self.assertRaises(ValueError):self.make()
        self.assertEqual(self.adapter.calls,[])
        self.stationary();self.step.arm_preflight=lambda *_:False
        self.settle_lift();self.feedback(10.4)
        self.assertTrue(self.step.poll()['stop_required'])
        self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))

    def test_stationary_observer_stale_or_cancelled_never_completes(self):
        for fault in ('stale','cancel'):
            self.setUp();self.stationary();self.settle_lift()
            if fault=='stale':self.now=11.
            r=self.step.poll(cancelled=fault=='cancel')
            self.assertTrue(r['stop_required'])
            self.assertFalse(r['new_observation_required'])
            self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))

if __name__=='__main__':unittest.main()
