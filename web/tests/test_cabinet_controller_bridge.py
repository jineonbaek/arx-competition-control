"""Real Controller logic with fake transport only; never starts ROS/CAN."""
import copy
import threading
from dataclasses import replace
import unittest
from test_control import configured, FakeAdapter, Controller
from agent.cabinet_gates import REQUIRED, Context, Decision
from agent.cabinet_controller_bridge import (
    ControllerBridge, BridgePolicy, PhysicalApproval, BridgeRejected,
)


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.adapter = FakeAdapter()
        self.controller = Controller(configured(), self.adapter, lambda: self.now)
        self.controller.wrist_reason = lambda side: ''
        self.driver = dict(driver_instance='driver-one', command_epoch=7,
                           request_id='', status='idle', ready=True, enabled=True)
        self.adapter.trajectory_status = lambda side: copy.deepcopy(self.driver)
        self.adapter.trajectory_keepalive = lambda side, handle: self.adapter.calls.append(('heartbeat', handle))
        self.confirm = lambda: None
        def start(side, plan, expected):
            self.adapter.calls.append(('trajectory', copy.deepcopy(plan)))
            handle = dict(driver_instance='driver-one', command_epoch=7, request_id='segment-one')
            return handle, self.confirm
        self.adapter.trajectory_start = start
        self.plan = dict(interpolation='segment_quintic_scalar',
                         time_from_start_s=[0., 2.], joint_waypoints_rad=[[0.]*6, [.01]*6])
        self.binding = dict(run_id='run', boot_id='boot', generation=1,
                            asset_hash='a'*64, held_identity_hash='b'*64)
        self.policy = BridgePolicy(joint_tolerance_rad=.002, gripper_tolerance_rad=.02,
                                  lift_tolerance_rad=.02, rest_speed_rad_s=.02,
                                  settle_s=.3, max_age_s=.2, max_poll_gap_s=.2, timeout_s=8.)
        self.approval_changes = {}
        self.bridge = ControllerBridge(self.controller, self.policy, physical_provider=self.provider)
        self.refresh()

    def refresh(self, q=None, *, grip=-1., height=5.):
        self.controller.update_arm('right', [0.]*6 if q is None else q, grip, [0.]*6)
        self.controller.update_arm('left', [0.]*6, -1., [0.]*6)
        self.controller.update_body(height, [0.]*4, 0., 0., 0.)

    def provider(self, request, state, now):
        # Synthetic test fixture for the trusted provider seam, never physical evidence.
        p = PhysicalApproval(request_hash=request['request_hash'], issued_at=now-.01,
                             expires_at=now+.15, verified_gates=frozenset(REQUIRED[request['phase']]),
                             scope='validated_physical', physical_stop_verified=True,
                             clearance_mm=50., error_upper_mm=2., reserve_mm=10.)
        return replace(p, **self.approval_changes)

    def start(self, phase='outside_align'):
        return self.bridge.start(phase, self.plan, self.binding)

    def running(self):
        self.driver.update(status='running', request_id='segment-one', command_epoch=8)

    def advance(self, dt=.1, q=None):
        self.now += dt
        self.refresh(q)
        return self.bridge.poll()

    def settle(self, *, q=None, count=6):
        result = None
        for _ in range(count):
            result = self.advance(q=q)
        return result

    def test_absent_provider_cannot_reserve_or_publish(self):
        self.bridge = ControllerBridge(self.controller, self.policy)
        with self.assertRaises(BridgeRejected): self.start()
        self.assertEqual(self.adapter.calls, [])
        self.assertIsNone(self.controller.motion_owner)

    def test_simulator_decision_is_never_a_physical_permission(self):
        ctx = Context('run', 'boot', 1, 'a'*64, 'b'*64)
        self.bridge.provider = lambda *args: Decision('outside_align', True, (), {}, ctx, 99., 101.)
        with self.assertRaises(BridgeRejected): self.start()
        self.assertEqual(self.adapter.calls, [])

    def test_approval_must_bind_plan_driver_and_all_phase_gates(self):
        for change in ({'request_hash':'0'*64}, {'scope':'simulation'},
                       {'verified_gates':frozenset({'G09'})}, {'physical_stop_verified':False},
                       {'expires_at':99.}, {'issued_at':101.}, {'error_upper_mm':float('nan')},
                       {'clearance_mm':-0.27}, {'clearance_mm':12.}, {'reserve_mm':-1.}):
            with self.subTest(change=change):
                self.approval_changes = change
                with self.assertRaises(BridgeRejected): self.start('insert')
        self.assertIsNone(self.controller.motion_owner)
        self.assertEqual(self.adapter.calls, [])

    def test_only_supported_timed_right_arm_plans_are_accepted(self):
        for phase in ('release', 'base_align', 'lift_align', 'withdraw_empty'):
            with self.assertRaises(BridgeRejected): self.start(phase)
        for field, value in [('time_from_start_s',[0.,0.]),
                             ('joint_waypoints_rad',[[0.]*6,[True]*6]),
                             ('time_from_start_s',[0.,float('nan')]),
                             ('interpolation','linear')]:
            plan = copy.deepcopy(self.plan); plan[field] = value
            with self.assertRaises(BridgeRejected): self.bridge.start('outside_align',plan,self.binding)
        self.assertEqual(self.adapter.calls, [])

    def test_full_plan_uses_existing_controller_and_owns_until_measured_arrival(self):
        self.start(); self.running()
        self.assertEqual(self.adapter.calls[0], ('trajectory', self.plan))
        self.assertTrue(self.controller.snapshot()['motion_reserved'])
        self.assertEqual(self.advance()['status'], 'running')
        self.driver['status'] = 'reference_complete'
        self.assertFalse(self.advance(q=[.01]*6)['completed'])
        result = self.settle(q=[.01]*6)
        self.assertTrue(result['completed'])
        self.assertFalse(self.controller.snapshot()['motion_reserved'])
        self.assertFalse(any(c[0]=='arm' for c in self.adapter.calls))

    def test_complete_reference_away_from_target_never_claims_arrival(self):
        self.start(); self.running(); self.driver['status']='reference_complete'
        self.advance()
        result = self.settle(q=[0.]*6)
        self.assertFalse(result['completed'])
        self.assertIsNotNone(self.controller.motion_owner)

    def test_duplicate_or_gapped_samples_cannot_prove_rest(self):
        self.start(); self.running(); self.driver['status']='reference_complete'
        self.advance(q=[.01]*6)
        for _ in range(3):
            self.now += .05; self.bridge.poll()
        self.assertIsNotNone(self.controller.motion_owner)
        self.now += .3; self.refresh([.01]*6)
        r = self.bridge.poll()
        self.assertFalse(r['completed'])
        self.assertIn(r['status'], ('stopping', 'recovery_required'))

    def test_expired_approval_stops_and_does_not_send_more_heartbeats(self):
        self.start(); self.running(); self.approval_changes={'expires_at':self.now}
        r=self.advance()
        self.assertIn(r['status'], ('stopping','recovery_required'))
        self.assertTrue(any(c[0]=='arm_stop' for c in self.adapter.calls))
        self.assertFalse(any(c[0]=='heartbeat' for c in self.adapter.calls))
        self.assertIsNotNone(self.controller.motion_owner)

    def test_stale_feedback_does_not_prevent_independent_stop(self):
        self.start(); self.running(); self.now += .15
        self.controller.arms['right']['received'] = self.now-5.
        self.bridge.poll()
        self.assertTrue(any(c[0]=='arm_stop' for c in self.adapter.calls))
        self.assertIsNotNone(self.controller.motion_owner)

    def test_clock_deadline_and_poll_gap_fail_closed(self):
        for delta in (-1., .25, 10.):
            with self.subTest(delta=delta):
                self.setUp();self.start();self.running();self.now += delta
                r=self.bridge.poll()
                self.assertFalse(r['completed'])
                self.assertTrue(any(c[0]=='arm_stop' for c in self.adapter.calls))

    def test_driver_restart_cannot_receive_old_keepalive_or_release(self):
        self.start(); self.running(); self.driver['driver_instance']='reboot'
        r=self.advance()
        self.assertFalse(r['completed'])
        self.assertFalse(any(c[0]=='heartbeat' for c in self.adapter.calls))
        self.driver['status']='idle';self.settle()
        self.assertIsNotNone(self.controller.motion_owner)

    def test_manual_stop_remains_latched_even_after_provider_recovers(self):
        self.start(); self.running()
        self.bridge.cancel('manual')
        self.driver['status']='idle'
        r=self.settle()
        self.assertEqual(r['status'],'cancelled')
        with self.assertRaises(BridgeRejected): self.start()
        self.assertEqual(sum(c[0]=='trajectory' for c in self.adapter.calls),1)

    def test_stop_ack_and_stationary_old_frame_do_not_release_ownership(self):
        self.start(); self.running(); self.bridge.cancel('manual');self.driver['status']='idle'
        self.now += .1
        r=self.bridge.poll()
        self.assertFalse(r['completed'])
        self.assertIsNotNone(self.controller.motion_owner)
        self.settle()
        self.assertIsNone(self.controller.motion_owner)

    def test_unknown_start_result_keeps_recovery_lock_even_after_fresh_rest(self):
        def unknown():raise RuntimeError('ack lost')
        self.confirm=unknown
        with self.assertRaises(BridgeRejected):self.start()
        self.driver.update(status='idle',command_epoch=8)
        r=self.settle()
        self.assertEqual(r['status'],'recovery_required')
        self.assertTrue(r['outcome_unknown'])
        self.assertIsNotNone(self.controller.motion_owner)

    def test_grip_lift_or_left_arm_motion_requests_stop_without_opening(self):
        for component in ('grip','lift','left'):
            with self.subTest(component=component):
                self.setUp();self.start();self.running();self.now += .1;self.refresh()
                if component=='grip':self.controller.arms['right']['gripper']=-1.1
                if component=='lift':self.controller.body['height']=6.
                if component=='left':self.controller.arms['left']['joints']=[.1]*6
                r=self.bridge.poll()
                self.assertFalse(r['completed'])
                self.assertTrue(any(c[0]=='arm_stop' for c in self.adapter.calls))
                self.assertFalse(any(c[0]=='arm' for c in self.adapter.calls))

    def test_no_approval_check_can_mutate_the_dispatched_plan(self):
        original=self.provider
        def provider(request,state,now):
            proof=original(request,state,now)
            request['plan']['joint_waypoints_rad'][-1]=[1.]*6
            state['arms']['right']['joints']=[1.]*6
            return proof
        self.bridge.provider=provider;self.start()
        self.assertEqual(self.adapter.calls[0][1],self.plan)

    def test_stop_request_failure_retains_control_and_can_retry(self):
        self.start(); self.running()
        saved=self.adapter.arm_stop
        self.adapter.arm_stop=lambda side: (_ for _ in ()).throw(RuntimeError('service unavailable'))
        self.bridge.cancel('manual');self.driver['status']='idle';self.settle()
        self.assertIsNotNone(self.controller.motion_owner)
        self.adapter.arm_stop=saved;self.bridge.cancel('retry stop');self.settle()
        self.assertIsNone(self.controller.motion_owner)

    def test_acknowledgement_cannot_outlive_physical_admission(self):
        def late_ack(): self.now += .4
        self.confirm=late_ack
        with self.assertRaises(BridgeRejected): self.start()
        self.assertTrue(any(c[0]=='arm_stop' for c in self.adapter.calls))
        self.assertIsNotNone(self.controller.motion_owner)

    def test_measured_rest_cannot_be_reused_after_feedback_changes_during_release(self):
        self.start();self.running();self.bridge.cancel('manual');self.driver['status']='idle'
        original=self.bridge._rested
        def changed_after_check(*args,**kwargs):
            passed=original(*args,**kwargs)
            if passed:self.controller.update_arm('right',[.2]*6,-1.,[0.]*6)
            return passed
        self.bridge._rested=changed_after_check
        self.settle()
        self.assertIsNotNone(self.controller.motion_owner)

    def test_driver_identity_is_rechecked_at_control_handoff(self):
        self.start();self.running();self.bridge.cancel('manual');self.driver['status']='idle'
        original=self.bridge._rested
        def restarted_after_check(*args,**kwargs):
            passed=original(*args,**kwargs)
            if passed:self.driver['driver_instance']='restarted'
            return passed
        self.bridge._rested=restarted_after_check
        self.settle()
        self.assertIsNotNone(self.controller.motion_owner)

    def test_changed_controller_configuration_invalidates_admission(self):
        self.start();self.running()
        self.controller.config['arms']['right']['max_delta_deg']=9.
        r=self.advance()
        self.assertIn(r['status'],('stopping','recovery_required'))
        self.assertFalse(any(c[0]=='heartbeat' for c in self.adapter.calls))

    def test_joint_limits_are_checked_before_reserving(self):
        plan=copy.deepcopy(self.plan);plan['joint_waypoints_rad'][-1]=[2.]*6
        with self.assertRaises(BridgeRejected):self.bridge.start('outside_align',plan,self.binding)
        self.assertIsNone(self.controller.motion_owner)
        self.assertEqual(self.adapter.calls,[])

    def test_stop_ignores_observation_mode_and_does_not_need_new_admission(self):
        self.start();self.running();self.controller.config['execution_enabled']=False
        self.bridge.provider=lambda *args: (_ for _ in ()).throw(RuntimeError('evidence lost'))
        self.bridge.cancel('manual stop')
        self.assertTrue(any(c[0]=='arm_stop' for c in self.adapter.calls))

    def test_slow_provider_cannot_publish_after_expiry(self):
        provider=self.provider;calls=[]
        def slow(request,state,now):
            calls.append(now);proof=provider(request,state,now)
            if len(calls)==2:self.now+=.4
            return proof
        self.bridge.provider=slow
        with self.assertRaises(BridgeRejected):self.start()
        self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))

    def test_approval_cannot_rebind_a_new_driver_epoch(self):
        provider=self.provider
        def changed_epoch(request,state,now):
            proof=provider(request,state,now);self.driver['command_epoch']=9;return proof
        self.bridge.provider=changed_epoch
        with self.assertRaises(BridgeRejected):self.start()
        self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))

    def test_cancel_sends_stop_without_waiting_for_confirmation_or_provider_lock(self):
        for blocked in ('confirmation','provider'):
            with self.subTest(blocked=blocked):
                self.setUp();entered=threading.Event();unblock=threading.Event();stop_sent=threading.Event()
                self.adapter.arm_stop=lambda side: (self.adapter.calls.append(('arm_stop',side)),stop_sent.set())
                if blocked=='confirmation':
                    def confirm():entered.set();unblock.wait(2.)
                    self.confirm=confirm
                else:
                    original=self.provider;calls=[]
                    def provider(request,state,now):
                        calls.append(now)
                        if len(calls)==2:entered.set();unblock.wait(2.)
                        return original(request,state,now)
                    self.bridge.provider=provider
                errors=[]
                def run():
                    try:self.start()
                    except BridgeRejected as e:errors.append(str(e))
                worker=threading.Thread(target=run);worker.start()
                self.assertTrue(entered.wait(1.))
                cancel=threading.Thread(target=lambda:self.bridge.cancel('manual'));cancel.start()
                try:self.assertTrue(stop_sent.wait(.2),'Independent stop blocked behind admission/ack')
                finally:unblock.set();worker.join(2.);cancel.join(2.)
                self.assertTrue(errors)
                if blocked=='provider':self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))

    def test_cancellation_discards_reference_complete_rest_history(self):
        self.start();self.running();self.driver['status']='reference_complete'
        for _ in range(4):self.advance(q=[.01]*6)
        self.assertIsNotNone(self.controller.motion_owner)
        self.bridge.cancel('manual');self.driver['status']='idle'
        self.advance(q=[.01]*6)
        self.assertIsNotNone(self.controller.motion_owner)

    def test_stop_speed_uses_each_components_actual_feedback_interval(self):
        self.start();self.running();self.bridge.cancel('manual');self.driver['status']='idle'
        for now,right_stamp,other_stamp,q in ((100.2,100.2,100.1,0.),(100.3,100.3,100.2,0.),
                                               (100.4,100.4,100.3,0.),(100.5,100.41,100.4,.001)):
            self.now=now;self.refresh([q]+[0.]*5)
            self.controller.arms['right']['received']=right_stamp
            self.controller.arms['left']['received']=other_stamp
            self.controller.body['received']=other_stamp
            self.bridge.poll()
        self.assertIsNotNone(self.controller.motion_owner)

    def test_controller_preserves_approved_epoch_after_preflight(self):
        token=self.controller.reserve_motion(self.controller.command_epoch)
        def changed_after_preflight(state,now):self.driver['command_epoch']=9;return True
        with self.assertRaises(ValueError):
            self.controller.start_trajectory('right',self.plan,token,preflight=changed_after_preflight,
                                             expected_driver={'driver_instance':'driver-one','command_epoch':7})
        self.assertFalse(any(c[0]=='trajectory' for c in self.adapter.calls))

    def test_incompatible_controller_contract_is_rejected_before_use(self):
        self.controller.start_trajectory=lambda side,plan,token,preflight:None
        with self.assertRaises(BridgeRejected):ControllerBridge(self.controller,self.policy)

    def test_task_deadline_expiring_inside_provider_prevents_heartbeat(self):
        self.policy=replace(self.policy,timeout_s=3.)
        self.bridge=ControllerBridge(self.controller,self.policy,physical_provider=self.provider)
        self.start();self.running()
        for i in range(1,29):
            self.now=100.+i*.1;self.refresh();self.bridge.poll()
        old_heartbeats=sum(c[0]=='heartbeat' for c in self.adapter.calls)
        provider=self.provider
        def slow(request,state,now):
            proof=provider(request,state,now);self.now+=.1;self.refresh();return proof
        self.bridge.provider=slow;self.now=102.9;self.refresh();result=self.bridge.poll()
        self.assertIn(result['status'],('stopping','recovery_required'))
        self.assertEqual(sum(c[0]=='heartbeat' for c in self.adapter.calls),old_heartbeats)

    def test_cancel_at_handoff_is_serialized_with_completion_and_stays_latched(self):
        self.start();self.running();self.driver['status']='reference_complete'
        self.advance(q=[.01]*6)
        original=self.controller.release_motion
        cancelled=[]
        def cancel_at_handoff(token):
            t=threading.Thread(target=lambda:cancelled.append(self.bridge.cancel('manual at handoff')))
            t.start();t.join(.5);self.assertFalse(t.is_alive(),'Independent cancel must not block on handoff')
            return original(token)
        self.controller.release_motion=cancel_at_handoff
        result=self.settle(q=[.01]*6)
        # Handoff has already committed a verified arrival before the concurrent
        # cancellation can commit its stop/rest boundary. It must not report
        # cancelled+released with erased post-stop proof.
        self.assertEqual(result['status'],'completed')
        self.assertTrue(result['cancellation_requested'])
        self.assertTrue(self.bridge.rest)
        self.assertFalse(any(x['status']=='stopping' for x in cancelled))
        with self.assertRaises(BridgeRejected):self.start()


if __name__=='__main__':unittest.main()
