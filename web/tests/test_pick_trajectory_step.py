"""Trajectory -> fresh settling -> single closure, using Controller with fake transport."""
import copy
import unittest
import test_pick_close_step as fixtures
from agent.pick_execution import PickTrajectoryStep


class PickTrajectoryTests(unittest.TestCase):
    feedback=fixtures.CloseStepTests.feedback
    def setUp(self):
        fixtures.CloseStepTests.setUp(self)
        self.c.wrist_reason=lambda side:''
        self.state=dict(driver_instance='test-driver',command_epoch=4,request_id='',
                        status='idle',enabled=True,ready=True)
        self.handle=dict(driver_instance='test-driver',command_epoch=4,request_id='test-job')
        self.adapter.trajectory_status=lambda side:copy.deepcopy(self.state)
        self.adapter.trajectory_keepalive=lambda side,handle:self.adapter.calls.append(('heartbeat',))
        def start(side,plan,expected):
            self.adapter.calls.append(('trajectory',copy.deepcopy(plan)))
            return self.handle,lambda:None
        self.adapter.trajectory_start=start
        self.plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=[0.,10.],
                       joint_waypoints_rad=[(self.q-.001).tolist(),self.q.tolist()])
        self.execution=PickTrajectoryStep(self.c,self.token,self.monitor,self.plan,
                                         preflight=lambda snapshot,now:True)

    def launch(self):
        self.feedback(10.)
        self.assertEqual(self.execution.start()['status'],'awaiting_trajectory')
        self.state.update(command_epoch=5,request_id='test-job',status='running')

    def test_long_reference_then_fresh_settling_then_close_once(self):
        self.launch()
        for t in (10.1,11.,15.,20.):
            self.feedback(t)
            self.assertEqual(self.execution.poll()['status'],'trajectory_running')
        self.assertFalse(any(call[0]=='arm' for call in self.adapter.calls))
        self.state['status']='reference_complete'
        self.feedback(20.1)
        self.assertEqual(self.execution.poll()['status'],'inserting')
        for t in (20.11,20.21,20.32):
            self.feedback(t);r=self.execution.poll()
        self.assertEqual(r['status'],'ready_to_close')
        self.feedback(20.42)
        self.assertEqual(self.execution.poll()['status'],'closing')
        for t in (20.43,20.53,20.64):
            self.feedback(t,-1.2);r=self.execution.poll()
        self.assertEqual(r['status'],'awaiting_hold_verification')
        self.assertFalse(r['grasp_confirmed'])
        self.assertEqual(sum(call[0]=='arm' for call in self.adapter.calls),1)
        with self.assertRaises(ValueError):self.execution.start()

    def test_failure_or_cancel_never_closes_and_cannot_restart(self):
        for fault in ('cancel','driver','preflight'):
            with self.subTest(fault=fault):
                self.setUp();self.launch();self.feedback(10.1)
                if fault=='driver':self.state['driver_instance']='new'
                if fault=='preflight':self.execution.preflight=lambda *_:False
                r=self.execution.poll(cancelled=fault=='cancel')
                self.assertTrue(r['stop_required']);self.assertFalse(r['grasp_confirmed'])
                before=len(self.adapter.calls)
                self.feedback(10.2);self.execution.poll()
                self.assertEqual(len(self.adapter.calls),before)
                self.assertFalse(any(c[0]=='arm' and abs(c[1][2]+1.2)<1e-6 for c in self.adapter.calls))
                with self.assertRaises(ValueError):self.execution.start()

    def test_plan_and_monitor_must_target_same_tool_endpoint(self):
        self.plan['joint_waypoints_rad'][-1][0]+=.2
        with self.assertRaises(ValueError):
            PickTrajectoryStep(self.c,self.token,self.monitor,self.plan,preflight=lambda *_:True)
        self.assertEqual(self.adapter.calls,[])

    def test_reference_end_does_not_hide_underreach(self):
        self.launch()
        self.state['status']='reference_complete'
        self.q[0]+=.1
        for t in (20.,20.01,20.11,20.22,20.32,20.42,20.52,20.62,20.72,20.92):
            self.feedback(t);r=self.execution.poll()
        self.assertEqual(r['status'],'stalled')
        self.assertTrue(r['stop_required'])
        self.assertFalse(any(c[0]=='arm' and abs(c[1][2]+1.2)<1e-6 for c in self.adapter.calls))

    def test_scene_revoked_after_reference_completion_prevents_close(self):
        self.launch();self.state['status']='reference_complete'
        self.feedback(20.);self.execution.poll()
        for t in (20.01,20.11,20.22):
            self.feedback(t);r=self.execution.poll()
        self.assertEqual(r['status'],'ready_to_close')
        self.execution.preflight=lambda *_:False
        self.feedback(20.32);r=self.execution.poll()
        self.assertTrue(r['stop_required'])
        self.assertFalse(any(c[0]=='arm' and abs(c[1][2]+1.2)<1e-6 for c in self.adapter.calls))

    def test_poll_before_start_does_not_close(self):
        self.feedback(10.1)
        with self.assertRaises(ValueError):self.execution.poll()
        self.assertEqual(self.adapter.calls,[])


if __name__=='__main__':unittest.main()
