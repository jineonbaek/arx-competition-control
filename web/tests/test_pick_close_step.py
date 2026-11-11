"""Real Controller boundary, simulated feedback and adapter; zero device transport."""
import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from test_control import Controller,FakeAdapter,configured
from agent.motion_feedback import PickMonitor
from agent.pick_execution import PickCloseStep

class CloseStepTests(unittest.TestCase):
    def setUp(self):
        self.now=10.;self.adapter=FakeAdapter()
        self.c=Controller(configured(),self.adapter,lambda:self.now)
        self.q=np.radians([0,20,40,10,0,-90])
        self.tcp=np.eye(4);self.tcp[:3,3]=[.12,.007,-.004]
        self.target=self.c.models['right'].wrist_transform(self.q,12)@self.tcp
        self.arm_policy=dict(tolerance=[1.,1.,1.,.5,.5,.5],max_speed=[1.]*6,
            settle_s=.2,timeout_s=3.,stall_s=.8,max_age_s=.15)
        self.grip_policy=dict(tolerance=[.02],max_speed=[.2],settle_s=.2,
            timeout_s=2.,stall_s=.8,max_age_s=.15)
        self.monitor=PickMonitor(self.c.models['right'],self.tcp,self.target,
            expected_lift_rad=12.,lift_tolerance_rad=.02,max_pair_skew_s=.05,
            started_at=10.,arm_policy=self.arm_policy,gripper_target=-1.2,gripper_policy=self.grip_policy)
        self.token=self.c.reserve_motion(self.c.command_epoch)
        self.step=PickCloseStep(self.c,self.token,self.monitor)

    def feedback(self,t,grip=-3.):
        self.now=t
        self.c.update_arm('right',self.q.tolist(),grip,[0.]*6)
        self.c.update_body(12.,[0.]*4,0.,0.,0.)

    def arrive(self):
        for t in (10.01,10.11,10.22):
            self.feedback(t);r=self.step.poll()
        self.assertEqual(r['status'],'ready_to_close')
        self.assertEqual(self.adapter.calls,[])

    def test_single_close_through_actual_controller_then_wait_for_hold(self):
        self.arrive();self.feedback(10.32)
        r=self.step.poll()
        self.assertEqual(r['status'],'closing')
        self.assertEqual(len(self.adapter.calls),1)
        self.assertAlmostEqual(self.adapter.calls[0][1][2],-1.2)
        for t in (10.33,10.43,10.54):
            self.feedback(t,-1.2);r=self.step.poll()
        self.assertEqual(r['status'],'awaiting_hold_verification')
        self.assertFalse(r['grasp_confirmed'])
        self.assertEqual(len(self.adapter.calls),1)

    def test_underreach_stops_once_without_closure(self):
        self.monitor.target[0,3]+=.013
        for t in (10.01,10.11,10.22,10.32,10.42,10.52,10.62,10.72,10.92):
            self.feedback(t);r=self.step.poll()
        self.assertEqual(r['status'],'stalled')
        self.assertEqual(r['stop_request'],'sent_unverified')
        self.assertEqual(len(self.adapter.calls),1)
        self.assertAlmostEqual(self.adapter.calls[0][1][2],-3.)
        self.feedback(11.);self.step.poll()
        self.assertEqual(len(self.adapter.calls),1)

    def test_manual_stop_between_arrival_and_close_prevents_close(self):
        self.arrive();self.feedback(10.32)
        self.c.submit({'type':'arm.stop','arm':'right'},'browser')
        r=self.step.poll()
        self.assertTrue(r['stop_required'])
        self.assertTrue(all(call[1][2]==-3. for call in self.adapter.calls))

    def test_feedback_drift_between_poll_and_dispatch_is_rechecked(self):
        self.arrive();self.feedback(10.32)
        submit=self.c.submit
        def drift(*args,**kwargs):
            self.q[0]+=.1
            self.feedback(10.33)
            return submit(*args,**kwargs)
        with patch.object(self.c,'submit',side_effect=drift):r=self.step.poll()
        self.assertTrue(r['stop_required'])
        self.assertTrue(all(call[1][2]==-3. for call in self.adapter.calls))

    def test_cancel_is_latched_and_stops_once(self):
        self.feedback(10.01)
        r=self.step.poll(cancelled=True)
        self.assertEqual(r['status'],'cancelled')
        self.feedback(10.11);self.step.poll()
        self.assertEqual(len(self.adapter.calls),1)

    def test_uncertain_close_reply_never_retries_and_stop_failure_is_reported(self):
        self.arrive();self.feedback(10.32)
        self.c.config['arms']['right']['command_result_topic']='fake-test-topic'
        calls=[]
        def uncertain(*args,**kwargs):
            calls.append(args)
            if len(calls)>1:raise RuntimeError('stop transport failed')
            def confirm():raise TimeoutError('close receipt unknown')
            return confirm
        with patch.object(self.adapter,'arm',side_effect=uncertain):
            r=self.step.poll();self.feedback(10.4);again=self.step.poll()
        self.assertEqual(r['status'],'execution_unknown')
        self.assertEqual(r['stop_request'],'failed')
        self.assertEqual(again['stop_request'],'failed')
        self.assertEqual(len(calls),2)
        self.assertEqual(sum(abs(call[2]+1.2)<1e-6 for call in calls),1)
        self.assertFalse(self.c.motion_valid(self.token))

    def test_manual_stop_during_confirmation_latches_interruption(self):
        self.arrive();self.feedback(10.32)
        self.c.config['arms']['right']['command_result_topic']='fake-test-topic'
        requests=[]
        def request(*args,**kwargs):
            requests.append(kwargs.get('kind'))
            if kwargs.get('kind')=='gripper':
                return lambda:self.c.submit({'type':'arm.stop','arm':'right'},'browser')
        with patch.object(self.adapter,'arm',side_effect=request):
            r=self.step.poll();self.step.poll()
        self.assertEqual(r['status'],'interrupted')
        self.assertEqual(requests.count('gripper'),1)
        self.assertFalse(self.c.motion_valid(self.token))

    def test_preflight_cannot_be_used_by_browser_or_for_other_commands(self):
        from control import Rejected
        calls=[]
        guard=lambda *_:calls.append(True) or True
        for command,client in (
                ({'type':'gripper','arm':'right','position':-1.2},'browser'),
                ({'type':'arm.wrist'},self.token),
                ({'type':'arm.stop','arm':'right'},self.token)):
            with self.assertRaises(Rejected):self.c.submit(command,client,preflight=guard)
        self.assertEqual(calls,[]);self.assertEqual(self.adapter.calls,[])

    def test_deferred_preflight_does_not_change_command_epoch(self):
        self.feedback(10.01);epoch=self.c.command_epoch
        result=self.c.submit({'type':'gripper','arm':'right','position':-1.2},
            self.token,preflight=lambda *_:False)
        self.assertTrue(result['deferred']);self.assertFalse(result['ok'])
        self.assertEqual(self.c.command_epoch,epoch);self.assertEqual(self.adapter.calls,[])

    def test_driver_confirmation_does_not_hold_feedback_lock(self):
        import threading
        self.arrive();self.feedback(10.32)
        self.c.config['arms']['right']['command_result_topic']='fake-test-topic'
        confirmed=[]
        def request(*args,**kwargs):
            def confirm():
                event=threading.Event()
                def feedback():
                    with self.c.lock:event.set()
                thread=threading.Thread(target=feedback,daemon=True);thread.start()
                self.assertTrue(event.wait(1.),'Controller lock held while waiting for ROS confirmation')
                thread.join();confirmed.append(True)
            return confirm
        with patch.object(self.adapter,'arm',side_effect=request):r=self.step.poll()
        self.assertEqual(r['status'],'closing');self.assertEqual(confirmed,[True])

if __name__=='__main__':unittest.main()
