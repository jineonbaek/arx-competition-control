"""Automatic opening uses the existing Controller with inert transport only."""
import unittest
from unittest.mock import patch
import test_pick_close_step as fixtures
from agent.gripper_open import OpenThenApproachStep

class Approach:
    def __init__(self,f):
        self.controller=f.c;self.token=f.token;self.monitor=f.monitor
        self.preflight=lambda *a:True;self.starts=0;self.cancels=0
    def start(self):self.starts+=1;return {'status':'awaiting_trajectory','stop_required':False}
    def poll(self,**kw):return {'status':'ready_to_reobserve','stop_required':False}
    def cancel(self):self.cancels+=1;return {'status':'cancelled','stop_required':True,'grasp_confirmed':False}

class OpenTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.CloseStepTests();self.f.setUp();self.f.feedback(10.,grip=-1.)
        self.f.c.wrist_reason=lambda *a:''
        self.f.c.config['arms']['right']['gripper_limits_raw']=[-3.14,0.]
        self.child=Approach(self.f)
        self.target=self.f.c.config['arms']['right']['gripper_open_raw']
        self.step=OpenThenApproachStep(self.child,self.f.q)
    def test_opens_once_waits_for_actual_feedback_then_starts_approach(self):
        self.assertEqual(self.f.adapter.calls,[])
        self.assertEqual(self.step.start()['status'],'opening_gripper')
        self.assertEqual(self.child.starts,0)
        self.assertAlmostEqual(self.f.adapter.calls[0][1][2],self.target)
        for t in (10.01,10.11,10.22):
            self.f.feedback(t,self.target);result=self.step.poll()
        self.assertEqual(result['status'],'awaiting_trajectory')
        self.assertEqual(self.child.starts,1)
        self.assertEqual(self.step.poll()['status'],'ready_to_reobserve')
        self.assertEqual(len(self.f.adapter.calls),1)
        with self.assertRaises(ValueError):self.step.start()
    def test_frozen_feedback_never_starts_approach(self):
        self.step.start();self.f.now=10.2
        self.assertTrue(self.step.poll()['stop_required']);self.assertEqual(self.child.starts,0)
    def test_drift_or_cancellation_never_starts_approach(self):
        for cancel in (False,True):
            if cancel:self.setUp()
            self.step.start()
            if not cancel:self.f.q[0]+=.1
            self.f.feedback(10.1,self.target)
            self.assertTrue(self.step.poll(cancelled=cancel)['stop_required'])
            self.assertEqual(self.child.starts,0);self.assertEqual(self.child.cancels,1)
    def test_unknown_submit_never_retries_opening(self):
        with patch.object(self.f.c,'submit',side_effect=TimeoutError('unknown')) as submit:
            self.assertTrue(self.step.start()['stop_required']);self.step.poll()
            submit.assert_called_once();self.assertEqual(self.child.starts,0)
    def test_moving_lift_blocks_open_request(self):
        self.f.c.wrist_reason=lambda *a:'lift moving'
        self.assertTrue(self.step.start()['stop_required'])
        self.assertEqual(self.f.adapter.calls,[])
        self.assertEqual(self.child.starts,0)

    def test_close_between_arrival_and_approach_dispatch_rejected(self):
        def start():
            self.f.feedback(10.23,grip=-1.)
            self.assertFalse(self.child.preflight(self.f.c.snapshot(),self.f.now))
            return {'status':'rejected','stop_required':True}
        self.child.start=start
        self.step.start()
        for t in (10.01,10.11,10.22):
            self.f.feedback(t,self.target);result=self.step.poll()
        self.assertTrue(result['stop_required'])

    def test_stop_failure_is_latched_and_never_retries_motion(self):
        self.step.start()
        with patch.object(self.child,'cancel',side_effect=RuntimeError('stop transport failed')) as stop:
            result=self.step.cancel()
            self.assertTrue(result['stop_required'])
            self.assertEqual(result['stop_request'],'failed')
            self.step.poll();stop.assert_called_once()
        self.assertEqual(self.child.starts,0)

    def test_backend_accepts_opening_stage_and_replays_without_new_command(self):
        import tempfile
        from pathlib import Path
        from agent.pick_backend import ApproachExecutionBackend
        payload={'plan_id':'p','plan':{'plan_id':'p','arm':'right'}}
        self.child.start=lambda:dict(status='opening_gripper',stop_required=False)
        self.child.poll=lambda:dict(status='ready_to_reobserve',stop_required=False,
                                   new_observation_required=True,close_permitted=False)
        with tempfile.TemporaryDirectory() as directory:
            backend=ApproachExecutionBackend(self.child,request_id='r',arguments=payload,
                journal=Path(directory)/'receipt.json',cancelled=lambda:False,timeout_s=1.)
            backend.invoke('execute_battery_approach',payload,'r')
            self.assertTrue(backend.wait(2.))
            self.assertEqual(backend.status('execute_battery_approach','r')['status'],'completed')
            with patch.object(self.child,'start') as start:
                replay=ApproachExecutionBackend(self.child,request_id='r',arguments=payload,
                    journal=Path(directory)/'receipt.json',cancelled=lambda:False,timeout_s=1.)
                self.assertEqual(replay.invoke('execute_battery_approach',payload,'r')['status'],'completed')
                start.assert_not_called()

    def test_preflight_rejection_and_invalid_target_send_nothing(self):
        self.step.preflight=lambda *a:False
        self.assertTrue(self.step.start()['stop_required']);self.assertEqual(self.f.adapter.calls,[])
        self.f.c.config['arms']['right']['gripper_open_raw']=float('nan')
        with self.assertRaises(ValueError):OpenThenApproachStep(self.child,self.f.q)

if __name__=='__main__':unittest.main()
