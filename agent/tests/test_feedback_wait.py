import copy
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch

from agent.feedback_wait import FeedbackWaitStep,feedback_pending
from agent.tests import test_grasp_pose_recovery as fixture
from agent.tests.test_battery_flow import review
from agent.battery_flow import BatteryFlow
from control import Controller


class FeedbackWaitTests(unittest.TestCase):
    def setUp(self):
        self.now=10.;self.valid=True
        self.data=dict(arms={'right':dict(joints=[0.]*6,gripper=-3.,fresh=True,received=10.)},
                       body=dict(height=12.,fresh=True,received=10.))
        self.c=SimpleNamespace(clock=lambda:self.now,snapshot=lambda:copy.deepcopy(self.data),
            motion_valid=lambda token:self.valid,log=Mock(),pause_trajectory_for_feedback=Mock(return_value=False))
        self.policy={'arm_policy':{'max_age_s':.3},'max_pair_skew_s':.15}
        self.step=Mock();self.step.retreat=None
        self.step.start.return_value={'status':'trajectory_running'}
        self.step.poll.return_value={'status':'trajectory_running'}
        self.factory=Mock(return_value=self.step)
        self.wait=FeedbackWaitStep(self.step,self.c,'owned',self.policy,'execute_pick',self.factory)

    def stale(self):self.data['body']['received']=9.8
    def fresh(self):
        self.data['body']['received']=self.now;self.data['arms']['right']['received']=self.now

    def test_existing_age_and_pair_limits_are_unchanged(self):
        self.assertFalse(feedback_pending(self.c,self.policy))
        self.stale();self.assertTrue(feedback_pending(self.c,self.policy))
        self.data['body']['received']=10.;self.data['arms']['right']['received']=9.6
        self.assertTrue(feedback_pending(self.c,self.policy))
        self.assertEqual(self.policy['arm_policy']['max_age_s'],.3)
        self.assertEqual(self.policy['max_pair_skew_s'],.15)

    def test_invalid_values_are_not_treated_as_temporary_delay(self):
        self.data['arms']['right']['joints'][0]=float('nan')
        with self.assertRaises(ValueError):feedback_pending(self.c,self.policy)

    def test_no_command_before_feedback_recovers(self):
        self.stale();self.assertTrue(self.wait.start()['waiting_for_feedback'])
        self.step.start.assert_not_called()
        for _ in range(3):
            self.now+=60.;self.assertTrue(self.wait.poll()['waiting_for_feedback'])
        self.c.pause_trajectory_for_feedback.assert_called_once()
        self.fresh()
        # A raw inert step has no arrival-monitor state to restart.
        self.step=SimpleNamespace(start=Mock(return_value={'status':'trajectory_running'}),poll=Mock(),cancel=Mock())
        self.wait.step=self.step
        self.assertEqual(self.wait.poll()['status'],'trajectory_running')
        self.step.start.assert_called_once()

    def test_stopped_moving_path_is_not_replayed_after_recovery(self):
        self.wait.start();self.c.pause_trajectory_for_feedback.return_value=True
        self.stale();self.assertTrue(self.wait.poll()['waiting_for_feedback'])
        self.fresh();self.data['arms']['right']['joints'][0]=.1;r=self.wait.poll()
        self.assertEqual(r['return_trace_rad'][0],[0.]*6)
        self.assertEqual(r['return_trace_rad'][-1],[.1,0.,0.,0.,0.,0.])
        self.assertTrue(r['feedback_replan_required'])
        self.step.poll.assert_not_called();self.step.start.assert_called_once()

    def test_manual_stop_during_wait_is_preserved(self):
        self.stale();self.wait.start();self.valid=False
        self.wait.poll();self.step.cancel.assert_called_once()

    def test_failed_stop_never_resumes(self):
        self.wait.start();self.stale()
        self.c.pause_trajectory_for_feedback.side_effect=RuntimeError('stop acknowledgement lost')
        with self.assertRaises(RuntimeError):self.wait.poll()
        self.assertFalse(self.wait.waiting)
        self.step.poll.assert_not_called()

    def test_controller_keeps_owner_only_after_stop_acknowledgement(self):
        c=SimpleNamespace(lock=threading.RLock(),motion_valid=lambda t:t=='owned',
            motion_trajectory={'side':'right'},config={'arms':{'right':{'stop_verified':True}}},
            adapter=SimpleNamespace(arm_stop_ready=lambda s:True,arm_stop=Mock()),
            begin_contact_stop=Mock(),_trajectory_abort=Mock())
        self.assertTrue(Controller.pause_trajectory_for_feedback(c,'owned'))
        self.assertIsNone(c.motion_trajectory);c.adapter.arm_stop.assert_called_once()
        c.begin_contact_stop.assert_not_called();c._trajectory_abort.assert_not_called()
        c.motion_trajectory={'side':'right'}
        c.adapter.arm_stop.side_effect=RuntimeError('lost acknowledgement')
        with self.assertRaises(RuntimeError):Controller.pause_trajectory_for_feedback(c,'owned')
        c.begin_contact_stop.assert_called_once();c._trajectory_abort.assert_called_once()


class FeedbackFlowTests(unittest.TestCase):
    setUp=fixture.FlowRecoveryTests.setUp

    def test_known_stopped_path_goes_to_point3_after_feedback_returns(self):
        base=self.tools.execute;failed=False
        def execute(name,args,ident):
            nonlocal failed
            if name=='execute_battery_approach' and not failed:
                failed=True
                self.store.state['feedback_restart']={'return_trace_rad':[[0.]*6]}
                return {'ok':False,'error':{'code':'feedback_replan_required','message':'feedback recovered'}}
            return base(name,args,ident)
        self.tools.execute=execute
        with patch.object(self.recovery,'recover',return_value={'completed_at':123.}) as recover:
            r=BatteryFlow(self.tools,review_plan=review,vision_mode='plan_f',recovery=self.recovery).run()
        self.assertEqual(r['status'],'completed');recover.assert_called_once()
        self.assertNotIn('feedback_restart',self.store.state)


class FeedbackReceiptTests(unittest.TestCase):
    def test_wait_freezes_execution_deadline_and_keeps_existing_request(self):
        import tempfile,time
        from pathlib import Path
        from agent.pick_backend import PickExecutionBackend
        ready=threading.Event()
        step=SimpleNamespace(start=lambda:{'status':'waiting_for_feedback','waiting_for_feedback':True},
            poll=lambda:({'status':'awaiting_hold_verification'} if ready.is_set() else
                         {'status':'waiting_for_feedback','waiting_for_feedback':True}),
            cancel=Mock(return_value={'status':'cancelled','stop_required':True}),timeout=Mock())
        with tempfile.TemporaryDirectory() as d:
            backend=PickExecutionBackend(step,request_id='run:wait',arguments={'plan_id':'p','plan':{'plan_id':'p','arm':'right'}},
                journal=Path(d)/'receipt.json',cancelled=lambda:False,timeout_s=.08)
            args={'plan_id':'p','plan':{'plan_id':'p','arm':'right'}}
            backend.invoke('execute_pick',args,'run:wait');time.sleep(.15)
            self.assertFalse(backend.done.is_set());step.timeout.assert_not_called()
            self.assertTrue(backend.status('execute_pick','run:wait')['data']['waiting_for_feedback'])
            ready.set();self.assertTrue(backend.wait(1.))
            self.assertEqual(backend.status('execute_pick','run:wait')['status'],'completed')

    def test_known_stop_receipt_clears_pending_without_claiming_motion_completed(self):
        import tempfile
        from agent.battery_tools import BatteryExecutionTools
        from agent.storage import RunStore
        from agent.contracts import AgentError
        with tempfile.TemporaryDirectory() as d:
            store=RunStore.create(d)
            try:
                store.state['mission']['stage']='PICK_APPROACH_PLANNED'
                store.state['pending']={'request_id':'request'}
                tools=BatteryExecutionTools.__new__(BatteryExecutionTools);tools.store=store
                receipt={'status':'rejected','execution_id':'request',
                         'data':{'feedback_replan_required':True,'return_trace_rad':[[0.]*6]}}
                with self.assertRaises(AgentError) as error:
                    tools._receipt('execute_battery_approach',{},receipt,physical=True,request_id='request')
                self.assertEqual(error.exception.code,'feedback_replan_required')
                self.assertIsNone(store.state['pending'])
                self.assertEqual(store.state['mission']['stage'],'PICK_APPROACH_PLANNED')
                self.assertTrue(store.state['feedback_restart']['return_trace_rad'])
            finally:store.close()


if __name__=='__main__':unittest.main()
