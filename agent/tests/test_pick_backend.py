"""Agent receipt integration with an inert execution step; no ROS or CAN."""
import copy
import tempfile
import threading
import unittest
from pathlib import Path
from agent.config import Config
from agent.storage import RunStore
from agent.tools import ToolRegistry
from agent.pick_backend import PickExecutionBackend


class Step:
    def __init__(self):
        self.calls=[]
        self.finish=threading.Event()
        self.seen=threading.Event()
        self.fail=False
    def start(self):
        self.calls.append('start')
        return dict(status='awaiting_trajectory',stop_required=False)
    def poll(self,*,cancelled=False):
        self.seen.set()
        if self.fail:raise RuntimeError('feedback failed')
        if cancelled:return self.cancel()
        return dict(status='awaiting_hold_verification' if self.finish.is_set() else 'trajectory_running',
                    stop_required=False,grasp_confirmed=False)
    def cancel(self):
        self.calls.append('stop')
        return dict(status='cancelled',stop_required=True,stop_request='sent_unverified',grasp_confirmed=False)


class PickBackendTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.store=RunStore.create(Path(self.temp.name));self.addCleanup(self.store.close)
        self.store.state['mission'].update(stage='PICK_PLANNED',task={},target={'target_id':'battery'},
            plan={'plan_id':'p','arm':'right','candidate_id':'c'})
        self.payload={'plan_id':'p','task':{},'target':{'target_id':'battery'},'slot':None,
                      'plan':self.store.state['mission']['plan']}
        self.ident=self.store.state['run_id']+':execute'
        self.journal=self.store.path/'pick-receipt.json'
        self.step=Step()
        self.backend=self.make()
        self.tools=ToolRegistry(Config(motion_enabled=True),self.store,observations=object(),
                               backends=self.backend,allowed_tools={'execute_pick','get_execution_status'})
    def make(self):
        backend=PickExecutionBackend(self.step,request_id=self.ident,arguments=self.payload,
            journal=self.journal,cancelled=lambda:self.store.cancelled,timeout_s=1.)
        self.addCleanup(lambda:backend.cancel_and_wait(2.))
        return backend
    def invoke(self):return self.backend.invoke('execute_pick',self.payload,self.ident)
    def test_tool_submit_and_query_advance_only_after_measured_closure(self):
        result=self.tools.execute('execute_pick',{'plan_id':'p'},'execute')
        self.assertFalse(result['ok']);self.assertIsNotNone(self.store.state['pending'])
        self.assertEqual(result['data']['execution_receipt']['execution_id'],self.ident)
        self.assertFalse(result['data']['execution_receipt']['data']['grasp_confirmed'])
        self.assertTrue(self.step.seen.wait(1.))
        self.assertEqual(self.store.state['mission']['stage'],'PICK_PLANNED')
        self.step.finish.set();self.assertTrue(self.backend.wait(2.))
        result=self.tools.execute('get_execution_status',{},'query')
        self.assertTrue(result['ok'],result)
        self.assertEqual(self.store.state['mission']['stage'],'PICK_EXECUTED')
        self.assertFalse(result['data']['grasp_confirmed'])
        self.assertEqual(result['data']['pick_stage'],'awaiting_hold_verification')
        self.assertIsNone(self.store.state['pending'])
        self.tools.execute('execute_pick',{'plan_id':'p'},'execute')
        self.assertEqual(self.step.calls,['start'])
    def test_duplicate_context_never_restarts_and_conflict_is_rejected(self):
        self.invoke();self.assertTrue(self.step.seen.wait(1.))
        self.invoke();self.assertEqual(self.step.calls,['start'])
        other=copy.deepcopy(self.payload);other['target']['target_id']='other'
        self.assertEqual(self.backend.invoke('execute_pick',other,self.ident)['status'],'rejected')
        self.assertEqual(self.backend.invoke('execute_pick',self.payload,'other')['status'],'rejected')
        self.assertEqual(self.backend.status('execute_pick','other')['status'],'unknown')
        self.assertEqual(self.step.calls,['start'])
    def test_cancel_or_exception_retains_pending_and_never_reports_success(self):
        for error in (False,True):
            with self.subTest(error=error):
                if error:self.setUp()
                self.tools.execute('execute_pick',{'plan_id':'p'},'execute')
                self.assertTrue(self.step.seen.wait(1.))
                if error:self.step.fail=True
                else:(self.store.path/'cancel.flag').touch()
                self.assertTrue(self.backend.wait(2.))
                result=self.tools.execute('get_execution_status',{},'query')
                self.assertFalse(result['ok']);self.assertIsNotNone(self.store.state['pending'])
                receipt=self.backend.status('execute_pick',self.ident)
                self.assertEqual(receipt['status'],'unknown')
                self.assertFalse(receipt['data']['grasp_confirmed'])
                self.assertEqual(self.step.calls.count('stop'),1)
                self.invoke();self.assertEqual(self.step.calls.count('start'),1)
    def test_existing_running_journal_is_unknown_after_restart_never_replayed(self):
        self.invoke();self.assertTrue(self.step.seen.wait(1.))
        second=self.make()
        self.assertEqual(second.invoke('execute_pick',self.payload,self.ident)['status'],'unknown')
        self.assertEqual(self.step.calls,['start'])
        self.step.finish.set();self.assertTrue(self.backend.wait(2.))
        self.assertEqual(second.status('execute_pick',self.ident)['status'],'completed')
    def test_pre_cancelled_and_unknown_operations_cannot_start(self):
        (self.store.path/'cancel.flag').touch()
        self.assertFalse(self.backend.available('verify_hold'))
        self.assertEqual(self.backend.invoke('verify_hold',self.payload,self.ident)['status'],'rejected')
        self.invoke();self.assertTrue(self.backend.wait(2.))
        self.assertEqual(self.backend.status('execute_pick',self.ident)['status'],'rejected')
        self.assertEqual(self.step.calls,[])
    def test_journal_failure_stops_once_and_retains_unknown_receipt(self):
        from unittest.mock import patch
        with patch('agent.pick_backend.atomic_json',side_effect=OSError('disk full')):
            self.invoke();self.assertTrue(self.backend.wait(2.))
        result=self.backend.status('execute_pick',self.ident)
        self.assertEqual(result['status'],'unknown')
        self.assertTrue(result['data']['recovery_required'])
        self.assertEqual(self.step.calls,['start','stop'])
        self.assertEqual(self.make().invoke('execute_pick',self.payload,self.ident)['status'],'unknown')
        self.assertEqual(self.step.calls,['start','stop'])

    def test_malformed_step_result_does_not_advance(self):
        self.step.poll=lambda **kwargs:dict(status='success')
        self.invoke();self.assertTrue(self.backend.wait(2.))
        result=self.backend.status('execute_pick',self.ident)
        self.assertEqual(result['status'],'unknown')
        self.assertFalse(result['data']['grasp_confirmed'])
        self.assertEqual(self.step.calls,['start','stop'])

    def test_deadline_cancels_even_with_repeated_running_receipts(self):
        self.invoke();self.assertTrue(self.backend.wait(2.))
        self.assertEqual(self.backend.status('execute_pick',self.ident)['status'],'unknown')
        self.assertEqual(self.step.calls,['start','stop'])

if __name__=='__main__':unittest.main()
