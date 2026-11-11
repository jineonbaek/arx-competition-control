"""Coarse task receipts, with an inert step and no hardware connections."""
import tempfile
import threading
import unittest
from pathlib import Path
from agent.pick_backend import CoarseExecutionBackend


class Step:
    def __init__(self):
        self.calls=[];self.finish=threading.Event();self.seen=threading.Event()
    def start(self):
        self.calls.append('start');return dict(status='lifting',stop_required=False)
    def poll(self):
        self.seen.set()
        return dict(status='ready_to_observe' if self.finish.is_set() else 'observing_motion',
                    stop_required=False,new_observation_required=True,achieved_lift_rad=6.)
    def cancel(self):
        self.calls.append('stop')
        return dict(status='cancelled',stop_required=True,stopped_verified=False,
                    stop_requests={'lift':'sent_unverified','right':'failed'})


class CoarseBackendTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.step=Step();self.payload={'plan_id':'p','plan':{'plan_id':'p','arm':'right'}}
        self.backend=self.make()
    def make(self):
        b=CoarseExecutionBackend(self.step,request_id='run:coarse',arguments=self.payload,
            journal=Path(self.temp.name)/'receipt.json',cancelled=lambda:False,timeout_s=.5)
        self.addCleanup(lambda:b.cancel_and_wait(2))
        return b
    def start(self):return self.backend.invoke('execute_coarse_alignment',self.payload,'run:coarse')
    def result(self):return self.backend.status('execute_coarse_alignment','run:coarse')
    def test_completion_requires_new_observation_not_grasp(self):
        self.start();self.assertTrue(self.step.seen.wait(1))
        self.assertEqual(self.result()['status'],'running')
        self.start();self.assertEqual(self.step.calls,['start'])
        self.step.finish.set();self.assertTrue(self.backend.wait(1))
        r=self.result();self.assertEqual(r['status'],'completed')
        self.assertEqual(r['data']['coarse_stage'],'ready_to_observe')
        self.assertTrue(r['data']['observer_ready']);self.assertTrue(r['data']['new_observation_required'])
        self.assertEqual(r['data']['achieved_lift_rad'],6.)
        self.assertFalse(r['data']['grasp_confirmed']);self.assertNotIn('gripper_settled',r['data'])
        self.assertEqual(self.make().invoke('execute_coarse_alignment',self.payload,'run:coarse')['status'],'completed')
        self.assertEqual(self.step.calls,['start'])
    def test_other_operation_journal_cannot_report_completion(self):
        import json
        path=Path(self.temp.name)/'receipt.json'
        path.write_text(json.dumps(dict(operation='execute_pick',payload=self.backend.payload,
            receipt=dict(status='completed',execution_id='run:coarse',data={}))))
        self.assertEqual(self.start()['status'],'unknown')
        self.assertEqual(self.step.calls,[])

    def test_wrong_operation_cannot_start(self):
        self.assertFalse(self.backend.available('execute_pick'))
        self.assertEqual(self.backend.invoke('execute_pick',self.payload,'run:coarse')['status'],'rejected')
        self.assertEqual(self.step.calls,[])
    def test_cancel_and_timeout_keep_multiaxis_stop_diagnostics(self):
        for cancel in (True,False):
            if not cancel:self.setUp()
            self.start();self.assertTrue(self.step.seen.wait(1))
            if cancel:self.backend.cancel_and_wait(1)
            else:self.assertTrue(self.backend.wait(1))
            r=self.result();self.assertEqual(r['status'],'unknown')
            self.assertTrue(r['data']['recovery_required']);self.assertFalse(r['data']['stopped_verified'])
            self.assertEqual(r['data']['stop_requests']['right'],'failed')
            self.assertEqual(self.step.calls,['start','stop'])
    def test_terminal_without_height_and_observation_evidence_fails(self):
        self.step.poll=lambda:dict(status='ready_to_observe',stop_required=False)
        self.start();self.assertTrue(self.backend.wait(1))
        self.assertEqual(self.result()['status'],'unknown')
        self.assertEqual(self.step.calls,['start','stop'])

if __name__=='__main__':unittest.main()
