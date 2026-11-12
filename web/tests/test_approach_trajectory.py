"""Pregrasp reuses trajectory feedback but cannot close the gripper."""
import tempfile
from pathlib import Path
import unittest
import test_pick_trajectory_step as fixtures
from agent.pick_execution import ApproachTrajectoryStep
from agent.pick_backend import ApproachExecutionBackend

class ApproachTests(unittest.TestCase):
    feedback=fixtures.PickTrajectoryTests.feedback
    launch=fixtures.PickTrajectoryTests.launch
    def setUp(self):
        fixtures.PickTrajectoryTests.setUp(self)
        self.execution=ApproachTrajectoryStep(self.c,self.token,self.monitor,self.plan,
                                             preflight=lambda *_:True)
    def test_reference_complete_still_requires_fresh_settling_and_never_closes(self):
        self.launch();self.state['status']='reference_complete'
        self.feedback(20.);self.assertEqual(self.execution.poll()['status'],'inserting')
        for t in (20.01,20.11,20.22):
            self.feedback(t);result=self.execution.poll()
        self.assertEqual(result['status'],'ready_to_reobserve')
        self.assertTrue(result['new_observation_required'])
        self.assertFalse(result['close_permitted'])
        self.feedback(20.32);self.execution.poll()
        self.assertFalse(any(c[0]=='arm' for c in self.adapter.calls))
        with self.assertRaises(ValueError):self.execution.start()
    def test_underreach_and_cancel_cannot_produce_success(self):
        fixtures.PickTrajectoryTests.test_reference_end_does_not_hide_underreach(self)
        self.setUp();self.launch();self.feedback(10.1)
        result=self.execution.poll(cancelled=True)
        self.assertTrue(result['stop_required'])
        self.assertNotEqual(result['status'],'ready_to_reobserve')
    def test_backend_preserves_new_frame_barrier_and_never_closes(self):
        self.feedback(10.)
        start=self.adapter.trajectory_start
        def finish(*args):
            result=start(*args)
            self.state.update(command_epoch=5,request_id='test-job',status='reference_complete')
            return result
        self.adapter.trajectory_start=finish
        poll=self.execution.poll
        def advance():
            self.feedback(self.now+.11)
            return poll()
        self.execution.poll=advance
        args={'plan_id':'p','plan':{'plan_id':'p','arm':'right'}}
        with tempfile.TemporaryDirectory() as directory:
            backend=ApproachExecutionBackend(self.execution,request_id='r',arguments=args,
                journal=Path(directory)/'receipt.json',cancelled=lambda:False,timeout_s=2.)
            try:
                backend.invoke('execute_battery_approach',args,'r')
                self.assertTrue(backend.wait(3.))
                result=backend.status('execute_battery_approach','r')
                self.assertEqual(result['status'],'completed',result)
                self.assertTrue(result['data']['new_observation_required'])
                self.assertTrue(result['data']['pose_reached'])
                self.assertFalse(result['data']['grasp_confirmed'])
                self.assertFalse(any(c[0]=='arm' for c in self.adapter.calls))
            finally:backend.cancel_and_wait(2.)
    def test_backend_rejects_success_without_observation_barrier(self):
        with self.assertRaises(ValueError):
            ApproachExecutionBackend._completed_data({})

if __name__=='__main__':unittest.main()
