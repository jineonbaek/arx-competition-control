"""Controller trajectory ownership with fake transport; no ROS or motor calls."""
import copy
import threading
import unittest
from unittest.mock import patch
from test_control import configured, FakeAdapter, Controller, Rejected


class TrajectoryOwnerTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.adapter=FakeAdapter()
        self.c=Controller(configured(),self.adapter,lambda:self.now)
        for side in ('left','right'):self.c.update_arm(side,[0.]*6,-1.,[0.]*6)
        self.c.update_body(5.,[0.]*4,0.,0.,0.)
        self.c.wrist_reason=lambda side:''
        self.token=self.c.reserve_motion(self.c.command_epoch)
        self.state=dict(driver_instance='one',command_epoch=1,request_id='',status='idle',ready=True,enabled=True)
        self.handle=dict(driver_instance='one',command_epoch=1,request_id='job')
        self.adapter.trajectory_status=lambda side:copy.deepcopy(self.state)
        self.adapter.trajectory_keepalive=lambda side,handle:self.adapter.calls.append(('heartbeat',handle))
        self.confirm=lambda:None
        def start(side,plan,expected):
            self.adapter.calls.append(('trajectory',plan))
            return self.handle,self.confirm
        self.adapter.trajectory_start=start
        self.plan={'fixture':True}

    def start(self):return self.c.start_trajectory('right',self.plan,self.token,preflight=lambda s,t:True)

    def running(self):self.state.update(request_id='job',command_epoch=2,status='running')

    def test_only_owner_with_preflight_can_publish_no_http_command(self):
        for token in ('browser',None,object()):
            with self.assertRaises(Rejected):self.c.start_trajectory('right',self.plan,token,preflight=lambda s,t:True)
        with self.assertRaises(Rejected):self.c.start_trajectory('right',self.plan,self.token,preflight=lambda s,t:False)
        with self.assertRaises(Rejected):self.c.submit({'type':'arm.trajectory','arm':'right'},'browser')
        self.assertEqual(self.adapter.calls,[])
        self.start()
        with self.assertRaises(Rejected):self.start()
        with self.assertRaises(Rejected):self.c.submit({'type':'lift','target':6.},self.token)
        self.assertEqual(len(self.adapter.calls),1)

    def test_receipt_latency_then_running_then_reference_complete_not_arrival(self):
        self.start()
        self.assertEqual(self.c.poll_trajectory(self.token,preflight=lambda s,t:True)['status'],'awaiting_driver_status')
        self.running()
        self.assertEqual(self.c.poll_trajectory(self.token,preflight=lambda s,t:True)['status'],'running')
        self.assertEqual(self.adapter.calls[-1][0],'heartbeat')
        self.state['status']='reference_complete'
        r=self.c.poll_trajectory(self.token,preflight=lambda s,t:True)
        self.assertFalse(r['arrived']);self.assertEqual(r['status'],'reference_complete')
        self.assertTrue(self.c.motion_valid(self.token));self.assertIsNone(self.c.motion_trajectory)

    def test_interrupt_during_confirmation_does_not_block_feedback_and_never_resumes(self):
        def confirm():
            done=threading.Event()
            def stop():
                self.c.submit({'type':'arm.stop','arm':'right'},'browser');done.set()
            t=threading.Thread(target=stop,daemon=True);t.start()
            self.assertTrue(done.wait(.5));t.join()
        self.confirm=confirm
        with self.assertRaises(Rejected):self.start()
        self.assertFalse(self.c.motion_valid(self.token))
        self.assertIsNone(self.c.release_motion(self.token))
        self.assertFalse(self.c.snapshot()['motion_reserved'])

    def test_fault_new_driver_and_check_failure_stop_once_keep_ownership(self):
        for fault in ('driver','failed','preflight','stale','initial_timeout','epoch','disabled','not_ready'):
            with self.subTest(fault=fault):
                self.setUp();self.start();self.running()
                if fault=='driver':self.state['driver_instance']='restarted'
                if fault=='epoch':self.state.update(status='reference_complete',command_epoch=3)
                if fault=='disabled':self.state.update(request_id='',command_epoch=1,status='idle',enabled=False)
                if fault=='not_ready':self.state.update(request_id='',command_epoch=1,status='idle',ready=False)
                if fault=='failed':self.state['status']='failed'
                if fault=='stale':self.adapter.trajectory_status=lambda side:None
                if fault=='initial_timeout':self.state.update(request_id='',command_epoch=1,status='idle');self.now+=.4
                for _ in range(2):
                    with self.assertRaises(Rejected):self.c.poll_trajectory(self.token,preflight=lambda s,t:fault!='preflight')
                self.assertFalse(self.c.motion_valid(self.token));self.assertIsNotNone(self.c.motion_owner)
                self.assertEqual(sum(call[0]=='arm_stop' for call in self.adapter.calls),1)
                self.assertFalse(any(call[0]=='heartbeat' for call in self.adapter.calls))

    def test_unknown_confirmation_requires_stop_verification(self):
        def confirm():raise Rejected('timeout unknown')
        self.confirm=confirm
        with self.assertRaisesRegex(Rejected,'timeout'):self.start()
        self.assertEqual(self.adapter.calls[-1][0],'arm_stop')
        self.assertIsNone(self.c.release_motion(self.token))
        self.assertFalse(self.c.snapshot()['motion_reserved'])

    def test_preflight_cannot_stop_then_still_publish(self):
        def check(s,t):self.c.interrupt_motion(self.token);return True
        with self.assertRaises(Rejected):self.c.start_trajectory('right',self.plan,self.token,preflight=check)
        self.assertEqual(self.adapter.calls,[])

if __name__=='__main__':unittest.main()
