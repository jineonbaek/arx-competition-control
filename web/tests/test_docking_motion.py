import copy
import unittest
from types import SimpleNamespace
import numpy as np
import threading
from docking_motion import DockingMotion


class Controller:
    def __init__(self):
        self.t=10.;self.sent=[];self.started=[];self.trajectory=False;self.follow_lift=True;self.lock=threading.RLock()
        self.target=5.;self.adapter=SimpleNamespace(pending_lift=None)
        self.state=dict(arms={s:dict(joints=[0.]*6,gripper=-.8,fresh=True,received=self.t) for s in ('left','right')},
                        body=dict(height=5.,fresh=True,received=self.t,wheels=[0.]*4))
    def clock(self):return self.t
    def snapshot(self):return copy.deepcopy(self.state)
    def motion_valid(self,token):return token=='task'
    def start_trajectory(self,side,plan,token,**kwargs):
        kwargs['preflight'](self.snapshot(),self.t);self.started.append(plan);self.trajectory=True
    def poll_trajectory(self,token,**kwargs):
        self.state['arms']['right']['joints']=self.started[-1]['joint_waypoints_rad'][-1]
        self.trajectory=False;return dict(status='reference_complete')
    def submit(self,message,token,**kwargs):
        if kwargs.get('preflight'):kwargs['preflight'](self.snapshot(),self.t)
        self.sent.append((message,token))
        if message['type']=='lift':
            self.target=message['target']
            self.adapter.pending_lift=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(results=[SimpleNamespace(successful=True)]))
        return dict(ok=True)
    def sleep(self,seconds):
        self.t+=seconds
        for arm in self.state['arms'].values():arm['received']=self.t
        self.state['body']['received']=self.t
        if self.follow_lift:self.state['body']['height']=self.target


class MotionTests(unittest.TestCase):
    def test_coarse_lift_accepts_measured_approximate_height_only_when_requested(self):
        c=Controller();c.follow_lift=False;c.state['body']['height']=9.7
        m=self.make(c,timeout_s=1.,lift_tolerance_rad=.35)
        m.lift(10.,lambda:None)
        self.assertEqual(c.state['body']['height'],9.7)
        self.assertEqual([x[0]['type'] for x in c.sent],['lift'])
    def make(self,c,**kwargs):return DockingMotion(c,'task',guard=lambda *a:True,sleep=c.sleep,**kwargs)
    def test_arm_dispatches_complete_checked_curve_and_keeps_claw(self):
        c=Controller();m=self.make(c)
        p=dict(interpolation='segment_quintic_scalar',joint_waypoints_rad=[[0.]*6,[.01]*6],time_from_start_s=[0.,1.])
        m.arm([p],lambda:None)
        self.assertEqual(c.started,[p]);self.assertEqual(c.sent,[])
    def test_cancel_mid_arm_requests_stop_but_never_gripper(self):
        c=Controller();m=self.make(c);calls=[0]
        def cancel():
            calls[0]+=1
            if calls[0]>2:raise InterruptedError('cancel')
        with self.assertRaises(InterruptedError):m.arm([dict(joint_waypoints_rad=[[0.]*6,[.01]*6],time_from_start_s=[0.,1.])],cancel)
        self.assertTrue(any(x[0]['type']=='arm.stop' for x in c.sent));self.assertFalse(any(x[0]['type']=='gripper' for x in c.sent))
    def test_lift_waits_measured_arrival_and_does_not_reissue_goal(self):
        c=Controller();m=self.make(c)
        m.lift(5.2,lambda:None)
        self.assertEqual(c.state['body']['height'],5.2)
        self.assertEqual([x[0]['type'] for x in c.sent],['lift'])
    def test_lift_ack_alone_is_not_arrival(self):
        c=Controller();c.follow_lift=False;m=self.make(c,timeout_s=.6)
        with self.assertRaises(ValueError):m.lift(5.2,lambda:None)
        self.assertEqual(c.sent[-1][0]['type'],'lift.stop')
    def test_invalid_lift_goal_rejected_before_publication(self):
        c=Controller();m=self.make(c)
        with self.assertRaises(ValueError):m.lift(18.1,lambda:None)
        self.assertEqual(c.sent,[])
    def test_start_drift_is_checked_inside_dispatch_preflight(self):
        c=Controller();m=self.make(c)
        original=c.start_trajectory
        def start(*a,**kw):
            c.state['arms']['right']['joints'][0]=.1
            return original(*a,**kw)
        c.start_trajectory=start
        p=dict(joint_waypoints_rad=[[0.]*6,[.01]*6],time_from_start_s=[0.,1.])
        with self.assertRaises(ValueError):m.arm([p],lambda:None,expected_start=dict(joints=[0.]*6,height=5.))
        self.assertEqual(c.started,[])
    def test_lift_start_drift_rejects_before_goal(self):
        c=Controller();m=self.make(c);c.state['body']['height']=5.1
        with self.assertRaises(ValueError):m.lift(5.2,lambda:None,expected_start=dict(joints=[0.]*6,height=5.))
        self.assertFalse(any(x[0]['type']=='lift' for x in c.sent))
    def test_slow_lift_has_movement_deadline_separate_from_arm_settle_window(self):
        c=Controller();c.follow_lift=False;original=c.sleep
        def slow(seconds):
            if c.t>=22.:c.follow_lift=True
            original(seconds)
        c.sleep=slow;m=self.make(c,timeout_s=30.)
        m.lift(5.2,lambda:None)
        self.assertGreater(c.t,22.);self.assertEqual(c.state['body']['height'],5.2)


if __name__=='__main__':unittest.main()
