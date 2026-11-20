"""Driver's real methods with inert dependencies; runnable without ROS or NumPy.

The AST loader omits imports and __init__ only: all driver method bodies execute
unchanged. WebChassis is the real task-scoped client with an inert submitter.
"""
import ast
import math
from pathlib import Path
import time
import types
import unittest
from unittest.mock import patch
from chassis_command import WebChassis
from recovery_budget import paused_budget

ROOT = Path(__file__).resolve().parents[1]

class Values(list):
    def all(self):return all(self)

class Array(list):
    def tolist(self):return list(self)

class Publish:
    def __init__(self):self.values=[]
    def publish(self,value):self.values.append(value)

def load_driver():
    tree=ast.parse((ROOT/'goal_drive.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Driver')
    cls.bases=[]
    ns=dict(math=math,time=time,paused_budget=paused_budget,rclpy=types.SimpleNamespace(ok=lambda:True),stamp=lambda s:s.sec+s.nanosec*1e-9,
            np=types.SimpleNamespace(isfinite=lambda xs:Values(math.isfinite(x) for x in xs)),
            planar=lambda p:Array([p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)]),
            Bool=lambda **kw:types.SimpleNamespace(**kw))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(ROOT/'goal_drive.py'),'exec'),ns)
    return ns['Driver']

Driver=load_driver()

class CancelGoalPolicy(unittest.TestCase):
    def setUp(self):
        self.calls=[];self.reject_ends=False
        def submit(payload):
            self.calls.append(dict(payload))
            return dict(ok=not(self.reject_ends and payload['type']=='chassis.end_output'))
        self.n=Driver.__new__(Driver);n=self.n
        n.transport=WebChassis(submit=submit);n.active=True;n.intent=Publish();n.finished=Publish()
        n.seq=n.seen=7;n.target=Array([1.,0.,0.]);n.route=object();n.arrow=self.goal()
        n.boot=0.;n.now=lambda:100.;n.ready_file=None;n.previous_alignment='alignment'
        self.finishes=[];self.overrides=[];self.messages=[];self.route_index=3
        n.status=self.messages.append
        n.get_logger=lambda:types.SimpleNamespace(warning=lambda *a,**kw:None)
        n.waypoints=types.SimpleNamespace(finish=self.finishes.append,
            from_self=lambda _:False,override=lambda:self.overrides.append(True),blocking=lambda:False)
        n.pending_output_ends=set();n.pending_route_finish=None;n._retrying_output_ends=False
        n.pending_output_task=None

    def goal(self,*,when=100.,frame='map',x=2.,q=1.):
        return types.SimpleNamespace(header=types.SimpleNamespace(frame_id=frame,
            stamp=types.SimpleNamespace(sec=int(when),nanosec=int((when-int(when))*1e9))),
            pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x,y=0.,z=0.),
            orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=q)))

    def test_cancel_retires_intent_before_transport_even_when_end_is_rejected(self):
        n=self.n;self.reject_ends=True;epoch=n.seq;old=n.transport.current
        observed=[];submit=n.transport.submit
        def capture(payload):
            observed.append((n.seq,n.target));return submit(payload)
        n.transport.submit=capture
        n.cancel('cancelled')
        self.assertGreater(n.seq,epoch);self.assertIsNone(n.target);self.assertIsNone(n.route)
        self.assertTrue(all(seq!=epoch and target is None for seq,target in observed))
        self.assertEqual(self.finishes,['cancelled']);self.assertEqual(n.pending_output_ends,{old})

    def test_independent_retry_confirms_same_scoped_end_without_reviving_goal(self):
        n=self.n;self.reject_ends=True;old=n.transport.current
        n.cancel('cancelled');n.retry_output_ends()
        self.reject_ends=False;n.retry_output_ends()
        self.assertFalse(n.pending_output_ends);self.assertIn(old,n.transport.ended);self.assertIsNone(n.target)
        self.assertEqual({(p['source_id'],p['task_id'],p['generation']) for p in self.calls},
            {(old.source_id,old.task_id,old.generation)})
        self.assertTrue(all(p['type']=='chassis.end_output' for p in self.calls))

    def test_valid_new_arrow_cannot_start_output_before_old_end_confirmation(self):
        n=self.n;self.reject_ends=True;n.cancel('cancelled');old=n.transport.current
        n.accept_goal(self.goal())
        self.assertEqual(n.target,[2.,0.,0.]);self.assertEqual(n.transport.current,old)
        self.assertTrue(all(p['type']=='chassis.end_output' for p in self.calls))
        self.reject_ends=False;n.retry_output_ends()
        self.assertEqual(n.target,[2.,0.,0.]);self.assertNotEqual(n.transport.current,old)

    def test_special_goal_waits_without_losing_semantic_target(self):
        n=self.n;self.reject_ends=True;n.cancel('cancelled');old=n.transport.current
        n.accept_special_goal(self.goal())
        self.assertEqual(n.target,[2.,0.,0.]);self.assertEqual(n.transport.current,old)
        self.assertTrue(n._special_execution)
        self.reject_ends=False;n.retry_output_ends()
        self.assertNotEqual(n.transport.current,old);self.assertEqual(n.target,[2.,0.,0.])

    def test_first_goal_also_quarantines_failure_to_end_initial_output_task(self):
        n=self.n;n.target=n.route=None;self.reject_ends=True;old=n.transport.current
        n.accept_goal(self.goal())
        self.assertEqual(n.target,[2.,0.,0.]);self.assertEqual(n.pending_output_ends,{old})
        self.reject_ends=False;n.retry_output_ends()
        self.assertNotEqual(n.transport.current,old)

    def test_output_end_wait_longer_than_execution_budget_does_not_spend_it(self):
        n=self.n;self.reject_ends=True;n.cancel('cancelled');n.accept_goal(self.goal())
        self.clock=100.;n.started=100.
        def spin(_):
            self.clock+=600.;self.reject_ends=False
        n.spin=spin
        with patch('recovery_budget.time.monotonic',lambda:self.clock):
            self.assertTrue(n.wait_output_ends(n.seq))
        self.assertEqual(self.clock-n.started,0.)

    def test_cancelling_queued_goal_cannot_allocate_or_revive_output_after_reconnect(self):
        n=self.n;self.reject_ends=True;n.cancel('cancelled');old=n.transport.current
        n.accept_goal(self.goal());n.cancel('cancelled')
        self.reject_ends=False;n.retry_output_ends()
        self.assertIsNone(n.target);self.assertEqual(n.transport.current,old)
        self.assertIsNone(n.pending_output_task)

    def test_arrival_does_not_advance_route_before_scoped_end_confirmation(self):
        n=self.n;self.reject_ends=True;n.cancel('arrived_stopped')
        self.assertEqual(self.finishes,[])
        self.reject_ends=False;n.retry_output_ends();n.retry_output_ends()
        self.assertEqual(self.finishes,['arrived_stopped'])

    def test_next_route_dispatch_during_end_retry_is_not_reported_ready_early(self):
        n=self.n;self.reject_ends=True;n.cancel('arrived_stopped');old=n.transport.current
        n.waypoints.finish=lambda _:n.accept_goal(self.goal())
        self.reject_ends=False
        self.assertFalse(n.retry_output_ends())
        self.assertEqual(n.transport.current,old);self.assertEqual(n.target,[2.,0.,0.])
        self.assertTrue(n.retry_output_ends());self.assertNotEqual(n.transport.current,old)

    def test_later_cancel_prevents_unconfirmed_arrival_from_advancing_route(self):
        n=self.n;self.reject_ends=True;n.cancel('arrived_stopped');n.cancel('cancelled')
        self.reject_ends=False;n.retry_output_ends()
        self.assertEqual(self.finishes,['cancelled'])

    def test_invalid_or_delayed_arrow_keeps_existing_goal_and_route(self):
        for goal in (self.goal(frame='odom'),self.goal(when=97.),self.goal(x=math.nan),
                     self.goal(q=0.),self.goal(when=101.)):
            with self.subTest(goal=goal):
                n=self.n;target=n.target;route=n.route;epoch=n.seq;old=n.transport.current
                n.plan_changed(goal)
                self.assertIs(n.target,target);self.assertIs(n.route,route);self.assertEqual(n.seq,epoch)
                self.assertEqual(n.transport.current,old);self.assertEqual(self.finishes,[])
                self.assertEqual(self.overrides,[]);self.assertEqual(self.calls,[])

    def test_direct_invalid_arrow_keeps_existing_goal(self):
        n=self.n;target=n.target;epoch=n.seq;n.accept_goal(self.goal(when=97.))
        self.assertIs(n.target,target);self.assertEqual(n.seq,epoch);self.assertEqual(self.calls,[])

if __name__=='__main__':unittest.main()
