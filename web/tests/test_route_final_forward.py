import threading,unittest
from types import SimpleNamespace as NS
from navigation_session import NavigationSession
class Clock:
 def __init__(self):self.t=0.
 def __call__(self):return self.t
class Cancel:
 def __init__(self,clock):self.clock=clock;self.cancel=False
 def is_set(self):return self.cancel
 def wait(self,delay):self.clock.t+=delay;return self.cancel
class FinalForward(unittest.TestCase):
 def test_body_forward_225_for_four_seconds_then_scoped_stop(self):
  clock=Clock();cancel=Cancel(clock);commands=[];ended=[]
  class Transport:
   def __init__(self,submit):pass
   def send(self,cmd,**kw):commands.append((clock.t,cmd,kw))
   def end_output(self):ended.append(clock.t)
  ctrl=NS(motion_valid=lambda _:True,snapshot=lambda:dict(body=dict(fresh=True),nav_sdk_ready=True),submit=lambda *a:None)
  nav=NavigationSession(ctrl,NS(),client=NS());ctx=dict(cancelled=cancel,controller=ctrl,token=object())
  nav.finish_forward(ctx,clock=clock,transport_factory=Transport)
  self.assertTrue(commands);self.assertTrue(all(tuple(c)==(.225,0.,0.) for _,c,_ in commands));self.assertLessEqual(max(t for t,_,_ in commands),4.);self.assertAlmostEqual(ended[0],4.,places=5)
  self.assertTrue(all(0<kw['ttl_s']<=.15 for _,_,kw in commands))
 def test_operator_stop_cannot_complete_forward_action(self):
  clock=Clock();cancel=Cancel(clock);cancel.cancel=True;ended=[]
  class Transport:
   def __init__(self,submit):pass
   def send(self,*a,**kw):raise AssertionError('must not move')
   def end_output(self):ended.append(True)
  ctrl=NS(motion_valid=lambda _:True,submit=lambda *a:None)
  with self.assertRaises(ValueError):NavigationSession(ctrl,NS(),client=NS()).finish_forward(dict(cancelled=cancel,controller=ctrl,token=object()),clock=clock,transport_factory=Transport)
  self.assertTrue(ended)
 def test_timeout_event_bypasses_only_old_task_attention(self):
  nav=NavigationSession(NS(),NS(),client=NS(event=lambda *a,**kw:dict(ok=True)))
  nav.visit=dict(visit_id='v');nav.active=True;nav.context=dict(run_id='run',cancelled=threading.Event());seen=[]
  nav.await_channel=lambda ctx:seen.append(ctx.get('t2_timed_out'))
  nav._event('timeout',control_released=True,navigation_ready=True)
  self.assertEqual(seen,[True])
class FailureReceipt(unittest.TestCase):
 def test_fail_event_is_bounded_even_when_navigation_has_attention(self):
  nav=NavigationSession(NS(),NS(),client=NS(event=lambda *a,**kw:dict(ok=True)))
  nav.visit=dict(visit_id='v');nav.active=True;nav.context=dict(run_id='run',cancelled=threading.Event())
  nav.await_channel=lambda ctx:(_ for _ in ()).throw(AssertionError('failure receipt must not block stop'))
  self.assertTrue(nav._event('fail',error='worker lease lost')['ok'])
class FinalStageIntegration(unittest.TestCase):
 def test_terminal_action_runs_after_navigation_with_owned_token(self):
  import tempfile,time
  from pathlib import Path
  from test_control import Controller,FakeAdapter,configured
  from test_competition_task import Module
  from test_t2_navigation_current_place import Navigation
  from t2_test_task import make_t2_navigation_task
  cfg=configured();cfg['chassis_backend']='local';ctrl=Controller(cfg,FakeAdapter());ctrl.update_body(.18,[0.]*4,0.,0.,0.)
  for side in ('left','right'):ctrl.update_arm(side,[0.]*6,-1.,[0.]*6)
  nav=Navigation();trace=[];place=Module(trace,'place');place.readiness=lambda:dict(reference_valid=True)
  entered=threading.Event();done=threading.Event()
  def finish(ctx):
   self.assertEqual(nav.trace[-1],'acknowledge_t2');self.assertFalse(nav.active);self.assertTrue(ctrl.motion_valid(ctx['token']));self.assertTrue(ctrl.snapshot()['competition_reserved'])
   entered.set();done.wait(1.)
  nav.finish_forward=finish
  with tempfile.TemporaryDirectory() as root:
   task=make_t2_navigation_task(ctrl,Path(root),nav,place);self.addCleanup(task.close);self.addCleanup(done.set)
   task.action('browser',dict(action='start',request_id='final',expected_epoch=ctrl.command_epoch,environment_clear=True));self.assertTrue(nav.started.wait(1));self.assertFalse(entered.is_set());nav.arrived.set();self.assertTrue(entered.wait(1));self.assertEqual(task.snapshot()['phase'],'finish_forward');done.set()
   task.active['worker'].join(2);self.assertEqual(task.snapshot()['status'],'completed');self.assertFalse(ctrl.snapshot()['competition_reserved']);self.assertEqual(task.snapshot()['tasks'][-1]['id'],'finish_forward')
