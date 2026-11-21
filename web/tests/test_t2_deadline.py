import tempfile,threading,unittest,uuid
from pathlib import Path
from types import SimpleNamespace as NS
from task_handoff import HandoffHub,ProtocolError
try:from t2_timed_placement import TimedT2Placement
except ModuleNotFoundError:TimedT2Placement=None
class Clock:
 def __init__(self):self.t=0.
 def __call__(self):return self.t
class Module:
 preview_only=False
 def __init__(self):self.starts=0;self.stops=0;self.status='running';self.stop_ok=True
 def readiness(self):return dict(reference_valid=True)
 def start(self,ctx):self.starts+=1;return self.inspect(ctx)
 def inspect(self,ctx):return dict(status=self.status,reason='failed early' if self.status=='failed' else '',**({'completed_at_monotonic':self.completed_at} if hasattr(self,'completed_at') else {}))
 def stop(self,ctx):self.stops+=1;return dict(stopped=self.stop_ok)
class Recovery:
 def __init__(self):self.starts=0;self.done=False;self.stops=0
 def start(self,ctx,visit,goal=None):self.starts+=1;return self.inspect(ctx)
 def inspect(self,ctx):return dict(status='completed' if self.done else 'running',phase='right_arm_return',right_arm_ready=self.done,chassis_ready=self.done)
 def stop(self,ctx):self.stops+=1;return dict(stopped=True)
class Deadline(unittest.TestCase):
 def setUp(self):
  self.assertIsNotNone(TimedT2Placement,'T2 timed placement adapter missing')
  self.clock=Clock();self.module=Module();self.recovery=Recovery();self.nav=NS(visit=dict(point_id='point_2',goal_pose=[1.,2.,0.],mission_id='m',visit_id='v'))
  self.ctx=dict(request_id='one',run_id='run',cancelled=threading.Event(),controller=NS(motion_valid=lambda _:True),token=object())
  self.timed=TimedT2Placement(self.module,self.nav,Path('/tmp'),recovery=self.recovery,clock=self.clock)
 def test_deadline_starts_at_real_work_and_does_not_skip_early(self):
  self.timed.start(self.ctx);self.clock.t=179.999;self.assertEqual(self.timed.inspect(self.ctx)['status'],'running');self.assertEqual(self.module.stops,0);self.assertEqual(self.recovery.starts,0)
 def test_timeout_stops_then_waits_for_arm_and_chassis_recovery_before_skip(self):
  self.timed.start(self.ctx);self.clock.t=180.;r=self.timed.inspect(self.ctx);self.assertEqual(r['status'],'running');self.assertEqual(self.module.stops,1);self.assertEqual(self.recovery.starts,1)
  self.recovery.done=True;r=self.timed.inspect(self.ctx);self.assertEqual(r['status'],'completed');self.assertEqual(r['outcome'],'timed_out');self.assertFalse(r['placement_confirmed']);self.assertTrue(r['recovery']['right_arm_ready']);self.assertEqual(self.module.starts,1)
 def test_unconfirmed_stop_cannot_start_recovery_or_continue(self):
  self.timed.start(self.ctx);self.module.stop_ok=False;self.clock.t=180.;self.assertEqual(self.timed.inspect(self.ctx)['status'],'running');self.assertEqual(self.recovery.starts,0)
 def test_normal_completion_has_no_recovery(self):
  self.timed.start(self.ctx);self.module.status='completed';self.clock.t=12.;self.assertEqual(self.timed.inspect(self.ctx)['status'],'completed');self.assertEqual(self.recovery.starts,0)
 def test_completed_before_deadline_survives_late_poll(self):
  self.timed.start(self.ctx);self.module.status='completed';self.module.completed_at=179.9;self.clock.t=180.1
  self.assertEqual(self.timed.inspect(self.ctx)['status'],'completed');self.assertEqual(self.recovery.starts,0)
 def test_completed_after_deadline_requires_recovery(self):
  self.timed.start(self.ctx);self.module.status='completed';self.module.completed_at=180.1;self.clock.t=180.2
  self.assertEqual(self.timed.inspect(self.ctx)['status'],'running');self.assertEqual(self.recovery.starts,1)
 def test_failed_early_waits_original_deadline_without_restarting_work(self):
  self.timed.start(self.ctx);self.module.status='failed';self.clock.t=10.;self.assertEqual(self.timed.inspect(self.ctx)['status'],'running');self.assertEqual(self.recovery.starts,0)
  self.clock.t=180.;self.timed.inspect(self.ctx);self.assertEqual(self.recovery.starts,1);self.assertEqual(self.module.starts,1)
 def test_operator_stop_prevents_timeout_recovery(self):
  self.timed.start(self.ctx);self.ctx['cancelled'].set();self.clock.t=181.;r=self.timed.inspect(self.ctx);self.assertEqual(r['status'],'stopped');self.assertEqual(self.recovery.starts,0)
class TimeoutProtocol(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.clock=Clock();self.h=HandoffHub(Path(self.tmp.name)/'journal.json',clock=self.clock)
  self.token=self.h.register(dict(worker_id='battery',instance_id=str(uuid.uuid4()),point_ids=['point_2','point_1']))['token']
 def begin(self,point='point_2'):
  self.a=self.h.begin(dict(mission_id=str(uuid.uuid4()),point_id=point,route_point_id='p',map_id='map',goal_pose=[0.,0.,0.],arrival_pose=[0.,0.,0.],pose_stamp_ns=1));self.event('accept')
 def event(self,event,**kw):return self.h.event(self.token,dict(request_id=str(uuid.uuid4()),event=event,**{k:self.a[k] for k in ['mission_id','visit_id','point_id']},**kw))
 def advance(self,t):
  while self.clock.t+1<t:self.clock.t+=1;self.h.heartbeat(self.token)
  self.clock.t=t;self.h.heartbeat(self.token)
 def test_authenticated_timeout_after180_is_resumable_but_not_placement_success(self):
  self.begin();self.advance(180.);r=self.event('timeout',navigation_ready=True,control_released=True)
  self.assertEqual(r['status'],'resume_pending');self.h.acknowledge(self.a['visit_id']);self.assertEqual(self.h.active['outcome'],'timed_out')
 def test_early_wrong_point_or_operator_cancel_cannot_skip(self):
  self.begin();self.advance(179.99)
  with self.assertRaises(ProtocolError):self.event('timeout',navigation_ready=True,control_released=True)
  self.advance(180.);self.h.cancel()
  with self.assertRaises(ProtocolError):self.event('timeout',navigation_ready=True,control_released=True)
if __name__=='__main__':unittest.main()
class TimeoutRouteIntegration(TimeoutProtocol):
 def test_recovered_timeout_releases_same_visit_to_tail_only_once(self):
  from mission_route import MissionRoute
  from navigation_session import NavigationSession
  route=MissionRoute('map')
  for p in [dict(id='entry',type='special',pose=[0.,0.,0.]),dict(id='t2',type='task',point_id='point_2',pose=[1.,0.,0.]),dict(id='tail',type='special',pose=[2.,0.,0.])]:route.add(p)
  route.start_t2(['point_2'],[-1.,0.,0.]);route.finish('arrived_stopped',[0.,0.,0.]);route.finish('arrived_stopped',[1.,0.,0.]);self.begin()
  ctrl=NS(motion_valid=lambda _:True,snapshot=lambda:dict(chassis_active=False,motion_reserved=False));ctx=dict(request_id='place',run_id='r',cancelled=threading.Event(),controller=ctrl,token=object())
  nav=NavigationSession(ctrl,NS(),client=NS());nav.visit=self.a;nav.context=ctx;nav.active=True;nav.latest=self.h.state()
  recovery=Recovery();placement=Module();timed=TimedT2Placement(placement,nav,Path('/tmp'),recovery=recovery,clock=self.clock);timed.start(ctx)
  self.advance(180.);self.assertEqual(timed.inspect(ctx)['status'],'running');self.assertEqual(route.state,'waiting_task');self.assertEqual(self.h.active['status'],'running')
  recovery.done=True;result=timed.inspect(ctx);self.assertEqual(result['outcome'],'timed_out')
  self.event('fail',error='original placement held');nav.latest=self.h.state()
  def event(visit,event,**values):
   return self.h.event(self.token,dict(event=event,**{k:visit[k] for k in ['mission_id','visit_id','point_id']},**values))
  nav.client=NS(event=event);nav.complete_visit(dict(ctx,t2_timed_out=True));self.assertEqual(self.h.active['status'],'resume_pending')
  self.h.acknowledge(self.a['visit_id']);self.assertEqual(self.h.active['outcome'],'timed_out');self.assertEqual(route.release_task([1.,0.,0.])['id'],'tail');self.assertTrue(route.running)
  route.finish('arrived_stopped',[2.,0.,0.]);self.assertEqual(route.state,'completed');self.assertEqual(placement.starts,1);self.assertEqual(recovery.starts,1)
 def test_point_one_timeout_rejected_even_after180(self):
  self.begin('point_1');self.advance(181.)
  with self.assertRaises(ProtocolError):self.event('timeout',navigation_ready=True,control_released=True)
 def test_new_visit_does_not_inherit_previous_timer(self):
  self.begin();self.advance(181.);self.event('complete',navigation_ready=True,control_released=True);self.h.acknowledge(self.a['visit_id']);self.begin()
  with self.assertRaises(ProtocolError):self.event('timeout',navigation_ready=True,control_released=True)
