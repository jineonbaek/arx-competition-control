import ast,tempfile,threading,time,unittest
from pathlib import Path
from test_control import Controller,FakeAdapter,configured
from test_competition_task import Module
class Navigation:
 def __init__(self):self.active=False;self.arrived=threading.Event();self.started=threading.Event();self.trace=[]
 def prepare(self,context):self.active=True;self.trace.append('register_t2')
 def begin(self,context):self.started.set();self.trace.append('navigate')
 def arrival(self,context,point):return dict(status='completed' if self.arrived.is_set() else 'running')
 def before_work(self,context,stage):
  if stage=='place':assert self.arrived.is_set();self.trace.append('accept_t2')
 def complete_route(self,context):self.trace.append('acknowledge_t2');self.active=False
 def stop(self,context):self.active=False;return dict(stopped=True)
 def close(self):self.active=False
 def check(self):return None
 def snapshot(self):return dict(route=dict(definition=dict(points=[dict(type='via'),dict(type='task',point_id='point_2')])))
class T2CurrentAfterNavigation(unittest.TestCase):
 def test_current_position_placement_only_starts_after_real_t2_arrival_stage(self):
  from t2_test_task import make_t2_navigation_task
  trace=[];config=configured();config['chassis_backend']='local';ctrl=Controller(config,FakeAdapter());ctrl.update_body(.18,[0.]*4,0.,0.,0.)
  for side in ('left','right'):ctrl.update_arm(side,[0.]*6,-1.,[0.]*6)
  nav=Navigation();place=Module(trace,'current_position_place');place.readiness=lambda:dict(reference_valid=True)
  with tempfile.TemporaryDirectory() as root:
   task=make_t2_navigation_task(ctrl,Path(root),nav,place);self.addCleanup(task.close)
   task.action('browser',dict(action='start',request_id='one',expected_epoch=ctrl.command_epoch,environment_clear=True))
   self.assertTrue(nav.started.wait(1));self.assertFalse(any(x[0]=='start' for x in trace));self.assertFalse(ctrl.snapshot()['motion_reserved'])
   nav.arrived.set();deadline=time.monotonic()+2
   while task.active['worker'].is_alive() and time.monotonic()<deadline:time.sleep(.01)
   self.assertFalse(task.active['worker'].is_alive());self.assertEqual(task.snapshot()['status'],'completed');self.assertEqual([x[1] for x in trace if x[0]=='start'],['current_position_place'])
   self.assertEqual(nav.trace,['register_t2','navigate','accept_t2','acknowledge_t2']);self.assertFalse(ctrl.snapshot()['competition_reserved'])
 def actual_task(self,offline=False):
  from t2_test_task import make_t2_navigation_task
  from t2_direct_task import T2CurrentPositionTask
  from t2_timed_placement import TimedT2Placement
  tree=ast.parse((Path(__file__).resolve().parents[1]/'src/server.py').read_text())
  assignment=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='t2_test_task' for t in n.targets))
  trace=[];adapter=FakeAdapter();config=configured();config['chassis_backend']='local';ctrl=Controller(config,adapter);ctrl.update_body(.18,[0.]*4,0.,0.,0.)
  for side in ('left','right'):ctrl.update_arm(side,[0.]*6,-1.,[0.]*6)
  nav=Navigation()
  if offline:
   def fail(context):raise ValueError('navigation offline; route unavailable')
   nav.prepare=fail
  place=Module(trace,'current_position_place');place.readiness=lambda:dict(reference_valid=True)
  tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
  bindings=dict(controller=ctrl,WEB_ROOT=Path(tmp.name)/'web',navigation=nav,fast_level=object(),make_t2_navigation_task=make_t2_navigation_task,T2CurrentPositionTask=T2CurrentPositionTask,TimedT2Placement=TimedT2Placement,CurrentPositionPlacement=lambda *args:place)
  task=eval(compile(ast.Expression(assignment.value),'<server T2 wiring>','eval'),bindings);self.addCleanup(task.close)
  return task,ctrl,nav,trace,adapter
 def wait_done(self,task):
  deadline=time.monotonic()+2
  while task.active['worker'].is_alive() and time.monotonic()<deadline:time.sleep(.01)
  self.assertFalse(task.active['worker'].is_alive())
 def test_actual_server_requires_navigation_and_never_places_when_offline(self):
  task,ctrl,nav,trace,adapter=self.actual_task(offline=True)
  task.action('browser',dict(action='start',request_id='one',expected_epoch=ctrl.command_epoch,environment_clear=True));self.wait_done(task)
  view=task.snapshot();self.assertEqual(view['status'],'needs_attention',view)
  self.assertIn('navigation offline',view['reason']);self.assertEqual(trace,[]);self.assertEqual(adapter.calls,[]);self.assertFalse(ctrl.snapshot()['competition_reserved'])
 def test_actual_server_waits_for_t2_arrival_before_place(self):
  task,ctrl,nav,trace,adapter=self.actual_task()
  task.action('browser',dict(action='start',request_id='one',expected_epoch=ctrl.command_epoch,environment_clear=True))
  self.assertTrue(nav.started.wait(1),'actual T2 endpoint skipped navigation');self.assertEqual(trace,[]);self.assertFalse(ctrl.snapshot()['motion_reserved'])
  nav.arrived.set();self.wait_done(task)
  self.assertEqual(task.snapshot()['status'],'completed');self.assertEqual([x[1] for x in trace if x[0]=='start'],['current_position_place']);self.assertEqual(nav.trace,['register_t2','navigate','accept_t2','acknowledge_t2']);self.assertEqual(adapter.calls,[])
 def test_page_requires_t2_navigation_before_real_task(self):
  page=(Path(__file__).resolve().parents[1]/'public/t2-test.html').read_text()
  self.assertIn('开始 T2 实测',page);self.assertIn('T2前一点',page);self.assertIn('到达T2后',page)
  self.assertNotIn('导航离线时也可单独放置',page);self.assertNotIn('不启动导航',page)
