import copy,tempfile,threading,time,unittest,sys
from pathlib import Path
from test_control import Controller,configured,FakeAdapter
from navigation_session import NavigationSession
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'navigation/polarbear_adapt'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'navigation'))
from mission_route import MissionRoute
from task_handoff import HandoffHub
from task_http import TaskServer
from task_client import TaskClient
class Route:
 def __init__(self,hub):
  self.hub=hub;self.r=MissionRoute('map');self.sent=[];self.results={}
  for i in range(1,7):self.r.add(dict(id=str(i),type='task' if i in (2,6) else 'special',pose=[float(i),0.,0.],**({'point_id':'point_1' if i==2 else 'point_2'} if i in (2,6) else {})))
  self.show()
 def show(self):
  if hasattr(self,'start_ident'):self.results[self.start_ident].update(state=self.r.state)
  self.hub.set_route(self.r.definition(),state=self.r.state,running=self.r.running,index=self.r.index)
 def snapshot(self):
  label={'draft':'待开始','running':'执行中','waiting_task':'等待任务','cancelled':'已取消','completed':'全部到达并停稳'}[self.r.state]
  return dict(route=dict(available=True,definition=self.r.definition(),status=label,drive_status=''))
 def command(self,action,ident,route_id=None,expected_definition=None,start_request_id=None):
  self.sent.append(action);self.results[ident]=dict(status='published')
  if action=='start_t2':
   assert expected_definition==self.r.definition() and route_id==self.r.route_id
   self.r.start_t2(self.hub.available_points(),[5.,0.,0.]);self.start_ident=ident;self.results[ident]=dict(status='started',mission_id=self.r.mission_id,state=self.r.state)
  elif action=='cancel':
   assert start_request_id==self.start_ident
   self.r.cancel();self.hub.cancel();self.results[ident]=dict(status='cancelled')
  else:raise AssertionError('T2 must not clear or start full route')
  self.show()
 def receipt(self,ident):return self.results.get(ident)
class T2SessionTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.h=HandoffHub(Path(self.tmp.name)/'state.json');self.server=TaskServer(self.h,0);self.addCleanup(self.server.close)
  c=configured();c['chassis_backend']='local';self.ctrl=Controller(c,FakeAdapter());self.ctrl.update_body(.18,[0.]*4,0.,0.,0.)
  self.lease=self.ctrl.reserve_competition(self.ctrl.command_epoch);self.route=Route(self.h)
  self.n=NavigationSession(self.ctrl,self.route,TaskClient('http://127.0.0.1:'+str(self.server.port)));self.addCleanup(self.n.close)
  self.context=dict(run_id='t2-test',competition_token=self.lease,controller=self.ctrl,cancelled=threading.Event(),token=None,route_scope='t2_test')
 def poll_arrival(self):
  deadline=time.monotonic()+2
  while time.monotonic()<deadline:
   result=self.n.arrival(self.context,'point_2')
   if result['status']=='completed':return result
   time.sleep(.03)
  self.fail(str(result))
 def test_real_point2_handoff_and_exit_without_t1_or_route_replacement(self):
  before=self.route.r.definition();self.n.prepare(self.context);self.assertEqual(self.h.state()['point_ids'],['point_2']);self.n.begin(self.context)
  self.assertEqual(self.route.sent,['start_t2']);visit=self.h.begin(dict(mission_id=self.route.r.mission_id,point_id='point_2',route_point_id='6',map_id='map',goal_pose=[6.,0.,0.],arrival_pose=[6.,0.,0.],pose_stamp_ns=1))
  self.poll_arrival();self.ctrl.set_competition_navigation(self.lease,False);token=self.ctrl.reserve_motion(self.ctrl.command_epoch,competition=self.lease);self.context['token']=token
  self.n.before_work(self.context,'place');self.assertEqual(self.h.state()['task']['status'],'running')
  self.ctrl.release_motion(token);self.context['token']=None;self.ctrl.set_competition_navigation(self.lease,True)
  error=[]
  def finish():
   try:self.n.complete_route(self.context)
   except Exception as e:error.append(e)
  thread=threading.Thread(target=finish);thread.start();deadline=time.monotonic()+2
  while self.h.state()['task']['status']!='resume_pending' and time.monotonic()<deadline:time.sleep(.01)
  self.assertEqual(self.h.state()['task']['status'],'resume_pending');self.h.acknowledge(visit['visit_id'])
  r=self.route.r;r.finish('arrived_stopped',[5.,0.,0.]);r.finish('arrived_stopped',[6.,0.,0.]);self.assertIsNone(r.release_task([6.,0.,0.]));self.route.show()
  thread.join(3);self.assertFalse(thread.is_alive());self.assertEqual(error,[]);self.assertFalse(self.n.active);self.assertEqual(r.definition(),before);self.ctrl.release_competition(self.lease)
 def test_edit_after_prepare_refuses_start_and_stop_preserves_full_draft(self):
  self.n.prepare(self.context);self.route.r.points[4]['pose'][1]=.2
  with self.assertRaises(ValueError):self.n.begin(self.context)
  self.assertEqual(self.route.sent,[]);self.n.stop(self.context);self.assertEqual(len(self.route.r.points),6);self.ctrl.release_competition(self.lease)
 def test_stop_after_start_uses_scoped_cancel_and_keeps_all_points(self):
  self.n.begin(self.context);self.assertTrue(self.n.stop(self.context)['stopped']);self.assertEqual(self.route.sent,['start_t2','cancel']);self.assertEqual(len(self.route.r.points),6);self.ctrl.release_competition(self.lease)
 def test_rejected_start_is_failure_and_cancelled_without_deleting_draft(self):
  def reject(action,ident,route_id=None,expected_definition=None,start_request_id=None):
   self.route.sent.append(action);self.route.results[ident]=dict(status='rejected' if action=='start_t2' else 'cancelled',reason='route changed')
  self.route.command=reject;self.n.begin(self.context)
  self.assertEqual(self.n.arrival(self.context,'point_2')['status'],'failed')
  self.assertTrue(self.n.stop(self.context)['stopped']);self.assertFalse(self.n.active);self.assertEqual(len(self.route.r.points),6)
 def test_previous_completed_view_does_not_fail_new_started_request(self):
  self.n.begin(self.context);original=self.route.snapshot
  self.route.snapshot=lambda:dict(route=original()['route']|dict(status='T2单独实测；全部到达并停稳 6/6'))
  self.assertEqual(self.n.arrival(self.context,'point_2')['status'],'running')
 def test_stop_serializes_with_begin_and_cancellation_prevents_late_publish(self):
  self.n.prepare(self.context);entered=threading.Event();release=threading.Event();original=self.n.client.heartbeat
  def held():entered.set();release.wait(2);return original()
  self.n.client.heartbeat=held;errors=[]
  def begin():
   try:self.n.begin(self.context)
   except ValueError as e:errors.append(str(e))
  starter=threading.Thread(target=begin);starter.start();self.assertTrue(entered.wait(1))
  self.context['cancelled'].set();stopper=threading.Thread(target=lambda:self.n.stop(self.context));stopper.start();release.set()
  starter.join(3);stopper.join(3);self.assertFalse(starter.is_alive());self.assertFalse(stopper.is_alive())
  self.assertEqual(self.route.sent,[]);self.assertFalse(self.n.active);self.assertTrue(errors)
if __name__=='__main__':unittest.main()
