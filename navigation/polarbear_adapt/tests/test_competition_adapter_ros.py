"""Actual 9983 navigation adapters + private ROS + loopback protocol.

Never instantiate Controller/BatteryTask. Robot arrivals and external task motion
are virtual observations, and WebChassis is the existing inert test transport.
"""
import importlib.util,json,os,threading,time,unittest,uuid
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='197', 'private ROS197 required'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import test_mission_ros as fixtures
from task_http import TaskServer
from task_client import TaskClient


def actual_adapter(name):
 root=Path(os.environ['ARX_TEST_WEB_SOURCE']).resolve()
 path=root/(name+'.py')
 spec=importlib.util.spec_from_file_location('isolated_'+name,path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


class ControllerObservations:
 def __init__(self):self.reserved=False;self.active=False;self.stops=[]
 def snapshot(self):return dict(chassis_active=self.active,motion_reserved=self.reserved,competition_reserved=True)
 def motion_valid(self,token):return self.reserved and token=='virtual-work-token'
 def stop_competition_navigation(self,token):self.stops.append(token)


class CompetitionAdapterROS(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.session_code=actual_adapter('navigation_session');cls.route_code=actual_adapter('navigation_route');rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 refresh=fixtures.MissionROS.refresh
 add=fixtures.MissionROS.add
 def setUp(self):
  fixtures.MissionROS.setUp(self)
  self.controller=ControllerObservations();self.observer=Node('isolated_competition_route_observer')
  root=Path(self.temp.name)/'mailbox';root.mkdir();self.relay=self.route_code.RouteRelay(root);self.relay.attach(self.observer)
  self.mailbox=self.route_code.RouteMailbox(root,lambda:dict(route=self.relay.snapshot()))
  self.server=TaskServer(self.n.waypoints.hub,port=0);self.client=TaskClient('http://127.0.0.1:'+str(self.server.port))
  self.session=self.session_code.NavigationSession(self.controller,self.mailbox,client=self.client)
  self.context=dict(run_id=str(uuid.uuid4()),cancelled=threading.Event(),token='virtual-work-token',competition_token='virtual-competition-token')
  self.add('task',.2,'point_1');self.add('task',.8,'point_2')
  self.wait(lambda:self.relay.available() and self.relay.status.startswith('待开始'))
 def tearDown(self):
  self.session.close();self.server.close();self.observer.destroy_node();fixtures.MissionROS.tearDown(self)
 def spin(self):
  rclpy.spin_once(self.n,timeout_sec=.005);rclpy.spin_once(self.observer,timeout_sec=.005)
 def wait(self,predicate,timeout=5.):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   self.spin()
   if predicate():return
  self.fail('private integration timeout: '+json.dumps(self.session.snapshot(),ensure_ascii=False))
 def notify_arrival(self,x):
  self.n.seen=self.n.seq;self.position[0]=x;self.refresh();self.n.cancel('arrived_stopped')
 def first_work(self):
  stage=self.session.stage('point_1');stage.start(self.context);self.wait(lambda:self.n.target is not None)
  self.assertTrue(self.n.waypoints.route.running);self.notify_arrival(.2)
  self.wait(lambda:stage.inspect(self.context)['status']=='completed')
  self.controller.reserved=True;self.session.before_work(self.context,'pick')
  self.assertEqual(self.n.waypoints.hub.active['status'],'running')
  return self.n.waypoints.hub.active['visit_id']
 def second_arrival(self):
  first=self.first_work();self.position[0]=.4;self.refresh()
  self.controller.active=True
  stage=self.session.stage('point_2');result=[]
  def release():
   try:result.append(stage.start(self.context))
   except Exception as exc:result.append(str(exc))
  worker=threading.Thread(target=release);worker.start()
  try:
   for _ in range(10):self.spin()
   self.assertEqual(result,[])
   self.assertEqual(self.n.waypoints.hub.active['status'],'running');self.assertIsNone(self.n.target)
   self.controller.active=False;self.wait(lambda:bool(result))
  finally:self.controller.active=False;worker.join(2.)
  self.assertEqual(result,[dict(status='running')])
  self.assertTrue(self.controller.reserved) # arm token is not a chassis permission
  self.assertEqual(self.n.waypoints.hub.active['status'],'resume_pending');self.assertIsNone(self.n.target)
  self.wait(lambda:self.n.target is not None)
  self.assertEqual(self.n.waypoints.hub.active['status'],'acknowledged');self.assertAlmostEqual(self.n.waypoints.last_resume_pose[0],.4)
  self.notify_arrival(.8);self.wait(lambda:stage.inspect(self.context)['status']=='completed')
  current=self.n.waypoints.hub.active;self.assertEqual(current['point_id'],'point_2');self.assertNotEqual(current['visit_id'],first)
  self.assertEqual(self.n.waypoints.route.state,'waiting_task')
 def test_real_adapter_task_1_handoff_then_task_2_and_acknowledged_completion(self):
  self.second_arrival();self.controller.reserved=True;self.session.before_work(self.context,'place');self.controller.reserved=False
  result=[]
  def finish():
   try:self.session.complete_route(self.context);result.append('completed')
   except Exception as e:result.append(type(e).__name__+':'+str(e))
  worker=threading.Thread(target=finish);worker.start()
  try:self.wait(lambda:bool(result))
  finally:worker.join(2.)
  self.assertEqual(result,['completed']);self.assertEqual(self.n.waypoints.route.state,'completed');self.assertEqual(self.n.waypoints.hub.active['outcome'],'completed')
  self.assertFalse(self.session.active);self.assertTrue(all(not any(c) for c in self.n.transport.calls))
 def test_terminal_pause_is_running_to_actual_9983_adapter_without_fake_arrival(self):
  stage=self.session.stage('point_1');stage.start(self.context);self.wait(lambda:self.n.target is not None)
  self.n.status('paused_terminal_correction outside_arrival_tolerance xy=0.0533 yaw=0.0646')
  self.n.waypoints.show('末端多次复核未通过，停车保留任务和目标')
  self.wait(lambda:self.relay.drive_status.startswith('paused_terminal_correction'))
  self.assertEqual(stage.inspect(self.context)['status'],'running')
  self.assertIsNone(self.n.waypoints.hub.active);self.assertTrue(self.n.waypoints.route.running)
  self.assertIsNotNone(self.n.target);self.assertTrue(self.session.active)
 def test_missing_place_fixture_uses_actual_clear_and_cancelled_not_success(self):
  self.second_arrival();result=[]
  # Only emulate framework cancellation on absent place; no place module is imported.
  def cancel():
   try:result.append(self.session.stop(self.context))
   except Exception as e:result.append(type(e).__name__+':'+str(e))
  worker=threading.Thread(target=cancel);worker.start()
  try:self.wait(lambda:bool(result))
  finally:worker.join(2.)
  self.assertEqual(result,[dict(stopped=True)]);self.assertEqual(self.controller.stops,['virtual-competition-token']);self.assertFalse(self.session.active)
  self.assertEqual(self.n.waypoints.route.state,'cancelled');self.assertEqual(self.n.waypoints.route.points,[])
  self.assertNotEqual(self.n.waypoints.hub.active.get('outcome'),'completed');self.assertTrue(all(not any(c) for c in self.n.transport.calls))
  self.wait(lambda:self.n.waypoints.hub.active['status']=='acknowledged')
  self.assertEqual(self.n.waypoints.hub.active['outcome'],'cancelled')
if __name__=='__main__':unittest.main()
