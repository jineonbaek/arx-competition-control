"""ROS driver integration; transport alone is inert, never hardware."""
import copy,io,json,math,os,tempfile,time,unittest,uuid
assert os.environ.get('ROS_DOMAIN_ID') in ('197','198')
import numpy as np,rclpy
from geometry_msgs.msg import PoseStamped,TransformStamped,PoseArray,Pose
from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import axis_trial
from path_audit import AuditGrid
from chassis_command import WebChassis
class Transport(WebChassis):
 sdk_units=native_sdk=True;origin='http://isolated.invalid'
 def __init__(self):
  self.calls=[];self.reserved=False;self.last_identity=None
  super().__init__(submit=self.capture);self.client=self
 def capture(self,m):
  key=(m['source_id'],m['task_id'],m['generation'])
  if m['type']=='chassis.end_output':
   if self.last_identity==key:self.calls.append((0.,0.,0.));self.last_identity=None
  else:self.calls.append((m['x'],m['y'],m['omega']));self.last_identity=key
  return dict(ok=True)
 def open(self,*a,**kw):return io.BytesIO(json.dumps(dict(execution_enabled=True,nav_sdk_ready=True,body=dict(fresh=True),chassis_active=False,motion_reserved=self.reserved,capabilities=dict(chassis=''),command_epoch=1)).encode())
axis_trial.WebChassis=Transport
from mppi_goal_drive import MppiDriver
class MissionROS(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();os.environ['ARX_TASK_JOURNAL']=self.temp.name+'/task.json';axis_trial.WebChassis=Transport
  self.n=MppiDriver();self.position=np.zeros(3);self.refresh();self.timer=self.n.create_timer(.04,self.refresh)
 def tearDown(self):self.n.destroy_node();self.temp.cleanup();os.environ.pop('ARX_TASK_JOURNAL',None)
 def refresh(self):
  n=self.n;stamp=n.get_clock().now().to_msg();grid=AuditGrid(np.zeros((200,200)),[-5,-5],.05,n.poly);n.grids={f:(stamp,grid) for f in ('map','odom')}
  for topic,kind in (('/pb/scan',LaserScan),('/localization/odometry',Odometry)):
   m=kind();m.header.stamp=stamp;n.sensor_message(topic,m)
  static=TransformStamped();static.header.frame_id='map';static.child_frame_id='odom';static.transform.rotation.w=1.;n.tf.set_transform_static(static,'test')
  tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.header.stamp=stamp;tf.transform.translation.x,tf.transform.translation.y=map(float,self.position[:2]);tf.transform.rotation.z=math.sin(self.position[2]/2);tf.transform.rotation.w=math.cos(self.position[2]/2);n.tf.set_transform(tf,'test')
  if hasattr(n,'receive_chassis_snapshot'):
   snap=PoseArray();snap.header.frame_id='map';snap.header.stamp=stamp;p=Pose();p.position.x,p.position.y=map(float,self.position[:2]);p.orientation=copy.deepcopy(tf.transform.rotation);snap.poses=[p,copy.deepcopy(p)];n.receive_chassis_snapshot(snap)
 def worker(self):
  h=self.n.waypoints.hub;token=h.register(dict(worker_id='battery',instance_id=str(uuid.uuid4()),point_ids=['point_1','point_2']))['token'];return h,token
 def add(self,kind,x,tag=None):
  p=dict(type=kind,pose=[x,0,0]);
  if tag:p['point_id']=tag
  self.n.waypoints.add_json(String(data=json.dumps(p)))
 def event(self,h,t,event,**v):
  a=h.active;p={k:a[k] for k in ('mission_id','visit_id','point_id')};p.update(request_id=str(uuid.uuid4()),event=event,**v);return h.event(t,p)
 def arrive_task(self):
  h,t=self.worker();self.add('task',.2,'point_1');self.add('stop',.8);self.n.waypoints.start(None);self.n.seen=self.n.seq;self.position[0]=.2;self.refresh();self.n.cancel('arrived_stopped');return h,t
 def test_task_point_requires_real_worker_registration(self):
  self.add('task',.2,'point_2');self.n.waypoints.start(None);self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
 def test_arrival_notifies_correct_point_and_blocks_arrows_without_brakes(self):
  h,t=self.arrive_task();self.assertEqual(h.active['point_id'],'point_1');self.assertEqual(self.n.waypoints.route.index,0);self.assertIsNone(self.n.target)
  old=len(self.n.transport.calls);m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg();m.pose.orientation.w=1.;m.pose.position.x=1.;self.n.plan_changed(m);self.n.spin(.15)
  self.assertIsNone(self.n.target);self.assertEqual(len(self.n.transport.calls),old);self.assertTrue(h.blocking())
 def test_completion_waits_for_actual_control_release_then_uses_new_pose(self):
  h,t=self.arrive_task();self.event(h,t,'accept');self.position[0]=.5;self.n.transport.reserved=True;self.refresh()
  self.event(h,t,'complete',navigation_ready=True,control_released=True);self.n.spin(.25);self.assertIsNone(self.n.target)
  self.n.transport.reserved=False
  until=time.monotonic()+2.5
  while self.n.target is None and time.monotonic()<until:h.heartbeat(t);self.n.spin(.08)
  np.testing.assert_allclose(self.n.target,[.8,0,0]);self.assertEqual(self.n.waypoints.route.index,1);self.assertEqual(h.active['status'],'acknowledged')
  np.testing.assert_allclose(self.n.waypoints.last_resume_pose,[.5,0,0]);self.assertTrue(all(np.linalg.norm(c)==0 for c in self.n.transport.calls))
 def test_expired_authoritative_batch_after_final_console_read_cannot_release_task(self):
  h,t=self.arrive_task();self.event(h,t,'accept');self.event(h,t,'complete',navigation_ready=True,control_released=True)
  rt=self.n.waypoints.runtime;rt.tick();rt.stop.add=lambda *a:True;rt.window.observe=lambda *a:True;rt.stamp=None
  original=rt.console;reads=[]
  def delayed_console():
   result=original();reads.append(1)
   if len(reads)==2:
    old=copy.deepcopy(self.n.pose_snapshot);old.header.stamp.sec-=1;self.n.pose_snapshot=old
   return result
  rt.console=delayed_console;rt.tick()
  self.assertEqual(len(reads),2);self.assertEqual(h.active['status'],'resume_pending');self.assertIsNone(self.n.target);self.assertEqual(self.n.waypoints.route.index,0)
 def test_cancel_and_late_complete_cannot_dispatch_next(self):
  h,t=self.arrive_task();self.event(h,t,'accept');self.n.waypoints.clear(None)
  with self.assertRaises(ValueError):self.event(h,t,'complete',navigation_ready=True,control_released=True)
  self.n.spin(.2);self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
 def test_existing_drive_cancel_also_cancels_the_task_visit(self):
  h,t=self.arrive_task();self.event(h,t,'accept');before=len(self.n.transport.calls)
  self.n.cancel('cancelled')
  self.assertEqual(h.active['status'],'cancelling');self.assertEqual(len(self.n.transport.calls),before) # already ended; no new all-car stop
  self.assertFalse(self.n.waypoints.route.running);self.assertIsNone(self.n.target)
 def test_start_console_timeout_is_contained_and_route_remains_draft(self):
  from urllib.error import URLError
  self.add('stop',.2)
  self.n.transport.open=lambda *a,**k:(_ for _ in ()).throw(URLError('test connection refused'))
  self.n.waypoints.start(None)
  self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running);self.assertEqual(self.n.waypoints.route.state,'draft')
 def test_task_cancel_transport_timeout_is_contained(self):
  h,t=self.arrive_task();self.event(h,t,'accept')
  self.n.transport.end_output=lambda *a:(_ for _ in ()).throw(TimeoutError('test stopping timeout'))
  self.n.waypoints.clear(None)
  self.assertEqual(h.active['status'],'cancelling');self.assertIsNone(self.n.target)
 def test_supported_large_route_definition_can_load_without_auto_start(self):
  w=self.n.waypoints
  for i in range(200):w.route.add(dict(id=str(i).zfill(4)+'a'*76,type='via',pose=[i+.12345678912345,0.,0.]))
  definition=w.route.definition();w.route.reset()
  payload=json.dumps(dict(action='load',definition=definition))
  self.assertGreater(len(payload),32768)
  w.edit(String(data=payload));self.assertEqual(len(w.route.points),200);self.assertFalse(w.route.running);self.assertIsNone(self.n.target)
 def test_continuous_via_target_is_terminal_and_shortcut_is_rejected(self):
  self.add('via',.3);self.add('stop',.8);self.n.waypoints.start(None);self.n.seen=self.n.seq
  np.testing.assert_allclose(self.n.target,[.8,0,0]);self.assertEqual(len(self.n.waypoints.route.vias()),1)
  m=Path();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg()
  for x,y in ((0.,0.),(0.,1.),(.8,1.),(.8,0.)):
   p=PoseStamped();p.header=m.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;m.poses.append(p)
  self.n.receive(m);self.assertIsNone(self.n.route);self.assertIsNotNone(self.n.target)
 def test_terminal_native_success_cannot_skip_vias(self):
  self.add('via',.3);self.add('task',.8,'point_1');h,t=self.worker();self.n.waypoints.start(None);self.n.seen=self.n.seq
  with self.assertRaisesRegex(ValueError,'replan_blocked'):self.n.verify_arrival()
  self.assertIsNone(h.active)
if __name__=='__main__':unittest.main()
