import copy,json,math,os
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty,String
from visualization_msgs.msg import Marker,MarkerArray
from rclpy.qos import QoSProfile,DurabilityPolicy
from mission_route import MissionRoute,point
from mission_runtime import TaskRuntime

class Waypoints:
 def __init__(self,driver):
  self.driver=driver;self.map_id=os.getenv('ARX_SAVED_MAP') or ('live:'+os.getenv('ARX_ADAPT_RUN','isolated'))
  self.route=MissionRoute(self.map_id);self.forwarded=set();self.kind='stop';self.phase=None;self.goal_stamp=0;self.last_resume_pose=None;self.accept_after_ns=0
  qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.status=driver.create_publisher(String,'/pb/waypoints/status',qos);self.markers=driver.create_publisher(MarkerArray,'/pb/waypoints/markers',qos)
  self.definition_pub=driver.create_publisher(String,'/pb/waypoints/definition',qos);self.constraints_pub=driver.create_publisher(String,'/pb/mission/constraints',qos)
  self.goals=driver.create_publisher(PoseStamped,'/pb/plan_goal',5)
  driver.create_subscription(PoseStamped,'/pb/waypoints/add',self.add,10)
  driver.create_subscription(String,'/pb/waypoints/add_typed',self.add_json,10)
  driver.create_subscription(String,'/pb/waypoints/kind',self.set_kind,10)
  driver.create_subscription(String,'/pb/waypoints/edit',self.edit,10)
  driver.create_subscription(Empty,'/pb/waypoints/start',self.start,5);driver.create_subscription(Empty,'/pb/waypoints/clear',self.clear,5)
  self.runtime=TaskRuntime(self);self.hub=self.runtime.hub;self.timer=driver.create_timer(.1,self.runtime.tick);self.show()
 def close(self):self.runtime.close()
 def blocking(self):return self.hub.blocking()
 def set_kind(self,msg):
  if msg.data in ('via','stop','point_1','point_2'):self.kind=msg.data
 def add(self,msg):
  p,q=msg.pose.position,msg.pose.orientation
  if msg.header.frame_id!='map' or not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) or abs(math.hypot(q.w,q.z)-1)>=.002 or abs(q.x)+abs(q.y)>=.002:return
  value=dict(type='task' if self.kind in ('point_1','point_2') else self.kind,pose=[p.x,p.y,2*math.atan2(q.z,q.w)])
  if value['type']=='task':value['point_id']=self.kind
  self._add(value)
 def _add(self,value):
  try:
   if self.blocking():raise ValueError('task handoff active')
   self.route.add(value);self.phase=None
  except ValueError as e:self.phase=str(e)
  self.show()
 def add_json(self,msg):
  try:
   if len(msg.data)>32768:raise ValueError('point too large')
   self._add(json.loads(msg.data))
  except (ValueError,TypeError) as e:self.phase=str(e);self.show()
 def edit(self,msg):
  try:
   if self.blocking() or self.route.running:raise ValueError('route running or handoff active')
   if len(msg.data)>262144:raise ValueError('edit too large')
   v=json.loads(msg.data);action=v['action']
   if action=='load':self.route.load(v['definition'],v.get('allow_map_change') is True)
   elif action=='delete':
    i=v['index'];assert type(i) is int and 0<=i<len(self.route.points);self.route.points.pop(i)
   elif action=='move':
    i,j=v['index'],v['to'];assert type(i) is int and type(j) is int and 0<=i<len(self.route.points) and 0<=j<len(self.route.points)
    self.route.points.insert(j,self.route.points.pop(i))
   else:raise ValueError('unknown edit')
   self.phase=None
  except (ValueError,TypeError,KeyError,AssertionError) as e:self.phase='编辑被拒绝 '+str(e)
  self.show()
 def show(self,phase=None):
  if phase is not None:self.phase=phase
  r=self.route;labels={'draft':'待开始','running':'执行中','completed':'全部到达并停稳','cancelled':'已取消','manual_override':'已由单点目标接管','waiting_task':'等待任务'}
  a=self.hub.state()['task']
  if self.blocking():
   states={'notified':'已到达，等待接管','running':'任务处理中','resume_pending':'任务完成，核验归还与停稳','needs_attention':'任务暂停需处理','cancelling':'取消任务，等待停车与归还','cancelled':'任务已取消，核验停稳'}
   label=a['point_id']+' '+states.get(a['status'],a['status'])
   if self.phase:label+='；'+self.phase
  else:label=self.phase or labels.get(r.state,r.state)
  self.status.publish(String(data=f'{label}  {r.index}/{len(r.points)}'))
  self.definition_pub.publish(String(data=json.dumps(r.definition(),ensure_ascii=False,allow_nan=False)))
  markers=[Marker(action=Marker.DELETEALL)]
  for i,p in enumerate(r.points):
   m=Marker();m.header.frame_id='map';m.ns='waypoints';m.id=i;m.type=Marker.ARROW;m.pose=self.pose_message(p).pose;m.pose.position.z=.16;m.scale.x,m.scale.y,m.scale.z=.28,.06,.06
   color=(.25,.75,.1) if i<r.index else ((.1,.8,1.) if p['type']=='via' else ((1.,.4,.2) if p.get('point_id')=='point_1' else ((.8,.2,1.) if p['type']=='task' else (1.,.75,.1))))
   m.color.r,m.color.g,m.color.b=color;m.color.a=1.
   t=copy.deepcopy(m);t.ns='waypoint_numbers';t.type=Marker.TEXT_VIEW_FACING;t.text=f"{i+1}:"+({'point_1':'T1','point_2':'T2'}.get(p.get('point_id'),'V' if p['type']=='via' else 'STOP'));t.pose.position.z=.36;t.scale.z=.17
   markers.extend([m,t])
  self.markers.publish(MarkerArray(markers=markers))
 def pose_message(self,p):
  m=PoseStamped();m.header.frame_id='map';m.pose.position.x,m.pose.position.y=map(float,p['pose'][:2]);a=p['pose'][2];m.pose.orientation.z=math.sin(a/2);m.pose.orientation.w=math.cos(a/2);return m
 def constraints(self):
  self.constraints_pub.publish(String(data=json.dumps(dict(terminal_stamp_ns=self.goal_stamp,accept_after_ns=self.accept_after_ns,mission_id=self.route.mission_id,vias=self.route.vias()),allow_nan=False)))
 def dispatch(self,p):
  if p is None:return
  if self.blocking():raise ValueError('task handoff active')
  msg=self.pose_message(p);msg.header.stamp=self.driver.get_clock().now().to_msg();self.goal_stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
  self.forwarded.add((msg.header.stamp.sec,msg.header.stamp.nanosec));self.phase=None;self.constraints()
  self.driver.accept_goal(msg);self.goals.publish(msg)
 def start(self,_):
  try:
   if self.blocking():raise ValueError('任务尚未归还控制权')
   if self.driver.ready_file is not None and not self.driver.ready_file.is_file():raise ValueError('定位未就绪')
   if self.route.running or self.driver.target is not None:self.show();return
   console=self.runtime.console()
   if not console['execution_enabled'] or not console['body']['fresh'] or console['chassis_active'] or console.get('motion_reserved',False):raise ValueError('底盘或任务控制权尚未就绪')
   self.dispatch(self.route.start(self.hub.available_points()))
  except (ValueError,OSError,KeyError,TypeError) as e:self.phase=('开始被拒绝：'+str(e)).replace('task worker unavailable:','任务程序未接入：')
  self.show()
 def clear(self,_):
  active=self.route.running;self.route.cancel()
  if self.blocking():self.runtime.cancel()
  if active and self.driver.target is not None:self.driver.cancel('cancelled')
  self.route.reset('cancelled');self.goal_stamp=0;self.constraints();self.phase=None;self.show()
 def from_self(self,msg):
  key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
  if key not in self.forwarded:return False
  self.forwarded.remove(key);return True
 def finish(self,reason):
  if self.blocking():
   if reason!='arrived_stopped':self.route.cancel();self.runtime.cancel()
   self.show();return
  if not self.route.running:return
  try:
   next_point=self.route.finish(reason)
   if self.route.state=='waiting_task':self.runtime.notify(self.route.points[self.route.index])
   else:self.dispatch(next_point)
  except Exception as e:self.route.running=False;self.route.state='needs_attention';self.phase=str(e)
  self.show()
 def observe(self,pose,odom,stamp):
  if self.route.observe(pose,odom,stamp):self.constraints();self.show()
 def override(self):
  if self.blocking():return False
  if self.route.running:self.route.reset('manual_override');self.goal_stamp=0;self.constraints();self.show()
  return True
