import copy,json,math,os
from pathlib import Path
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Empty,String
from visualization_msgs.msg import Marker,MarkerArray
from rclpy.qos import QoSProfile,DurabilityPolicy
from mission_route import MissionRoute,point
from mission_runtime import TaskRuntime

class Waypoints:
 def __init__(self,driver,mapping_only=False):
  self.mapping_only=mapping_only
  self.driver=driver;self.map_id=os.getenv('ARX_SAVED_MAP') or ('live:'+os.getenv('ARX_ADAPT_RUN','isolated'))
  self.route=MissionRoute(self.map_id);self.forwarded=set();self.kind='stop';self.phase=None;self.goal_stamp=0;self.last_resume_pose=None;self.accept_after_ns=0;self.mode_revision=0;self.capture_ids=set()
  qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.status=driver.create_publisher(String,'/pb/waypoints/status',qos);self.markers=driver.create_publisher(MarkerArray,'/pb/waypoints/markers',qos)
  self.definition_pub=driver.create_publisher(String,'/pb/waypoints/definition',qos);self.constraints_pub=driver.create_publisher(String,'/pb/mission/constraints',qos)
  self.goals=driver.create_publisher(PoseStamped,'/pb/plan_goal',5)
  self.t2_receipts={};self.t2_cancelled=set();self.t2_execution=None
  self.t2_receipt_pub=driver.create_publisher(String,'/pb/waypoints/t2_receipt',QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  driver.create_subscription(String,'/pb/waypoints/start_t2',self.start_t2,5)
  driver.create_subscription(PoseStamped,'/pb/waypoints/add',self.add,10)
  driver.create_subscription(String,'/pb/waypoints/add_typed',self.add_json,10)
  driver.create_subscription(String,'/pb/waypoints/add_current',self.add_current,10)
  driver.create_subscription(String,'/pb/waypoints/kind',self.set_kind,10)
  driver.create_subscription(String,'/pb/waypoints/edit',self.edit,10)
  driver.create_subscription(Empty,'/pb/waypoints/start',self.start,5);driver.create_subscription(Empty,'/pb/waypoints/clear',self.clear,5)
  if mapping_only:
   from mapping_route_editor import EditorRuntime
   self.runtime=EditorRuntime();self.hub=self.runtime.hub;self.timer=None
  else:
   self.runtime=TaskRuntime(self);self.hub=self.runtime.hub;self.timer=driver.create_timer(.1,self.runtime.tick)
   from route_start import RouteStarter
   self.starter=RouteStarter(self);self.start_timer=driver.create_timer(.1,self.starter.tick)
  self.show()
 def close(self):self.runtime.close()
 def blocking(self):return self.hub.blocking()
 def set_kind(self,msg):
  if msg.data in ('via','stop','special','point_1','point_2'):self.kind=msg.data
 def special_active(self):return bool(getattr(self.driver,'_special_execution',False))
 def direct_ready(self,pose,after_task=False):return self.route.can_enter_direct(pose,after_task=after_task)
 def add_current(self,msg):
  try:
   if len(msg.data)>32768:raise ValueError('request too large')
   v=json.loads(msg.data)
   if not isinstance(v,dict) or set(v)-{'request_id','kind','name'}:raise ValueError('invalid capture fields')
   ident=v['request_id'];kind=v['kind']
   if not isinstance(ident,str) or not 1<=len(ident)<=80 or kind not in ('via','special','point_1','point_2'):raise ValueError('invalid capture request')
   if ident in self.capture_ids:return
   if self.blocking() or self.route.running:raise ValueError('route running or handoff active')
   if len(self.capture_ids)>=512:self.capture_ids.clear()
   self.capture_ids.add(ident)
   poses=self.driver.special_poses()
   value=dict(type='task' if kind in ('point_1','point_2') else kind,pose=poses[0].tolist())
   if kind in ('point_1','point_2'):value['point_id']=kind
   if 'name' in v:value['name']=v['name']
   self.route.add(self.working_point(value));self.phase='已添加当前车位'
  except Exception as e:self.phase='当前位置添加被拒绝：'+str(e)
  self.show()
 def add(self,msg):
  p,q=msg.pose.position,msg.pose.orientation
  if msg.header.frame_id!='map' or not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) or abs(math.hypot(q.w,q.z)-1)>=.002 or abs(q.x)+abs(q.y)>=.002:return
  value=dict(type='task' if self.kind in ('point_1','point_2') else self.kind,pose=[p.x,p.y,2*math.atan2(q.z,q.w)])
  if value['type']=='task':value['point_id']=self.kind
  self._add(value)
 def working_point(self,value):
  if value.get('type')=='task' and any(p.get('test_task_id') for p in self.route.points):
   value=dict(value);value['type']='special';value['test_task_id']=value.pop('point_id')
  return value
 def _add(self,value):
  try:
   if self.blocking():raise ValueError('task handoff active')
   self.route.add(self.working_point(value));self.phase=None
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
   if action=='test_mode':
    if self.driver.target is not None:raise ValueError('请先结束当前单点导航')
    from route_test_mode import test_definition
    if not any(p['type']=='task' or p.get('test_task_id') for p in self.route.points):raise ValueError('当前路线没有可测试的任务点')
    self.route.load(test_definition(self.route.definition(),v['enabled']))
   elif action=='replace_current':
    if self.driver.target is not None:raise ValueError('请先结束当前单点导航')
    if v.get('route_id')!=self.route.route_id:raise ValueError('路线已改变，请重新选择点')
    matches=[p for p in self.route.points if p['id']==v.get('id')]
    if len(matches)!=1 or matches[0]!=v.get('expected_point'):raise ValueError('选中点已改变，请重新选择')
    ident=v.get('request_id')
    if not isinstance(ident,str) or not 1<=len(ident)<=80:raise ValueError('invalid replacement request')
    if ident in self.capture_ids:return
    poses=self.driver.special_poses()
    updated=dict(matches[0],pose=poses[0].tolist());updated=point(updated)
    if len(self.capture_ids)>=512:self.capture_ids.clear()
    self.capture_ids.add(ident)
    self.route.points[self.route.points.index(matches[0])]=updated
   elif action=='replace_if_unchanged':
    if self.driver.target is not None or v['expected_definition']!=self.route.definition():raise ValueError('路线已改变或单点导航未结束，未重加载')
    self.route.load(v['definition'])
   elif action=='load_if_empty':
    if (self.route.state!='draft' or self.route.points or self.driver.target is not None
        or v.get('expected_route_id')!=self.route.route_id):raise ValueError('当前路线已改变，保留现有编辑或目标')
    self.route.load(v['definition'])
   elif action=='load':self.route.load(v['definition'],v.get('allow_map_change') is True)
   elif action=='delete':
    i=v['index'];assert type(i) is int and 0<=i<len(self.route.points);self.route.points.pop(i)
   elif action=='move':
    if 'id' in v:
     if v.get('route_id')!=self.route.route_id:raise ValueError('路线已改变，请重新选择点')
     delta=v.get('delta')
     if type(delta) is not int or delta not in (-1,1):raise ValueError('invalid move direction')
     matches=[i for i,p in enumerate(self.route.points) if p['id']==v['id']]
     if len(matches)!=1:raise ValueError('选中点不存在')
     i=matches[0];j=max(0,min(len(self.route.points)-1,i+delta))
    else:
     i,j=v['index'],v['to'];assert type(i) is int and type(j) is int and 0<=i<len(self.route.points) and 0<=j<len(self.route.points)
    self.route.points.insert(j,self.route.points.pop(i))
   else:raise ValueError('unknown edit')
   self.phase=None
  except (ValueError,TypeError,KeyError,AssertionError) as e:self.phase='编辑被拒绝 '+str(e)
  self.show()
 def show(self,phase=None):
  if phase is not None:self.phase=phase
  r=self.route;labels={'draft':'待开始','running':'执行中','completed':'全部到达并停稳','cancelled':'已取消','manual_override':'已由单点目标接管','waiting_task':'等待任务'}
  self.hub.set_route(r.definition(),state=r.state,running=r.running,index=r.index)
  a=self.hub.state()['task']
  if self.blocking():
   states={'notified':'已到达，等待接管','running':'任务处理中','resume_pending':'任务完成，核验归还与停稳','needs_attention':'任务暂停需处理','cancelling':'取消任务，等待停车与归还','cancelled':'任务已取消，核验停稳'}
   label=a['point_id']+' '+states.get(a['status'],a['status'])
   if self.phase:label+='；'+self.phase
  else:
   label=labels.get(r.state,r.state)
   if self.phase:label=(label+'；' if r.state=='draft' else '')+self.phase
  if any(p.get('test_task_id') for p in r.points):label='测试模式（无真实作业）；'+label
  if r.execution_end is not None:label='T2单独实测；'+label
  self.status.publish(String(data=f'{label}  {r.index}/{len(r.points)}'))
  self.t2_state()
  self.definition_pub.publish(String(data=json.dumps(r.definition(),ensure_ascii=False,allow_nan=False)))
  markers=[Marker(action=Marker.DELETEALL)]
  for i,p in enumerate(r.points):
   m=Marker();m.header.frame_id='map';m.ns='waypoints';m.id=i;m.type=Marker.ARROW;m.pose=self.pose_message(p).pose;m.pose.position.z=.16;m.scale.x,m.scale.y,m.scale.z=.28,.06,.06
   color=(.25,.75,.1) if i<r.index else ((.1,.8,1.) if p['type']=='via' else ((1.,.4,.2) if p.get('point_id')=='point_1' else ((.8,.2,1.) if p['type']=='task' else (1.,.75,.1))))
   m.color.r,m.color.g,m.color.b=color;m.color.a=1.
   t=copy.deepcopy(m);t.ns='waypoint_numbers';t.type=Marker.TEXT_VIEW_FACING;t.text=f"{i+1}:"+({'point_1':'T1','point_2':'T2'}.get(p.get('point_id'),'V' if p['type']=='via' else 'SPECIAL' if p['type']=='special' else 'LEGACY STOP'))+(' '+p['name'] if p.get('name') else '');t.pose.position.z=.36;t.scale.z=.17
   if p.get('test_task_id'):t.text=f"{i+1}:TEST "+('T1后退' if p['test_task_id']=='point_1' else 'T2直达下一点')
   markers.extend([m,t])
  for i,(a,b) in enumerate(zip(r.points,r.points[1:])):
   task=b['type']=='task'
   if not task and (a['type']!='special' or b['type']!='special'):continue
   line=Marker();line.header.frame_id='map';line.ns='task_direct_approach' if task else 'special_straight_segments';line.id=i;line.type=Marker.LINE_STRIP;line.pose.orientation.w=1.;line.scale.x=.025
   line.color.r,line.color.g,line.color.b,line.color.a=1.,.5,.05,1.
   line.points=[Point(x=float(p['pose'][0]),y=float(p['pose'][1]),z=.20) for p in (a,b)]
   markers.append(line)
  self.markers.publish(MarkerArray(markers=markers))
 def pose_message(self,p):
  m=PoseStamped();m.header.frame_id='map';m.pose.position.x,m.pose.position.y=map(float,p['pose'][:2]);a=p['pose'][2];m.pose.orientation.z=math.sin(a/2);m.pose.orientation.w=math.cos(a/2);return m
 def constraints(self):
  self.mode_revision=max(self.mode_revision+1,self.driver.get_clock().now().nanoseconds)
  vias=self.route.vias()
  if vias and self.route.target_index is not None and vias[-1]['id']==self.route.points[self.route.target_index]['id']:vias=vias[:-1]
  self.constraints_pub.publish(String(data=json.dumps(dict(terminal_stamp_ns=self.goal_stamp,accept_after_ns=self.accept_after_ns,mission_id=self.route.mission_id,vias=vias,execution_mode='direct' if self.special_active() else 'navigation',mode_generation_ns=self.mode_revision),allow_nan=False)))
 def dispatch(self,p):
  if p is None:return
  if self.blocking():raise ValueError('task handoff active')
  msg=self.pose_message(p);msg.header.stamp=self.driver.get_clock().now().to_msg();self.goal_stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
  self.phase=None
  if self.route.mode in ('direct','task_direct'):
   if self.route.mode=='task_direct':
    self.route.direct_source=dict(pose=self.driver.special_poses()[0].tolist())
   self.driver.accept_special_goal(msg);self.constraints()
  else:
   self.forwarded.add((msg.header.stamp.sec,msg.header.stamp.nanosec));self.constraints();self.driver.accept_goal(msg);self.goals.publish(msg)
 def t2_state(self):
  execution=self.t2_execution
  if execution and execution['mission_id']==self.route.mission_id:
   record=self.t2_receipts.get(execution['request_id'])
   if record:
    record[1].update(state=self.route.state)
    self.t2_receipt_pub.publish(String(data=json.dumps(record[1])))
 def start_t2(self,msg):
  request=None;value=None;result=None;created=False
  try:
   if len(msg.data)>262144:raise ValueError('request too large')
   value=json.loads(msg.data)
   if not isinstance(value,dict):raise ValueError('invalid T2 command')
   request=value.get('request_id');action=value.get('action')
   if not isinstance(request,str) or not 1<=len(request)<=128:raise ValueError('invalid T2 request')
   old=self.t2_receipts.get(request)
   if old:
    if old[0]!=value:raise ValueError('请求编号不可改变内容')
    self.t2_receipt_pub.publish(String(data=json.dumps(old[1])));return
   fields={'action','request_id','route_id','expected_definition'} if action=='start' else {'action','request_id','route_id','start_request_id'}
   if set(value)!=fields or action not in ('start','cancel'):raise ValueError('invalid T2 command')
   if action=='cancel':
    start=value['start_request_id']
    if not isinstance(start,str) or not 1<=len(start)<=128:raise ValueError('invalid T2 start identity')
    self.t2_cancelled.add((value['route_id'],start))
    execution=self.t2_execution
    # Only stop the exact execution created by this start request. An old
    # cancellation must never stop a newer mission or a manual single goal.
    if (execution and execution['request_id']==start and execution['route_id']==value['route_id']
        and self.route.route_id==value['route_id'] and self.route.mission_id==execution['mission_id']
        and self.route.running and self.route.execution_end is not None and self.route.state!='manual_override'):
     self.route.cancel()
     if self.blocking():self.runtime.cancel()
     if self.driver.target is not None:self.driver.cancel('cancelled')
     self.phase=None;self.goal_stamp=0;self.constraints();self.show()
    result=dict(request_id=request,status='cancelled',start_request_id=start)
   else:
    if (value['route_id'],request) in self.t2_cancelled:raise ValueError('本次实测已停止，不再启动')
    if value['route_id']!=self.route.route_id or value['expected_definition']!=self.route.definition():raise ValueError('路线已改变，请重新开始T2实测')
    if self.mapping_only:raise ValueError('纯建图只编辑路线')
    if self.blocking() or self.driver.target is not None:raise ValueError('请先结束原导航任务')
    if self.driver.ready_file is not None and not self.driver.ready_file.is_file():raise ValueError('定位未就绪')
    console=self.runtime.console()
    if not console['execution_enabled'] or not console['body']['fresh'] or (console.get('chassis_scheduler') or {}).get('maintenance'):raise ValueError('底盘执行接口未就绪或维护中')
    target=self.route.start_t2(self.hub.available_points(),pose=self.driver.special_poses()[0])
    created=True
    self.t2_execution=dict(request_id=request,route_id=self.route.route_id,mission_id=self.route.mission_id)
    result=dict(request_id=request,status='started',mission_id=self.route.mission_id,state=self.route.state)
    self.t2_receipts[request]=(copy.deepcopy(value),result)
    self.dispatch(target);self.show()
  except Exception as e:
   result=dict(request_id=request,status='rejected',reason=str(e))
   execution=self.t2_execution
   if created and execution and execution['request_id']==request and execution['mission_id']==self.route.mission_id:
    self.route.cancel()
    if self.blocking():self.runtime.cancel()
    if self.driver.target is not None:self.driver.cancel('cancelled')
    result['mission_id']=execution['mission_id']
   self.phase='开始被拒绝：'+str(e);self.show()
  if request and result:
   if request not in self.t2_receipts or self.t2_receipts[request][0]==value:self.t2_receipts[request]=(copy.deepcopy(value),result)
   self.t2_receipt_pub.publish(String(data=json.dumps(result)))
 def start(self,_):
  if self.mapping_only:self.show('纯建图只编辑路线，请切换导航模式后执行');return
  self.starter.request()
 def clear(self,_):
  if not self.mapping_only:self.starter.cancel()
  active=self.route.running;self.route.cancel()
  if self.blocking():self.runtime.cancel()
  if active and self.driver.target is not None:self.driver.cancel('cancelled')
  self.route.reset('cancelled');self.goal_stamp=0;self.constraints();self.phase=None;self.show()
 def from_self(self,msg):
  key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
  if key not in self.forwarded:return False
  self.forwarded.remove(key);return True
 def finish(self,reason):
  if not self.mapping_only and reason!='arrived_stopped':self.starter.cancel('导航已取消')
  if self.blocking():
   if reason!='arrived_stopped':self.route.cancel();self.runtime.cancel()
   self.show();return
  if not self.route.running:return
  try:
   next_point=self.route.finish(reason,pose=self.driver.pose('map',self.driver.now()) if reason=='arrived_stopped' else None)
   if self.route.state=='waiting_task':self.runtime.notify(self.route.points[self.route.index])
   else:
    if next_point is None:self.goal_stamp=0;self.constraints()
    self.dispatch(next_point)
  except Exception as e:self.route.running=False;self.route.state='needs_attention';self.phase=str(e)
  self.show()
 def observe(self,pose,odom,stamp):
  if self.route.observe(pose,odom,stamp):self.constraints();self.show()
 def override(self):
  if not self.mapping_only:self.starter.cancel('单点目标已接管')
  if self.blocking():return False
  if self.route.running:self.route.reset('manual_override');self.goal_stamp=0;self.constraints();self.show()
  return True
