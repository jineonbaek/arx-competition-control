"""ROS-thread handoff safety gate; HTTP thread never mutates goals or hardware."""
import copy,json,math,os,tempfile,time
from pathlib import Path
from urllib.error import HTTPError,URLError
from rclpy.time import Time
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile,DurabilityPolicy
from task_handoff import HandoffHub,ProtocolError
from task_http import TaskServer
from mppi_trial.stationary import StopWindow
from mppi_trial.recovery import RecoveryWindow,input_unavailable

class TaskRuntime:
 def __init__(self,waypoints):
  self.waypoints=waypoints;self.driver=waypoints.driver;self.temp=None
  path=os.getenv('ARX_TASK_JOURNAL')
  if not path:
   if os.getenv('ROS_DOMAIN_ID')=='192':path=str(Path(__file__).resolve().parents[1]/'state/navigation-task.json')
   else:self.temp=tempfile.TemporaryDirectory();path=str(Path(self.temp.name)/'task.json')
  self.hub=HandoffHub(path);self.server=None
  self.lock_pub=self.driver.create_publisher(Bool,'/pb/mission/handoff_active',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.stop=None;self.window=None;self.stamp=None;self.stationary=False;self.phase=None
  self.last_state=None;self.stop_requested=None
  self.resume_intent=None;self.pending_notification=None
  if os.getenv('ROS_DOMAIN_ID')=='192' and os.getenv('ARX_TASK_API','1')=='1':self.server=TaskServer(self.hub,int(os.getenv('ARX_TASK_PORT','9985')))
  self.publish_lock()
 def publish_lock(self):self.lock_pub.publish(Bool(data=self.hub.blocking()))
 def defer_notification(self,p,worker,mp=None,od=None):
  if not worker:raise ProtocolError('task worker unavailable')
  n=self.driver;route=self.waypoints.route
  pending=getattr(self,'pending_notification',None)
  if not (pending and pending['point']==p and pending['mission_id']==route.mission_id
          and pending['route_id']==route.route_id and pending['instance_id']==worker['instance_id']):
   pending=dict(point=copy.deepcopy(p),mission_id=route.mission_id,route_id=route.route_id,
    instance_id=worker['instance_id'],arrival_map=None,arrival_odom=None,
    stop=StopWindow(after=n.now()),map_stop=StopWindow(after=n.now()),window=RecoveryWindow(time.monotonic()),stamp=None,stationary=False,stop_signature=None)
   self.pending_notification=pending
  # A failed first source proof leaves these unset: never anchor to invalid data.
  if mp is not None and pending['arrival_map'] is None:
   pending.update(arrival_map=mp.tolist(),arrival_odom=od.tolist())
  self.hub.set_navigation_check('waiting','等待原worker、底盘输出归还确认及实测停稳')
 def drain_output_ends(self):
  retry=getattr(self.driver,'retry_output_ends',None)
  try:
   if retry is None or retry() is not False:return True
   reason='等待原底盘输出归还确认'
  except Exception as exc:reason='等待原底盘输出归还确认：'+str(exc)
  # No stopped-motion evidence gathered before output cleanup may authorize work.
  if self.window is not None:
   self.stop=StopWindow(after=self.driver.now());self.stamp=None;self.stationary=False
   self.window.observe(time.monotonic(),False,(0,0))
  pending=getattr(self,'pending_notification',None)
  if pending is not None:
   pending['stop']=StopWindow(after=self.driver.now());pending['stamp']=None
   pending['map_stop']=StopWindow(after=self.driver.now());pending['stationary']=False;pending['stop_signature']=None
   pending['window'].observe(time.monotonic(),False,(0,0))
  self.hub.set_navigation_check('waiting',reason[:300]);return False
 def notify(self,p,_proof=None):
  n=self.driver;worker=copy.deepcopy(self.hub.worker)
  # Retain the exact unfinished visit intent before any source lookup can fail.
  self.defer_notification(p,worker)
  try:
   ended=n.end_output()
   mp,od,source=n.motion_poses() if _proof is None else _proof;s=n.pose_snapshot.header.stamp
   n.validate_motion_proof(source)
   self.defer_notification(p,worker,mp,od)
   if ended is False or not self.drain_output_ends():return
   n.validate_motion_proof(source)
   self.hub.begin(dict(mission_id=self.waypoints.route.mission_id,point_id=p['point_id'],route_point_id=p['id'],map_id=self.waypoints.map_id,goal_pose=p['pose'],arrival_pose=mp.tolist(),pose_stamp_ns=s.sec*1000000000+s.nanosec))
  except Exception as exc:
   if input_unavailable(str(exc)) or isinstance(exc,ProtocolError) and str(exc)=='task worker unavailable':
    self.hub.set_navigation_check('waiting',('等待原到点新定位证明：'+str(exc))[:300]);return
   self.pending_notification=None;raise
  self.pending_notification=None
  self.publish_lock()
 def retry_notification(self):
  pending=self.pending_notification;n=self.driver;route=self.waypoints.route
  if (not route.running or route.state!='waiting_task' or route.mission_id!=pending['mission_id']
      or route.route_id!=pending['route_id'] or route.points[route.index]!=pending['point']):
   self.pending_notification=None;return
  try:
   if not self.drain_output_ends():return
   if self.hub.blocking():return
   worker=self.hub.worker
   if not worker or worker['instance_id']!=pending['instance_id']:return
   if pending['point']['point_id'] not in self.hub.available_points():return
   if n.ready_file is not None and not n.ready_file.is_file():return
   state=self.console()
   ready=(state['execution_enabled'] and state['body']['fresh'] and not state['chassis_active']
    and state.get('nav_sdk_ready',False) and not state['capabilities']['chassis'])
   mp,od,stamp=n.motion_poses()
   unchanged=lambda p,old:(math.hypot(p[0]-old[0],p[1]-old[1])<=.009 and
    abs(math.atan2(math.sin(p[2]-old[2]),math.cos(p[2]-old[2])))<=.02)
   if pending['arrival_map'] is None:
    # Without a trustworthy original batch, require the original task goal.
    # Conservative regular task tolerances; do not infer T1 fallback arrival.
    goal=pending['point']['pose']
    located=(math.hypot(mp[0]-goal[0],mp[1]-goal[1])<=.045 and
     abs(math.atan2(math.sin(mp[2]-goal[2]),math.cos(mp[2]-goal[2])))<=.09)
   else:located=unchanged(mp,pending['arrival_map']) and unchanged(od,pending['arrival_odom'])
   safe=ready and not n.special_stale_inputs() and located
   signature=tuple(mp)+tuple(od)
   if stamp==pending['stamp'] and signature!=pending.get('stop_signature'):
    pending['stop']=StopWindow(after=n.now());pending['map_stop']=StopWindow(after=n.now())
    pending['stamp']=None;pending['stationary']=False;pending['stop_signature']=None
    pending['window'].observe(time.monotonic(),False,(0,0))
   if stamp!=pending['stamp']:
    odom_stopped=pending['stop'].add(stamp,od);map_stopped=pending['map_stop'].add(stamp,mp)
    pending['stationary']=odom_stopped and map_stopped;pending['stamp']=stamp;pending['stop_signature']=signature
   stationary=pending.get('stationary',False)
   if not pending['window'].observe(time.monotonic(),safe and stationary,(int(stamp*1e9),int(stamp*1e9))):return
   final=self.console();n.validate_motion_proof(stamp)
   if (not final['execution_enabled'] or not final['body']['fresh'] or final['chassis_active']
       or not final.get('nav_sdk_ready',False) or final['capabilities']['chassis'] or n.special_stale_inputs()):return
   # Registration may race the console reads; begin is serialized with registration.
   with self.hub.lock:
    if self.hub.worker['instance_id']!=pending['instance_id']:return
    if n.ready_file is not None and not n.ready_file.is_file():return
    if not self.drain_output_ends():return
    n.validate_motion_proof(stamp)
    self.notify(pending['point'],_proof=(mp,od,stamp))
  except Exception as exc:
   network=(isinstance(exc,HTTPError) and exc.code in (502,503,504) or
    isinstance(exc,(TimeoutError,URLError)) and not isinstance(exc,HTTPError))
   if not (input_unavailable(str(exc)) or network):raise
   pending['stop']=StopWindow(after=n.now());pending['map_stop']=StopWindow(after=n.now());pending['stamp']=None;pending['stationary']=False;pending['stop_signature']=None
   pending['window'].observe(time.monotonic(),False,(0,0))
   self.hub.set_navigation_check('waiting',('等待原到点通知核验：'+str(exc))[:300])
 def dispatch_accepted(self,intent):
  arrow=getattr(self.driver,'arrow',None);goal_stamp=self.waypoints.goal_stamp
  if arrow is None or not goal_stamp or goal_stamp==intent['previous_goal_stamp']:return False
  stamp=arrow.header.stamp
  return stamp.sec*1000000000+stamp.nanosec==goal_stamp
 def resume_route(self,intent,pose):
  if not self.drain_output_ends():return
  route=self.waypoints.route
  if route.route_id!=intent['route_id'] or route.mission_id!=intent['mission_id']:
   raise ValueError('交接后路线身份已变化，不重放剩余目标')
  if not route.running and route.state!='completed':
   self.resume_intent=None;return
  if self.dispatch_accepted(intent):self.resume_intent=None;return
  if route.index==intent['index'] and route.state=='waiting_task':route.release_task(pose=pose)
  # release_task can advance successfully and then fail at an observing boundary.
  if route.state=='completed':self.resume_intent=None;return
  if route.index!=intent['index']+1 or route.state!='running' or route.target_index is None:
   raise ValueError('剩余路线推进状态无法核实，不重复推进')
  point=route.points[route.target_index]
  if intent.get('point') is None:intent['point']=copy.deepcopy(point)
  if point!=intent['point']:raise ValueError('交接后的下一目标已变化')
  self.waypoints.dispatch(intent['point'])
  self.resume_intent=None
 def cancel(self):
  if not self.hub.blocking():return
  visit=self.hub.state()['task']['visit_id']
  try:self.hub.cancel()
  except Exception as exc:
   self.waypoints.phase='取消记录失败，保持暂停：'+str(exc)
   self.driver.get_logger().error(self.waypoints.phase)
  if self.stop_requested!=visit:
   try:
    self.driver.transport.end_output();self.driver.active=False;self.stop_requested=visit
   except Exception as exc:
    self.waypoints.phase='停车请求未确认，保持暂停：'+str(exc)
    self.driver.get_logger().error(self.waypoints.phase)
  self.publish_lock()
 def console(self):
  n=self.driver
  with n.transport.client.open(n.transport.origin+'/api/state',timeout=.3) as reply:return json.load(reply)
 def tick(self):
  n=self.driver;self.hub.tick()
  if not self.drain_output_ends():return
  a=self.hub.state()['task']
  if getattr(self,'pending_notification',None) is not None:self.retry_notification();return
  intent=getattr(self,'resume_intent',None)
  if intent and (not a or a['visit_id']!=intent['visit_id']):
   self.resume_intent=None;intent=None
  retrying=bool(intent and a and a['visit_id']==intent['visit_id'] and a['status']=='acknowledged' and a.get('outcome')=='completed')
  if retrying and (not self.waypoints.route.running or self.dispatch_accepted(intent)):
   self.resume_intent=None;return
  key=None if a is None else (a['visit_id'],a['status'],a['revision'])
  if key!=self.last_state:self.last_state=key;self.publish_lock();self.waypoints.show()
  if not a or (a['status'] not in ('resume_pending','cancelled','cancelling') and not retrying):
   if a and a['status']=='acknowledged':self.hub.set_navigation_check('cancelled' if a.get('outcome')=='cancelled' else 'verified','取消已确认，不恢复旧目标' if a.get('outcome')=='cancelled' else '导航已确认交接，按剩余路线处理')
   else:self.hub.set_navigation_check('idle','等待作业回执' if a else '尚未产生到点交接')
   return
  phase=(a['visit_id'],a['status'])
  if phase!=self.phase:
   self.phase=phase;self.stop=StopWindow(after=n.now());self.window=RecoveryWindow(time.monotonic());self.stamp=None;self.stationary=False
  try:
   if n.ready_file is not None and not n.ready_file.is_file():
    self.hub.set_navigation_check('waiting','导航执行门尚未就绪');self.window.observe(time.monotonic(),False,(0,0));return
   s=self.console()
   ready=(s['execution_enabled'] and s['body']['fresh'] and not s['chassis_active'] and s.get('nav_sdk_ready',False) and not s['capabilities']['chassis'])
   mp,op,stamp=n.motion_poses();poses={'map':mp,'odom':op}
   route=self.waypoints.route;i=route.index
   final_task=(route.running and route.state=='waiting_task' and route.mode=='task_direct'
               and 0<=i<len(route.points) and i+1==len(route.points)
               and route.points[i]['type']=='task' and route.points[i]['id']==a['route_point_id']
               and route.points[i]['point_id']==a['point_id'] and route.mission_id==a['mission_id']
               and route.map_id==a['map_id'])
   # No motion follows the last completed direct task. Occupancy must not
   # block the final receipt; stop/source/control proofs below still apply.
   direct=(self.waypoints.direct_ready(poses['map'],after_task=True) or final_task or
    (retrying and route.mode in ('direct','task_direct'))) and (a['status']=='resume_pending' or retrying)
   # Cancellation acknowledges a stopped handoff; it does not authorize travel.
   no_geometry=direct or a['status'] in ('cancelled','cancelling')
   stale=n.special_stale_inputs() if no_geometry else n.stale_inputs()
   if direct:n.special_poses()
   if not ready or stale:
    reason='等待作业结束底盘输出或执行接口就绪' if not ready else '定位、扫描或代价地图数据不新鲜'
    self.hub.set_navigation_check('waiting',reason);self.window.observe(time.monotonic(),False,(0,0));return
   if not no_geometry and any(n.grids[f][1].inspect(poses[f])['first_cell'] for f in poses):
    self.hub.set_navigation_check('waiting','当前整车轮廓与代价地图占用相交');self.window.observe(time.monotonic(),False,(0,0));return
   if stamp!=self.stamp:self.stationary=self.stop.add(stamp,poses['odom']);self.stamp=stamp
   revisions=(int(stamp*1e9),int(stamp*1e9)) if no_geometry else tuple(n.grids[f][0].sec*1000000000+n.grids[f][0].nanosec for f in ('map','odom'))
   if not self.window.observe(time.monotonic(),self.stationary,revisions):self.hub.set_navigation_check('checking','等待实测停稳及新地图验证');return
   # Re-read control state after the proof; no cached ownership is sufficient.
   final=self.console()
   if (not final['execution_enabled'] or not final['body']['fresh'] or final['chassis_active'] or not final.get('nav_sdk_ready',False) or final['capabilities']['chassis'] or (n.special_stale_inputs() if no_geometry else n.stale_inputs())):self.hub.set_navigation_check('waiting','最终底盘输出或数据时效复核尚未通过');return
   if n.ready_file is not None and not n.ready_file.is_file():
    self.hub.set_navigation_check('waiting','导航执行门已关闭');self.window.observe(time.monotonic(),False,(0,0));return
   # HTTP may have blocked after the geometric and stopped-motion proof.
   # Never acknowledge a batch that expired, changed or was withdrawn meanwhile.
   if not self.drain_output_ends():return
   n.validate_motion_proof(stamp)
   if a['status']=='cancelling':self.hub.operator_cancel_verified(a['visit_id'])
   self.waypoints.accept_after_ns=n.get_clock().now().nanoseconds
   if a['status']=='resume_pending' and not intent:
    intent=dict(visit_id=a['visit_id'],route_id=route.route_id,mission_id=route.mission_id,index=route.index,
     previous_goal_stamp=self.waypoints.goal_stamp,point=None)
    self.resume_intent=intent
   if not retrying:self.hub.acknowledge(a['visit_id'])
   self.publish_lock()
   self.hub.set_navigation_check('verified' if a['status']=='resume_pending' else 'cancelled','导航已确认交接，按剩余路线处理' if a['status']=='resume_pending' else '取消已确认，不恢复旧目标')
   self.waypoints.last_resume_pose=poses['map'].copy();self.waypoints.phase=None
   if (a['status']=='resume_pending' or retrying) and self.waypoints.route.running:self.resume_route(intent,poses['map'])
   self.waypoints.show()
  except Exception as exc:
   self.hub.set_navigation_check('waiting',('等待导航复核：'+str(exc))[:300])
   self.window.observe(time.monotonic(),False,(0,0));n.get_logger().warn('Task resume waiting: '+str(exc),throttle_duration_sec=3.)
 def close(self):
  if self.server:self.server.close()
  if self.temp:self.temp.cleanup()
