"""ROS-thread handoff safety gate; HTTP thread never mutates goals or hardware."""
import json,os,tempfile,time
from pathlib import Path
from rclpy.time import Time
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile,DurabilityPolicy
from task_handoff import HandoffHub
from task_http import TaskServer
from mppi_trial.stationary import StopWindow
from mppi_trial.recovery import RecoveryWindow

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
  if os.getenv('ROS_DOMAIN_ID')=='192' and os.getenv('ARX_TASK_API','1')=='1':self.server=TaskServer(self.hub,int(os.getenv('ARX_TASK_PORT','9985')))
  self.publish_lock()
 def publish_lock(self):self.lock_pub.publish(Bool(data=self.hub.blocking()))
 def notify(self,p):
  n=self.driver;n.end_output();mp,_,source=n.motion_poses();s=n.pose_snapshot.header.stamp
  n.validate_motion_proof(source)
  self.hub.begin(dict(mission_id=self.waypoints.route.mission_id,point_id=p['point_id'],route_point_id=p['id'],map_id=self.waypoints.map_id,goal_pose=p['pose'],arrival_pose=mp.tolist(),pose_stamp_ns=s.sec*1000000000+s.nanosec))
  self.publish_lock()
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
  n=self.driver;self.hub.tick();a=self.hub.state()['task']
  key=None if a is None else (a['visit_id'],a['status'],a['revision'])
  if key!=self.last_state:self.last_state=key;self.publish_lock();self.waypoints.show()
  if not a or a['status'] not in ('resume_pending','cancelled','cancelling'):
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
   direct=(self.waypoints.direct_ready(poses['map'],after_task=True) or final_task) and a['status']=='resume_pending'
   stale=n.special_stale_inputs() if direct else n.stale_inputs()
   if direct:n.special_poses()
   if not ready or stale:
    reason='等待作业结束底盘输出或执行接口就绪' if not ready else '定位、扫描或代价地图数据不新鲜'
    self.hub.set_navigation_check('waiting',reason);self.window.observe(time.monotonic(),False,(0,0));return
   if not direct and any(n.grids[f][1].inspect(poses[f])['first_cell'] for f in poses):
    self.hub.set_navigation_check('waiting','当前整车轮廓与代价地图占用相交');self.window.observe(time.monotonic(),False,(0,0));return
   if stamp!=self.stamp:self.stationary=self.stop.add(stamp,poses['odom']);self.stamp=stamp
   revisions=(int(stamp*1e9),int(stamp*1e9)) if direct else tuple(n.grids[f][0].sec*1000000000+n.grids[f][0].nanosec for f in ('map','odom'))
   if not self.window.observe(time.monotonic(),self.stationary,revisions):self.hub.set_navigation_check('checking','等待实测停稳及新地图验证');return
   # Re-read control state after the proof; no cached ownership is sufficient.
   final=self.console()
   if (not final['execution_enabled'] or not final['body']['fresh'] or final['chassis_active'] or not final.get('nav_sdk_ready',False) or final['capabilities']['chassis'] or (n.special_stale_inputs() if direct else n.stale_inputs())):self.hub.set_navigation_check('waiting','最终底盘输出或数据时效复核尚未通过');return
   if n.ready_file is not None and not n.ready_file.is_file():
    self.hub.set_navigation_check('waiting','导航执行门已关闭');self.window.observe(time.monotonic(),False,(0,0));return
   # HTTP may have blocked after the geometric and stopped-motion proof.
   # Never acknowledge a batch that expired, changed or was withdrawn meanwhile.
   n.validate_motion_proof(stamp)
   if a['status']=='cancelling':self.hub.operator_cancel_verified(a['visit_id'])
   self.waypoints.accept_after_ns=n.get_clock().now().nanoseconds
   self.hub.acknowledge(a['visit_id']);self.publish_lock()
   self.hub.set_navigation_check('verified' if a['status']=='resume_pending' else 'cancelled','导航已确认交接，按剩余路线处理' if a['status']=='resume_pending' else '取消已确认，不恢复旧目标')
   self.waypoints.last_resume_pose=poses['map'].copy();self.waypoints.phase=None
   if a['status']=='resume_pending' and self.waypoints.route.running:self.waypoints.dispatch(self.waypoints.route.release_task(pose=poses['map']))
   self.waypoints.show()
  except Exception as exc:
   self.hub.set_navigation_check('waiting',('等待导航复核：'+str(exc))[:300])
   self.window.observe(time.monotonic(),False,(0,0));n.get_logger().warn('Task resume waiting: '+str(exc),throttle_duration_sec=3.)
 def close(self):
  if self.server:self.server.close()
  if self.temp:self.temp.cleanup()
