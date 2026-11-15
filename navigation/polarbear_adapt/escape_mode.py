"""Goal-preserving Local withdrawal and short-lived sparse planning leases."""
import json,math,os,time
from pathlib import Path
import numpy as np
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header,Bool
from visualization_msgs.msg import Marker
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from path_audit import AuditGrid
from continuous_track import swept_poses
from axis_trial import body_delta
from escape_policy import choose,contacts,admissible,key,sparse_cells
from mppi_trial.session_policy import slew_effort

class EscapeMode:
 def __init__(self,driver,config=None):
  self.d=driver;path=Path(__file__).resolve().parents[1]/'config/low_speed_escape.json'
  self.config=config if config is not None else (json.loads(path.read_text()) if path.exists() else {'enabled':False})
  self.enabled=self.config.get('enabled') is True;self.attempted=False;self.busy=False;self.pins=[];self.reference=None;self.reference_at=0.;self.reference_stamp=0.;self.failed=None;self.control=None;self.checked_at=0.;self.last_search=0.;self.owner=None
  self.enabled=bool(driver.declare_parameter('low_speed_escape_enabled',self.enabled,ParameterDescriptor(read_only=True)).value)
  driver.declare_parameter('low_speed_escape_sparse',bool(self.config.get('allow_sparse',False)),ParameterDescriptor(read_only=True))
  driver.get_logger().info(f'Low-speed escape enabled={self.enabled}; sparse={self.config.get("allow_sparse",False)}; XY SDK cap=.14; max6 cells/2 clusters')
  self.pub=driver.create_publisher(PointCloud2,'/pb/escape/sparse_lease',1)
  driver.create_subscription(Costmap,'/pb/escape/local_reference',self.observe,1)
  self.timer=driver.create_timer(.05,self.tick)
 def log(self,event,**data):
  value=dict(event=event,time=time.time(),goal_seq=self.d.seq,**data);run=os.getenv('ARX_ADAPT_RUN')
  if run:
   with (Path(run)/'escape-events.jsonl').open('a') as f:f.write(json.dumps(value,allow_nan=False)+'\n')
  self.d.status('escape_'+event)
 def observe(self,m):
  if m.header.frame_id!='odom' or m.metadata.size_x*m.metadata.size_y!=len(m.data):return
  stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if stamp<=self.reference_stamp:return
  v=m.metadata;a=np.asarray(m.data,dtype=np.uint8).reshape(v.size_y,v.size_x);a=np.where(a==254,100,np.where(a==255,-1,0));self.reference=AuditGrid(a,[v.origin.position.x,v.origin.position.y],v.resolution,self.d.poly);self.reference_stamp=stamp;self.reference_at=time.monotonic()
 def publish(self):
  self.pub.publish(create_cloud_xyz32(Header(frame_id='odom',stamp=self.d.get_clock().now().to_msg()),np.asarray([[x,y,0.] for x,y in self.pins],dtype=np.float32).reshape(-1,3)))
  m=Marker();m.header.frame_id='odom';m.header.stamp=self.d.get_clock().now().to_msg();m.ns='escape_sparse_lease';m.id=0;m.type=Marker.CUBE_LIST;m.action=Marker.ADD if self.pins else Marker.DELETE;m.scale.x=m.scale.y=.02;m.scale.z=.04;m.pose.orientation.w=1.;m.color.r=1.;m.color.g=.45;m.color.a=.9;m.points=[Point(x=float(x),y=float(y),z=.05) for x,y in self.pins];self.d.mark.publish(m)
 def clear(self):
  had=bool(self.pins);self.pins=[];self.publish();self.control=None
  if had:self.log('sparse_withdrawn')
 def reset(self):
  self.clear();self.attempted=False;self.failed=None;self.owner=None;self.last_search=0.
 def console(self,begin=False):
  with self.d.transport.client.open(self.d.transport.origin+'/api/state',timeout=.18) as f:s=json.load(f)
  if (not s['execution_enabled'] or not s['body']['fresh'] or not s.get('nav_sdk_ready',False) or s.get('motion_reserved',False) or s['capabilities']['chassis'] or (s['chassis_active'] and not self.d.active)):raise ValueError('console_not_ready_or_busy')
  current=(s.get('command_epoch'),self.d.transport.sequence)
  if begin:self.control=current
  elif self.control is not None and current[0]!=self.control[0]+current[1]-self.control[1]:raise ValueError('external_command_during_escape')
  self.checked_at=time.monotonic()
 def tick(self):
  if not self.pins:return
  try:
   if self.d.target is None or self.d.seq!=self.owner or self.d.waypoints.blocking():self.clear();return
   if self.d.stale_inputs():raise ValueError('escape_inputs_stale')
   if time.monotonic()-self.checked_at>.25:self.console()
   if self.reference is not None and self.reference_stamp>=self.lease_start:
    if time.monotonic()-self.reference_at>.35:raise ValueError('escape_reference_stale')
    eligible=sparse_cells(self.reference);p=self.d.pose('odom',self.d.now());remaining=[]
    for xy in self.pins:
     k=key(xy,self.reference.res);i,j=np.floor((np.asarray(xy)-self.reference.origin)/self.reference.res).astype(int)
     if not (0<=j<self.reference.grid.shape[0] and 0<=i<self.reference.grid.shape[1]):continue
     value=self.reference.grid[j,i]
     if value==0 or np.linalg.norm(np.asarray(xy)-p[:2])>1.2:continue
     if value!=100 or k not in eligible:raise ValueError('escape_exception_changed')
     remaining.append(xy)
    self.pins=remaining
   self.publish()
  except Exception as e:
   self.failed=str(e);self.clear()
 def target_odom(self,map_pose,odom):
  vias=self.d.waypoints.route.vias();goal=np.asarray(vias[0]['pose'] if vias else self.d.target,float);angle=odom[2]-map_pose[2];c,s=math.cos(angle),math.sin(angle);result=odom.copy();result[:2]=odom[:2]+(goal[:2]-map_pose[:2])@np.array([[c,s],[-s,c]]);return result
 def maybe_run(self,epoch):
  if not self.enabled or self.attempted or self.pins or time.monotonic()-self.last_search<1.:return False
  try:mp=self.d.checked(occupancy=False);op=self.d.pose('odom',self.d.now())
  except Exception:return False
  g=self.d.grids['odom'][1];local,unknown=contacts(g,op)
  if unknown:return False
  if not local and not self.d.grids['map'][1].inspect(mp)['first_cell']:return False
  self.last_search=time.monotonic();self.d.brake();target=self.target_odom(mp,op);selection=choose(g,op,target,self.config.get('allow_sparse',False))
  if selection is None:self.d.waypoints.show('无可行脱困方向');return False
  self.d.spin(.05);self.d.current_intent(epoch)
  self.console(begin=True);self.attempted=True;self.owner=epoch;self.busy=True
  self.log('selected',mode=selection['mode'],direction=selection['direction'].tolist(),ignored=selection['ignored'],target_direction=(target[:2]-op[:2]).tolist())
  try:
   if selection['mode']=='sparse':self.arm_sparse(selection,epoch)
   else:self.withdraw(selection,epoch)
   return True
  finally:self.busy=False
 def arm_sparse(self,selection,epoch):
  self.pins=selection['ignored'];self.lease_start=self.d.now();self.reference=None;self.reference_at=0.;self.failed=None;self.publish();self.d.waypoints.show(f'低速孤点许可 {len(self.pins)}格，等待重规划')
  end=time.monotonic()+3.
  while time.monotonic()<end:
   self.d.current_intent(epoch)
   if self.failed:
    error=self.failed;self.failed=None;raise ValueError(error)
   self.d.spin(.05)
   if not self.pins:return
   try:
    self.d.checked()
    if self.reference is not None and self.reference_stamp>self.lease_start:return
   except Exception:pass
  self.clear();raise ValueError('escape_lease_unavailable')
 def withdraw(self,selection,epoch):
  d=self.d;d.waypoints.show('低速脱离占用');start=d.pose('odom',d.now());previous_pose=start.copy();best_progress=0.;baseline=selection['baseline'];direction=selection['direction'];prior=np.zeros(3);measured=np.zeros(3);last=time.monotonic();sample=start.copy();sample_at=last;progress_at=last;progress_pose=start.copy();began=last;clear_since=None;last_revision=None
  try:
   while time.monotonic()-began<6.:
    tick=time.monotonic();dt=tick-last;last=tick;d.current_intent(epoch)
    if d.active and dt>.30:raise ValueError('control_cycle_overrun')
    mp=d.checked(occupancy=False);op=d.pose('odom',d.now());g=d.grids['odom'][1];_,op,source=d.motion_poses();d.waypoints.observe(mp,op,source)
    if tick-self.checked_at>.20:self.console()
    delta=body_delta(start,op)
    if np.linalg.norm(delta[:2])>=.18:break
    if abs(delta[2])>.05:raise ValueError('escape_heading_changed')
    along=float((op[:2]-start[:2])@direction);cross=abs(float(np.cross(direction,op[:2]-start[:2])))
    if along<best_progress-.004 or cross>.025:raise ValueError('escape_unexpected_motion')
    best_progress=max(best_progress,along)
    ok,reason=admissible(g,baseline,np.asarray([previous_pose,op]));previous_pose=op.copy()
    if not ok:raise ValueError('escape_'+reason)
    now_contacts,unknown=contacts(g,op);glob=d.grids['map'][1].inspect(mp)
    if not now_contacts and not unknown and not glob['first_cell']:
     d.brake();revision=d.grids['odom'][0].sec*10**9+d.grids['odom'][0].nanosec
     if clear_since is None:clear_since=tick;last_revision=revision
     if tick-clear_since>.25 and revision!=last_revision:self.log('withdraw_completed');return
     d.spin(.05);continue
    clear_since=None
    if tick-sample_at>.20:measured=body_delta(sample,op)/(tick-sample_at);sample=op.copy();sample_at=tick
    if tick-progress_at>1.5:
     if np.linalg.norm(op[:2]-progress_pose[:2])<.008:raise ValueError('escape_no_progress')
     progress_pose=op.copy();progress_at=tick
    angle=op[2];c,s=math.cos(angle),math.sin(angle);body=direction@np.array([[c,-s],[s,c]]);effort=.14*min(1.,.08/max(np.linalg.norm(measured[:2]),.001));command=np.r_[body*effort,0.];command=np.asarray(slew_effort(prior,command,dt))
    for sweep in swept_poses(op,command,measured):
     ok,reason=admissible(g,baseline,np.asarray(sweep))
     if not ok:raise ValueError('escape_'+reason)
    if d.stale_inputs() or time.monotonic()-tick>.10:d.brake();prior=np.zeros(3);d.spin(.025);continue
    d.current_intent(epoch);d.validate_motion_proof(source)
    d.intent.publish(Bool(data=True));d.active=True;d.transport.send(command);prior=command;d.spin(max(.001,.05-(time.monotonic()-tick)))
   self.log('withdraw_limit_waiting_replan')
  finally:d.brake();self.control=None
 def before_command(self,command):
  if self.failed:
   error=self.failed;self.failed=None;raise ValueError(error)
  if not self.pins:return command
  if self.reference is None or time.monotonic()-self.reference_at>.35:raise ValueError('escape_reference_stale')
  result=command.copy();result[:2]*=min(1.,.14/max(np.linalg.norm(result[:2]),1e-9));return result
 def restore_for_arrival(self):
  if self.failed:
   error=self.failed;self.failed=None;raise ValueError(error)
  if not self.pins:return
  self.clear();cut=self.d.now();end=time.monotonic()+2.
  while time.monotonic()<end:
   self.d.spin(.05)
   if all(m.sec+m.nanosec*1e-9>cut for m,_ in self.d.grids.values()):return
  raise ValueError('escape_restore_stale')
