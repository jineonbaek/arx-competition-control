"""Read-only sensor filtering or chassis odometry. No actuator publishers."""
import argparse,copy,json,os,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from sensor_msgs_py.point_cloud2 import read_points_numpy,create_cloud_xyz32
from livox_ros_driver2.msg import CustomMsg
from core import body_state,initial_gauge
from odom_guard import OdomGuard,map_output_allowed
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'dual_flex'))
from mesh_filter import MeshFilter
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rear_sector import rear_points,mask_scan


def tf_message(parent,child,T,stamp):
 m=TransformStamped();m.header.frame_id=parent;m.child_frame_id=child;m.header.stamp=stamp
 m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=map(float,T[:3,3]);q=Rotation.from_matrix(T[:3,:3]).as_quat();m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=map(float,q);return m

class Runtime(Node):
 def __init__(self,mode,cfg,mesh,run):
  super().__init__('bottom_upright_'+mode);self.run=run;self.cfg=cfg;self.mode=mode;self.last=None;self.failed=False;self.G=None;self.count=0;self.total_points=0;self.removed=0;self.max_ms=0.;self.sum_ms=0.;self.last_map=0
  self.sensor=np.eye(4);self.sensor[:3,3]=cfg['bottom']['lidar_origin_m'];self.imu=np.eye(4);self.imu[:3,3]=cfg['bottom']['imu_origin_m']
  if mode=='filter':
   z=np.load(mesh);self.rigid=MeshFilter(z['rigid'],np.empty((0,3)),z['nominal_head'],float(z['pitch']));self.head=MeshFilter(np.empty((0,3)),z['head'],z['nominal_head'],float(z['pitch']));self.nominal_head=z['nominal_head']
   self.pub=self.create_publisher(CustomMsg,'/single/lidar_filtered',20);self.sub=self.create_subscription(CustomMsg,'/single/lidar_raw',self.filter,qos_profile_sensor_data)
  elif mode=='body':
   self.guard=OdomGuard();self.gap_warned=False;self.output_state={'healthy':False}
   self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self);self.pub=self.create_publisher(Odometry,'/localization/odometry',20)
   self.sub=self.create_subscription(Odometry,'/single/odometry_raw',self.odom,50)
  else:
   # Map conversion/publication can block; it must never stall odometry/TF.
   self.map_pub=self.create_publisher(PointCloud2,'/mapping/cloud_map',2)
   self.map_sub=self.create_subscription(PointCloud2,'/single/map_raw',self.map,2)
   self.map_count=0;self.map_max_ms=0.

 def filter(self,msg):
  start=time.monotonic();stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
  if self.failed:return
  if self.last is not None and stamp<=self.last:
   self.failed=True;self.get_logger().error('LiDAR timestamp reversed; filter latched closed, restart stationary');return
  self.last=stamp
  pts=np.array([(p.x,p.y,p.z) for p in msg.points],dtype=np.float32).reshape(-1,3)
  # Nominal head geometry gets a larger exclusion margin for untracked flex.
  mask=self.rigid.mask(pts,self.sensor,self.nominal_head,.02)|self.head.mask(pts,self.sensor,self.nominal_head,.10)|~np.isfinite(pts).all(axis=1)
  mask |= rear_points(pts)
  n=len(msg.points);msg.points=[p for p,drop in zip(msg.points,mask) if not drop];msg.point_num=len(msg.points);msg.header.frame_id='bottom_livox'
  self.pub.publish(msg);self.count+=1;self.total_points+=n;self.removed+=int(mask.sum());ms=(time.monotonic()-start)*1000;self.max_ms=max(self.max_ms,ms);self.sum_ms+=ms
  if self.count%10==0:
   (self.run/'filter-health.json').write_text(json.dumps({'frames':self.count,'last_stamp_ns':stamp,'input_points':self.total_points,'removed_points':self.removed,'mean_callback_ms':self.sum_ms/self.count,'max_callback_ms':self.max_ms,'head_margin_m':.10,'rigid_margin_m':.02,'point_time_offsets_preserved':True,'commands_sent':0},indent=2))

 def odom(self,msg):
  stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
  if self.failed:return
  po=msg.pose.pose;tw=msg.twist.twist
  pos=[po.position.x,po.position.y,po.position.z];q=[po.orientation.x,po.orientation.y,po.orientation.z,po.orientation.w];v=np.array([tw.linear.x,tw.linear.y,tw.linear.z,tw.angular.x,tw.angular.y,tw.angular.z])
  if msg.header.frame_id!='lio_world' or msg.child_frame_id!='bottom_imu' or not np.isfinite(np.r_[pos,q,v]).all():
   self.failed=True;self.output_status(False);self.get_logger().error('Unexpected odometry frame or nonfinite value');return
  p,q,vel,J,Jp=body_state(pos,q,v,self.cfg['bottom']['imu_origin_m']);T=np.eye(4);T[:3,:3]=Rotation.from_quat(q).as_matrix();T[:3,3]=p
  decision=self.guard.accept(stamp,self.get_clock().now().nanoseconds,p,q,vel)
  if decision=='fault':
   self.failed=True;self.output_status(False);self.get_logger().error(self.guard.reason+'; output latched closed');return
  self.last=stamp
  if decision=='wait':
   self.output_status(False)
   if self.guard.pending and not self.gap_warned:
    self.gap_warned=True;self.get_logger().warn('Positive odometry gap: holding output until 3 fresh stationary samples')
   return
  if self.gap_warned:self.get_logger().info('Stationary odometry stream recovered with unchanged world gauge');self.gap_warned=False
  if self.G is None:
   self.G=initial_gauge(T);internal=np.eye(4);internal[:3,3]=np.array(self.cfg['bottom']['lidar_origin_m'])-self.cfg['bottom']['imu_origin_m']
   self.static.sendTransform([tf_message('odom','lio_world',self.G,msg.header.stamp),tf_message('base_link','bottom_imu',self.imu,msg.header.stamp),tf_message('bottom_imu','bottom_livox',internal,msg.header.stamp)])
   sys.path.insert(0,str(root/'fixed_dual'))
   from geometry import enabled,transforms
   if enabled(root):self.static.sendTransform(tf_message('base_link','rear_livox',transforms()[1],msg.header.stamp))
   (self.run/'world-gauge.json').write_text(json.dumps({'odom_from_lio_world':self.G.tolist(),'base_link_definition':'user O ground projection, not original URDF base_link','configuration':self.cfg['bottom']},indent=2))
  T=self.G@T;out=copy.deepcopy(msg);out.header.frame_id='odom';out.child_frame_id='base_link';out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,T[:3,3]);qq=Rotation.from_matrix(T[:3,:3]).as_quat();out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,qq)
  out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,vel[:3]);out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,vel[3:]);out.twist.covariance=(J@np.array(msg.twist.covariance).reshape(6,6)@J.T).ravel().tolist()
  A=np.zeros((6,6));A[:3,:3]=A[3:,3:]=self.G[:3,:3];out.pose.covariance=(A@Jp@np.array(msg.pose.covariance).reshape(6,6)@Jp.T@A.T).ravel().tolist();self.pub.publish(out);self.tf.sendTransform(tf_message('odom','base_link',T,msg.header.stamp));self.count+=1
  self.output_state.update(last_published_ns=stamp,position_raw_m=list(map(float,p)),quaternion_raw_xyzw=list(map(float,q)))
  self.output_status(True)
  if self.count%10==0:(self.run/'body-health.json').write_text(json.dumps({'frames':self.count,'last_stamp_ns':stamp,'position_m':T[:3,3].tolist(),'commands_sent':0}))

 def output_status(self,healthy):
  self.output_state['healthy']=healthy
  temporary=self.run/f'body-output-state-{os.getpid()}.tmp'
  temporary.write_text(json.dumps(self.output_state));temporary.replace(self.run/'body-output-state.json')

 def map(self,msg):
  try:state=json.loads((self.run/'body-output-state.json').read_text())
  except (FileNotFoundError,ValueError):return
  if not map_output_allowed(state,self.get_clock().now().nanoseconds):return
  if self.G is None and self.mode=='map':
   try:
    saved=json.loads((self.run/'world-gauge.json').read_text())
    if saved['configuration']!=self.cfg['bottom']:raise ValueError('Map gauge/configuration mismatch')
    self.G=np.asarray(saved['odom_from_lio_world'],float)
   except FileNotFoundError:return
  if self.G is None or self.failed or time.monotonic()-self.last_map<1:return
  if msg.header.frame_id!='lio_world':return
  self.last_map=time.monotonic();begin=self.last_map;pts=read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True).reshape(-1,3);pts=pts@self.G[:3,:3].T+self.G[:3,3];header=copy.deepcopy(msg.header);header.frame_id='odom';self.map_pub.publish(create_cloud_xyz32(header,pts.astype(np.float32)))
  ms=(time.monotonic()-begin)*1000;self.map_count+=1;self.map_max_ms=max(self.map_max_ms,ms)
  (self.run/'map-relay-health.json').write_text(json.dumps({'frames':self.map_count,'points':len(pts),'callback_ms':ms,'max_callback_ms':self.map_max_ms,'commands_sent':0}))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['filter','body','map']);p.add_argument('--config',required=True);p.add_argument('--mesh',required=True);p.add_argument('--run',required=True);a=p.parse_args();rclpy.init();node=Runtime(a.mode,json.loads(Path(a.config).read_text()),a.mesh,Path(a.run))
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.try_shutdown()
