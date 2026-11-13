#!/usr/bin/env python3
"""Read mapping streams and request PCD snapshots; no robot command publisher."""
import json,math,time,sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Imu
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg
from std_srvs.srv import Trigger
root=Path(sys.argv[1]);rclpy.init();node=Node('arm_mapping_observer');qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
stats={};poses=[];map_points=0;first_pose=None;last_pose=None;max_displacement=0.;last_save=None;pending=None;last_save_request=0.;started=time.monotonic();next_report=started+5

def count(key,msg):
 s=stats.setdefault(key,{'messages':0,'first':time.monotonic(),'last':0});s['messages']+=1;s['last']=time.monotonic()

def cloud(msg):
 global map_points
 count('map',msg);map_points=msg.width*msg.height

def odom(msg):
 global first_pose,last_pose,max_displacement
 count('odometry',msg);p=msg.pose.pose.position;v=[p.x,p.y,p.z]
 if not all(math.isfinite(x) for x in v):raise RuntimeError('Nonfinite LIO pose')
 if first_pose is None:first_pose=v
 last_pose=v;max_displacement=max(max_displacement,math.dist(first_pose,v))
subs=[node.create_subscription(CustomMsg,'/mapping_test/lidar',lambda m:count('lidar',m),qos),node.create_subscription(Imu,'/mapping_test/imu',lambda m:count('imu',m),qos),node.create_subscription(PointCloud2,'/mapping_test/cloud_registered',lambda m:count('registered_cloud',m),qos),node.create_subscription(PointCloud2,'/mapping_test/map',cloud,qos),node.create_subscription(Odometry,'/mapping_test/odometry',odom,qos)]
client=node.create_client(Trigger,'/mapping_test/map_save')
try:
 while rclpy.ok():
  rclpy.spin_once(node,timeout_sec=.1);now=time.monotonic()
  if pending is not None and pending.done():
   response=pending.result();last_save={'success':response.success,'message':response.message,'at_unix':time.time()};pending=None
  if map_points>100 and pending is None and (last_save_request==0 or now-last_save_request>=60) and client.service_is_ready():
   pending=client.call_async(Trigger.Request());last_save_request=now
  if now>=next_report:
   result={'updated_unix':time.time(),'uptime_s':round(now-started,1),'map_points':map_points,'first_pose_imu_m':first_pose,'last_pose_imu_m':last_pose,'max_displacement_from_first_m':max_displacement,'last_save':last_save,'no_robot_command_publishers':True,'topics':{k:{'messages':v['messages'],'mean_hz':round((v['messages']-1)/(v['last']-v['first']),2) if v['messages']>1 and v['last']>v['first'] else 0,'age_s':round(now-v['last'],3)} for k,v in stats.items()}}
   temporary=root/'live-status.json.tmp';temporary.write_text(json.dumps(result,indent=2)+'\n');temporary.replace(root/'live-status.json');print(json.dumps({'map_points':map_points,'pose_m':last_pose,'last_save':last_save}),flush=True);next_report=now+5
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
