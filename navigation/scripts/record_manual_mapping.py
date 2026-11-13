#!/usr/bin/env python3
"""Passive manual-drive mapping recorder; never publishes robot commands."""
import argparse,csv,json,math,time,os
from pathlib import Path
import numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=300);p.add_argument('--odom-topic',default='/mapping_test/odometry');p.add_argument('--map-topic',default='/mapping_test/map');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);(a.output/'recorder.pid').write_text(str(os.getpid())+'\n')
rclpy.init();node=Node('manual_mapping_recorder');qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE);rows=[];map_points=[];last_receive=None;begin=time.monotonic()
f=(a.output/'trajectory.csv').open('w',newline='');w=csv.writer(f);w.writerow(['arrival_monotonic','header_sec','x','y','z','qx','qy','qz','qw'])
def odom(m):
 global last_receive
 last_receive=time.monotonic();p=m.pose.pose.position;q=m.pose.pose.orientation;row=[last_receive,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w]
 if not np.isfinite(row).all():raise RuntimeError('Nonfinite pose')
 rows.append(row);w.writerow(row)
def cloud(m):map_points.append(m.width*m.height)
subs=[node.create_subscription(Odometry,a.odom_topic,odom,qos),node.create_subscription(PointCloud2,a.map_topic,cloud,qos)]
try:
 next_status=begin
 while rclpy.ok() and time.monotonic()-begin<a.duration:
  rclpy.spin_once(node,timeout_sec=.1);now=time.monotonic()
  if now-begin>10 and last_receive is None:raise RuntimeError('No mapping pose stream')
  if last_receive is not None and now-last_receive>2:raise RuntimeError('Mapping pose stream stalled')
  if now>=next_status:
   status={'elapsed_s':round(now-begin,1),'samples':len(rows),'map_points':map_points[-1] if map_points else None,'robot_commands_sent':False,'last_pose_m':rows[-1][2:5] if rows else None}
   if rows:status['displacement_from_start_m']=math.dist(rows[0][2:5],rows[-1][2:5])
   (a.output/'progress.json').write_text(json.dumps(status,indent=2));f.flush();next_status=now+2
except KeyboardInterrupt:pass
finally:
 if len(rows)>1:
  values=np.asarray(rows);xyz=values[:,2:5];q=values[:,5:9];q/=np.linalg.norm(q,axis=1)[:,None];ang=float(np.degrees(2*np.arccos(np.clip(abs(q[0]@q[-1]),0,1))))
  result={'samples':len(rows),'duration_s':float(values[-1,0]-values[0,0]),'end_displacement_from_start_m':float(np.linalg.norm(xyz[-1]-xyz[0])),'max_displacement_from_start_m':float(np.linalg.norm(xyz-xyz[0],axis=1).max()),'axis_span_m':np.ptp(xyz,axis=0).tolist(),'end_orientation_change_deg':ang,'first_map_points':map_points[0] if map_points else None,'last_map_points':map_points[-1] if map_points else None,'operator_return_to_start_confirmed':None,'robot_commands_sent':False,'note':'Trajectory displacement is not independently measured accuracy; endpoint difference is a return error only if the operator confirms return to the same physical pose.'}
  (a.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
 f.close();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
