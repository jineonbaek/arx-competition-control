#!/usr/bin/env python3
"""Finite passive static-pose test; requires operator-confirmed stillness."""
import argparse,csv,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--duration',type=float,default=90);parser.add_argument('--warmup',type=float,default=20);args=parser.parse_args()
args.output.mkdir(parents=True,exist_ok=True);rclpy.init();node=Node('static_mapping_validation');poses=[];started=time.monotonic();collect_after=started+args.warmup;last_received=None
csvfile=(args.output/'static-pose-samples.csv').open('w',newline='');writer=csv.writer(csvfile);writer.writerow(['arrival_monotonic','header_sec','x','y','z','qx','qy','qz','qw'])
def callback(msg):
 global last_received
 now=time.monotonic();last_received=now
 if now<collect_after:return
 p=msg.pose.pose.position;q=msg.pose.pose.orientation;row=[now,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w]
 if not np.isfinite(row).all():raise RuntimeError('Non-finite odometry')
 poses.append(row);writer.writerow(row)
sub=node.create_subscription(Odometry,'/mapping_test/odometry',callback,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE))
status={'stage':'warming_up','robot_commands_sent':False}
try:
 end=collect_after+args.duration;next_report=started
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.1);now=time.monotonic()
  if now>=next_report:
   status.update(stage='sampling' if now>=collect_after else 'warming_up',elapsed_s=now-started,samples=len(poses),last_message_age_s=None if last_received is None else now-last_received)
   (args.output/'static-test-progress.json').write_text(json.dumps(status,indent=2));next_report=now+5
  if now-started>15 and last_received is None:raise RuntimeError('No new LIO odometry; test not valid')
  if last_received is not None and now-last_received>2:raise RuntimeError('LIO data stream stalled; test not valid')
 if len(poses)<args.duration*5:raise RuntimeError('Too few samples for a valid static test')
 a=np.asarray(poses);xyz=a[:,2:5];q=a[:,5:9];norms=np.linalg.norm(q,axis=1)
 if np.any(norms<1e-9):raise RuntimeError('Invalid zero quaternion')
 q=q/norms[:,None]
 displacement=np.linalg.norm(xyz-xyz[0],axis=1);angles=np.degrees(2*np.arccos(np.clip(np.abs(q@q[0]),0,1)))
 t=a[:,0]-a[0,0];trend=[float(np.polyfit(t,xyz[:,i],1)[0]) for i in range(3)]
 result={'valid':True,'operator_stillness_required':True,'duration_s':float(t[-1]),'samples':len(a),'observed_hz':float((len(a)-1)/t[-1]),'end_displacement_m':float(displacement[-1]),'max_displacement_from_first_m':float(displacement.max()),'axis_peak_to_peak_m':np.ptp(xyz,axis=0).tolist(),'position_std_m':np.std(xyz,axis=0).tolist(),'end_orientation_change_deg':float(angles[-1]),'max_orientation_change_deg':float(angles.max()),'linear_trend_m_per_min':[x*60 for x in trend],'max_header_gap_s':float(np.max(np.diff(a[:,1]))),'header_time_backwards':int(np.sum(np.diff(a[:,1])<0)),'robot_commands_sent':False,'note':'Static precision result requires operator-confirmed stillness throughout; no manufacturer accuracy threshold is asserted.'}
 (args.output/'static-result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
except Exception as error:
 (args.output/'static-result.json').write_text(json.dumps({'valid':False,'reason':str(error),'robot_commands_sent':False},indent=2));raise
finally:
 csvfile.close();node.destroy_node();rclpy.shutdown()
