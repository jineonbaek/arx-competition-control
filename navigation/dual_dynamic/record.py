"""Read-only dual LIO/IMU timing and trajectory capture; raw scene data stays on NUC."""
import argparse,json,time,signal
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos_profile_sensor_data=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=30);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=Node('dual_independent_readonly_recorder');rows={};subs=[];stopping=False

def ns(st):return st.sec*10**9+st.nanosec

def callback(msg,key,kind):
    recv=time.time_ns(); stamp=ns(msg.header.stamp)
    row={'t':stamp,'recv':recv}
    if kind=='odom':
        v=msg.pose.pose;row.update(p=[v.position.x,v.position.y,v.position.z],q=[v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w],frame=msg.header.frame_id,child=msg.child_frame_id)
    elif kind=='imu':
        v=msg.angular_velocity;b=msg.linear_acceleration;row.update(w=[v.x,v.y,v.z],a=[b.x,b.y,b.z])
    else:row.update(timebase=int(msg.timebase),points=int(msg.point_num))
    rows[key].append(row)

for name,raw in [('top','top'),('bottom','arm')]:
 for kind,typ,topic in [('odom',Odometry,f'/dual/{name}/odometry'),('imu',Imu,f'/dual/{raw}/imu'),('cloud',CustomMsg,f'/dual/{raw}/lidar')]:
    key=name+'_'+kind;rows[key]=[]
    subs.append(node.create_subscription(typ,topic,lambda m,k=key,ty=kind:callback(m,k,ty),qos_profile_sensor_data))
start=time.monotonic();last_progress=-1
def stop(*_):
 global stopping
 stopping=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
try:
 while not stopping and time.monotonic()-start<a.seconds:
    rclpy.spin_once(node,timeout_sec=.05)
    if int(time.monotonic()-start)!=last_progress:
        last_progress=int(time.monotonic()-start)
        (out/'progress.json').write_text(json.dumps({'seconds':time.monotonic()-start,'counts':{k:len(v) for k,v in rows.items()},'commands_sent':0}))
finally:
 summary={'seconds':time.monotonic()-start,'streams':{},'commands_sent':0,'hardware_sync_verified':False,'fusion_enabled':False}
 for k,rs in rows.items():
    (out/(k+'.json')).write_text(json.dumps(rs))
    if not rs:summary['streams'][k]={'count':0};continue
    ts=np.array([r['t'] for r in rs],dtype=np.int64);ds=np.diff(ts)/1e9
    age=np.array([(r['recv']-r['t'])/1e9 for r in rs]);v={'count':len(rs),'hz':(len(rs)-1)/((ts[-1]-ts[0])/1e9) if len(rs)>1 and ts[-1]>ts[0] else None,'max_gap_sec':float(ds.max()) if len(ds) else None,'nonincreasing':int(np.sum(ds<=0)),'age_median_sec':float(np.median(age)),'age_p99_sec':float(np.percentile(age,99))}
    if k.endswith('odom'):v['position_span_m']=np.ptp(np.array([r['p'] for r in rs]),axis=0).tolist()
    summary['streams'][k]=v
 (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
 node.destroy_node();rclpy.shutdown()
