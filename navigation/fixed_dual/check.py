import json,sys,time
from collections import defaultdict
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu,PointCloud2,LaserScan
from nav_msgs.msg import Odometry
rclpy.init();n=Node('fixed_dual_read_only_check');rows=defaultdict(list);details={};subs=[]
def receive(k,m):
 t=m.header.stamp.sec+1e-9*m.header.stamp.nanosec;rows[k].append(t)
 if isinstance(m,CustomMsg):
  offsets=np.array([p.offset_time for p in m.points]);details[k]=dict(points=m.point_num,clock_error_ns=round(t*1e9)-m.timebase,offsets_sorted=bool(np.all(np.diff(offsets)>=0)),offset_max_ns=int(offsets.max()) if len(offsets) else 0)
 if isinstance(m,Odometry):
  p=m.pose.pose.position;details.setdefault(k,[]).append([p.x,p.y,p.z])
 if isinstance(m,LaserScan):details.setdefault(k,{}).update({m.header.frame_id:int(np.isfinite(m.ranges).sum())})
spec=[('/single/lidar_raw',CustomMsg),('/unused_head/lidar',CustomMsg),('/single/imu',Imu),('/unused_head/imu',Imu),('/fixed_dual/lidar',CustomMsg),('/localization/odometry',Odometry),('/fixed_dual/bottom/cloud_body',PointCloud2),('/fixed_dual/head/cloud_body',PointCloud2),('/pb/bottom/stvl_evidence',PointCloud2),('/pb/head/stvl_evidence',PointCloud2),('/pb/mapping_scan',LaserScan),('/pb/head/direct_scan',LaserScan)]
for k,cls in spec:subs.append(n.create_subscription(cls,k,lambda m,k=k:receive(k,m),qos_profile_sensor_data))
end=time.monotonic()+float(sys.argv[2] if len(sys.argv)>2 else 12)
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
report={}
for k,_ in spec:
 a=np.array(rows[k]);item=dict(count=len(a))
 if len(a)>1:item.update(hz=(len(a)-1)/(a[-1]-a[0]) if a[-1]>a[0] else 0,nonincreasing=int((np.diff(a)<=0).sum()),max_gap_ms=1000*float(np.diff(a).max()))
 item['details']=details.get(k)
 if k=='/localization/odometry' and k in details:item['position_span_mm']=(np.ptp(details[k],axis=0)*1000).tolist();item['details']=None
 report[k]=item
Path(sys.argv[1]).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
