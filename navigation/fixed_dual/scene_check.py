import json,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg
from geometry import transforms
rclpy.init();n=Node('fixed_rear_scene_check');cloud={};sub=[]
def receive(s,m):
 p=np.array([(p.x,p.y,p.z) for p in m.points[::5]],float);T=transforms()[s];p=p@T[:3,:3].T+T[:3,3];cloud[s]=p[np.isfinite(p).all(1)]
for i,k in enumerate(['/single/lidar_raw','/unused_head/lidar']):sub.append(n.create_subscription(CustomMsg,k,lambda m,s=i:receive(s,m),qos_profile_sensor_data))
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
report={}
for s,p in cloud.items():
 floor=p[(np.linalg.norm(p[:,:2],axis=1)>.7)&(p[:,2]>-.1)&(p[:,2]<.08)]
 report[str(s)]=dict(points=len(p),near_ground=len(floor),ground_z_median_m=float(np.median(floor[:,2])) if len(floor) else None)
if len(cloud)==2:
 a,b=[p[(p[:,2]>.12)&(p[:,2]<1.3)&(np.linalg.norm(p[:,:2],axis=1)>1.)] for p in (cloud[0],cloud[1])]
 d=cKDTree(a).query(b)[0];report['overlap']=dict(samples=len(d),nearest_median_m=float(np.median(d)),within_10cm_fraction=float(np.mean(d<.1)),note='Sparse views and occlusion; diagnostic, not calibrated accuracy')
print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
