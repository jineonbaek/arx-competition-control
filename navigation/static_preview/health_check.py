"""Bounded passive preview metrics; raw scene points are not exported."""
import os,time,json,argparse
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import PointCloud2,LaserScan
from sensor_msgs_py.point_cloud2 import read_points_numpy
from nav_msgs.msg import OccupancyGrid,Path as NavPath
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=20);p.add_argument('--output',default='preview-check.json');args=p.parse_args()
rclpy.init();n=Node('preview_readonly_inspection');rows={};last={};subs=[]
def cb(m,k):
 rows[k].append(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
 if isinstance(m,PointCloud2):
  pts=read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
  last[k]={'points':len(pts),'frame':m.header.frame_id,'z_quantiles':np.quantile(pts[:,2],[.05,.5,.95]).tolist() if len(pts) else []}
 elif isinstance(m,LaserScan):last[k]={'frame':m.header.frame_id,'finite':int(np.isfinite(m.ranges).sum()),'beams':len(m.ranges),'inf':int(np.isinf(m.ranges).sum())}
 elif isinstance(m,OccupancyGrid):
  a=np.array(m.data);last[k]={'frame':m.header.frame_id,'size':[m.info.width,m.info.height],'resolution':m.info.resolution,'free':int((a==0).sum()),'unknown':int((a<0).sum()),'occupied':int((a>=99).sum()),'inflated':int(((a>0)&(a<99)).sum())}
 else:last[k]={'poses':len(m.poses)}
for k,t in [('/preview/level_cloud',PointCloud2),('/preview/ground',PointCloud2),('/preview/obstacle_raw',PointCloud2),('/preview/obstacles',PointCloud2),('/preview/scan',LaserScan),('/preview/slam_map',OccupancyGrid),('/mapping/grid',OccupancyGrid),('/local_costmap/costmap',OccupancyGrid),('/global_costmap/costmap',OccupancyGrid),('/preview/path',NavPath)]:
 rows[k]=[];q=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
 if t==OccupancyGrid:q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 subs.append(n.create_subscription(t,k,lambda m,k=k:cb(m,k),q))
a=time.monotonic()
while time.monotonic()-a<args.seconds:rclpy.spin_once(n,timeout_sec=.05)
result={}
for k,v in rows.items():
 d=np.diff(v);result[k]={'count':len(v),'hz':float(len(d)/(v[-1]-v[0])) if len(d) and v[-1]>v[0] else None,'nonincreasing':int((d<=0).sum()),'last':last.get(k)}
Path(os.environ['ARX_PREVIEW_RUN'],args.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
