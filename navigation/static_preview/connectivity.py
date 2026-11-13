"""Read-only connectivity audit and candidate for plan-only test."""
import os,json,time
from pathlib import Path
import numpy as np,rclpy
from scipy.ndimage import label
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer,TransformListener
rclpy.init();n=Node('preview_connectivity');b=Buffer();l=TransformListener(b,n);maps=[]
s=n.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:maps.append(m),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL));end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
m=maps[-1];t=b.lookup_transform('preview_map','base_link',rclpy.time.Time());x=t.transform.translation.x;y=t.transform.translation.y;grid=np.array(m.data).reshape(m.info.height,m.info.width);r=m.info.resolution;ox=m.info.origin.position.x;oy=m.info.origin.position.y;i=int((y-oy)/r);j=int((x-ox)/r)
mask=(grid>=0)&(grid<99);mask[i,j]=True;labs,num=label(mask);comp=np.argwhere(labs==labs[i,j]);xy=np.column_stack([ox+(comp[:,1]+.5)*r,oy+(comp[:,0]+.5)*r]);dist=np.linalg.norm(xy-[x,y],axis=1);target=xy[dist.argmax()];near=grid[max(0,i-2):i+3,max(0,j-2):j+3]
out={'robot':[x,y],'start_cell_cost':int(grid[i,j]),'connected_cells':len(comp),'max_reachable_center_distance_m':float(dist.max()),'candidate':target.tolist(),'neighborhood_costs':near.tolist(),'lethal_cells':int((grid==100).sum()),'inscribed_cells':int((grid==99).sum()),'unknown_cells':int((grid<0).sum())};Path(os.environ['ARX_PREVIEW_RUN'],'connectivity.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
