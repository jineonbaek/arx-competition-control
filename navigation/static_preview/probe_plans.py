"""Read costmap and test ComputePathToPose. Never executes paths."""
import os,time,json,math
from pathlib import Path
import numpy as np,rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from tf2_ros import Buffer,TransformListener
rclpy.init();n=Node('preview_plan_probe');b=Buffer();l=TransformListener(b,n);maps=[]
s=n.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:maps.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE));client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
def spinuntil(f,seconds=8):
 end=time.monotonic()+seconds
 while not f() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 return f()
spinuntil(lambda:bool(maps) and b.can_transform('preview_map','base_link',rclpy.time.Time()),10)
if not maps:raise RuntimeError('No costmap')
t=b.lookup_transform('preview_map','base_link',rclpy.time.Time());x=t.transform.translation.x;y=t.transform.translation.y
m=maps[-1];grid=np.array(m.data).reshape(m.info.height,m.info.width);res=m.info.resolution;ox=m.info.origin.position.x;oy=m.info.origin.position.y
ij=np.argwhere(grid==0);points=np.column_stack([ox+(ij[:,1]+.5)*res,oy+(ij[:,0]+.5)*res]);distance=np.linalg.norm(points-[x,y],axis=1);candidates=points[(distance>.8)&(distance<3.)][::max(1,len(points)//20)][:12];rows=[]
for target in list(candidates)+[[ox-5,oy-5]]:
 g=ComputePathToPose.Goal();g.goal=PoseStamped();g.goal.header.frame_id='preview_map';g.goal.header.stamp=n.get_clock().now().to_msg();g.goal.pose.position.x=float(target[0]);g.goal.pose.position.y=float(target[1]);g.goal.pose.orientation.w=1.;g.planner_id='GridBased';g.use_start=False
 f=client.send_goal_async(g)
 if not spinuntil(f.done):rows.append({'target':list(target),'timeout':'accept'});continue
 h=f.result()
 if not h.accepted:rows.append({'target':list(target),'rejected':True});continue
 f=h.get_result_async()
 if not spinuntil(f.done):h.cancel_goal_async();rows.append({'target':list(target),'timeout':'result'});continue
 result=f.result();rows.append({'target':list(target),'status':result.status,'error_code':result.result.error_code,'poses':len(result.result.path.poses)})
 if rows[-1]['poses']>0:break
# Always test outside map after any successful candidate.
g=ComputePathToPose.Goal();g.goal.header.frame_id='preview_map';g.goal.pose.position.x=ox-5.;g.goal.pose.position.y=oy-5.;g.goal.pose.orientation.w=1.;g.planner_id='GridBased';f=client.send_goal_async(g);spinuntil(f.done)
if f.done() and f.result().accepted:
 h=f.result();f=h.get_result_async();spinuntil(f.done)
 if f.done():r=f.result();rows.append({'case':'outside_map','status':r.status,'error_code':r.result.error_code,'poses':len(r.result.path.poses)})
 else:h.cancel_goal_async()
Path(os.environ['ARX_PREVIEW_RUN'],'plan-probe.json').write_text(json.dumps({'robot':[x,y],'results':rows,'commands_sent':0},indent=2));print(json.dumps(rows,indent=2));n.destroy_node();rclpy.shutdown()
