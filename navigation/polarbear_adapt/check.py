"""Bounded read-only deployed-chain check. No publishers or action clients."""
import sys,time,json,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,LaserScan
from nav_msgs.msg import OccupancyGrid,Odometry
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
rclpy.init();n=Node('pb_readonly_check');buf=Buffer();listener=TransformListener(buf,n);rows={};latest={};subs=[]
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
def recv(m,k):
 rows[k].append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9);latest[k]=m
for topic,typ in [('/pb/level_cloud',PointCloud2),('/pb/obstacles',PointCloud2),('/pb/scan',LaserScan),('/pb/mapping_scan',LaserScan),('/localization/odometry',Odometry),('/pb/map',OccupancyGrid),('/global_costmap/costmap',OccupancyGrid),('/local_costmap/costmap',OccupancyGrid)]:
 rows[topic]=[];subs.append(n.create_subscription(typ,topic,lambda m,k=topic:recv(m,k),q if typ==OccupancyGrid else qos_profile_sensor_data))
end=time.monotonic()+15
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
out={'commands_sent':0,'topics':{}}
for k,ts in rows.items():
 d=[b-a for a,b in zip(ts,ts[1:])];v={'count':len(ts),'publishers':n.count_publishers(k),'hz':len(d)/(ts[-1]-ts[0]) if d and ts[-1]>ts[0] else None,'nonincreasing':sum(x<=0 for x in d)};m=latest.get(k)
 if isinstance(m,LaserScan):v.update(finite=sum(math.isfinite(x) for x in m.ranges),inf=sum(math.isinf(x) for x in m.ranges),frame=m.header.frame_id)
 if isinstance(m,OccupancyGrid):
  v.update(free=sum(x==0 for x in m.data),occupied=sum(x==100 for x in m.data),unknown=sum(x<0 for x in m.data),frame=m.header.frame_id)
  try:
   t=buf.lookup_transform(m.header.frame_id,'base_link',Time());p=t.transform.translation;x=int(math.floor((p.x-m.info.origin.position.x)/m.info.resolution));y=int(math.floor((p.y-m.info.origin.position.y)/m.info.resolution));v['robot_cell']=m.data[y*m.info.width+x] if 0<=x<m.info.width and 0<=y<m.info.height else 'outside'
  except Exception as e:v['tf_error']=str(e)
 out['topics'][k]=v
out['single_publishers_and_received']=all(v['count']>0 and v['publishers']==1 for v in out['topics'].values())
Path(sys.argv[1]).write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
