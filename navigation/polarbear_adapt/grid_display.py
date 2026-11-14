"""Original cube-list colors/material, with cached objects for changed cells."""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import ColorRGBA
from grid_visuals import cell_labels
COLORS={1:(1.,1.,1.,1.),2:(.25,0.,.3,1.),3:(.1,.9,.9,.8),4:(1.,.55,.7,.6)}
class Display(Node):
 def __init__(self):
  super().__init__('pb_grid_display');self.cache={};self.previous={};self.pixels={};self.pubs={};self.subs=[]
  q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
  for key,topic in [('map','/pb/map'),('global','/global_costmap/costmap'),('local','/local_costmap/costmap'),('projection','/mapping/grid')]:
   self.pubs[key]=self.create_publisher(MarkerArray,'/pb/display/'+key,q);self.subs.append(self.create_subscription(OccupancyGrid,topic,lambda m,k=key:self.cache.update({k:m}),q))
  self.timer=self.create_timer(.2,self.tick)
 def tick(self):
  changed,self.cache=self.cache,{}
  for key,m in changed.items():
   try:labels=cell_labels(m.data,m.info.width,m.info.height,key in ('global','local'))
   except ValueError as exc:self.get_logger().warn(str(exc));continue
   p,q=m.info.origin.position,m.info.origin.orientation
   signature=(m.header.frame_id,m.info.width,m.info.height,m.info.resolution,p.x,p.y,p.z,q.x,q.y,q.z,q.w)
   previous=self.previous.get(key)
   if previous is not None and previous[0]==signature and np.array_equal(previous[1],labels):continue
   same_grid=previous is not None and previous[0][1:4]==signature[1:4]
   if not same_grid:
    self.pixels[key]={g:{} for g in COLORS};indices=np.flatnonzero(labels)
   else:indices=np.flatnonzero(labels.ravel()!=previous[1].ravel())
   pixels=self.pixels[key];flat=labels.ravel()
   for index in indices:
    index=int(index)
    if same_grid:
     old=int(previous[1].flat[index])
     if old:pixels[old].pop(index,None)
    group=int(flat[index])
    if group:pixels[group][index]=Point(x=(index%m.info.width+.5)*m.info.resolution,y=(index//m.info.width+.5)*m.info.resolution,z=0.)
   # Always publish a complete snapshot for late/transient-local RViz subscribers.
   marks=MarkerArray()
   for group,color in COLORS.items():
    marker=Marker();marker.header=copy.deepcopy(m.header);marker.header.stamp.sec=0;marker.header.stamp.nanosec=0;marker.ns=key;marker.id=group-1;marker.type=Marker.CUBE_LIST;marker.action=Marker.ADD;marker.pose=copy.deepcopy(m.info.origin)
    marker.scale.x=marker.scale.y=m.info.resolution;marker.scale.z=.005
    marker.pose.position.z+=-.04 if key in ('map','projection') else .01 if key=='global' else .02
    marker.color=ColorRGBA(r=color[0],g=color[1],b=color[2],a=color[3]);marker.points=list(pixels[group].values())
    if not marker.points:marker.action=Marker.DELETE
    marks.markers.append(marker)
   self.pubs[key].publish(marks);self.previous[key]=(signature,labels)
if __name__=='__main__':
 rclpy.init();n=Display()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
