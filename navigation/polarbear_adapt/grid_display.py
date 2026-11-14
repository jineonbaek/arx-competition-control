"""Exact cell-union RViz fallback. Merge geometry; never resample the costmap."""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import ColorRGBA
from grid_visuals import cell_labels,merged_rectangles,geometry
COLORS={1:(1.,1.,1.,1.),2:(.25,0.,.3,1.),3:(.1,.9,.9,.8),4:(1.,.55,.7,.6)}
class Display(Node):
 def __init__(self):
  super().__init__('pb_grid_display');self.cache={};self.previous={};self.pubs={};self.subs=[]
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
   shapes=geometry(labels,merged_rectangles(labels),m.info.resolution);marks=MarkerArray()
   for group,color in COLORS.items():
    kind,points=shapes[group];marker=Marker();marker.header=copy.deepcopy(m.header);marker.header.stamp.sec=0;marker.header.stamp.nanosec=0;marker.ns=key;marker.id=group-1;marker.action=Marker.ADD;marker.pose=copy.deepcopy(m.info.origin)
    marker.pose.position.z+=-.04 if key in ('map','projection') else .01 if key=='global' else .02
    marker.color=ColorRGBA(r=color[0],g=color[1],b=color[2],a=color[3])
    if kind=='triangles':
     marker.type=Marker.TRIANGLE_LIST;marker.scale.x=marker.scale.y=marker.scale.z=1.;marker.pose.position.z+=.0025
    else:
     marker.type=Marker.CUBE_LIST;marker.scale.x=marker.scale.y=m.info.resolution;marker.scale.z=.005
    marker.points=[Point(x=float(x),y=float(y),z=0.) for x,y in points]
    if not marker.points:marker.action=Marker.DELETE
    marks.markers.append(marker)
   self.pubs[key].publish(marks);self.previous[key]=(signature,labels)
if __name__=='__main__':
 rclpy.init();n=Display()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
