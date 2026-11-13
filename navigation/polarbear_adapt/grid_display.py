"""RViz cube-list fallback for affected indexed-map GLSL drivers; no map changes."""
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import ColorRGBA
class Display(Node):
 def __init__(self):
  super().__init__('pb_grid_display');self.cache={};self.pubs={};self.subs=[]
  q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
  for key,topic in [('map','/pb/map'),('global','/global_costmap/costmap'),('local','/local_costmap/costmap'),('projection','/mapping/grid')]:
   self.pubs[key]=self.create_publisher(MarkerArray,'/pb/display/'+key,q);self.subs.append(self.create_subscription(OccupancyGrid,topic,lambda m,k=key:self.cache.update({k:m}),q))
  self.timer=self.create_timer(.2,self.tick)
 def tick(self):
  changed, self.cache = self.cache, {}
  for key,m in changed.items():
   marks=MarkerArray();groups={}
   for i,v in enumerate(m.data):
    if v<0:continue
    if key in ('global','local') and v==0:continue
    group='free' if v==0 else 'occupied' if v>=100 else 'inscribed' if v==99 else 'inflated'
    groups.setdefault(group,[]).append(i)
   colors={'free':(1.,1.,1.,1.),'occupied':(.25,0.,.3,1.),'inscribed':(.1,.9,.9,.8),'inflated':(1.,.55,.7,.6)}
   for j,group in enumerate(colors):
    marker=Marker();marker.header=copy.deepcopy(m.header);marker.header.stamp.sec=0;marker.header.stamp.nanosec=0;marker.ns=key;marker.id=j;marker.type=Marker.CUBE_LIST;marker.action=Marker.ADD;marker.pose=copy.deepcopy(m.info.origin);marker.scale.x=marker.scale.y=m.info.resolution;marker.scale.z=.005
    marker.pose.position.z+=-.04 if key in ('map','projection') else .01 if key=='global' else .02
    marker.color=ColorRGBA(r=colors[group][0],g=colors[group][1],b=colors[group][2],a=colors[group][3])
    marker.points=[Point(x=(i%m.info.width+.5)*m.info.resolution,y=(i//m.info.width+.5)*m.info.resolution,z=0.) for i in groups.get(group,[])]
    if not marker.points:marker.action=Marker.DELETE
    marks.markers.append(marker)
   self.pubs[key].publish(marks)
if __name__=='__main__':
 rclpy.init();n=Display()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
