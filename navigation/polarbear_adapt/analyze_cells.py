"""Compare robot-cell static/global evidence; only numerical summary exported."""
import sys,json,collections,math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from pathlib import Path
root=Path(sys.argv[1]);r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(root/'replay-output'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={t.name:get_message(t.type) for t in r.get_all_topics_and_types()};selected=['/localization/odometry','/tf','/pb/map','/global_costmap/costmap','/pb/scan'];r.set_filter(rosbag2_py.StorageFilter(topics=selected))
pose=None;T=None;static=None;counts=collections.Counter();scans=[];first=None;last=None
def cell(m,p):
 x=int(math.floor((p[0]-m.info.origin.position.x)/m.info.resolution));y=int(math.floor((p[1]-m.info.origin.position.y)/m.info.resolution))
 return int(m.data[y*m.info.width+x]) if 0<=x<m.info.width and 0<=y<m.info.height else -999
while r.has_next():
 topic,data,_=r.read_next();m=deserialize_message(data,types[topic])
 if topic=='/localization/odometry':
  p=m.pose.pose.position;pose=np.array([p.x,p.y,p.z])
 elif topic=='/tf':
  for t in m.transforms:
   if t.header.frame_id=='map' and t.child_frame_id=='odom':
    q=t.transform.rotation;v=t.transform.translation;T=(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),np.array([v.x,v.y,v.z]))
 elif topic=='/pb/map':
  static=m;a=np.asarray(m.data);row={'free_m2':float(np.sum(a==0)*m.info.resolution**2),'occupied_cells':int(np.sum(a==100)),'unknown_cells':int(np.sum(a<0)),'size':[m.info.width,m.info.height]};first=first or row;last=row
 elif topic=='/pb/scan':scans.append(int(np.isfinite(m.ranges).sum()))
 elif topic=='/global_costmap/costmap' and pose is not None and T is not None and static is not None:
  p=T[0]@pose+T[1];counts[(cell(static,p),cell(m,p))]+=1
out={'static_global_robot_cell_pairs':[{'static':a,'global':b,'count':n} for (a,b),n in counts.most_common()],'first_map':first,'last_map':last,'finite_beam_quantiles':np.quantile(scans,[0,.5,.95,1]).tolist() if scans else [],'commands_sent':0}
(root/'cell-analysis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
