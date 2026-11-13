import os,json,time
from pathlib import Path
import numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
rclpy.init();n=Node('preview_ground_audit');rows=[]
def cb(m):
 p=read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).reshape(-1,3);d=np.linalg.norm(p[:,:2],axis=1);mask=(p[:,2]>-.28)&(p[:,2]<-.18)
 rows.append({'count':len(p),'z_min':float(p[:,2].min()),'z_q':np.quantile(p[:,2],[0,.01,.05,.5]).tolist(),'ground_band':int(mask.sum()),'ground_range_q':np.quantile(d[mask],[0,.5,1]).tolist() if mask.any() else []})
s=n.create_subscription(PointCloud2,'/preview/level_cloud',cb,qos_profile_sensor_data);a=time.monotonic()
while time.monotonic()-a<5:rclpy.spin_once(n,timeout_sec=.1)
out={'frames':len(rows),'last':rows[-1] if rows else None,'ground_band_mean':float(np.mean([r['ground_band'] for r in rows])) if rows else None};Path(os.environ['ARX_PREVIEW_RUN'],'ground-audit.json').write_text(json.dumps(out,indent=2));print(out)
