"""Closed-loop check of head-specific evidence counters used by the navigation map."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from live_static_map import OnlineMap

grid=np.zeros((100,100));xy=np.array([[2.725,.025],[1.525,.025],[2.725,.525]])
n=SimpleNamespace(grid=grid,xy=xy,origin=np.zeros(2),ref=SimpleNamespace(info=SimpleNamespace(resolution=.05)),
                  head_times=np.zeros((20,3),np.int64),head_streak=np.zeros(3,int),head_last=0,tf=Mock())
n.reset_head=lambda:OnlineMap.reset_head(n)
t=TransformStamped();t.transform.rotation.w=1.;n.tf.lookup_transform.return_value=t
pts=np.array([[0,.025,1.19],[3,.025,.015]])
def tick(i,points):
    ns=10**9+i*10**8
    h=Header(frame_id='base_link',stamp=Time(sec=ns//10**9,nanosec=ns%10**9))
    n.head=create_cloud_xyz32(h,points.astype(np.float32))
    OnlineMap.head_observations(n,ns+10**7)
for i in range(19):tick(i,pts)
assert list(n.head_streak)==[19,0,0]
tick(18,pts);assert n.head_streak[0]==19
tick(19,pts);assert list(n.head_streak)==[20,0,0]
tick(20,np.vstack([pts,[2.725,.025,.3]]));assert n.head_veto[0] and not n.head_streak.any()
tick(21,np.vstack([pts,[1.4,.025,.4]]));assert not n.head_streak.any()
tick(22,pts);assert n.head_streak[0]==1
OnlineMap.head_observations(n,5*10**9);assert not n.head_streak.any()
for i in range(50,90):
    tick(i,pts if i%2 else np.array([[0,.025,1.19],[3,1.,.015]]))
assert list(n.head_streak)==[20,0,0]
tick(90,np.vstack([pts,[2.725,.025,.3]]));assert n.head_veto[0] and not n.head_streak.any()
print('PASS: 20 unique low crossing rays, high crossing excluded, adjacent/occluded cells retained, live hit restores occupancy, stale reset')
