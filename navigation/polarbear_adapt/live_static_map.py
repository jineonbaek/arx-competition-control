"""Project positive visibility onto a navigation copy of the current reference map."""
from array import array
from pathlib import Path
import copy
import os
import math

import numpy as np
import shapely
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException

from chassis_projection import RigidProjection
from base_envelope import BaseEnvelope
from clear_votes import FrameVotes
from head_visibility import evidence


def bottom_crossings(xy, origin, rays, hits, floor_z, res):
    """Use the actual low beams; cap their length at the closest obstacle in that direction."""
    delta = rays-origin
    radius = np.linalg.norm(delta[:,:2],axis=1)
    low = (rays[:,2]>floor_z+.06)&(rays[:,2]<floor_z+.31)&(radius>.35)
    if not low.any() or not floor_z+.06 < origin[2] < floor_z+.31:
        return np.empty(0,int)
    bins=720
    def angles(v):
        return np.floor((np.arctan2(v[:,1],v[:,0])+np.pi)*bins/(2*np.pi)).astype(int)%bins
    cap=np.full(bins,np.inf)
    hd=hits-origin
    np.minimum.at(cap,angles(hd),np.linalg.norm(hd[:,:2],axis=1))
    dirs=delta[low,:2]/radius[low,None]
    ids=angles(dirs)
    lengths=np.minimum(cap[ids],radius[low])
    order=np.lexsort((-lengths,ids))
    unique,first=np.unique(ids[order],return_index=True)
    vectors=np.zeros((bins,2))
    limit=np.zeros(bins)
    vectors[unique]=dirs[order[first]]
    limit[unique]=lengths[order[first]]
    rel=xy-origin[:2]
    nearby=(angles(rel)[:,None]+np.arange(-2,3))%bins
    v=vectors[nearby]
    cross=np.abs(rel[:,0,None]*v[:,:,1]-rel[:,1,None]*v[:,:,0])
    along=(rel[:,None,:]*v).sum(axis=2)
    return np.flatnonzero(((along>.35)&(cross<.42*res)&(limit[nearby]>along+.22)).any(axis=1))


class OnlineMap(Node):
    def __init__(self):
        super().__init__('pb_live_static_map')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(OccupancyGrid,'/pb/map',qos)
        self.persistent=self.create_publisher(OccupancyGrid,'/pb/persistent_map',qos)
        path=Path(__file__).resolve().parent/'chassis_unknown'/'projection.wkt'
        self.projection=RigidProjection(shapely.from_wkt(path.read_text()))
        self.envelope=BaseEnvelope.load()
        if self.envelope is not None:self.projection=self.envelope.projection
        self.normal=None
        self.temporary=()
        self.projection_pose=None
        self.map_alignment=None
        self.hold_stamp=None
        self.awaiting_amcl=None
        self.saved_map=bool(os.getenv('ARX_SAVED_MAP'))
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.relocalizing,10)
        if self.saved_map:
            self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.localized,qos_profile_sensor_data)
        # The 10 Hz odometry can arrive with only 25–40 ms of its freshness window left.
        self.create_timer(.02,self.refresh_projection)
        self.create_subscription(OccupancyGrid,'/pb/reference_map',self.reference,qos)
        for source in ('head','bottom'):
            self.create_subscription(PointCloud2,'/pb/'+source+'_observations',
                                     lambda msg,s=source:self.observe(s,msg),qos_profile_sensor_data)
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self)
        self.grid=None
        self.checkpoint=os.getenv('ARX_OBSTACLE_CHECKPOINT')
        self.anchor=None
        self.cleared=set()
        self.windows={s:FrameVotes() for s in ('head','bottom')}
        self.aligned=-1
        self.last_pose=None

    def reference(self,m):
        if m.header.frame_id!='map' or abs(m.info.origin.orientation.w)<.99999:return
        self.ref=m
        self.grid=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width)
        self.distance=distance_transform_edt(self.grid<100)*m.info.resolution
        self.origin=np.array([m.info.origin.position.x,m.info.origin.position.y])
        res=m.info.resolution
        if self.anchor is None or abs(self.res-res)>1e-6:
            self.anchor=self.origin.copy();self.cleared.clear()
            self.windows={s:FrameVotes() for s in self.windows}
        self.res=res
        if self.checkpoint:
            from obstacle_checkpoint import restore_map_clears
            restore_map_clears(self,self.checkpoint)
            self.checkpoint=None
        self.iy,self.ix=np.where(self.grid==100)
        self.xy=self.origin+(np.c_[self.ix,self.iy]+.5)*res
        ij=np.floor((self.xy-self.anchor)/res).astype(np.int64)
        self.keys=(ij[:,0]<<32)|(ij[:,1]&0xffffffff)
        self.publish()

    def publish(self):
        self.normal=self.grid.copy()
        if self.cleared:
            selected=np.isin(self.keys,np.fromiter(self.cleared,np.int64))
            self.normal[self.iy[selected],self.ix[selected]]=0
        m=self.map_message(self.normal)
        self.persistent.publish(m)
        self.refresh_projection(force=True)

    def map_message(self, data):
        m=copy.copy(self.ref)
        m.header=copy.deepcopy(self.ref.header)
        m.header.stamp=self.get_clock().now().to_msg()
        values=array('b')
        values.frombytes(data.tobytes())
        m.data=values
        return m

    def relocalizing(self, msg):
        if msg.header.frame_id!='map':return
        ns=self.get_clock().now().nanoseconds
        self.hold_stamp=ns
        if self.saved_map:self.awaiting_amcl=ns
        self.projection_pose=None
        if self.normal is not None:
            self.temporary=()
            self.pub.publish(self.map_message(self.normal))

    def localized(self, msg):
        if (self.awaiting_amcl is not None and msg.header.frame_id=='map' and
                Time.from_msg(msg.header.stamp).nanoseconds>=self.awaiting_amcl):
            self.awaiting_amcl=None

    @staticmethod
    def planar(tf):
        t,q=tf.translation,tf.rotation
        values=np.array([t.x,t.y,q.x,q.y,q.z,q.w])
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[2:])-1)>.01:return None
        return (t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def fresh_pose(self):
        if self.awaiting_amcl is not None:return None
        try:
            body=self.tf.lookup_transform('map','base_link',Time())
            stamp=Time.from_msg(body.header.stamp)
            age=(self.get_clock().now()-stamp).nanoseconds
            if not -100_000_000<=age<=650_000_000:return None
            alignment=self.planar(self.tf.lookup_transform('map','odom',stamp).transform)
        except TransformException:return None
        pose=self.planar(body.transform)
        if alignment is None or pose is None:return None
        old=self.map_alignment
        self.map_alignment=alignment
        if old is not None:
            dx,dy,dyaw=np.subtract(alignment,old)
            if math.hypot(dx,dy)>.18 or abs(math.atan2(math.sin(dyaw),math.cos(dyaw)))>.13:
                self.hold_stamp=stamp.nanoseconds
        if self.hold_stamp is not None:
            if stamp.nanoseconds<=self.hold_stamp:return None
            self.hold_stamp=None
        return pose

    def refresh_projection(self, force=False):
        if self.normal is None:return
        pose=self.fresh_pose()
        if not force and pose==self.projection_pose:return
        self.projection_pose=pose
        select=self.projection.covered_cells if self.envelope is not None else self.projection.unknown_cells
        cells=tuple(select(self.normal,self.origin,self.res,pose)) if pose is not None else ()
        if not force and cells==self.temporary:return
        self.temporary=cells
        data=self.normal
        if cells:
            data=data.copy()
            data.flat[list(cells)]=0
        self.pub.publish(self.map_message(data))

    def observe(self,source,msg):
        if self.grid is None or msg.header.frame_id!='odom' or msg.point_step!=16 or not msg.width:return
        stamp=Time.from_msg(msg.header.stamp);ns=stamp.nanoseconds;history=self.windows[source]
        if ns<=history.last:return
        # Unknown or unusable frames never become votes for free space.
        def miss():
            history.add(ns,())
        if not -.05<=(self.get_clock().now()-stamp).nanoseconds/1e9<=.65:miss();return
        try:
            t=self.tf.lookup_transform('map','odom',stamp).transform
            body=self.tf.lookup_transform('map','base_link',stamp).transform
        except TransformException:miss();return
        q=t.rotation;rot=Rotation.from_quat([q.x,q.y,q.z,q.w])
        pose=np.array([t.translation.x,t.translation.y,rot.as_euler('xyz')[2]])
        if self.last_pose is not None:
            change=pose-self.last_pose
            if np.linalg.norm(change[:2])>.18 or abs(math.atan2(math.sin(change[2]),math.cos(change[2])))>.13:
                self.windows={s:FrameVotes() for s in self.windows};self.aligned=-1
        self.last_pose=pose
        a=np.frombuffer(msg.data,'<f4').reshape(-1,4)
        if not np.isfinite(a[0]).all():miss();return
        a=a[np.isfinite(a).all(axis=1)]
        points=rot.apply(a[:,:3])+[t.translation.x,t.translation.y,t.translation.z]
        rays=points[a[:,3]==1];hits=points[a[:,3]==2];origin=points[0]
        if source=='bottom':
            ij=np.unique(np.floor((hits[:,:2]-self.origin)/self.res).astype(int),axis=0)
            ij=ij[(ij[:,0]>=0)&(ij[:,0]<self.ref.info.width)&(ij[:,1]>=0)&(ij[:,1]<self.ref.info.height)]
            if len(ij)<25 or (self.distance[ij[:,1],ij[:,0]]<=.15).mean()<.40:
                self.aligned=-1;self.windows[source].add(ns,());return
            self.aligned=ns
            ids=bottom_crossings(self.xy,origin,rays,hits,body.translation.z,self.res)
        else:
            if not 0<=abs(ns-self.aligned)<500_000_000:miss();return
            ids,_=evidence(self.grid,self.xy,self.origin,rays,origin,body.translation.z,self.res,False)
        clear=self.windows[source].add(ns,self.keys[ids])
        if clear-self.cleared:
            self.cleared.update(clear)
            self.publish()


if __name__=='__main__':
    rclpy.init();n=OnlineMap()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.try_shutdown()
