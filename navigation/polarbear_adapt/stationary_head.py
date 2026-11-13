"""Publish short-lived head observations only after the parked chassis and mast settle."""
import json
import math
import os
import sys
from collections import deque
from pathlib import Path
from urllib.request import build_opener, ProxyHandler

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells,Odometry
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu,PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from std_msgs.msg import String,Bool
from std_srvs.srv import SetBool
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer,TransformListener

from chassis_filter import ChassisFilter
from shared_obstacles import packet
from shared_voxels import without_self_shadow
from stationary_scene import align,confirmed,thin
# Per-axis medians from 3,050 bottom and 3,048 head IMU samples in the
# confirmed stationary 2026-11-07 recording. Applied only to the parked gate.
PARKED_GYRO_BIAS=np.array([[.00532718,-.00374093,.01279131],
                           [-.02257931,-.00349548,.04043360]])
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'dual_flex'))
from mesh_filter import MeshFilter


def stamp(m):
    return m.header.stamp.sec+1e-9*m.header.stamp.nanosec


class Supplement(Node):
    def __init__(self):
        super().__init__('pb_stationary_head')
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.output=self.create_publisher(GridCells,'/pb/parked_obstacles',1)
        self.visual={k:self.create_publisher(Marker,"/pb/head_"+k,1) for k in ("scene","confirmed")}
        self.shared=self.create_publisher(PointCloud2,"/pb/head_observations",2)
        self.rays=self.create_publisher(PointCloud2,"/pb/parked_visibility",1)
        self.state=self.create_publisher(String,'/pb/parked_state',1)
        self.http=build_opener(ProxyHandler({}))
        self.console_ok=False;self.console_stamp=0.;self.driving=False;self.intent_time=0.
        self.imu=[deque(maxlen=160),deque(maxlen=160)]
        self.poses=deque(maxlen=22);self.clouds=[None,None];self.recent=[deque(maxlen=4),deque(maxlen=4)]
        self.pairs=deque(maxlen=4);self.anchor=None
        self.history=deque(maxlen=2);self.good_since=None;self.correction=None
        self.fit_time=0.;self.last_head=0.;self.last_mode=None;self.fit_metrics={}
        mesh=ROOT/'models/lift2s_bottom_upright_20261105/body_voxels.npz'
        self.mesh=MeshFilter.load(mesh);self.chassis=ChassisFilter(mesh)
        self.top=np.eye(4);self.top[:3,:3]=np.diag([1.,-1.,-1.])
        cad=json.loads((ROOT/'config/cad_nominal_20261105.json').read_text())
        self.top[:3,3]=cad['top']['lidar_origin_m']
        self.bottom=np.array(json.loads((ROOT/'config/bottom_upright_20261105.json').read_text())['bottom']['lidar_origin_m'])
        for j,(imu,lidar) in enumerate([('/single/imu','/single/lidar_filtered'),('/unused_head/imu','/unused_head/lidar')]):
            self.create_subscription(Imu,imu,lambda m,j=j:self.on_imu(j,m),qos_profile_sensor_data)
            self.create_subscription(CustomMsg,lidar,lambda m,j=j:self.on_cloud(j,m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/localization/odometry',self.on_pose,qos_profile_sensor_data)
        self.create_subscription(Bool,'/pb/head_motion_intent',self.drive_state,1)
        self.preference=ROOT/'config'/'head_obstacles.enabled'
        self.enabled=not self.preference.exists() or self.preference.read_text().strip()!='0'
        self.create_service(SetBool,'/pb/head_enabled',self.set_enabled)
        self.create_timer(.14,self.console)
        self.create_timer(.10,self.tick)

    def set_enabled(self,req,res):
        temp=self.preference.with_suffix('.tmp')
        try:
            temp.write_text('1' if req.data else '0')
            temp.replace(self.preference)
        except OSError as exc:
            res.success=False;res.message=str(exc);return res
        self.enabled=req.data
        if not self.enabled:self.off('disabled')
        res.success=True;res.message='parked only' if self.enabled else 'disabled'
        return res

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def drive_state(self,m):
        self.driving=m.data;self.intent_time=self.now()
        if self.driving:self.off('drive_requested')

    def console(self):
        try:
            with self.http.open('http://127.0.0.1:9983/api/state',timeout=.06) as f:s=json.load(f)
            self.console_ok=bool(s['body']['fresh']) and not s['chassis_active']
            self.console_stamp=self.now()
        except Exception:self.console_ok=False
        if not self.console_ok:self.off('console_busy_or_stale')

    def on_imu(self,j,m):
        w=m.angular_velocity;a=m.linear_acceleration
        gyro=np.array([w.x,w.y,w.z])
        residual=np.linalg.norm(gyro-PARKED_GYRO_BIAS[j])
        raw=np.linalg.norm(gyro)
        self.imu[j].append((stamp(m),residual,np.linalg.norm([a.x,a.y,a.z]),raw))
        if raw>.14 or residual>.12:self.off('gyro_motion')

    def on_cloud(self,j,m):
        self.clouds[j]=m
        self.recent[j].append(m)

    def timed_pair(self,now):
        candidates=[list(q) if q else ([m] if m is not None else []) for q,m in zip(self.recent,self.clouds)]
        for h in reversed(candidates[1]):
            if not self.last_head<stamp(h)<=now or now-stamp(h)>.32:continue
            bottom=[b for b in candidates[0] if 0<=now-stamp(b)<.32 and abs(stamp(h)-stamp(b))<=.12]
            if not bottom:continue
            try:t=self.tf.lookup_transform('odom','base_link',Time.from_msg(h.header.stamp)).transform
            except Exception:continue
            return min(bottom,key=lambda b:abs(stamp(b)-stamp(h))),h,t
        return None

    def on_pose(self,m):
        p=m.pose.pose;q=p.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.poses.append((stamp(m),p.position.x,p.position.y,yaw))
        if len(self.poses)>1 and np.hypot(self.poses[-1][1]-self.poses[-2][1],self.poses[-1][2]-self.poses[-2][2])>.008:
            self.off('odom_motion')

    def quiet(self,now):
        if (self.driving and now-self.intent_time<.5) or not self.console_ok or now-self.console_stamp>.4:return False
        for series in self.imu:
            a=np.asarray(series)
            if len(a)<30 or a[-1,0]>now+.025:return False
            a=a[(a[:,0]>now-.40)&(a[:,0]<=now)]
            if len(a)<30 or now-a[-1,0]>=.25 or not np.isfinite(a).all():return False
            if np.sqrt(np.mean(a[:,1]**2))>.050 or np.std(a[:,2])>.12 or max(a[:,3])>.14 or max(a[:,1])>.12:return False
        a=np.asarray(self.poses)
        if len(a)<9 or not 0<=now-a[-1,0]<.3:return False
        a=a[a[:,0]>now-1.2]
        return len(a)>=9 and np.linalg.norm(np.ptp(a[:,1:3],axis=0))<.010 and np.ptp(np.unwrap(a[:,3]))<.018

    def publish(self,xy,ts=None):
        m=GridCells();m.header.frame_id='odom'
        m.header.stamp=ts if ts is not None else self.get_clock().now().to_msg()
        m.cell_width=m.cell_height=.05
        m.cells=[Point(x=float(x),y=float(y),z=0.) for x,y in xy]
        self.output.publish(m)

    def draw(self,scene=None,confirmed=None,ts=None):
        for key,points in [('scene',scene),('confirmed',confirmed)]:
            m=Marker();m.header.frame_id='odom';m.header.stamp=ts or self.get_clock().now().to_msg()
            m.ns='parked_head_'+key;m.id=0;m.pose.orientation.w=1.
            if points is None or not len(points):m.action=Marker.DELETE
            else:
                m.type=Marker.POINTS;m.action=Marker.ADD;m.lifetime.nanosec=450000000
                red=key=='confirmed';m.scale.x=m.scale.y=.085 if red else .045
                m.color.a=1.;m.color.r=float(red);m.color.g=.10 if red else 1.
                m.pose.position.z=.075 if red else .045
                m.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in points]
            self.visual[key].publish(m)

    def mode(self,mode,count=0):
        if mode!=self.last_mode or self.now()-getattr(self,'last_report',0)>2:
            self.last_mode=mode;self.last_report=self.now()
            msg=dict(mode=mode,head_cells=count,time=self.last_report,**self.fit_metrics)
            self.state.publish(String(data=json.dumps(msg)))
            if os.getenv('ARX_ADAPT_RUN'):
                Path(os.environ['ARX_ADAPT_RUN'],'parked-head.json').write_text(json.dumps(msg))

    def off(self,why):
        if self.last_mode=='parked_fused':
            self.publish([]);self.draw()
            self.rays.publish(create_cloud_xyz32(Header(frame_id='base_link',stamp=self.get_clock().now().to_msg()),np.empty((0,3),np.float32)))
        self.last_head=0.;self.pairs.clear();self.anchor=None;self.history.clear();self.good_since=None;self.correction=None;self.fit_metrics={}
        self.mode(why)

    def points(self,m):
        p=np.array([(q.x,q.y,q.z) for q in m.points[::2] if (q.tag & 0x30) in (0,0x10)],float).reshape(-1,3)
        r=np.linalg.norm(p,axis=1)
        return p[np.isfinite(p).all(axis=1)&(r>.35)&(r<7.)]

    def tick(self):
        now=self.now()
        if not self.enabled:self.off('disabled');return
        if not self.quiet(now):self.off('moving_or_unconfirmed');return
        if self.good_since is None:self.good_since=now
        if now-self.good_since<1.2:self.mode('settling');return
        pair=self.timed_pair(now)
        if pair is None:
            if self.last_head and now-self.last_head>.32:self.off('unpaired_or_stale')
            elif not self.last_head:self.mode('waiting_for_timed_scan')
            return
        b,h,t=pair
        if self.last_head and not 0<stamp(h)-self.last_head<.23:
            self.off("scan_gap");return
        self.last_head=stamp(h)
        bp=self.points(b)+self.bottom
        hp=self.points(h)
        own=self.mesh.mask(hp,self.top,self.mesh.nominal_head,.055)
        blocked=hp[own]@self.top[:3,:3].T+self.top[:3,3]
        hp=hp[~own]
        hp=hp@self.top[:3,:3].T+self.top[:3,3]
        self.pairs.append((bp,hp))
        if len(self.pairs)<3:self.mode('collecting_overlap');return
        try:
            if self.correction is None or now-self.fit_time>.8:
                area=lambda p:p[(np.linalg.norm(p[:,:2],axis=1)>.85)&(p[:,2]>-.12)&(p[:,2]<2.4)]
                rot,tr,info=align(area(np.vstack([v[0] for v in self.pairs])),area(np.vstack([v[1] for v in self.pairs])))
                if self.correction is not None:
                    oldrot,oldt=self.correction
                    if np.linalg.norm(oldt-tr)>.05 or Rotation.from_matrix(oldrot.T@rot).magnitude()>.035:
                        raise ValueError('unstable_alignment')
                    rot=Rotation.from_rotvec(.3*Rotation.from_matrix(rot@oldrot.T).as_rotvec()).as_matrix()@oldrot
                    tr=.7*oldt+.3*tr
                self.correction=(rot,tr);self.fit_time=now;self.fit_metrics=info
            rot,tr=self.correction
            p=hp@rot.T+tr
            inside=self.mesh.mask(p,np.eye(4),self.mesh.nominal_head,.055)|self.chassis.contains(p)
            origin=self.top[:3,3]@rot.T+tr
            blocked=np.vstack((blocked@rot.T+tr,p[inside]))
            scene=without_self_shadow(p[~inside],blocked,origin)
            # Rear blind sector plus additional elevations/directions missed by the bottom.
            from scipy.spatial import cKDTree
            p=scene[(scene[:,2]>.065)&(scene[:,2]<1.32)]
            all_hits=p
            rear=(p[:,0]<0)&(np.abs(p[:,1])<-p[:,0]*math.tan(math.radians(42.5)))
            if len(bp):p=p[rear | (cKDTree(bp).query(p,distance_upper_bound=.09)[0]>.09)]
            q=t.rotation;v=t.translation
            body_rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            body_xyz=np.array([v.x,v.y,v.z])
            if self.anchor is None:self.anchor=(body_rot,body_xyz)
            ar,at=self.anchor
            if np.linalg.norm(body_xyz-at)>.018 or Rotation.from_matrix(ar.T@body_rot).magnitude()>.025:
                raise ValueError('parked_pose_changed')
            p=thin(p,.045,3500)@ar.T+at
            keep=confirmed(p,self.history);self.history.append(p)
            xy=(np.unique(np.floor(keep[:,:2]/.05),axis=0)+.5)*.05
            if self.now()-stamp(h)>.38:raise ValueError('processing_stale')
            self.publish(xy,h.header.stamp);self.mode('parked_fused',len(xy))
            self.draw(thin(scene,.065,1400)@ar.T+at,keep,h.header.stamp)
            # Internal XYZ packet: row 0 is the fitted optical origin in base_link;
            # remaining rows are actual measured endpoints in the same frame.
            origin=self.top[:3,3]@rot.T+tr
            self.shared.publish(packet(h.header.stamp,origin@ar.T+at,scene@ar.T+at,all_hits@ar.T+at,extra=scene[(scene[:,2]<=.065)|(scene[:,2]>=1.32)]@ar.T+at))
            self.rays.publish(create_cloud_xyz32(Header(frame_id='base_link',stamp=h.header.stamp),
                              np.vstack((origin,scene)).astype(np.float32)))
        except Exception as exc:
            self.off(str(exc)[:120])

if __name__=='__main__':
    rclpy.init();node=Supplement()
    try:rclpy.spin(node)
    finally:node.publish([]);node.destroy_node();rclpy.try_shutdown()
