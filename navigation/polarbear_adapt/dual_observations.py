"""Project each lidar's gated observation into Nav2 without retaining a second obstacle map."""
import os
from collections import deque
import json
import math
from pathlib import Path
from scipy.spatial.transform import Rotation
from chassis_solid import ChassisSolid
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException
from clear_votes import TransientEvidence, SIZE
from shared_voxels import World, credible
from shared_obstacles import packet
from trail_evidence import SensorTrail, append_trail_evidence


def planar_ranges(points, origin, yaw, blind):
    delta = points[:, :2] - origin[:2]
    ranges = np.linalg.norm(delta, axis=1)
    valid = (ranges >= blind) & (ranges < 8.)
    angles = (np.arctan2(delta[valid, 1], delta[valid, 0]) - yaw + np.pi) % (2*np.pi)
    bins = np.rint(angles * 720 / (2*np.pi)).astype(int) % 720
    out = np.full(720, np.inf, dtype=np.float32)
    np.minimum.at(out, bins, ranges[valid])
    return out


class Observations(Node):
    def __init__(self, sensors_only=False):
        super().__init__('pb_nav_observations' if sensors_only else 'pb_dual_observations')
        self.sensors_only=sensors_only
        self.stvl=sensors_only and os.getenv("ARX_STVL_EVIDENCE")=="1"
        self.evidence={}
        self.live_alignment=None
        self.create_subscription(TFMessage,'/tf',self.on_alignment,50)
        self.trails = {s: SensorTrail() for s in ('bottom','head')} if self.stvl else {}
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.tf = TransformBroadcaster(self) if sensors_only else None
        self.blind = float(self.declare_parameter('blind', .35).value)
        self.filters = {s: World() for s in ('bottom', 'head')}
        self.prev = {}
        self.scan = {}
        self.mark = {}
        self.dynamic = None if sensors_only else TransientEvidence(ChassisSolid(Path(__file__).resolve().parents[1]/"models/lift2s_bottom_upright_20261105/base_interior.npz"))
        checkpoint=os.getenv('ARX_OBSTACLE_CHECKPOINT')
        if self.dynamic is not None and checkpoint:
            from obstacle_checkpoint import load_history
            count=load_history(self.dynamic,checkpoint)
            self.get_logger().info(f'Restored {count} historical voxels from this LIO session')
        self.legacy_mode=os.getenv('ARX_SOLID_LIVE_HANDOFF')=='1'
        topic='/pb/chassis_checked_decisions' if self.legacy_mode else '/pb/dynamic_decisions'
        self.decisions=self.create_publisher(PointCloud2,topic,1) if not sensors_only else None
        self.prior_pose={}
        self.protected=None if self.legacy_mode else set()
        self.handoff_start=None
        self.last_report=0
        if self.legacy_mode and not sensors_only:
            self.create_subscription(PointCloud2,'/pb/dynamic_decisions',self.legacy,1)
        self.compat_mark=(self.create_publisher(PointCloud2,'/pb/bottom/mark',2)
                          if sensors_only and (self.legacy_mode or os.getenv('ARX_EXTERNAL_GOAL_DRIVER')=='1') else None)
        self.head_pose = None
        self.head_stamp = None
        self.pending={s:deque(maxlen=12) for s in self.filters}
        for s in self.filters:
            if self.stvl:
                self.evidence[s]=self.create_publisher(PointCloud2,f'/pb/{s}/stvl_evidence',10)
            if sensors_only:
                self.scan[s] = self.create_publisher(LaserScan, f'/pb/{s}/nav_scan', 3)
                self.mark[s] = self.create_publisher(PointCloud2, f'/pb/{s}/nav_mark', 3)
            self.create_subscription(PointCloud2, f'/pb/{s}_observations',
                                     lambda m,s=s:self.receive(s,m) if self.sensors_only and not self.stvl else self.pending[s].append(m), qos_profile_sensor_data)
        if sensors_only:
            self.create_timer(.1, self.expire_head)
        if not sensors_only or self.stvl:
            self.create_timer(.01,self.drain)

    def on_alignment(self,msg):
        for t in msg.transforms:
            if t.header.frame_id=='map' and t.child_frame_id=='odom':
                self.live_alignment=(self.get_clock().now().nanoseconds,t.transform)

    def legacy(self,msg):
        if msg.header.frame_id!='odom' or msg.point_step!=12:return
        points=np.frombuffer(msg.data,'<f4').reshape(-1,3)
        occupied=set(map(tuple,np.floor(points[points[:,2]>0,:2]/SIZE[:2]).astype(int)))
        now=self.get_clock().now().nanoseconds
        if self.protected is None:
            self.protected=occupied
            self.handoff_start=now
        elif now-self.handoff_start<2_000_000_000:
            self.protected.update(occupied)
        else:
            cleared=set(map(tuple,np.floor(points[points[:,2]<0,:2]/SIZE[:2]).astype(int)))
            self.dynamic.cleared_xy.update(self.protected & cleared)
            self.protected.difference_update(cleared)

    def frames(self, s, origin, yaw, stamp):
        transforms = []
        for suffix, z in [('sensor', origin[2]), ('scan', .20)]:
            tr = TransformStamped()
            tr.header = Header(frame_id='odom', stamp=stamp)
            tr.child_frame_id = f'pb_nav_{s}_{suffix}'
            tr.transform.translation.x, tr.transform.translation.y = map(float, origin[:2])
            tr.transform.translation.z = float(z)
            tr.transform.rotation.z = math.sin(yaw/2)
            tr.transform.rotation.w = math.cos(yaw/2)
            transforms.append(tr)
        self.tf.sendTransform(transforms)

    def publish_scan(self, s, stamp, ranges):
        m = LaserScan()
        m.header = Header(frame_id=f'pb_nav_{s}_scan', stamp=stamp)
        m.angle_increment = 2*math.pi/720
        m.angle_min = -math.pi
        m.angle_max = m.angle_min + 719*m.angle_increment
        m.range_min, m.range_max = self.blind, 8.
        m.ranges = ranges.tolist()
        self.scan[s].publish(m)

    def expire_head(self):
        now = self.get_clock().now()
        if self.head_stamp is not None and (now-self.head_stamp).nanoseconds > 400000000:
            self.frames('head', *self.head_pose, now.to_msg())
            self.publish_scan('head', now.to_msg(), np.full(720,np.inf))
            self.prev.pop('head',None)

    def drain(self):
        for source, queue in self.pending.items():
            if not queue:continue
            msg=queue[0]
            stamp=Time.from_msg(msg.header.stamp)
            age=(self.get_clock().now()-stamp).nanoseconds/1e9
            if not -.025 <= age <= .65:
                queue.popleft()
                continue
            try:
                tr=self.buf.lookup_transform('odom','base_link',stamp).transform
                to_map=None
                if self.stvl:
                    try:to_map=self.buf.lookup_transform('map','odom',stamp).transform
                    except TransformException:
                        # SLAM periodically rebroadcasts its current map->odom correction
                        # at the last accepted scan stamp while stationary. Require a
                        # fresh broadcaster receipt, never an abandoned cached transform.
                        from pathlib import Path
                        import json
                        cfg=Path(__file__).resolve().parents[1]/'config/fixed_dual.json'
                        fixed=cfg.exists() and json.loads(cfg.read_text()).get('enabled')
                        if not fixed or self.live_alignment is None or self.get_clock().now().nanoseconds-self.live_alignment[0]>300000000:raise
                        to_map=self.live_alignment[1]
            except TransformException:continue
            queue.popleft()
            self.receive(source,msg,tr,to_map)

    def receive(self, s, m, tr=None,to_map=None):
        stamp = Time.from_msg(m.header.stamp)
        if m.header.frame_id!='odom' or m.point_step!=16 or not m.width:return
        if tr is None:
            if not -.02 <= (self.get_clock().now()-stamp).nanoseconds/1e9 <= .65:return
            try:tr=self.buf.lookup_transform('odom','base_link',stamp).transform
            except TransformException:return
        q=tr.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        a=np.frombuffer(m.data,'<f4').reshape(-1,4)
        if not np.isfinite(a[0]).all():return
        origin=a[0,:3].astype(float)
        a=a[np.isfinite(a).all(axis=1)]
        rays,hits,ground,extra=(a[a[:,3]==k,:3] for k in (1,2,3,4))
        if not len(rays) and not len(hits):
            return
        t=stamp.nanoseconds/1e9
        previous_t,previous=self.prev.get(s,(-1,None))
        if t<=previous_t:return
        hits=self.filters[s].floor_filter(s,t,origin,rays,hits,ground,extra)
        supported=credible(hits,previous if t-previous_t<.35 else None)
        self.prev[s]=(t,hits)
        if self.stvl:
            qmap=to_map.rotation;omap=to_map.translation
            rot=Rotation.from_quat([qmap.x,qmap.y,qmap.z,qmap.w]).as_matrix()
            delta=np.array([omap.x,omap.y,omap.z])
            self.trails[s].observe_alignment(rot, delta)
            map_origin = origin @ rot.T + delta
            map_rays = rays @ rot.T + delta
            map_hits = supported @ rot.T + delta
            owned, vacated = self.trails[s].evidence(t, s, map_origin, map_rays, map_hits)
            msg = packet(m.header.stamp, map_origin, map_rays, map_hits)
            append_trail_evidence(msg, map_hits[owned], vacated)
            msg.header.frame_id='map'
            self.evidence[s].publish(msg)
        if self.sensors_only:
            self.frames(s,origin,yaw,m.header.stamp)
            self.publish_scan(s,m.header.stamp,planar_ranges(supported,origin,yaw,self.blind))
            header=Header(frame_id='odom',stamp=m.header.stamp)
            message=create_cloud_xyz32(header,supported.astype(np.float32))
            self.mark[s].publish(message)
            if self.compat_mark is not None and s=='bottom':self.compat_mark.publish(message)
            if s=='head':
                self.head_stamp=stamp
                self.head_pose=(origin,yaw)
            return
        rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        translation=np.array([tr.translation.x,tr.translation.y,tr.translation.z])
        pose=None
        prev=self.prior_pose.get(s)
        if prev is not None:
            old_time,old_rot,old_translation=prev
            if (0<t-old_time<=.5 and np.linalg.norm(translation-old_translation)<.12
                    and Rotation.from_matrix(old_rot.T@rot).magnitude()<.12):
                pose=(rot,translation)
        self.prior_pose[s]=(t,rot,translation)
        self.dynamic.update(s,stamp.nanoseconds,origin,rays,supported,pose)
        header=Header(frame_id='odom',stamp=m.header.stamp)
        if self.protected is None:return
        decisions=self.dynamic.projection()
        if self.protected:
            xy=(np.asarray(list(self.protected),float)+.5)*SIZE[:2]
            # Sources from the existing process are 2-D. Keep its confirmed
            # occupied columns until it explicitly clears them in its own 3-D map.
            decisions=np.vstack((decisions,np.column_stack((xy,np.ones(len(xy)))))).astype(np.float32)
        self.decisions.publish(create_cloud_xyz32(header,decisions))
        if t-self.last_report>3 and os.getenv('ARX_ADAPT_RUN'):
            self.last_report=t
            report=Path(os.environ['ARX_ADAPT_RUN'])/'solid-filter.json'
            tmp=report.with_suffix('.tmp')
            tmp.write_text(json.dumps(dict(time=t,temporary_vote_rule='4_effective_or_5_at_80pct_in_1s',solid_removed=self.dynamic.solid_removed,
                                protected_legacy_xy=len(self.protected),measured_voxels=len(self.dynamic.cells))))
            tmp.replace(report)


if __name__=='__main__':
    rclpy.init()
    n=Observations(os.getenv('ARX_FAST_SENSOR_NODE')=='1')
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.try_shutdown()
