"""Deskew each lidar independently with the joint LIO trajectory.

Keep source optical origins for ground processing and Nav2 ray clearing. Never
use merged LIO cloud_body as a single-origin obstacle scan.
"""
import os,json,copy
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs_py.point_cloud2 import create_cloud
from sensor_msgs.msg import PointCloud2,PointField
from livox_ros_driver2.msg import CustomMsg
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer,TransformListener,TransformException
from geometry import transforms,compensate
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rear_sector import rear_forward_points

class Observations(Node):
    def __init__(self):
        super().__init__('fixed_dual_deskew_'+os.getenv('ARX_DESKEW_SOURCE','both'))
        self.buf=Buffer();self.listener=TransformListener(self.buf,self)
        self.sensor=dict(zip(('bottom','head'),transforms()))
        selected=os.getenv('ARX_DESKEW_SOURCE')
        if selected:self.sensor={selected:self.sensor[selected]}
        self.pending={s:deque(maxlen=12) for s in self.sensor};self.pub={};self.count={};self.drops={};self.subs=[]
        self.run=Path(os.environ['ARX_SINGLE_RUN'])
        for s in self.sensor:
            # Raw measured returns retain intercepted self points for occlusion checks downstream.
            topic='/single/lidar_raw' if s=='bottom' else '/unused_head/lidar'
            self.pub[s]=self.create_publisher(PointCloud2,f'/fixed_dual/{s}/cloud_body',3)
            self.subs.append(self.create_subscription(CustomMsg,topic,lambda m,s=s:self.pending[s].append(m),qos_profile_sensor_data))
        self.create_timer(.01,self.tick)
    def pose(self,t):
        msg=self.buf.lookup_transform('odom','base_link',Time(nanoseconds=int(t)))
        a=msg.transform
        p=a.translation;q=a.rotation;out=np.eye(4)
        out[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();out[:3,3]=[p.x,p.y,p.z]
        return out
    def tick(self):
        now=self.get_clock().now().nanoseconds
        for s,queue in self.pending.items():
            if not queue:continue
            m=queue[0];start=int(m.timebase)
            if not -25_000_000<=now-start<=800_000_000:
                queue.popleft();self.drops[s]=self.drops.get(s,0)+1;continue
            pts=[p for p in m.points[::2] if (p.tag&0x30) in (0,0x10)]
            if not pts:queue.popleft();continue
            times=np.array([int(p.offset_time) for p in pts],np.int64)+start
            end=int(times.max());xyz=np.array([(p.x,p.y,p.z) for p in pts],np.float64)
            valid=np.isfinite(xyz).all(1)
            if s=='head':valid &= ~rear_forward_points(xyz)
            xyz=xyz[valid];times=times[valid]
            if not len(xyz):queue.popleft();continue
            intensity=np.array([p.reflectivity for p in pts],float)[valid]
            try:
                latest=self.buf.lookup_transform('odom','base_link',Time())
                supported=latest.header.stamp.sec*10**9+latest.header.stamp.nanosec
                reference=min(end,supported)
                if end-reference>2_000_000 or reference<=start:
                    raise TransformException('Trajectory does not yet support scan end within existing 2ms hold')
                # Query only supported times. The existing terminal <=2ms pose
                # hold remains explicit; acquisition timestamps are not changed.
                keys=np.unique(np.r_[start,(times//5_000_000)*5_000_000,reference])
                keys=keys[(keys>=start)&(keys<=reference)]
                poses=np.array([self.pose(t) for t in keys])
            except TransformException as exc:
                if not hasattr(self,'last_error') or now-self.last_error>2_000_000_000:
                    self.get_logger().warn(str(exc));self.last_error=now
                continue
            if len(keys)<2:queue.popleft();continue
            from scipy.spatial.transform import Slerp
            relative=(keys-start)*1e-9;sample=(np.minimum(times,reference)-start)*1e-9
            per=np.tile(np.eye(4),(len(times),1,1))
            per[:,:3,:3]=Slerp(relative,Rotation.from_matrix(poses[:,:3,:3]))(sample).as_matrix()
            per[:,:3,3]=np.column_stack([np.interp(sample,relative,poses[:,k,3]) for k in range(3)])
            out=compensate(xyz,per,poses[-1],self.sensor[s]);queue.popleft()
            header=Header(stamp=Time(nanoseconds=reference).to_msg(),frame_id='bottom_livox' if s=='bottom' else 'rear_livox')
            fields=[PointField(name=k,offset=4*j,datatype=PointField.FLOAT32,count=1) for j,k in enumerate(('x','y','z','intensity'))]
            self.pub[s].publish(create_cloud(header,fields,np.c_[out,intensity].astype(np.float32)))
            self.count[s]=self.count.get(s,0)+1
            if self.count[s]%10==0:(self.run/f'fixed-deskew-{s}.json').write_text(json.dumps(dict(frames=self.count[s],stale_drops=self.drops.get(s,0),stamp_ns=reference,age_ms=(now-reference)/1e6,points=len(out),measurement_start_ns=start,measurement_end_ns=end,reference_stamp_ns=reference,terminal_hold_ns=end-reference,terminal_held_points=int((times>reference).sum()))))

if __name__=='__main__':
    rclpy.init();n=Observations()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.try_shutdown()
