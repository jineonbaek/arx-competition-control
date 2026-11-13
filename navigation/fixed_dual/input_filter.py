"""Per-device self filter; preserve native point coordinates and exact timestamps."""
import json,sys,time,os
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg
from geometry import transforms,configuration
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'dual_flex'));sys.path.insert(0,str(ROOT))
from mesh_filter import MeshFilter
from rear_sector import rear_points
sys.path.insert(0,str(ROOT/'polarbear_adapt'))
from base_envelope import BaseEnvelope

class Filter(Node):
    def __init__(self):
        super().__init__('fixed_dual_input_filter')
        cfg=configuration(ROOT)
        self.mesh=MeshFilter.load(ROOT/cfg['mesh'])
        self.envelope=BaseEnvelope.load()
        self.T=dict(zip(('bottom','head'),transforms()))
        self.pub={};self.last={};self.count={};self.subs=[]
        self.run=Path(os.environ['ARX_SINGLE_RUN'])
        for source,topic in [('bottom','/single/lidar_raw'),('head','/unused_head/lidar')]:
            self.pub[source]=self.create_publisher(CustomMsg,f'/fixed_dual/{source}/filtered',10)
            self.subs.append(self.create_subscription(CustomMsg,topic,lambda m,s=source:self.receive(s,m),qos_profile_sensor_data))
    def receive(self,source,m):
        begin=time.monotonic();stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if stamp<=self.last.get(source,0):
            self.get_logger().error('Nonmonotonic device clock: '+source);return
        self.last[source]=stamp
        xyz=np.asarray([(p.x,p.y,p.z) for p in m.points],np.float32).reshape(-1,3)
        mask=self.mesh.mask(xyz,self.T[source],self.mesh.nominal_head,.02)
        if self.envelope is not None:
            transform=self.T[source]
            mask|=self.envelope.mask(xyz@transform[:3,:3].T+transform[:3,3])
        mask|=~np.isfinite(xyz).all(1)
        if source=='bottom':mask|=rear_points(xyz)
        m.points=[p for p,drop in zip(m.points,mask) if not drop and (p.tag&0x30) in (0,0x10)]
        m.point_num=len(m.points);m.header.frame_id='bottom_livox' if source=='bottom' else 'rear_livox'
        self.pub[source].publish(m)
        count=self.count.get(source,0)+1;self.count[source]=count
        if count%10==0:
            (self.run/f'fixed-filter-{source}.json').write_text(json.dumps(dict(frames=count,input=len(xyz),kept=m.point_num,callback_ms=1000*(time.monotonic()-begin),stamp_ns=stamp)))

if __name__=='__main__':
    rclpy.init();n=Filter()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.try_shutdown()
