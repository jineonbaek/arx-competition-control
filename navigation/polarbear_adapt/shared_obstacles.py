"""Both sensors submit time-stamped finite rays and obstacle hits to one voxel map."""
from collections import deque
import json
import os
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header,String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from shared_voxels import World,Cell


def packet(stamp,origin,rays,hits,ground=None,extra=None):
    ground=np.empty((0,3)) if ground is None else ground
    extra=np.empty((0,3)) if extra is None else extra
    data=np.empty((1+len(rays)+len(hits)+len(ground)+len(extra),4),dtype='<f4')
    data[0]=[*origin,0];data[1:1+len(rays),:3]=rays;data[1:1+len(rays),3]=1
    start=1+len(rays)
    for kind,points in ((2,hits),(3,ground),(4,extra)):
        data[start:start+len(points),:3]=points;data[start:start+len(points),3]=kind
        start+=len(points)
    m=PointCloud2(header=Header(frame_id='odom',stamp=stamp),height=1,width=len(data),
                  point_step=16,row_step=16*len(data),is_dense=True,
                  fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                          for i,name in enumerate(['x','y','z','kind'])])
    m.data=data.tobytes();return m


class Shared(Node):
    def __init__(self):
        super().__init__('pb_shared_obstacles')
        self.world=World();self.queue=[];self.frames={'bottom':0,'head':0}
        path=os.getenv('ARX_GROUND_BOOTSTRAP')
        if path:
            snapshot=np.load(path,allow_pickle=False)
            actual=Path('reports/active-bottom-single.txt').read_text().strip()
            if str(snapshot['odom_run'])!=actual:raise ValueError('obstacle seed belongs to a different odometry run')
            for k,row in zip(snapshot['keys'],snapshot['values']):
                lo,hi,bt,ht=row
                self.world.cells[tuple(k)]=Cell(max(bt,ht),0,zmin=lo,zmax=hi,bottom_seen=bt,head_seen=ht,hits=2,last_hit=max(bt,ht))
            for k in snapshot['unknown_xy']:
                self.world.legacy[tuple(k)]=(np.ones(18,bool),np.full(18,-1.))
        self.out=self.create_publisher(PointCloud2,'/pb/shared_obstacles',1)
        for sensor in self.frames:
            self.create_subscription(PointCloud2,'/pb/'+sensor+'_observations',
                                     lambda m,s=sensor:self.receive(m,s),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(.045,self.process);self.published=0.
        self.reported=0.
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.detail=self.create_publisher(String,'/pb/obstacle_detail',1)
        self.label=self.create_publisher(Marker,'/pb/path_issue',3)
        self.grids={}
        grid_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in ('/pb/map','/local_costmap/costmap','/global_costmap/costmap'):
            self.create_subscription(OccupancyGrid,name,
                lambda grid,topic=name:self.grids.update({topic:grid}),grid_qos)
        self.pending_clicks=deque(maxlen=32)
        self.create_subscription(PointStamped,'/clicked_point',self.click,5)
        self.create_subscription(PointStamped,'/pb/inspect_point',self.click,5)
        self.create_timer(.05,self.inspect)

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def click(self,m):
        self.pending_clicks.append((m,self.now()))
        self.inspect()

    def inspect(self):
        if not self.pending_clicks:return
        m,received=self.pending_clicks[0]
        try:
            t=self.tf.lookup_transform('odom',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
        except TransformException as exc:
            if self.now()-received>4.:
                self.get_logger().warning('Point inspection TF unavailable: '+str(exc));self.pending_clicks.popleft()
            return
        self.pending_clicks.popleft()
        q=t.rotation;v=t.translation
        pos=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([m.point.x,m.point.y,m.point.z])+[v.x,v.y,v.z]
        info=self.world.inspect(pos[:2],self.now())
        info.update(frame=m.header.frame_id,clicked=[m.point.x,m.point.y,m.point.z],time=self.now())
        info['grids']={}
        for name,grid in self.grids.items():
            try:
                tr=self.tf.lookup_transform(grid.header.frame_id,'odom',Time()).transform
            except TransformException:
                info['grids'][name]=None
                continue
            def rotation(q):
                return Rotation.from_quat([q.x,q.y,q.z,q.w])
            v=tr.translation
            origin=grid.info.origin
            point=rotation(tr.rotation).apply(pos)+[v.x,v.y,v.z]
            local=rotation(origin.orientation).inv().apply(point-
                [origin.position.x,origin.position.y,origin.position.z])
            ix,iy=np.floor(local[:2]/grid.info.resolution).astype(int)
            value=None
            if 0<=ix<grid.info.width and 0<=iy<grid.info.height:
                value=int(grid.data[iy*grid.info.width+ix])
            info['grids'][name]=dict(value=value,stamp=grid.header.stamp.sec+grid.header.stamp.nanosec/1e9)
        text=json.dumps(info,allow_nan=False);self.detail.publish(String(data=text))
        folder=Path(os.environ['ARX_ADAPT_RUN'])
        (folder/'clicked-obstacle.json').write_text(text)
        with (folder/'clicked-obstacles.jsonl').open('a') as f:f.write(text+'\n')
        rows=info['voxels'];src=sorted({s for row in rows for s in row['sources']})
        levels=[z for row in rows for z in (row['height_above_floor'] or [])]
        label=Marker();label.header.frame_id='odom';label.ns='shared_cell_detail';label.id=0
        label.type=Marker.TEXT_VIEW_FACING;label.pose.orientation.w=1.;label.scale.z=.12
        label.pose.position.x=float(pos[0]);label.pose.position.y=float(pos[1]);label.pose.position.z=.60
        label.color.r=label.color.g=label.color.b=label.color.a=1.;label.lifetime.sec=15
        label.text=(' / '.join(src) if src else 'predicted' if info['prediction_only'] else 'old XY - height unknown' if info['legacy_height_unknown'] else 'no shared hit')
        if levels:label.text+=f'  {min(levels):.2f}..{max(levels):.2f} m'
        elif rows:label.text+='  floor unknown'
        if rows:label.text+=f"  {min(row['age_s'] for row in rows):.1f} s ago"
        self.label.publish(label)

    def receive(self,m,s):
        t=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        if m.header.frame_id=='odom' and m.point_step==16 and m.width and -.01<=self.now()-t<.65:
            self.queue.append((t,s,m))
            self.queue=self.queue[-24:]

    def process(self):
        now=self.now()
        freshest={}
        for t,s,m in self.queue:
            if 0<=now-t<=.60 and (s not in freshest or freshest[s][0]<t):freshest[s]=(t,m)
        self.queue=[]
        if freshest:
            # Let subscription callbacks run between expensive voxel updates. Keep
            # the newest packet for each sensor, even while the history is large.
            s=min(freshest,key=lambda name:self.world.last.get(name,-np.inf))
            t,m=freshest.pop(s)
            self.queue.extend((ts,k,msg) for k,(ts,msg) in freshest.items())
            a=np.frombuffer(m.data,'<f4').reshape(-1,4)
            if len(a) and np.isfinite(a[0]).all():
                a=a[np.isfinite(a).all(axis=1)]
                if self.world.update(s,t,a[0,:3],a[a[:,3]==1,:3],a[a[:,3]==2,:3],a[a[:,3]==3,:3],a[a[:,3]==4,:3]):self.frames[s]+=1
        if now-self.published>=.075:
            self.publish();self.published=now

    def publish(self):
        now=self.now()
        if 'bottom' not in self.world.last:return
        xy=self.world.projection(now)
        from rclpy.time import Time
        bottom_stamp=Time(nanoseconds=int(self.world.last['bottom']*1e9)).to_msg()
        points=np.column_stack((xy,np.full(len(xy),.075))).astype('<f4')
        m=PointCloud2(header=Header(frame_id='odom',stamp=bottom_stamp),height=1,width=len(xy),
                      point_step=12,row_step=12*len(xy),is_dense=True,
                      fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                              for i,k in enumerate(('x','y','z'))])
        m.data=points.tobytes();self.out.publish(m)
        if now-self.reported>2:
            data=dict(frames=self.frames,voxels=len(self.world.cells),xy=len(xy),ray_removed=self.world.ray_removed,
                      legacy_xy=len(self.world.legacy),grazing_filtered=self.world.grazing_filtered,grazing_removed=self.world.grazing_removed,floor_tiles=len(self.world.floor.planes),floor_filtered=self.world.floor_filtered,floor_removed=self.world.floor_removed,track_removed=self.world.track_removed,moving_tracks=sum(t.moving for t in self.world.tracks.values()),
                      ages={s:round(now-t,3) for s,t in self.world.last.items()})
            report=Path(os.environ['ARX_ADAPT_RUN'],'shared-obstacles.json')
            temp=report.with_suffix('.tmp');temp.write_text(json.dumps(data));temp.replace(report)
            self.reported=now


if __name__=='__main__':
    rclpy.init();n=Shared()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.try_shutdown()
