"""Read-only RViz planning. Publishes paths and diagnostics; only calls ComputePathToPose."""
import copy
import json
import math
import os
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import PointCloud2, LaserScan
from visualization_msgs.msg import Marker
from nav2_msgs.action import ComputePathToPose
from path_audit import AuditGrid, heading, planar

ERRORS={205:'start_occupied',206:'goal_occupied',208:'no_valid_path'}

def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class Planner(Node):
    def __init__(self):
        super().__init__('pb_plan_only')
        self.run=Path(os.environ['ARX_ADAPT_RUN'])
        footprint=json.loads(Path(__file__).with_name('footprint.json').read_text())
        poly=np.asarray(footprint['polygon_m']); self.poly=poly+np.sign(poly)*footprint['padding_m']
        self.buf=Buffer(); self.listener=TransformListener(self.buf,self)
        self.client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.path_pub=self.create_publisher(NavPath,'/pb/path',1)
        self.issue_pub=self.create_publisher(Marker,'/pb/path_issue',10)
        self.grid=None; self.times={}; self.target=None; self.pending=False
        self.dock=None; self.retreat=0
        self.audit_index=0; self.seq=0; self.goal_handle=None; self.last=0.; self.sent=0.
        self.last_target_xy=None
        for cls,topic in [(Odometry,'/localization/odometry'),(PointCloud2,'/pb/level_cloud'),(LaserScan,'/pb/scan')]:
            self.create_subscription(cls,topic,lambda m,k=topic:self.times.update({k:seconds(m.header.stamp)}),qos_profile_sensor_data)
        self.require_bottom_mark=os.getenv('ROS_DOMAIN_ID')=='192'
        if self.require_bottom_mark:self.create_subscription(PointCloud2,'/pb/bottom/nav_mark',lambda m:self.times.update({'bottom_mark':seconds(m.header.stamp)}),1)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_map,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointStamped,'/clicked_point',self.point_goal,5)
        self.create_subscription(PoseStamped,'/pb/plan_goal',self.set_goal,5)
        self.create_subscription(PoseStamped,'/pb/dock_goal',self.dock_goal,5)
        self.create_subscription(PoseStamped,'/pb/drive_finished',self.finished,5)
        self.create_timer(.2,self.tick)

    def finished(self,m):
        if self.target is None or m.header.stamp!=self.target.header.stamp:return
        self.seq+=1
        if self.goal_handle:self.goal_handle.cancel_goal_async()
        self.goal_handle=None;self.target=None;self.pending=False;self.clear();self.report('execution_finished')

    def on_map(self,m):
        if m.header.frame_id=='map' and abs(m.info.origin.orientation.w)>0.999999:
            self.grid=m

    def fresh(self):
        now=self.get_clock().now().nanoseconds/1e9
        return (len(self.times)==(4 if self.require_bottom_mark else 3) and all(0<=now-t<1 for t in self.times.values()) and
                self.grid is not None and -.1<=now-seconds(self.grid.header.stamp)<2.)

    def report(self,status,**extra):
        if 'target' in extra:self.last_target_xy=extra['target'][:2]
        if self.last_target_xy is not None:
            label=Marker();label.header.frame_id='map';label.ns='plan_result';label.id=0
            label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.
            label.pose.position.x,label.pose.position.y=map(float,self.last_target_xy)
            label.pose.position.z=.35;label.scale.z=.13;label.color.a=1.;label.color.r=1.;label.color.g=.9
            label.text={'start_occupied':'Start occupied','goal_occupied':'Goal occupied',
                        'path_ready':'Path ready','candidate_with_unknown':'Path touches unknown',
                        'candidate_with_lethal':'Path touches obstacle'}.get(status,status.replace('_',' '))
            self.issue_pub.publish(label)
        data=dict(status=status,time=time.time(),commands_sent=0,**extra)
        tmp=self.run/'plan-status.tmp';tmp.write_text(json.dumps(data,indent=2));tmp.replace(self.run/'plan-status.json')
        with (self.run/'plan-events.jsonl').open('a') as out:out.write(json.dumps(data)+'\n')

    def mark(self,issue=None):
        m=Marker();m.header.frame_id='map';m.ns='first_path_issue';m.id=0;m.action=Marker.DELETE
        if issue:
            m.action=Marker.ADD;m.type=Marker.SPHERE;m.pose.orientation.w=1.
            m.pose.position.x,m.pose.position.y=issue['first_cell'];m.pose.position.z=.08
            m.scale.x=m.scale.y=m.scale.z=.12;m.color.a=1.;m.color.r=1.
            m.color.g=0. if issue['lethal'] else .7
        self.issue_pub.publish(m)

    def clear(self):
        p=NavPath();p.header.frame_id='map';self.path_pub.publish(p);self.mark()

    def point_goal(self,m):
        p=PoseStamped();p.header=m.header;p.pose.position=m.point;p.pose.orientation.w=1.
        self.set_goal(p)

    def dock_goal(self,m):
        self.set_goal(m,dock=True)

    def set_goal(self,m,dock=False):
        q=m.pose.orientation;p=m.pose.position
        if m.header.frame_id!='map' or not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all() or abs(np.linalg.norm([q.x,q.y,q.z,q.w])-1)>.002:
            self.report('invalid_goal');return
        m.pose.position.z=0.
        self.last_target_xy=[p.x,p.y]
        self.seq+=1
        if self.goal_handle:self.goal_handle.cancel_goal_async()
        self.dock=planar(m.pose) if dock else None;self.retreat=0
        self.goal_handle=None;self.pending=False;self.target=m;self.last=0.;self.clear();self.tick()

    def try_next_dock(self):
        if self.dock is None:return
        self.retreat+=1
        if self.retreat>12:
            self.report('no_verified_dock_candidate',nominal=self.dock.tolist(),search_m=.60)
            self.target=None;self.dock=None
            return
        x,y,a=self.dock
        self.target.pose.position.x=float(x-.05*self.retreat*math.cos(a))
        self.target.pose.position.y=float(y-.05*self.retreat*math.sin(a))
        self.last=0.

    def short_omni_path(self,start,goal,grid,ctx):
        if self.dock is not None:return False
        delta=goal[:2]-start[:2]
        yaw=math.atan2(math.sin(goal[2]-start[2]),math.cos(goal[2]-start[2]))
        forward=delta@[math.cos(start[2]),math.sin(start[2])]
        lateral=delta@[-math.sin(start[2]),math.cos(start[2])]
        if not ((abs(forward)<.035 and abs(lateral)<=.38 and abs(yaw)<.09)
                or np.linalg.norm(delta)<.035):
            return False
        steps=max(1,math.ceil((np.linalg.norm(delta)+grid.radius*abs(yaw))/.025))
        poses=[np.array(start)+t*np.r_[delta,yaw] for t in np.linspace(0,1,steps+1)]
        audit=grid.along(poses)
        if audit['first'] is not None:return False
        path=NavPath()
        path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        for x,y,a in poses:
            p=PoseStamped();p.header=path.header
            p.pose.position.x=float(x);p.pose.position.y=float(y)
            p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2)
            path.poses.append(p)
        self.path_pub.publish(path);self.mark()
        self.report('path_ready',source='short_omni',poses=len(poses),audit=audit,**ctx)
        return True

    def tick(self):
        if self.target is None:return
        if not self.fresh():
            self.clear()
            if self.pending:
                self.seq+=1;self.pending=False
                if self.goal_handle:self.goal_handle.cancel_goal_async()
                self.goal_handle=None
            if time.monotonic()-self.last>2:self.report('waiting_for_fresh_data');self.last=time.monotonic()
            return
        if self.pending:
            if time.monotonic()-self.sent>12:
                self.seq+=1;self.pending=False;self.clear();self.report('planner_timeout')
                if self.goal_handle:self.goal_handle.cancel_goal_async()
                self.goal_handle=None
            return
        if time.monotonic()-self.last<2:return
        self.last=time.monotonic()
        if not self.client.server_is_ready():self.report('waiting_for_planner');return
        m=self.grid
        try:
            # The grid stamp is its publication time. Nav2 plans from the latest robot pose.
            pose = self.buf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            self.report('waiting_for_robot_tf');return
        now = self.get_clock().now().nanoseconds / 1e9
        if not 0 <= now - seconds(pose.header.stamp) <= .4:
            self.report('waiting_for_fresh_robot_tf');return
        tf = pose.transform
        o=m.info.origin.position
        grid=AuditGrid(np.array(m.data).reshape(m.info.height,m.info.width),[o.x,o.y],m.info.resolution,self.poly)
        start=[tf.translation.x,tf.translation.y,heading(tf.rotation)]
        goal=planar(self.target.pose)
        ctx=dict(grid_stamp=seconds(m.header.stamp),pose_stamp=seconds(pose.header.stamp),
                 start=start,target=goal.tolist(),
                 start_cells=grid.inspect(start),goal_cells=grid.inspect(goal))
        if self.short_omni_path(start,goal,grid,ctx):return
        if self.dock is not None:
            ctx.update(nominal_dock=self.dock.tolist(),retreat_m=.05*self.retreat,grasp_reach='unverified')
            if ctx['goal_cells']['first_cell'] is not None:
                self.clear();self.mark(ctx['goal_cells']);self.report('dock_footprint_blocked',**ctx)
                self.try_next_dock();return
        self.audit_index+=1
        ctx['grid_file']=f"plan-grid-{self.audit_index%12:02d}.npz"
        ctx['grid_sequence']=self.audit_index
        np.savez_compressed(self.run/ctx['grid_file'],grid=grid.grid,origin=grid.origin,
                            resolution=grid.res,stamp=ctx['grid_stamp'],sequence=self.audit_index)
        g=ComputePathToPose.Goal();g.planner_id='GridBased';g.goal=copy.deepcopy(self.target);g.goal.header.stamp=self.get_clock().now().to_msg()
        g.start.header = pose.header
        g.start.pose.position.x, g.start.pose.position.y, g.start.pose.position.z = tf.translation.x, tf.translation.y, tf.translation.z
        g.start.pose.orientation = tf.rotation
        g.use_start = True
        seq=self.seq;self.pending=True;self.sent=time.monotonic()
        self.client.send_goal_async(g).add_done_callback(lambda f:self.accepted(f,seq,ctx,grid))

    def accepted(self,f,seq,ctx,grid):
        try:
            h=f.result()
            if seq!=self.seq:
                if h.accepted:h.cancel_goal_async()
                return
            if not h.accepted:self.pending=False;self.clear();self.report('rejected',**ctx);return
            self.goal_handle=h
            h.get_result_async().add_done_callback(lambda result:self.done(result,seq,ctx,grid))
        except Exception as e:
            if seq==self.seq:self.pending=False;self.clear();self.report('action_error',detail=str(e),**ctx)

    def done(self,f,seq,ctx,grid):
        if seq!=self.seq:return
        self.pending=False;self.goal_handle=None
        try:
            response=f.result();r=response.result
            if not self.fresh():self.clear();self.report('result_stale',**ctx);return
            if response.status!=4 or r.error_code or not r.path.poses:
                self.clear();issue=ctx['start_cells'] if r.error_code==205 else ctx['goal_cells'] if r.error_code==206 else {}
                if issue.get('first_cell'):self.mark(issue)
                self.report(ERRORS.get(r.error_code,'planner_failed'),error_code=int(r.error_code),**ctx)
                if r.error_code in (206,208):self.try_next_dock()
                return
            poses=[planar(p.pose) for p in r.path.poses];audit=grid.along(poses)
            self.path_pub.publish(r.path);self.mark(audit['first'])
            self.report('candidate_with_lethal' if audit['max_lethal'] else 'candidate_with_unknown' if audit['max_unknown'] else 'path_ready',
                        poses=len(poses),audit=audit,**ctx)
            if self.dock is not None and (audit['max_unknown'] or audit['max_lethal']):self.try_next_dock()
        except Exception as e:self.clear();self.report('audit_error',detail=str(e),**ctx)


if __name__=='__main__':
    rclpy.init();node=Planner()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
