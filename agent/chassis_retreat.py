"""Task-owned short retreat using the existing navigation tracker and pose authority."""
import os
import json
from pathlib import Path
import sys
import uuid
import numpy as np
from navigation.pose_freshness import MOTION_MAX_AGE_NS
from .pose_wait import PoseWait,pose_unavailable
from navigation.polarbear_adapt.chassis_command import WebChassis
from navigation.polarbear_adapt.continuous_track import tracking_command, PERIOD
from navigation.polarbear_adapt.mppi_trial.stationary import StopWindow


def _retreat_stale_inputs(node):
    from follow_preview import Follow
    return [source for source in Follow.stale_inputs(node) if source not in ('map_costmap','odom_costmap')]


class _PoseReader:
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from geometry_msgs.msg import PoseArray
        from nav_msgs.msg import OccupancyGrid,Odometry
        from sensor_msgs.msg import LaserScan,PointCloud2
        from std_msgs.msg import Bool
        directory=Path(__file__).resolve().parents[1]/'navigation/polarbear_adapt'
        if str(directory) not in sys.path:sys.path.insert(0,str(directory))
        from goal_drive import Driver
        from follow_preview import Follow
        from axis_trial import body_delta
        from chassis_pose_policy import SnapshotLease
        # Reuse the driver's source freshness and invalidation rules verbatim.
        class Receiver(Node):
            motion_poses=Driver.motion_poses
            validate_motion_proof=Driver.validate_motion_proof
            pose=Driver.pose
            checked=Driver.checked
            receive_dual_origin_health=Driver.receive_dual_origin_health
            grid=Follow.grid
            sensor_message=Follow.sensor_message
            stale_inputs=_retreat_stale_inputs
            now=Follow.now
            def receive(self,message):
                self.received_pose=True
                Driver.receive_chassis_snapshot(self,message)
        self.ros=rclpy;self.context=Context()
        rclpy.init(context=self.context,domain_id=int(os.getenv('NAV_ROS_DOMAIN_ID','192')))
        self.node=Receiver('grasp_retreat_pose',context=self.context)
        self.node.pose_snapshot=None;self.node.pose_lease=SnapshotLease(max_age_ns=MOTION_MAX_AGE_NS);self.node.received_pose=False
        self.body_delta=body_delta;self.previous_pose=None;self.measured=np.zeros(3)
        config=json.loads((directory/'footprint.json').read_text());poly=np.asarray(config['polygon_m'])
        self.node.poly=poly+np.sign(poly)*config['padding_m']
        self.node.grids={};self.node.sensor={};self.node.sensor_received={}
        self.node.seq=self.node.seen=0
        self.node.target=None;self.node.watch_slam=os.getenv('ARX_DESKTOP_MODE')=='mapping-navigation'
        self.node.previous_alignment=None;self.node.require_bottom_mark=True;self.node.bottom_mark_stamp=0.
        self.node.require_dual_origin=os.getenv('ARX_MAPPING_ENGINE')=='dual-origin';self.node.dual_origin_health=None
        self.node.create_subscription(PointCloud2,'/pb/bottom/nav_mark',
            lambda m:setattr(self.node,'bottom_mark_stamp',m.header.stamp.sec+m.header.stamp.nanosec*1e-9),1)
        if self.node.require_dual_origin:
            self.node.create_subscription(Bool,'/pb/dual_origin/ready',self.node.receive_dual_origin_health,1)
        from rclpy.qos import qos_profile_sensor_data
        for frame,name in [('map','global'),('odom','local')]:
            self.node.create_subscription(OccupancyGrid,f'/{name}_costmap/costmap',
                lambda m,f=frame:self.node.grid(m,f),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for topic,kind in [('/pb/scan',LaserScan),('/localization/odometry',Odometry)]:
            self.node.create_subscription(kind,topic,lambda m,k=topic:self.node.sensor_message(k,m),qos_profile_sensor_data)
        self.node.create_subscription(PoseArray,'/pb/chassis_pose',self.node.receive,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)

    def now(self):return self.node.get_clock().now().nanoseconds*1e-9

    def read(self):
        for _ in range(8):self.executor.spin_once(timeout_sec=0.)
        if not self.node.received_pose:return None
        sample=self.node.motion_poses()
        if self.node.target is None:
            if self.node.stale_inputs():return None
            if self.node.require_bottom_mark and not self.node.bottom_mark_stamp:return None
            if self.node.require_dual_origin and self.node.dual_origin_health is None:return None
            self.node.target=sample[0].copy()
            self.node.target[:2]-=.200*np.array([np.cos(sample[0][2]),np.sin(sample[0][2])])
        self.node.checked(occupancy=False)
        if self.previous_pose is not None and sample[2]>self.previous_pose[1]:
            self.measured=self.body_delta(self.previous_pose[0],sample[1])/(sample[2]-self.previous_pose[1])
        self.previous_pose=(sample[1].copy(),sample[2])
        return sample

    def check_motion(self,command,source):
        # Extraction skips costmap collision checks; keep existing pose authority checks.
        self.node.checked(occupancy=False)
        self.node.validate_motion_proof(source)

    def close(self):
        self.executor.shutdown();self.executor.remove_node(self.node)
        self.node.destroy_node();self.context.shutdown()


class ChassisRetreatStep:
    def __init__(self,controller,token):
        self.controller=controller;self.token=token
        self.reader=None;self.transport=None;self.target=None;self.route=None
        self.previous=np.zeros(3);self.last=None;self.stop=None;self.done=False
        self.pose_wait=None;self.moved=False

    def start(self):
        # Finish ROS discovery before WebChassis timestamps its first source-stamped command.
        if getattr(self.controller,'config',{}).get('chassis_backend')=='ros':
            self.controller.chassis.session(self.token,'navigation').ensure()
        self.transport=WebChassis(source_id='navigation-'+uuid.uuid4().hex,
            submit=lambda command:self.controller.submit(command,self.token))
        self.transport.sdk_units=True;self.transport.native_sdk=True
        self.reader=_PoseReader();self.pose_wait=PoseWait(self.transport)

    def poll(self):
        try:return self._poll()
        except Exception as exc:
            if not pose_unavailable(exc):raise
            self.pose_wait.begin(self.moved);self.pose_wait.hold()
            return dict(status='backing_chassis',waiting_for_pose=True)

    def _poll(self):
        if self.done:return dict(status='chassis_stopped')
        sample=self.reader.read()
        if sample is None:
            self.pose_wait.begin(self.moved);self.pose_wait.hold()
            return dict(status='backing_chassis',waiting_for_pose=True)
        if self.pose_wait.waiting:
            if not self.pose_wait.ready(sample,self.reader.now(),allow_motion=self.stop is None):
                return dict(status='backing_chassis',waiting_for_pose=True)
            if self.stop is not None and self.pose_wait.window is not None:self.stop=self.pose_wait.window
            self.previous=np.zeros(3);self.last=self.controller.clock()-PERIOD
        _,odom,source=sample;tick=self.controller.clock()
        if self.target is None:
            self.target=odom.copy()
            self.target[:2]-=.200*np.array([np.cos(odom[2]),np.sin(odom[2])])
            self.route=np.asarray([odom.copy(),self.target.copy()]);self.last=tick-PERIOD
        dt=tick-self.last
        if dt<PERIOD:return dict(status='backing_chassis')
        self.last=tick
        start=self.route[0]
        retreat=float((start[:2]-odom[:2])@np.array([np.cos(start[2]),np.sin(start[2])]))
        if self.stop is not None:
            if self.stop.add(source,odom):
                if not .200 < retreat < .400:
                    raise ValueError(f'chassis_retreat_out_of_range distance={retreat:.3f}m')
                self.done=True;self.close()
                return dict(status='chassis_stopped')
            return dict(status='backing_chassis')
        if retreat > .200:
            self.transport.end_output()
            self.stop=StopWindow(after=self.reader.now())
            return dict(status='backing_chassis')
        command,arrived=tracking_command(self.route,odom,self.target,self.previous,dt)
        if arrived:command=self.previous
        self.reader.check_motion(command,source)
        self.transport.send(command);self.previous=command;self.moved=True
        return dict(status='backing_chassis')

    def close(self):
        if self.transport is not None:self.transport.end_output()
        if self.reader is not None:
            self.reader.close();self.reader=None

    def cancel(self):
        try:
            if self.transport is not None:self.transport.end_output()
        finally:self.close()
