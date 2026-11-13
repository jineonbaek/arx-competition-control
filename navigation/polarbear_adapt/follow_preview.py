"""Follow one audited RViz path. Default output is a diagnostic TwistStamped only."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

from chassis_command import WebChassis, follow_velocity
from path_audit import AuditGrid, heading, planar


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def fresh(now, stamp, limit):
    return 0 <= now - stamp.sec - stamp.nanosec * 1e-9 <= limit


class Follow(Node):
    def __init__(self, drive):
        super().__init__('pb_follow_once')
        if drive and self.get_parameter('use_sim_time').value:
            raise ValueError('实车执行需要实时时钟')
        config = json.loads(Path(__file__).with_name('footprint.json').read_text())
        poly = np.array(config['polygon_m'])
        self.poly = poly + np.sign(poly) * config['padding_m']
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.grids = {}
        self.sensor = {}
        self.sensor_received = {}
        self.path = None
        self.goal = None
        self.index = 0
        self.done = False
        self.best = float('inf')
        self.progress = self.created = time.monotonic()
        self.web = WebChassis() if drive else None
        self.attempted_move = False
        self.pub = self.create_publisher(TwistStamped, '/pb/chassis_preview', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for frame, name in [('map','global'), ('odom','local')]:
            self.create_subscription(OccupancyGrid, f'/{name}_costmap/costmap',
                                     lambda m, f=frame: self.grid(m, f), qos)
        for topic, cls in [('/pb/scan', LaserScan), ('/localization/odometry', Odometry)]:
            self.create_subscription(cls, topic, lambda m, k=topic: self.sensor_message(k, m),
                                     qos_profile_sensor_data)
        self.create_subscription(RosPath, '/pb/path', self.receive, 1)
        self.create_subscription(PoseStamped, '/pb/plan_goal', self.plan_changed, 1)
        self.create_subscription(PoseStamped, '/pb/dock_goal', self.changed, 1)
        self.create_subscription(PointStamped, '/clicked_point', self.changed, 1)
        self.create_timer(.1, self.step)

    def plan_changed(self, message):
        self.changed(message)

    def changed(self, message):
        if self.path is not None:
            self.stop('收到新目标')
        elif isinstance(message, PoseStamped) and message.header.frame_id == 'map':
            self.goal = planar(message.pose)
        else:
            self.goal = None

    def grid(self, m, frame):
        if m.header.frame_id != frame or abs(m.info.origin.orientation.w) < .999999:
            self.grids.pop(frame, None)
            return
        o = m.info.origin.position
        self.grids[frame] = (m.header.stamp, AuditGrid(np.array(m.data).reshape(m.info.height,m.info.width),
                                                      [o.x,o.y],m.info.resolution,self.poly))

    def pose(self, frame, now):
        t = self.tf.lookup_transform(frame,'base_link',Time())
        if not fresh(now,t.header.stamp,.4):
            raise ValueError('定位 TF 过期')
        v = t.transform
        return np.array([v.translation.x,v.translation.y,heading(v.rotation)])

    def receive(self, m):
        if self.done or self.goal is None:
            return
        if not m.poses:
            if self.path is not None:
                self.stop('规划器撤回路径')
            return
        if m.header.frame_id != 'map' or not fresh(self.now(), m.header.stamp, 2.):
            return
        route = np.array([planar(p.pose) for p in m.poses])
        if np.linalg.norm(route[-1,:2]-self.goal[:2]) > .08 or abs(angle(route[-1,2]-self.goal[2])) > .25:
            if self.path is not None:
                self.stop('终点与所选箭头不一致')
            return
        if not np.isfinite(route).all():
            self.stop('路径数据无效')
        elif self.path is None and self.ready():
            audit = self.grids['map'][1].along(route)
            if audit['first']:
                self.stop('全路径轮廓覆盖障碍或未知格')
            else:
                self.path = route
                self.progress = time.monotonic()
        elif self.path is not None:
            if self.grids['map'][1].along(route)['first']:
                self.stop('更新的规划路径未通过轮廓检查')
                return
            if np.linalg.norm(route[-1,:2]-self.path[-1,:2]) > .03 or abs(angle(route[-1,2]-self.path[-1,2])) > .08:
                self.stop('规划终点变化')
                return
            remaining = self.path[max(0,self.index-1):,:2]
            if any(np.min(np.linalg.norm(remaining-p[:2],axis=1)) > .15 for p in route):
                self.stop('路径走廊变化，请重新选择目标')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sensor_message(self, topic, msg):
        stamp = msg.header.stamp
        old = self.sensor.get(topic)
        if old is None or (stamp.sec, stamp.nanosec) > (old.sec, old.nanosec):
            self.sensor[topic] = stamp
            self.sensor_received[topic] = time.monotonic()

    def stale_inputs(self):
        now, received = self.now(), time.monotonic()
        stale = []
        # The filtered scan is published after deskew and ground segmentation.
        for topic, limit in (('/pb/scan', .58), ('/localization/odometry', .4)):
            stamp = self.sensor.get(topic)
            if stamp is None or not fresh(now, stamp, limit) or received-self.sensor_received[topic] > .2:
                stale.append(topic)
        for frame in ('map', 'odom'):
            if frame not in self.grids or not fresh(now, self.grids[frame][0], 1.5):
                stale.append(f'{frame}_costmap')
        return stale

    def ready(self):
        return not self.stale_inputs()

    def output(self, velocity):
        m = TwistStamped()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = map(float,velocity)
        self.pub.publish(m)
        if self.web and (any(velocity) or self.attempted_move):
            self.attempted_move = True
            self.web.send(velocity)

    def stop(self, why):
        if not self.done:
            self.done = True
            self.get_logger().info(why)
            self.output((0.,0.,0.))

    def step(self):
        if self.done:
            return
        if self.path is None:
            if time.monotonic()-self.created > 90:
                self.stop('等待新鲜路径超时')
            return
        if not self.ready():
            self.stop('雷达、里程计或代价地图过期')
            return
        try:
            poses = {f:self.pose(f,self.now()) for f in self.grids}
        except (ValueError,TransformException):
            self.stop('定位不可用')
            return
        base = poses['map']
        target = self.path[self.index]
        delta = target[:2]-base[:2]
        dist = np.linalg.norm(delta)
        dyaw = angle(target[2]-base[2])
        if dist > .35:
            self.stop('车体偏离路径')
            return
        if dist < .045 and abs(dyaw)<.09:
            self.index += 1
            if self.index == len(self.path):
                self.stop('已到达箭头终点')
            self.best = float('inf')
            self.progress = time.monotonic()
            return
        error = dist + .25*abs(dyaw)
        if error < self.best-.005:
            self.best = error
            self.progress = time.monotonic()
        if time.monotonic()-self.progress > 4:
            self.stop('底盘未取得进展')
            return
        c,s = math.cos(base[2]),math.sin(base[2])
        body = np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
        for frame, pose in poses.items():
            c,s = math.cos(pose[2]),math.sin(pose[2])
            end = pose + [c*body[0]-s*body[1],s*body[0]+c*body[1],dyaw]
            if self.grids[frame][1].along([pose,end])['first']:
                self.stop('当前行进段覆盖障碍或未知格')
                return
        if not self.ready():
            self.stop('轮廓检查期间数据过期')
            return
        self.output(follow_velocity(*body,dyaw))


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--drive',action='store_true',help='使用本机网页底盘接口执行一次路径')
    args = parser.parse_args()
    rclpy.init()
    node = Follow(args.drive)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop('跟踪结束')
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
