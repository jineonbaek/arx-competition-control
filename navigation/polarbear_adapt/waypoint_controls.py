import copy
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from waypoint_queue import Route


class Waypoints:
    def __init__(self, driver):
        self.driver = driver
        self.route = Route()
        self.forwarded = set()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = driver.create_publisher(String, '/pb/waypoints/status', qos)
        self.markers = driver.create_publisher(MarkerArray, '/pb/waypoints/markers', qos)
        self.goals = driver.create_publisher(PoseStamped, '/pb/plan_goal', 5)
        driver.create_subscription(PoseStamped, '/pb/waypoints/add', self.add, 10)
        driver.create_subscription(Empty, '/pb/waypoints/start', self.start, 5)
        driver.create_subscription(Empty, '/pb/waypoints/clear', self.clear, 5)
        self.show()

    def add(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        if (msg.header.frame_id == 'map' and math.isfinite(p.x) and math.isfinite(p.y)
                and abs(math.hypot(q.w, q.z)-1) < .002 and abs(q.x)+abs(q.y) < .002):
            self.route.add(msg)
        self.show()

    def show(self):
        r = self.route
        label = {'draft':'待开始','running':'执行中','completed':'全部到达并停稳',
                 'cancelled':'已取消','manual_override':'已由单点目标接管'}.get(r.state,r.state)
        self.status.publish(String(data=f'{label}  {r.index}/{len(r.points)}'))
        markers = [Marker(action=Marker.DELETEALL)]
        for i, point in enumerate(r.points):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type = 'waypoints', i, Marker.ARROW
            m.pose = copy.deepcopy(point.pose)
            m.pose.position.z = .16
            m.scale.x, m.scale.y, m.scale.z = .28, .06, .06
            m.color.r, m.color.g, m.color.b, m.color.a = (.25 if i < r.index else 1.), .75, .1, 1.
            text = copy.deepcopy(m)
            text.ns, text.type, text.text = 'waypoint_numbers', Marker.TEXT_VIEW_FACING, str(i+1)
            text.pose.position.z, text.scale.z = .36, .20
            markers.extend([m, text])
        self.markers.publish(MarkerArray(markers=markers))

    def dispatch(self, point):
        if point is None:
            return
        msg = copy.deepcopy(point)
        msg.header.stamp = self.driver.get_clock().now().to_msg()
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.forwarded.add(stamp)
        # Begin locally before notifying the planner. A cancel racing the DDS
        # loopback cannot revive the waypoint because the echoed stamp is ignored.
        self.driver.accept_goal(msg)
        self.goals.publish(msg)

    def start(self, _):
        if self.driver.target is None:
            self.dispatch(self.route.start())
        self.show()

    def clear(self, _):
        active = self.route.running
        self.route.reset('cancelled')
        if active:
            self.driver.cancel('cancelled')
        self.show()

    def from_self(self, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if key not in self.forwarded:
            return False
        self.forwarded.remove(key)
        return True

    def finish(self, reason):
        if self.route.running:
            self.dispatch(self.route.finish(reason))
            self.show()

    def override(self):
        if self.route.running:
            self.route.reset('manual_override')
            self.show()
