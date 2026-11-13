"""Build the reference map using the physical origins of both lidars."""
from array import array
from collections import Counter, deque
import json
import math
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from pathlib import Path
import os
import yaml
from PIL import Image
from nav2_msgs.srv import SaveMap
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster

def planar(transform):
    t, q = transform.translation, transform.rotation
    return np.array([t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])

def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])

SOURCES = {
    'bottom': ('/fixed_dual/bottom/mapping_scan', 'bottom_livox'),
    'head': ('/fixed_dual/head/mapping_scan', 'rear_livox'),
}
RESOLUTION = .05
MAX_CELLS = 4_000_000


def project(scan, transform):
    t, q = transform.translation, transform.rotation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    ranges = np.asarray(scan.ranges)
    keep = (ranges > max(.35, scan.range_min)) & (ranges < min(8., scan.range_max))
    angles = scan.angle_min + np.flatnonzero(keep) * scan.angle_increment + yaw
    origin = np.array([t.x, t.y])
    points = origin + ranges[keep, None] * np.column_stack((np.cos(angles), np.sin(angles)))
    return origin, points


class Grid:
    def __init__(self):
        self.low = np.zeros(2, dtype=np.int32)
        self.passes = np.zeros((0, 0), dtype=np.uint32)
        self.hits = np.zeros_like(self.passes)

    def insert(self, origin, points):
        cells = np.floor(points / RESOLUTION).astype(np.int32)
        start = np.floor(origin / RESOLUTION).astype(np.int32)
        low = np.minimum(cells.min(axis=0), start) - 4
        high = np.maximum(cells.max(axis=0), start) + 5
        if self.passes.size:
            low = np.minimum(self.low, low)
            high = np.maximum(self.low + self.passes.shape[::-1], high)
        w, h = high - low
        if int(w) * int(h) > MAX_CELLS:
            return False
        if self.passes.shape != (h, w):
            x, y = self.low - low
            old_h, old_w = self.passes.shape
            for name in ('passes', 'hits'):
                grown = np.zeros((h, w), dtype=np.uint32)
                grown[y:y+old_h, x:x+old_w] = getattr(self, name)
                setattr(self, name, grown)
            self.low = low
        start = tuple(start - self.low)
        ends = np.unique(cells - self.low, axis=0)
        mask = np.zeros_like(self.passes, dtype=np.uint8)
        for x, y in ends:
            cv2.line(mask, start, (int(x), int(y)), 1)
        self.passes += mask
        self.hits[ends[:, 1], ends[:, 0]] += 1
        return True

    def occupancy(self):
        data = np.full(self.passes.shape, -1, dtype=np.int8)
        known = self.passes > 2
        data[known] = 0
        data[known & (self.hits > .3 * self.passes)] = 100
        return data

class DualOriginMapping(Node):
    def __init__(self):
        super().__init__('pb_dual_origin_mapping')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/pb/reference_map', qos)
        self.health_pub = self.create_publisher(Bool, '/pb/dual_origin/ready', 1)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.create_service(SaveMap, '/pb/dual_origin/save_map', self.save)
        self.alignment = None
        self.resume_pose = None
        self.tf = Buffer(cache_time=Duration(seconds=10), node=self)
        self.listener = TransformListener(self.tf, self)
        self.grid = Grid()
        if checkpoint := os.getenv('ARX_DUAL_ORIGIN_RESUME'):
            with np.load(checkpoint, allow_pickle=False) as data:
                low, passes, hits = (data[key] for key in ('low', 'passes', 'hits'))
            if (low.shape != (2,) or low.dtype != np.int32 or passes.ndim != 2
                    or not 0 < passes.size <= MAX_CELLS or hits.shape != passes.shape
                    or passes.dtype != np.uint32 or hits.dtype != np.uint32
                    or np.any(hits > passes)):
                raise ValueError('Invalid dual-origin map checkpoint')
            self.grid.low, self.grid.passes, self.grid.hits = low, passes, hits
            seed = json.loads(Path(os.environ['ARX_MAP_SEED']).read_text())
            self.resume_pose = np.asarray(seed['base_pose'], dtype=float)
            if not seed['acceptable'] or self.resume_pose.shape != (3,) or not np.isfinite(self.resume_pose).all():
                raise ValueError('Invalid dual-origin map alignment')

        self.queue = {s: deque(maxlen=14) for s in SOURCES}
        self.stats = {s: Counter() for s in SOURCES}
        self.lag = {s: deque(maxlen=2400) for s in SOURCES}
        self.cost = deque(maxlen=2400)
        self.latest = {}
        self.last_clock = None
        self.dirty = False
        self.saved_grid = None
        self.subs = [
            self.create_subscription(LaserScan, topic, lambda scan, s=s: self.receive(s, scan),
                                     qos_profile_sensor_data)
            for s, (topic, _) in SOURCES.items()
        ]
        self.create_timer(.025, self.process)
        self.create_timer(.5, self.publish)
        self.create_timer(.2, self.publish_health)
        self.create_timer(5., self.report)

    def initialize_alignment(self):
        if self.alignment is not None:
            return True
        try:
            body = self.tf.lookup_transform('odom', 'base_link', Time())
        except TransformException:
            return False
        age = (self.get_clock().now()-Time.from_msg(body.header.stamp)).nanoseconds
        if not -100_000_000 <= age <= 700_000_000:
            return False
        odom_pose = planar(body.transform)
        yaw = self.resume_pose[2]-odom_pose[2] if self.resume_pose is not None else 0.
        shift = self.resume_pose[:2]-rotation(yaw)@odom_pose[:2] if self.resume_pose is not None else np.zeros(2)
        self.alignment = (rotation(yaw), shift)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x, transform.transform.translation.y = map(float, shift)
        transform.transform.rotation.z = math.sin(yaw/2)
        transform.transform.rotation.w = math.cos(yaw/2)
        self.broadcaster.sendTransform(transform)
        return True

    def sensors_fresh(self):
        now = self.get_clock().now().nanoseconds
        return all(source in self.latest and 0 <= now-self.latest[source] <= 1_600_000_000 for source in SOURCES)

    def publish_health(self):
        self.health_pub.publish(Bool(data=self.saved_grid is not None and self.sensors_fresh()))

    def save(self, request, response):
        if self.saved_grid is None:
            self.get_logger().warning('Save rejected: there is no completed map yet')
            return response
        prefix = Path(request.map_url).resolve()
        maps = Path(__file__).resolve().parent.parent/'maps'
        if prefix.name != 'room' or prefix.parent.parent != maps or not (prefix.parent/'INCOMPLETE').is_file():
            self.get_logger().error('Save rejected: invalid map reservation')
            return response
        try:
            low, passes, hits, data = self.saved_grid
            pixels = np.select([data == 0, data == 100], [254, 0], default=205).astype(np.uint8)
            Image.fromarray(pixels[::-1]).save(prefix.with_suffix('.pgm'))
            origin = [float(v*RESOLUTION) for v in low] + [0.]
            config = dict(image='room.pgm', mode='trinary', resolution=RESOLUTION, origin=origin,
                          negate=0, occupied_thresh=.65, free_thresh=.196)
            prefix.with_suffix('.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
            np.savez_compressed(prefix.with_suffix('.dual-origin.npz'), low=low, passes=passes, hits=hits)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Map save failed: {exc}')
            return response
        response.result = True
        return response

    def receive(self, source, scan):
        self.stats[source]['received'] += 1
        if scan.header.frame_id != SOURCES[source][1]:
            self.stats[source]['frame_reject'] += 1
            return
        queue = self.queue[source]
        if len(queue) == queue.maxlen:
            self.stats[source]['overflow'] += 1
        queue.append((time.monotonic(), scan))

    def process(self):
        now = self.get_clock().now()
        if not self.initialize_alignment():
            return
        if self.last_clock is not None and now.nanoseconds < self.last_clock:
            raise RuntimeError('ROS clock moved backwards during mapping')
        self.last_clock = now.nanoseconds
        for source, queue in self.queue.items():
            stats = self.stats[source]
            while queue:
                arrived, scan = queue[0]
                stamp = Time.from_msg(scan.header.stamp)
                age = (now.nanoseconds - stamp.nanoseconds) * 1e-9
                if time.monotonic() - arrived > .8 or age > 1.2:
                    queue.popleft()
                    stats['tf_or_age_drop'] += 1
                    continue
                if stamp.nanoseconds < self.latest.get(source, -500_000_000) + 500_000_000:
                    queue.popleft()
                    stats['sampled_out'] += 1
                    continue
                try:
                    tf = self.tf.lookup_transform('odom', scan.header.frame_id, stamp)
                except TransformException:
                    break
                queue.popleft()
                begin = time.monotonic()
                origin, points = project(scan, tf.transform)
                if not len(points) or not np.isfinite(points).all() or not np.isfinite(origin).all():
                    stats['empty_or_invalid'] += 1
                    continue
                rot, shift = self.alignment
                if self.resume_pose is not None:
                    origin, points = origin @ rot.T + shift, points @ rot.T + shift
                if not self.grid.insert(origin, points):
                    stats['extent_reject'] += 1
                    continue
                self.latest[source] = stamp.nanoseconds
                stats['inserted'] += 1
                self.lag[source].append(age * 1000)
                self.cost.append((time.monotonic() - begin) * 1000)
                self.dirty = True

    def publish(self):
        if not self.dirty or not self.sensors_fresh():
            return
        self.dirty = False
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        if self.latest:
            grid.header.stamp = Time(nanoseconds=max(self.latest.values())).to_msg()
        grid.info.resolution = RESOLUTION
        data = self.grid.occupancy()
        if not np.any(data == 0) or not np.any(data == 100):
            return
        self.saved_grid = (self.grid.low.copy(), self.grid.passes.copy(), self.grid.hits.copy(), data)
        grid.info.height, grid.info.width = data.shape
        grid.info.origin.position.x, grid.info.origin.position.y = map(float, self.grid.low * RESOLUTION)
        grid.info.origin.orientation.w = 1.
        grid.data = array('b', data.tobytes())
        self.map_pub.publish(grid)

    def report(self):
        def percentiles(samples):
            return np.round(np.percentile(samples, [50, 95, 99]), 2).tolist() if samples else []
        self.get_logger().info(json.dumps({
            'counts': self.stats,
            'scan_to_insert_ms_p50_p95_p99': {s: percentiles(l) for s, l in self.lag.items()},
            'compute_ms_p50_p95_p99': percentiles(self.cost),
            'size': list(self.grid.passes.shape),
        }))


def main():
    rclpy.init()
    node = DualOriginMapping()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
