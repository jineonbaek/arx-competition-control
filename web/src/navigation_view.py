"""Read-only ROS map and TF projection for the operator display."""
import copy
import math
import threading
import time
import json
from pathlib import Path

STATE_FILE = Path(__file__).resolve().parents[1] / "hardware/runtime/navigation-view.json"

def read_snapshot():
    try:
        data = json.loads(STATE_FILE.read_text())
        elapsed = max(0, time.monotonic() - data.pop("updated"))
        if elapsed > 3:
            data["error"] = "地图观察服务数据已过期"
        if data.get("pose"):
            data["pose"]["age_s"] += elapsed
            data["pose"]["valid"] = data["pose"]["valid"] and elapsed <= 3 and data["pose"]["age_s"] <= 2
        if data.get("path"):
            data["path"]["age_s"] += elapsed
        return data
    except (OSError, ValueError, KeyError, TypeError):
        return dict(map=None, pose=None, path=None, error="等待导航地图观察服务")



class NavigationView:
    def __init__(self):
        self.lock = threading.Lock()
        self.grid = None
        self.path = None
        self.pose = None
        self.error = '等待地图与定位'
        self.buffer = None
        self.last_grid_update = 0.0

    def attach(self, node):
        from nav_msgs.msg import OccupancyGrid, Path
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from tf2_ros import Buffer, TransformListener
        self.node = node
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node, spin_thread=False)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.map_subscription = node.create_subscription(OccupancyGrid, '/pb/map', self.on_map, qos)
        self.path_subscription = node.create_subscription(Path, '/pb/path', self.on_path, 1)
        self.timer = node.create_timer(0.25, self.update_pose)

    @staticmethod
    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_map(self, msg):
        import numpy as np
        width, height = msg.info.width, msg.info.height
        if not width or not height or len(msg.data) != width * height or msg.info.resolution <= 0:
            return
        # Coarse display cells preserve any occupied cell in each block.
        stride = max(1, math.ceil(max(width, height) / 480))
        if time.monotonic() - self.last_grid_update < 1.0:
            return
        self.last_grid_update = time.monotonic()
        rows, columns = math.ceil(height / stride), math.ceil(width / stride)
        padded = np.full((rows * stride, columns * stride), -1, dtype=np.int16)
        padded[:height, :width] = np.asarray(msg.data).reshape(height, width)
        blocks = padded.reshape(rows, stride, columns, stride)
        maximum = blocks.max(axis=(1, 3))
        minimum = blocks.min(axis=(1, 3))
        cells = np.where(maximum > 0, maximum, minimum).ravel().tolist()
        origin = msg.info.origin
        grid = dict(frame=msg.header.frame_id, width=math.ceil(width / stride),
                    height=math.ceil(height / stride), resolution=msg.info.resolution * stride,
                    origin=[origin.position.x, origin.position.y, self.yaw(origin.orientation)], cells=cells)
        with self.lock:
            self.grid = grid

    def on_path(self, msg):
        points = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        with self.lock:
            self.path = dict(frame=msg.header.frame_id, points=points, received=time.monotonic())

    def update_pose(self):
        from rclpy.time import Time
        try:
            with self.lock:
                frame = self.grid['frame'] if self.grid else 'map'
            transform = self.buffer.lookup_transform(frame, 'base_link', Time())
            stamp = transform.header.stamp
            age = (self.node.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9)
            t = transform.transform
            pose = dict(x=t.translation.x, y=t.translation.y, yaw=self.yaw(t.rotation),
                        frame=frame, source_age=age, received=time.monotonic())
            if not all(math.isfinite(pose[k]) for k in ('x', 'y', 'yaw', 'source_age')):
                raise ValueError('Invalid TF')
            with self.lock:
                self.pose, self.error = pose, ''
        except Exception:
            with self.lock:
                self.error = '定位 TF 暂不可用'

    def snapshot(self):
        with self.lock:
            result = copy.deepcopy(dict(map=self.grid, path=self.path, pose=self.pose, error=self.error))
        now = time.monotonic()
        pose = result['pose']
        if pose:
            pose['age_s'] = max(0, pose['source_age']) + now - pose.pop('received')
            pose['valid'] = not result['error'] and -0.5 <= pose['source_age'] and pose['age_s'] <= 2
        if result['path']:
            result['path']['age_s'] = now - result['path'].pop('received')
        return result


def main():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('arx_web_map_observer')
    view = NavigationView()
    view.attach(node)
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    def publish():
        data = view.snapshot()
        if data['map']:
            cells = data['map'].pop('cells')
            runs = []
            for value in cells:
                if runs and runs[-2] == value:
                    runs[-1] += 1
                else:
                    runs.extend([value, 1])
            data['map']['runs'] = runs
        data['updated'] = time.monotonic()
        tmp = STATE_FILE.with_suffix('.tmp')
        tmp.write_text(json.dumps(data, allow_nan=False))
        tmp.replace(STATE_FILE)
    node.create_timer(0.5, publish)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
