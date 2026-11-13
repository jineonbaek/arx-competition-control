"""Transform display point clouds into map at their acquisition stamps."""
from array import array
from collections import deque
from functools import partial
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
import yaml
from tf2_ros import Buffer, TransformException, TransformListener
from base_envelope import BaseEnvelope

STREAMS = {
    "/fixed_dual/head/cloud_body": "/pb/rviz/rear_cloud_map",
    "/single/cloud_registered": "/pb/rviz/lio_cloud_map",
}

def filter_display_cloud(cloud,body_transform,envelope):
    if envelope is None:return True
    xyz=coordinate_views(cloud,cloud.data,('x','y','z'))
    if not xyz:return False
    q,t=body_transform.rotation,body_transform.translation
    points=np.stack(xyz,axis=-1).reshape(-1,3)
    body=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points)+[t.x,t.y,t.z]
    keep=~envelope.mask(body)
    records=np.ndarray((cloud.height,cloud.width,cloud.point_step),dtype=np.uint8,
        buffer=cloud.data,strides=(cloud.row_step,cloud.point_step,1)).reshape(-1,cloud.point_step)
    cloud.data=array('B',records[keep].copy().tobytes())
    cloud.height=1;cloud.width=int(keep.sum());cloud.row_step=cloud.width*cloud.point_step
    return True


def coordinate_views(cloud, data, names):
    fields = {field.name: field for field in cloud.fields}
    if any(name not in fields or fields[name].datatype != PointField.FLOAT32
           or fields[name].count != 1 or fields[name].offset + 4 > cloud.point_step
           for name in names):
        return ()
    dtype = ">f4" if cloud.is_bigendian else "<f4"
    return tuple(np.ndarray(
        (cloud.height, cloud.width), dtype=dtype, buffer=data, offset=fields[name].offset,
        strides=(cloud.row_step, cloud.point_step)
    ) for name in names)


def project(cloud, transform):
    if (not cloud.width or not cloud.height or not cloud.point_step
            or cloud.row_step < cloud.width * cloud.point_step
            or len(cloud.data) != cloud.row_step * cloud.height):
        return False
    data = array("B", cloud.data)
    xyz = coordinate_views(cloud, data, ("x", "y", "z"))
    if not xyz:
        return False

    q, t = transform.rotation, transform.translation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    points = np.stack(xyz, axis=-1).reshape(-1, 3)
    points = rotation.apply(points) + [t.x, t.y, t.z]
    for index, field in enumerate(xyz):
        field[:] = points[:, index].reshape(field.shape)

    normals = coordinate_views(cloud, data, ("normal_x", "normal_y", "normal_z"))
    if normals:
        vectors = rotation.apply(np.stack(normals, axis=-1).reshape(-1, 3))
        for index, field in enumerate(normals):
            field[:] = vectors[:, index].reshape(field.shape)
    cloud.data = data
    cloud.header.frame_id = "map"
    return True


def footprint_polygons(config):
    grid = config["global_costmap"]["global_costmap"]["ros__parameters"]
    vertices = np.asarray(json.loads(grid["footprint"]), dtype=float)
    if vertices.ndim != 2 or vertices.shape[0] < 3 or vertices.shape[1] != 2:
        raise ValueError("Expected a polygon in the global costmap footprint")
    padding = float(grid["footprint_padding"])
    return vertices, vertices + np.sign(vertices) * padding


class CloudDisplay(Node):
    def __init__(self):
        super().__init__("pb_cloud_display")
        self.envelope=BaseEnvelope.load()
        self.transforms = Buffer()
        self.listener = TransformListener(self.transforms, self)
        self.pending = {source: deque(maxlen=10) for source in STREAMS}
        self.last_stamp = {source: -1 for source in STREAMS}
        self.output = {
            source: self.create_publisher(PointCloud2, target, qos_profile_sensor_data)
            for source, target in STREAMS.items()
        }
        self.cloud_subscriptions = [
            self.create_subscription(PointCloud2, source, partial(self.receive, source), qos_profile_sensor_data)
            for source in STREAMS
        ]
        self.create_timer(0.02, self.publish_ready)
        self.configure_footprints()
        self.create_timer(0.5, self.publish_footprints)

    def configure_footprints(self):
        config = yaml.safe_load(Path(__file__).with_name("nav2.yaml").read_text())
        topics = ("/pb/rviz/footprint", "/pb/rviz/padded_footprint")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.footprints = []
        for topic, points in zip(topics, footprint_polygons(config)):
            publisher = self.create_publisher(PolygonStamped, topic, qos)
            polygon = PolygonStamped()
            polygon.header.frame_id = "base_link"
            polygon.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in points]
            self.footprints.append((publisher, polygon))

    def publish_footprints(self):
        try:
            pose = self.transforms.lookup_transform("map", "base_link", Time())
        except TransformException:
            return
        for publisher, polygon in self.footprints:
            polygon.header.stamp = pose.header.stamp
            publisher.publish(polygon)

    def receive(self, source, cloud):
        self.pending[source].append((time.monotonic(), cloud))

    def publish_ready(self):
        now = time.monotonic()
        for source, queue in self.pending.items():
            while queue:
                arrived, cloud = queue[0]
                stamp = Time.from_msg(cloud.header.stamp)
                if stamp.nanoseconds <= self.last_stamp[source] or now - arrived > 0.5:
                    queue.popleft()
                    continue
                try:
                    transform = self.transforms.lookup_transform("map", cloud.header.frame_id, stamp).transform
                    body_transform=(self.transforms.lookup_transform('base_link',cloud.header.frame_id,stamp).transform
                                    if self.envelope is not None else None)
                except TransformException:
                    break
                queue.popleft()
                filtered=filter_display_cloud(cloud,body_transform,self.envelope)
                if filtered and not cloud.width:
                    cloud.header.frame_id='map'
                if filtered and (not cloud.width or project(cloud, transform)):
                    self.output[source].publish(cloud)
                    self.last_stamp[source] = stamp.nanoseconds
                else:
                    self.get_logger().warn(f"Unexpected point layout on {source}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = CloudDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
