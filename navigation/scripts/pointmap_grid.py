#!/usr/bin/env python3
"""Offline PCD projection or live point-map -> OccupancyGrid, no actuator access."""
import argparse
import csv
from pathlib import Path
import time
import numpy as np
from grid_core import read_pcd, fit_floor, project, save_grid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd', type=Path)
    parser.add_argument('--trajectory', type=Path)
    parser.add_argument('--cloud-topic', default='/dual/arm/map')
    parser.add_argument('--odom-topic', default='/dual/arm/odometry')
    parser.add_argument('--map-topic', default='/mapping/grid')
    parser.add_argument('--frame', default='arm_lidar_odom')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resolution', type=float, default=0.1)
    parser.add_argument('--floor', type=float, nargs=3)
    args = parser.parse_args()
    floor = None if args.floor is None else np.array(args.floor)
    meta = {'frame_id': args.frame, 'navigation_validated': False, 'floor_source': 'explicit' if floor is not None else 'automatic'}
    if args.pcd:
        points = read_pcd(args.pcd)
        if floor is None:
            floor, extra = fit_floor(points)
            meta.update(extra)
        origins = None
        if args.trajectory:
            rows = list(csv.DictReader(args.trajectory.open()))
            origins = np.array([[float(r['x']), float(r['y'])] for r in rows])[::5]
        grid, origin = project(points, floor, args.resolution, origins)
        save_grid(args.output, grid, origin, args.resolution, dict(meta, source_pcd=str(args.pcd)))
        print(args.output.with_suffix('.json').read_text())
        return

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    rclpy.init()
    node = Node('pointmap_grid')
    pub = node.create_publisher(OccupancyGrid, args.map_topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    state = {'cloud': None, 'origins': [], 'last': 0, 'last_saved': 0}
    def odom(msg):
        p = msg.pose.pose.position
        if not state['origins'] or np.linalg.norm(np.array([p.x, p.y])-state['origins'][-1]) > 0.2:
            state['origins'].append(np.array([p.x, p.y]))
            state['origins'] = state['origins'][-10000:]
    def cloud(msg):
        if msg.header.frame_id != args.frame:
            node.get_logger().error('Wrong map frame; projection refused')
            return
        state['cloud'] = msg
    def tick():
        nonlocal floor
        msg = state['cloud']
        if msg is None:
            return
        try:
            points = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
            if floor is None:
                floor, extra = fit_floor(points)
                meta.update(extra)
                node.get_logger().info('Floor estimate: '+str(floor.tolist()))
            grid, origin = project(points, floor, args.resolution, state['origins'])
            out = OccupancyGrid()
            out.header = msg.header
            out.info.map_load_time = node.get_clock().now().to_msg()
            out.info.resolution = args.resolution
            out.info.width, out.info.height = grid.shape[1], grid.shape[0]
            out.info.origin.position.x, out.info.origin.position.y = map(float, origin)
            out.info.origin.orientation.w = 1.0
            out.data = grid.ravel().tolist()
            pub.publish(out)
            if time.monotonic()-state['last_saved'] > 10:
                save_grid(args.output, grid, origin, args.resolution, meta)
                state['last_saved'] = time.monotonic()
        except ValueError as e:
            node.get_logger().warning(str(e))
    subs = [node.create_subscription(PointCloud2, args.cloud_topic, cloud, 2), node.create_subscription(Odometry, args.odom_topic, odom, 50)]
    timer = node.create_timer(2.0, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
