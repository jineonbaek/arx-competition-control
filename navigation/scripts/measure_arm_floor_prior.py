#!/usr/bin/env python3
"""Ground-height prior plus local surface normals rejects horizontal wall slices."""
import argparse
from pathlib import Path
import json
import time
import numpy as np
from scipy.spatial import cKDTree


def analyze(points, acceleration, height=.5):
    up = np.asarray(acceleration).mean(axis=0)
    up /= np.linalg.norm(up)
    xaxis = np.array([1., 0., 0.])
    e1 = xaxis-(xaxis@up)*up
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    B = np.column_stack([e1, e2, up])
    points = np.asarray(points)
    points = points[np.isfinite(points).all(axis=1)]
    world = points @ B
    radius = np.linalg.norm(world[:, :2], axis=1)
    keep = (radius > 1.5) & (radius < 30) & (world[:, 2] < -(height-.12)) & (world[:, 2] > -(height+.12))
    world = world[keep]
    if len(world) < 100:
        return {'valid': False, 'reason': 'Too few points compatible with approximate 0.5m height'}
    _, idx = np.unique(np.floor(world/.025).astype(np.int32), axis=0, return_index=True)
    world = world[idx]
    tree = cKDTree(world)
    distance, indices = tree.query(world, k=18, workers=2)
    neighbor = world[indices]
    delta = neighbor-neighbor.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', delta, delta)/18
    values, vectors = np.linalg.eigh(covariance)
    normals = vectors[:, :, 0]
    curvature = values[:, 0]/np.maximum(values.sum(axis=1), 1e-12)
    good = (distance[:, -1] < .30) & (abs(normals[:, 2]) > np.cos(np.radians(15))) & (curvature < .025)
    floor = world[good]
    info = {'height_prior_m': height, 'height_prior_half_width_m': .12,
            'imu_up_native': up.tolist(), 'native_lidar_x_downward_angle_deg': float(np.degrees(np.arcsin(-up[0]))),
            'mount_tilt_from_level_deg': float(np.degrees(np.arccos(np.clip(up[2],-1,1)))),
            'unique_height_compatible_points': len(world), 'horizontal_surface_points': len(floor),
            'horizontal_surface_distance_quantiles_m': np.percentile(np.linalg.norm(floor[:, :2], axis=1), [0,5,50,95,100]).tolist() if len(floor) else [],
            'theoretical_level_nearest_horizontal_m': height/np.tan(np.radians(7))}
    if len(floor) < 100:
        return dict(info, valid=False, reason='Insufficient near-horizontal surface points; no confirmed ground plane')
    A = np.column_stack([floor[:, :2], np.ones(len(floor))])
    rng = np.random.default_rng(81)
    best = np.zeros(len(floor), dtype=bool)
    for _ in range(500):
        i = rng.choice(len(floor), 3, replace=False)
        if np.linalg.cond(A[i]) > 1000:
            continue
        abc = np.linalg.solve(A[i], floor[i, 2])
        if np.linalg.norm(abc[:2]) > .09 or not height-.12 < -abc[2] < height+.12:
            continue
        mask = abs(A @ abc-floor[:, 2]) < .02
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < 100:
        return dict(info, valid=False, reason='No dominant floor plane in horizontal surface points')
    abc = np.linalg.lstsq(A[best], floor[best, 2], rcond=None)[0]
    inliers = floor[abs(A @ abc-floor[:, 2]) < .02]
    span = np.percentile(inliers[:, :2], 95, axis=0)-np.percentile(inliers[:, :2], 5, axis=0)
    cells = np.unique(np.floor(inliers[:, :2]/.2).astype(int), axis=0)
    info.update(candidate_height_m=float(-abc[2]), candidate_inliers=len(inliers),
                candidate_span_90pct_m=span.tolist(), candidate_20cm_cells=len(cells),
                candidate_rms_m=float(np.sqrt(np.mean((np.column_stack([inliers[:, :2], np.ones(len(inliers))])@abc-inliers[:,2])**2))))
    if min(span) < 1 or len(cells) < 30 or best.mean() < .4:
        return dict(info, valid=False, reason='Plane support too narrow or inconsistent; nearest ground cannot be confirmed')
    # Only well-supported local floor surfaces, not arbitrary height-band points.
    r = np.linalg.norm(inliers[:, :2], axis=1)
    j = r.argmin()
    native = inliers[j] @ B.T
    return dict(info, valid=True, lidar_origin_height_m=float(-abc[2]),
                plane_gravity_aligned_z_ax_by_c=abc.tolist(),
                nearest_confirmed_floor_horizontal_m=float(r[j]),
                nearest_confirmed_floor_slant_m=float(np.linalg.norm(inliers[j])),
                floor_distance_p05_m=float(np.percentile(r,5)),
                nearest_confirmed_floor_point_native_m=native.tolist())


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--prior', type=float, default=.5)
    p.add_argument('--duration', type=float, default=20)
    p.add_argument('--npz', type=Path)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    if a.npz:
        data = np.load(a.npz)
        result = analyze(data['points'], data['acceleration'], a.prior)
    else:
        import rclpy
        from rclpy.node import Node
        from livox_ros_driver2.msg import CustomMsg
        from sensor_msgs.msg import Imu
        rclpy.init()
        node = Node('arm_ground_prior_measurement')
        points, acceleration, stamps = [], [], []
        def cloud(m):
            points.append(np.array([[p.x,p.y,p.z] for p in m.points], dtype=np.float32))
            stamps.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
        def imu(m):
            acceleration.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
        subs = [node.create_subscription(CustomMsg,'/livox/lidar_192_168_1_3',cloud,30),
                node.create_subscription(Imu,'/livox/imu_192_168_1_3',imu,500)]
        begin = time.monotonic()
        while time.monotonic()-begin < a.duration:
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.shutdown()
        if len(points)<50 or len(acceleration)<1000:
            raise RuntimeError('Insufficient live sensor data')
        pieces = [np.concatenate([points[i] for i in indices])
                  for indices in np.array_split(np.arange(len(points)),3)]
        all_points = np.concatenate(points)
        acceleration = np.array(acceleration)
        np.savez_compressed(a.output/'arm-raw-samples.npz',points=all_points,acceleration=acceleration)
        result = analyze(all_points,acceleration,a.prior)
        result['windows'] = [analyze(v,acceleration,a.prior) for v in pieces]
        result['lidar_hz'] = (len(stamps)-1)/(stamps[-1]-stamps[0])
        result['max_lidar_gap_s'] = float(np.diff(stamps).max())
        if result['valid'] and not all(r['valid'] for r in result['windows']):
            result.update(valid=False,repeatability_note='Some windows failed independent ground confirmation')
        heights = [r['lidar_origin_height_m'] for r in result['windows'] if r.get('valid')]
        if len(heights) == 3:
            result['three_window_height_span_m'] = float(np.ptp(heights))
            if np.ptp(heights) > .02:
                result.update(valid=False,repeatability_note='Window heights differ by more than 2 cm')
    result['robot_commands_sent'] = False
    (a.output/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
