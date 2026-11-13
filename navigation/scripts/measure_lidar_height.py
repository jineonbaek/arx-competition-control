#!/usr/bin/env python3
"""Passive raw-LiDAR/IMU floor measurement, including inverted sensors.

Plane n.p+d=0 is expressed in native LiDAR coordinates. IMU acceleration
determines the upward normal; the known internal IMU/LiDAR axes are aligned.
No chassis frame, map, motion command, or guessed mounting height is used.
"""
import argparse
from pathlib import Path
import time
import json
import numpy as np


def plane_fit(points, up):
    up = np.asarray(up, dtype=float)
    up /= np.linalg.norm(up)
    points = np.asarray(points, dtype=float)
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    vertical = points @ up
    horizontal = np.linalg.norm(points-vertical[:, None]*up, axis=1)
    points = points[(horizontal > .8) & (horizontal < 30) & (vertical < -.01) & (vertical > -4)]
    if len(points) < 1000:
        raise ValueError('Too few below-sensor points for a reliable ground fit')
    _, idx = np.unique(np.floor(points/.03).astype(np.int32), axis=0, return_index=True)
    points = points[idx]
    rng = np.random.default_rng(15)
    sample = points if len(points) <= 22000 else points[rng.choice(len(points), 22000, replace=False)]
    heights = -sample @ up
    hist, edges = np.histogram(heights, np.arange(.05, 4.025, .025))
    peaks = [i for i in range(1, len(hist)-1) if hist[i] >= hist[i-1] and hist[i] >= hist[i+1] and hist[i] >= 30]
    seeds = sorted(set(np.argsort(hist)[-20:].tolist()+peaks))
    hypotheses = []
    for i in seeds:
        subset = sample[abs(heights-(edges[i]+.0125)) < .075]
        if len(subset) < 60:
            continue
        for _ in range(25):
            three = subset[rng.choice(len(subset), 3, replace=False)]
            n = np.cross(three[1]-three[0], three[2]-three[0])
            if np.linalg.norm(n) < 1e-6:
                continue
            n /= np.linalg.norm(n)
            if n @ up < 0:
                n = -n
            if n @ up < np.cos(np.radians(7)):
                continue
            d = -n @ three[0]
            if not .08 < d < 4:
                continue
            residual = abs(sample @ n+d)
            count = int((residual < .025).sum())
            hypotheses.append((count, n, d))
    if not hypotheses:
        raise ValueError('No broad near-horizontal ground plane found')
    hypotheses.sort(key=lambda x: x[0], reverse=True)
    candidates = []
    for count, n, d in hypotheses:
        if any(abs(d-c['d']) < .06 for c in candidates):
            continue
        for _ in range(4):
            mask = abs(points @ n+d) < .025
            ground = points[mask]
            center = ground.mean(axis=0)
            _, _, vt = np.linalg.svd(ground-center, full_matrices=False)
            n = vt[-1]
            if n @ up < 0:
                n = -n
            d = -n @ center
        tangent = vt[:2]
        xy = ground @ tangent.T
        span = np.percentile(xy, 95, axis=0)-np.percentile(xy, 5, axis=0)
        cells = np.unique(np.floor(xy/.25).astype(int), axis=0)
        residual = abs(points @ n+d)
        core = int((residual < .025).sum())
        shell = int(((residual >= .05) & (residual < .075)).sum())
        candidates.append({'d': float(d), 'normal': n.tolist(), 'inliers': len(ground),
                           'plane_density_contrast': core/max(shell, 1),
                           'coverage_cells_25cm': len(cells), 'support_span_90pct_m': span.tolist(),
                           'rms_m': float(np.sqrt(np.mean((ground @ n+d)**2))),
                           'tilt_from_imu_up_deg': float(np.degrees(np.arccos(np.clip(n @ up, -1, 1))))})
        if len(candidates) >= 20:
            break
    valid = [c for c in candidates if min(c['support_span_90pct_m']) > 1.5 and c['inliers'] > 300 and c['tilt_from_imu_up_deg'] < 7 and c['plane_density_contrast'] >= 3]
    if not valid:
        return {'valid': False, 'reason': 'No distinct broad ground plane: wall slices must not be interpreted as ground',
                'candidates': candidates, 'filtered_unique_points': len(points)}
    best = max(valid, key=lambda c: c['coverage_cells_25cm'])
    n = np.array(best['normal'])
    cosine = n @ up
    # Official internal offset: IMU origin expressed in native LiDAR coordinates.
    lidar_to_imu_origin = np.array([.011, .02329, -.04412])
    return dict(best, valid=True, plane_native_lidar_nx_ny_nz_d=best['normal']+[best['d']],
                lidar_origin_perpendicular_height_m=best['d'],
                lidar_origin_vertical_height_m=best['d']/cosine,
                imu_origin_vertical_height_m=(best['d']+n @ lidar_to_imu_origin)/cosine,
                filtered_unique_points=len(points), candidates=candidates,
                source_origin='native LiDAR coordinate origin, NOT mounting bottom center')


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from livox_ros_driver2.msg import CustomMsg
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--duration', type=float, default=18)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('raw_dual_lidar_height_measurement')
    start = time.monotonic()
    points = {n: [[], [], []] for n in ('arm', 'top')}
    imu = {n: [] for n in points}
    gyro = {n: [] for n in points}
    counts = {n: 0 for n in points}
    times = {n: [] for n in points}
    def cloud(name, msg):
        block = min(2, int((time.monotonic()-start)/a.duration*3))
        arr = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        if arr.size:
            points[name][block].append(arr)
        counts[name] += 1
        times[name].append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    def inertial(name, msg):
        imu[name].append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro[name].append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    subs = []
    for name in points:
        ip = '192_168_1_3' if name == 'arm' else '192_168_1_164'
        subs += [node.create_subscription(CustomMsg, '/livox/lidar_'+ip, lambda m, n=name: cloud(n, m), 30),
                 node.create_subscription(Imu, '/livox/imu_'+ip, lambda m, n=name: inertial(n, m), 500)]
    while time.monotonic()-start < a.duration:
        rclpy.spin_once(node, timeout_sec=.05)
    node.destroy_node()
    rclpy.shutdown()
    result = {'duration_s': time.monotonic()-start, 'robot_commands_sent': False, 'sources': {}}
    for name in points:
        acceleration = np.array(imu[name])
        assert len(acceleration) > 1000 and counts[name] > 50, 'Insufficient live sensor data'
        up = acceleration.mean(axis=0)
        up /= np.linalg.norm(up)
        blocks = [np.concatenate(v) for v in points[name] if v]
        all_points = np.concatenate(blocks)
        np.savez_compressed(a.output/(name+'-raw-samples.npz'), points=all_points, acceleration=acceleration, gyro=np.array(gyro[name]))
        fits = []
        for v in blocks:
            try:
                fits.append(plane_fit(v, up))
            except ValueError as error:
                fits.append({'valid': False, 'reason': str(error)})
        whole = plane_fit(all_points, up)
        heights = np.array([f['lidar_origin_vertical_height_m'] for f in fits if f.get('valid')])
        g = np.array(gyro[name])
        whole.update(imu_up_in_lidar_axes=up.tolist(), imu_acceleration_mean=acceleration.mean(axis=0).tolist(),
                     imu_acceleration_std=acceleration.std(axis=0).tolist(), gyro_mean_rad_s=g.mean(axis=0).tolist(),
                     gyro_rms_rad_s=float(np.sqrt(np.mean(np.sum(g*g, axis=1)))),
                     lidar_messages=counts[name], imu_messages=len(acceleration),
                     lidar_hz=(counts[name]-1)/(times[name][-1]-times[name][0]),
                     max_lidar_gap_s=float(np.diff(times[name]).max()),
                     three_window_heights_m=[f.get('lidar_origin_vertical_height_m') for f in fits],
                     window_height_span_m=float(np.ptp(heights)) if len(heights)>1 else None)
        if len(heights) != 3 or (len(heights)>1 and np.ptp(heights)>.02):
            whole['valid'] = False
            whole['repeatability_note'] = 'Three consistent valid ground fits are required for accepted height'
        result['sources'][name] = whole
        (a.output/(name+'-plane.json')).write_text(json.dumps(whole, indent=2))
    (a.output/'height-result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
