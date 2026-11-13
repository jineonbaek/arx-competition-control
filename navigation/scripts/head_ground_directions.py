#!/usr/bin/env python3
"""Ground echo ranges: head LiDAR +X forward; gravity defines up and left."""
import argparse
from pathlib import Path
import time
import json
import numpy as np
from scipy.spatial import cKDTree
from measure_lidar_height import plane_fit


def directions(points, up, plane):
    points = np.asarray(points)
    n = np.array(plane['normal'])
    ground = points[abs(points@n+plane['d']) < .03]
    _, i = np.unique(np.floor(ground/.02).astype(int), axis=0, return_index=True)
    ground = ground[i]
    if len(ground) < 100:
        raise ValueError('Ground support too sparse')
    e1 = np.array([1.,0.,0.]) - up[0]*up
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up,e1)
    xy = ground @ np.column_stack([e1,e2])
    distance, _ = cKDTree(xy).query(xy,k=8,workers=2)
    keep = distance[:,-1] < .12
    ground,xy = ground[keep],xy[keep]
    angle = np.arctan2(xy[:,1],xy[:,0])
    radius = np.linalg.norm(xy,axis=1)
    result = {}
    for name,center in [('front',0),('left',np.pi/2),('rear',np.pi),('right',-np.pi/2)]:
        delta = (angle-center+np.pi)%(2*np.pi)-np.pi
        mask = abs(delta) <= np.radians(15)
        count = int(mask.sum())
        r = radius[mask]
        if count < 30:
            result[name] = {'valid':False,'points':count,'reason':'No dense confirmed ground echo in this sector'}
            continue
        g = ground[mask]
        j = r.argmin()
        result[name] = {'valid':True,'points':count,'min_horizontal_m':float(r[j]),
                        'p05_horizontal_m':float(np.percentile(r,5)),
                        'nearest_slant_m':float(np.linalg.norm(g[j])),
                        'nearest_native_xyz_m':g[j].tolist()}
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=20)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from livox_ros_driver2.msg import CustomMsg
    rclpy.init();node=Node('head_ground_echo_ranges');clouds=[];acc=[];stamps=[]
    def cloud(m):
        clouds.append(np.array([[v.x,v.y,v.z] for v in m.points],dtype=np.float32))
        stamps.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    def imu(m):acc.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
    subs=[node.create_subscription(CustomMsg,'/dual/top/lidar',cloud,50),node.create_subscription(Imu,'/dual/top/imu',imu,500)]
    begin=time.monotonic()
    while time.monotonic()-begin<a.duration:rclpy.spin_once(node,timeout_sec=.05)
    node.destroy_node();rclpy.shutdown()
    if len(clouds)<100 or len(acc)<1000:raise RuntimeError('Insufficient live head sensor data')
    points=np.concatenate(clouds);acc=np.array(acc);up=acc.mean(axis=0);up/=np.linalg.norm(up)
    np.savez_compressed(a.output/'head-samples.npz',points=points,acceleration=acc)
    plane=plane_fit(points,up)
    if not plane.get('valid'):raise RuntimeError('Ground plane not independently confirmed')
    ranges=directions(points,up,plane)
    windows=[directions(np.concatenate([clouds[i] for i in indices]),up,plane)
             for indices in np.array_split(np.arange(len(clouds)),3)]
    for name,v in ranges.items():
        values=[w[name]['min_horizontal_m'] for w in windows if w[name]['valid']]
        v['window_min_horizontal_m']=values
        v['window_min_span_m']=float(np.ptp(values)) if values else None
    result={'robot_commands_sent':False,'body_forward':'head native +X, confirmed by operator',
            'body_left':'gravity-up cross forward; approximately head native -Y when inverted',
            'sector_half_width_deg':15,'height_lidar_origin_m':plane['lidar_origin_vertical_height_m'],
            'ground_rms_m':plane['rms_m'],'plane_native':plane['plane_native_lidar_nx_ny_nz_d'],
            'imu_up_native':up.tolist(),'directions':ranges,'windows':windows,
            'lidar_hz':(len(stamps)-1)/(stamps[-1]-stamps[0]),'max_lidar_gap_s':float(np.diff(stamps).max()),
            'note':'Ranges measured horizontally from LiDAR ground projection, not body edge; minima within +/-15deg sectors, not exact cardinal rays. Nearby space without ground echoes is not certified obstacle-free.'}
    (a.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':main()
