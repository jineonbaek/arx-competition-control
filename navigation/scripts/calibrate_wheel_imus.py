#!/usr/bin/env python3
"""Static gyro bias calibration; no control or sensor configuration writes."""
from pathlib import Path
import argparse
import datetime
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def main():
    p=argparse.ArgumentParser();p.add_argument('--confirm-static',action='store_true');p.add_argument('--duration',type=float,default=22);a=p.parse_args()
    if not a.confirm_static:raise SystemExit('Operator must confirm chassis, lift and sensors stationary on level ground')
    root=Path(__file__).resolve().parents[1];folder=root/'config/wheel_imu_fusion';folder.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=Node('dual_imu_static_calibration');acc={k:[] for k in ['arm','top']};gyro={k:[] for k in acc};stamps={k:[] for k in acc}
    def cb(k,m):
        acc[k].append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]);gyro[k].append([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]);stamps[k].append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    subs=[n.create_subscription(Imu,f'/dual/{k}/imu',lambda m,k=k:cb(k,m),1000) for k in acc];end=time.monotonic()+a.duration
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
    result={'operator_confirmed_static':True,'created_unix':time.time(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'native_acceleration_unit':'g','scope':'planar body-Z angular velocity, rigid mounts, initial level chassis','sensors':{}}
    for k in acc:
        assert len(acc[k])>a.duration*120,'Insufficient live '+k+' IMU data'
        A,G,T=np.array(acc[k]),np.array(gyro[k]),np.array(stamps[k]);up=A.mean(axis=0);norm=float(np.linalg.norm(up));up/=norm;C=np.cov(G.T);delta=np.diff(T)
        valid=bool(.8<norm<1.2 and np.isfinite([*A.ravel(),*G.ravel()]).all() and (delta>0).all() and delta.max()<.05 and abs(G.mean(axis=0)@up)<.2)
        assert valid,'Invalid stationary '+k+' data'
        result['sensors'][k]={'samples':len(A),'up_axis_native':up.tolist(),'gyro_bias_native_rad_s':G.mean(axis=0).tolist(),'gyro_covariance_native':C.tolist(),'acceleration_norm_g':norm,'acceleration_std_g':A.std(axis=0).tolist(),'rate_hz':(len(T)-1)/(T[-1]-T[0]),'max_stamp_gap_s':float(delta.max()),'z_gyro_variance_rad2_s2':float(up@C@up),'valid':valid}
    f=folder/('imu-static-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.json');f.write_text(json.dumps(result,indent=2));(folder/'latest-imu-calibration.txt').write_text(str(f)+'\n');print(f);print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
