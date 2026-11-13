"""Find distinct planar laser poses on a saved map and validate on later scans."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from PIL import Image
from rclpy.qos import qos_profile_sensor_data
from scipy.ndimage import distance_transform_edt
from scipy.optimize import minimize
from sensor_msgs.msg import LaserScan


def collect():
    node = rclpy.create_node('room_match_scan_sampler')
    scans = []
    def receive(m):
        if m.header.frame_id != 'bottom_livox':
            return
        a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        r = np.asarray(m.ranges)
        ok = np.isfinite(r) & (r > .45) & (r < 8.) & (np.abs(a)<np.deg2rad(137.5))
        if np.count_nonzero(ok) >= 25:
            scans.append(np.column_stack((r[ok]*np.cos(a[ok]),r[ok]*np.sin(a[ok]))))
    node.create_subscription(LaserScan,'/pb/mapping_scan',receive,qos_profile_sensor_data)
    end = time.monotonic()+16
    while len(scans)<40 and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
    node.destroy_node()
    if len(scans)<20:
        raise ValueError('insufficient live scan coverage')
    return [np.unique(np.round(np.vstack(group)/.09),axis=0)*.09 for group in
            (scans[:len(scans)//2], scans[len(scans)//2:])]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('map',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--live-hint', type=Path)
    args=parser.parse_args()
    cfg=yaml.safe_load(args.map.read_text())
    img=np.asarray(Image.open(args.map.parent/cfg['image']))[::-1]
    res=cfg['resolution']; origin=np.array(cfg['origin'][:2])
    if abs(cfg['origin'][2])>1e-6 or cfg['negate']:
        raise ValueError('unsupported saved map orientation')
    dist=distance_transform_edt(img>=(255*(1-cfg['occupied_thresh'])))*res
    rclpy.init()
    try:
        train,test=collect()
    finally:
        rclpy.shutdown()

    def offsets(points,yaw):
        c,s=np.cos(yaw),np.sin(yaw)
        return points @ np.array([[c,s],[-s,c]]) / res

    def sample(indices):
        x,y=indices[...,0],indices[...,1]
        inside=(x>=0)&(y>=0)&(x<dist.shape[1])&(y<dist.shape[0])
        return np.where(inside,dist[y.clip(0,dist.shape[0]-1),x.clip(0,dist.shape[1]-1)],1.)
    def independent(a,b):
        return np.linalg.norm(a[:2]-b[:2])>.35 or abs(np.angle(np.exp(1j*(a[2]-b[2]))))>.20

    offset=np.array([.18533,-.000003])
    hint=None
    if args.live_hint:
        try:
            data=json.loads(args.live_hint.read_text())
            pose=np.array([data[k] for k in ('x_m','y_m','yaw_rad')])
            if data['frame']=='map' and data['child_frame']=='base_link' and np.isfinite(pose).all():
                hint=pose
        except (OSError, KeyError, TypeError, ValueError):
            pass
    if hint is not None:
        yaw=hint[2]
        laser=hint.copy()
        laser[:2]+=offset @ np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
        seeds=[laser]
    else:
        rows,cols=np.where((img>240)&(dist>.24))
        keep=(rows%3==0)&(cols%3==0)
        xy=np.column_stack((cols[keep],rows[keep]))+.5
        coarse=train[::max(1,len(train)//120)]
        best=[]
        for deg in np.arange(-180,180,5):
            off=offsets(coarse,np.deg2rad(deg))
            distances=sample(np.floor(xy[:,None,:]+off).astype(int))
            scores=np.mean(np.minimum(distances,.50),axis=1)
            for i in np.argsort(scores)[:10]:
                best.append((scores[i],*(origin+xy[i]*res),np.deg2rad(deg)))
        seeds=[]
        for _,x,y,yaw in sorted(best):
            p=np.array([x,y,yaw])
            if all(independent(p,b) for b in seeds):seeds.append(p)
            if len(seeds)==16:break
    def errors(p,points):
        return sample(np.floor((p[:2]-origin)/res+offsets(points,p[2])).astype(int))
    modes=[]
    for p in seeds:
        fit=minimize(lambda u:np.minimum(errors(u,train),.50).mean(),p,method='Powell',
                     bounds=list(zip(p-[.20,.20,.12],p+[.20,.20,.12])),options={'maxiter':30})
        d=errors(fit.x,test)
        modes.append((np.minimum(d,.50).mean(),fit.x,np.median(d),np.mean(d<=.10)))
    modes.sort(key=lambda row:row[0])
    score,laser,median,fraction=modes[0]
    second=next((float(m[0]) for m in modes[1:] if independent(laser,m[1])),None)
    yaw=laser[2];base=laser.copy()
    base[:2]-=offset @ np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
    match_distinct=second is not None and second-score>=.035
    if hint is not None:
        match_distinct=np.linalg.norm(base[:2]-hint[:2])<=.20 and abs(base[2]-hint[2])<=.12
    report=dict(source='live_pose_hint' if hint is not None else 'global_scan_match',laser_pose=laser.tolist(),base_pose=base.tolist(),median_m=float(median),
                within_10cm=float(fraction),score_m=float(score),next_distinct_score_m=second,
                points=[len(train),len(test)],acceptable=bool(median<=.12 and fraction>=.45 and match_distinct),
                candidates=[dict(score_m=float(m[0]),pose=m[1].tolist()) for m in modes[:6]])
    args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps(report))

if __name__=='__main__':
    main()
