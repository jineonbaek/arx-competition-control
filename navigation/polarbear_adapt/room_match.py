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
from localization_obstacles import MIN_MATCH_POINTS, evidence, rank_by_region_evidence, scores, selected_region


def collect():
    node = rclpy.create_node('room_match_scan_sampler')
    scans = []
    def receive(m):
        if m.header.frame_id != 'bottom_livox':
            return
        a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        r = np.asarray(m.ranges)
        ok = np.isfinite(r) & (r > .45) & (r < 8.) & (np.abs(a)<np.deg2rad(137.5))
        if np.count_nonzero(ok) >= MIN_MATCH_POINTS:
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
    occupied=img<(255*(1-cfg['occupied_thresh']))
    region=selected_region(args.map,img.shape,res,origin)
    dist=region.distances(occupied,res) if region else distance_transform_edt(~occupied)*res
    rclpy.init()
    try:
        train,test=collect()
    finally:
        rclpy.shutdown()

    report=match_scan(args,img,dist,res,origin,train,test,region)
    if region:
        report['obstacle_bounds_m']=list(region.bounds)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print(json.dumps(report,ensure_ascii=False))

def match_scan(args,img,dist,res,origin,train,test,region=None):
    def offsets(points,yaw):
        c,s=np.cos(yaw),np.sin(yaw)
        return points @ np.array([[c,s],[-s,c]]) / res

    def sample(indices):
        if region:
            return region.sample(dist,indices)
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
        stride=max(1,len(train)//120)
        coarse=train[::stride]
        best=[]
        for deg in np.arange(-180,180,5):
            off=offsets(coarse,np.deg2rad(deg))
            distances=sample(np.floor(xy[:,None,:]+off).astype(int))
            losses=scores(distances,minimum=max(8,int(np.ceil(MIN_MATCH_POINTS/stride)))) if region else np.minimum(distances,.50).mean(axis=1)
            for i in np.argsort(losses)[:10]:
                if not region or losses[i]<1.:
                    best.append((losses[i],*(origin+xy[i]*res),np.deg2rad(deg)))
        seeds=[]
        for _,x,y,yaw in sorted(best):
            p=np.array([x,y,yaw])
            if all(independent(p,b) for b in seeds):seeds.append(p)
            if len(seeds)==16:break
    def errors(p,points):
        return sample(np.floor((p[:2]-origin)/res+offsets(points,p[2])).astype(int))
    modes=[]
    for p in seeds:
        def objective(u):
            d=errors(u,train)
            return scores(d).item() if region else np.minimum(d,.50).mean()
        fit=minimize(objective,p,method='Powell',
                     bounds=list(zip(p-[.20,.20,.12],p+[.20,.20,.12])),options={'maxiter':30})
        candidate=p if region and objective(p)<objective(fit.x) else fit.x
        if region and objective(candidate)>=1.:
            continue
        metrics=evidence(errors(candidate,test),region is not None)
        if metrics:
            modes.append(dict(metrics,pose=candidate.tolist()))
    if region:
        rank_by_region_evidence(modes)
    modes.sort(key=lambda row:row['score_m'])
    source='live_pose_hint' if hint is not None else 'global_scan_match'
    if not modes:
        return dict(source=source,acceptable=False,points=[len(train),len(test)],
                    reason='截图区域内有效雷达回波不足' if region else '没有有效定位候选',candidates=[])
    best=modes[0]
    score=best['score_m'];laser=np.asarray(best['pose'])
    second=next((m['score_m'] for m in modes[1:] if independent(laser,m['pose'])),None)
    yaw=laser[2];base=laser.copy()
    base[:2]-=offset @ np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
    match_distinct=second is not None and second-score>=.035
    if hint is not None:
        match_distinct=np.linalg.norm(base[:2]-hint[:2])<=.20 and abs(base[2]-hint[2])<=.12
    stats={key:value for key,value in best.items() if key!='pose'}
    return dict(stats,source=source,laser_pose=laser.tolist(),base_pose=base.tolist(),next_distinct_score_m=second,
                points=[len(train),len(test)],acceptable=bool(best['median_m']<=.12 and best['within_10cm']>=.45 and match_distinct),
                candidates=modes[:6])

if __name__=='__main__':
    main()
