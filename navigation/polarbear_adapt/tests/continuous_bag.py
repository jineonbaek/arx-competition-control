"""Read-only shadow replay of recorded Nav2 paths and recorded footprint grids."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from continuous_track import SDK_XY_MAX,SDK_YAW_MAX,SDK_XY_ACCEL,SDK_YAW_ACCEL,PERIOD, tracking_command, swept_poses, wrap
from path_audit import planar, AuditGrid

def messages(bag, topics):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        name,buf,ts=reader.read_next()
        yield ts/1e9,name,deserialize_message(buf,types[name])

def replay_path(bag):
    paths=[]; poses=[]
    for ts,name,m in messages(bag,['/amcl_pose','/pb/path']):
        if name=='/pb/path':
            paths.append((ts,np.array([planar(p.pose) for p in m.poses])))
        else:poses.append((ts,planar(m.pose.pose)))
    times=[t for t,p in poses];xyz=np.array([p for t,p in poses]);xyz[:,2]=np.unwrap(xyz[:,2])
    prev=np.zeros(3); active=mixed=three=arrived=off=0
    for i,(t,path) in enumerate(paths):
        if not len(path):prev*=0;continue
        end=min(paths[i+1][0] if i+1<len(paths) else times[-1],t+3,times[-1])
        for now in np.arange(max(t,times[0]),end,PERIOD):
            p=np.array([np.interp(now,times,axis) for axis in xyz.T])
            try:cmd,done=tracking_command(path,p,path[-1],prev,PERIOD)
            except ValueError:off+=1;prev*=0;continue
            assert np.isfinite(cmd).all() and np.linalg.norm(cmd[:2])<=SDK_XY_MAX+.00001 and abs(cmd[2])<=SDK_YAW_MAX+.00001
            if not done:
                assert np.linalg.norm(cmd[:2]-prev[:2]) <= SDK_XY_ACCEL*PERIOD+.00001
                assert abs(cmd[2]-prev[2]) <= SDK_YAW_ACCEL*PERIOD+.00001
                active+=1
                axes=np.sum(abs(cmd)>.01)
                mixed+=int(axes>=2);three+=int(axes==3)
            else:
                assert not cmd.any()
                arrived+=1
            prev=cmd
    assert active>0
    return dict(path_messages=len(paths),pose_messages=len(poses),moving_ticks=active,
                multi_axis_ticks=mixed,three_axis_ticks=three,arrival_ticks=arrived,off_path_ticks=off)

def replay_grids(bag):
    config=json.loads((Path(__file__).resolve().parents[1]/'footprint.json').read_text())
    poly=np.array(config['polygon_m']);poly+=np.sign(poly)*config['padding_m']
    counts={}
    for ts,name,m in messages(bag,['/local_costmap/costmap','/global_costmap/costmap']):
        if name in counts:continue
        data=np.array(m.data).reshape(m.info.height,m.info.width)
        origin=np.array([m.info.origin.position.x,m.info.origin.position.y]);res=m.info.resolution
        grid=AuditGrid(data,origin,res,poly)
        stats={'clear':0,'blocked':0}
        # Compare static obstacle and free floor in the same recorded costmap.
        from scipy.ndimage import distance_transform_edt
        usable=(data>=0)&(data<100)
        clearance=distance_transform_edt(usable)
        for value in [0,100]:
            indices=np.argwhere(clearance >= max(1., clearance.max()-3) if value==0 else data==100)
            for row,col in indices[::max(1,len(indices)//70)][:70]:
                pose=np.r_[origin+res*(np.array([col,row])+.5),.25]
                sweeps=swept_poses(pose,np.array([.21,.19,.17]),np.array([.03,.03,.1]))
                blocked=any(grid.along(s)['first'] for s in sweeps)
                if value==100:assert blocked
                stats['blocked' if blocked else 'clear']+=1
        if stats['clear'] and stats['blocked']:counts[name]=stats
        if len(counts)==2:break
    assert len(counts)==2
    return counts

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('grid');a.add_argument('bags',nargs='+');args=a.parse_args()
    paths={p:replay_path(p) for p in args.bags}
    assert sum(r['three_axis_ticks'] for r in paths.values())>0
    print(json.dumps(dict(paths=paths,grids=replay_grids(args.grid)),indent=2))
