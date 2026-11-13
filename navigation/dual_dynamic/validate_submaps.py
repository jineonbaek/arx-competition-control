"""Offline time compensated submaps, temporal support and spatial holdout."""
import argparse,json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
from relative import interpolate
from scene_registration import transform,voxel,prepare_target,register
from submap_core import compensate,persistent_points,spatial_split
p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args();root=Path(a.run);out=root/'submap-validation';out.mkdir(exist_ok=True)
previous=json.loads((root/'scene-validation/results.json').read_text());rows=previous['samples'];centers=np.array([round(r['time_s']*1e9) for r in rows],np.int64)+int(np.load(root/'relative-diagnostic.npz')['time_ns'][0]);G=np.array(previous['initial_scene_world_alignment']);arrays={}
for n in ['top','bottom']:
 rr=json.loads((root/'trajectories'/(n+'_odom.json')).read_text());arrays[n]=(np.array([x['t'] for x in rr],np.int64),np.array([x['p'] for x in rr]),np.array([x['q'] for x in rr]))
cache=out/'window-clouds.npz';clouds={n:[] for n in arrays}
if cache.exists():
 z=np.load(cache)
 for n in clouds:
  for i,t in enumerate(z[n+'_times']):clouds[n].append((int(t),z[f'{n}_{i}']))
else:
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 while reader.has_next():
  topic,data,_=reader.read_next()
  if topic not in ['/dual/top/cloud_body','/dual/bottom/cloud_body']:continue
  n='top' if '/top/' in topic else 'bottom';m=deserialize_message(data,PointCloud2);t=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  if np.min(np.abs(centers-t))>550000000:continue
  assert m.header.frame_id==n+'_lio_imu'
  off={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[off[c] for c in ['x','y','z']],'itemsize':m.point_step});v=np.frombuffer(m.data,dtype=dt,count=m.width*m.height);xyz=np.column_stack([v[c] for c in ['x','y','z']]);rr=np.linalg.norm(xyz,axis=1);clouds[n].append((t,voxel(xyz[(rr>.8)&(rr<8)],.06)))
 save={}
 for n,items in clouds.items():
  save[n+'_times']=np.array([t for t,_ in items],np.int64)
  for i,(_,points) in enumerate(items):save[f'{n}_{i}']=points
 np.savez_compressed(cache,**save)
print('extracted', {n:len(x) for n,x in clouds.items()},flush=True)
def make_map(n,center,half,near):
 ref=interpolate(*arrays[n],center);frames=[]
 for t,points in clouds[n]:
  if abs(t-center)>half*1e9:continue
  try:pose=interpolate(*arrays[n],t)
  except ValueError:continue
  points=points[np.linalg.norm(points,axis=1)>near];frames.append(compensate(points,pose,ref))
 if len(frames)<4:return np.empty((0,3)),len(frames)
 pts=persistent_points(frames,min_frames=max(3,int(np.ceil(len(frames)*.5))),min_range=0)
 # Keep only locally planar support; downsample horizontal planes to prevent floor domination.
 if len(pts)<60:return np.empty((0,3)),len(frames)
 _,_,normals,valid=prepare_target(pts);up=ref[:3,:3].T@np.array([0,0,1]);horizontal=np.abs(normals@up)>.9;idx=np.arange(len(pts));pts=pts[valid&(~horizontal|(idx%4==0))]
 return pts,len(frames)
def validation(source,target,T):
 if min(len(source),len(target))<50:return {'support':0.,'rms_m':None}
 pts,tree,normals,valid=prepare_target(target);x=transform(source,T);dist,ids=tree.query(x,workers=1);mask=(dist<.25)&valid[ids];res=np.sum((x[mask]-pts[ids[mask]])*normals[ids[mask]],axis=1)
 return {'support':float(mask.mean()),'rms_m':float(np.sqrt(np.mean(res**2))) if len(res) else None,'count':len(res)}
results=[]
for i,center in enumerate(centers):
 center=int(center)
 try:B=interpolate(*arrays['bottom'],center);H=interpolate(*arrays['top'],center)
 except ValueError:continue
 pred=np.linalg.inv(H)@np.linalg.inv(G)@B
 for variant,half,near in [('0.5s_1.2m',.25,1.2),('1s_1.2m',.5,1.2),('1s_1.6m',.5,1.6)]:
  source,nb=make_map('bottom',center,half,near);target,nh=make_map('top',center,half,near)
  if min(len(source),len(target))<120:
   results.append({'index':i,'variant':variant,'status':'insufficient_persistent_geometry','source_count':len(source),'target_count':len(target)});continue
  st,sv=spatial_split(source,transform(source,pred));tt,tv=spatial_split(target)
  if min(len(st),len(tt),len(sv),len(tv))<50:continue
  prep=prepare_target(tt);seeds=[pred,np.array(rows[i]['T_head_bottom'])]
  for axis in range(3):
   for sign in [-1,1]:
    s=pred.copy();s[axis,3]+=sign*.06;seeds.append(s)
  fits=[register(st,prep,s) for s in seeds];best=min(fits,key=lambda r:r['score']);T=best['T'];similar=[f for f in fits if f['score']<best['score']+.003];spread=max(np.linalg.norm(f['T'][:3,3]-T[:3,3]) for f in similar);angle=max(Rotation.from_matrix(f['T'][:3,:3]@T[:3,:3].T).magnitude()*180/np.pi for f in similar)
  delta=np.linalg.inv(pred)@T;vfit=validation(sv,tv,T);vpred=validation(sv,tv,pred)
  row={k:v for k,v in best.items() if k!='T'};row.update(index=i,variant=variant,time_s=rows[i]['time_s'],T_head_bottom=T.tolist(),T_lio=pred.tolist(),head_frames=nh,bottom_frames=nb,head_points=len(target),bottom_points=len(source),spatial_holdout_fit=vfit,spatial_holdout_lio=vpred,multistart_spread_mm=float(spread*1000),multistart_spread_deg=float(angle),difference_from_lio_mm=float(np.linalg.norm(delta[:3,3])*1000),difference_from_lio_deg=float(Rotation.from_matrix(delta[:3,:3]).magnitude()*180/np.pi),status='computed')
  row['provisional_usable']=bool(best['rank']==6 and best['condition']<1000 and spread<.02 and angle<1 and vfit['support']>.2 and vfit['rms_m'] is not None and vfit['rms_m']<.03)
  results.append(row);(out/'results.json').write_text(json.dumps({'samples':results,'dynamic_extrinsic_validated':False,'body_filter':'conservative per-sensor radius exclusion, not measured chassis footprint','static_filter':'at least half window frames in 12cm voxels; locally planar; horizontal support thinned 4x','validation':'held out 60cm spatial blocks; shared surfaces/own-LIO deskew still correlated'},indent=2));print(i,variant,'n',len(source),len(target),'val',round(vfit['rms_m'] or 1,3),'diff',round(row['difference_from_lio_mm'],1),'usable',row['provisional_usable'],flush=True)
(out/'results.json').write_text(json.dumps({'samples':results,'dynamic_extrinsic_validated':False,'body_filter':'conservative per-sensor radius exclusion; no calibrated chassis mask','static_filter':'15cm neighborhood multi-frame support, planar surfaces, horizontal balancing','validation':'60cm spatial block holdout; own-LIO deskew/time compensation not independent truth'},indent=2))
