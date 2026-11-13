"""Offline time compensated submaps, temporal support and spatial holdout."""
import argparse,json,sys
sys.path.insert(0,str(__import__('pathlib').Path(__file__).resolve().parents[1]/'dual_dynamic'))
from flex_core import installation,body_mask
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
from relative import interpolate
from scene_registration import transform,voxel,prepare_target,register
from submap_core import compensate,persistent_points,spatial_split
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--mesh');a=p.parse_args();root=Path(a.run);out=root/('mesh-flex-scenes' if a.mesh else 'cad-flex-scenes');out.mkdir(exist_ok=True)
mesh_filter=None
if a.mesh:
 from mesh_filter import MeshFilter
 mesh_filter=MeshFilter.load(a.mesh)
filter_audit={};geometry_audit={}
arrays={}
for n in ['top','bottom']:
 rr=json.loads((root/'trajectories'/(n+'_odom.json')).read_text());arrays[n]=(np.array([x['t'] for x in rr],np.int64),np.array([x['p'] for x in rr]),np.array([x['q'] for x in rr]))
AB,AH=installation(root)
t0=max(int(arrays['bottom'][0][0]),int(arrays['top'][0][0]));Binit=interpolate(*arrays['bottom'],t0);Hinit=interpolate(*arrays['top'],t0);G=Binit@np.linalg.inv(AB)@AH@np.linalg.inv(Hinit)
if (root/'geometry.json').exists():
 end=min(int(v[0][-1]) for v in arrays.values())
 centers=np.linspace(t0+1_000_000_000,end-1_000_000_000,8).astype(np.int64)
 # Include windows during short maneuvers even when most of the bag is stationary.
 times,positions,_=arrays['bottom']
 step=np.linalg.norm(np.diff(positions,axis=0),axis=1)
 distance=np.r_[0.,np.cumsum(np.where(step>.003,step,0.))]
 if distance[-1]>.25:
  motion_times=times[np.searchsorted(distance,np.linspace(.05,.95,12)*distance[-1])]
  centers=np.unique(np.r_[centers,motion_times[(motion_times>centers[0])&(motion_times<centers[-1])]])
 rows=[{'time_s':(int(t)-t0)/1e9} for t in centers]
else:
 previous=json.loads((root/'scene-validation/results.json').read_text());rows=previous['samples'];centers=np.array([round(r['time_s']*1e9) for r in rows],np.int64)+int(np.load(root/'relative-diagnostic.npz')['time_ns'][0])
cache=out/'window-clouds.npz';clouds={n:[] for n in arrays}
if cache.exists() and (not (root/'geometry.json').exists() or
                       np.array_equal(np.load(cache).get('centers'),centers)):
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
  off={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[off[c] for c in ['x','y','z']],'itemsize':m.point_step});v=np.frombuffer(m.data,dtype=dt,count=m.width*m.height);xyz=np.column_stack([v[c] for c in ['x','y','z']]);rr=np.linalg.norm(xyz,axis=1);xyz=xyz[np.isfinite(xyz).all(axis=1)&(rr>.05)&(rr<8)];clouds[n].append((t,xyz if mesh_filter else voxel(xyz,.06)))
 save={'centers':centers}
 for n,items in clouds.items():
  save[n+'_times']=np.array([t for t,_ in items],np.int64)
  for i,(_,points) in enumerate(items):save[f'{n}_{i}']=points
 np.savez_compressed(cache,**save)
print('extracted', {n:len(x) for n,x in clouds.items()},flush=True)
def make_map(n,center,half,near):
 ref=interpolate(*arrays[n],center);frames=[];used_times=[]
 for t,points in clouds[n]:
  if abs(t-center)>half*1e9:continue
  try:pose=interpolate(*arrays[n],t)
  except ValueError:continue
  try: head_in_body=AB@np.linalg.inv(interpolate(*arrays['bottom'],t))@G@interpolate(*arrays['top'],t)
  except ValueError:continue
  sensor_in_body=AB if n=='bottom' else head_in_body
  box=body_mask(points,sensor_in_body,padding=near)
  mask=mesh_filter.mask(points,sensor_in_body,head_in_body,padding=near) if mesh_filter else box
  if mesh_filter:
   filter_audit[(n,t,near)]={'sensor':n,'time_ns':t,'padding_m':near,'input':len(points),'mesh_removed':int(mask.sum()),'box_removed':int(box.sum()),'mesh_only_removed':int((mask&~box).sum()),'box_only_removed':int((box&~mask).sum())}
  points=points[~mask]
  if mesh_filter:points=voxel(points,.06)
  frames.append(compensate(points,pose,ref));used_times.append(t)
 if len(frames)<4:return np.empty((0,3)),len(frames),False
 pts=persistent_points(frames,min_frames=max(3,int(np.ceil(len(frames)*.5))),min_range=0)
 # Keep only locally planar support; downsample horizontal planes to prevent floor domination.
 if len(pts)<60:return np.empty((0,3)),len(frames),False
 _,_,normals,valid=prepare_target(pts);up=ref[:3,:3].T@np.array([0,0,1]);horizontal=np.abs(normals@up)>.9;idx=np.arange(len(pts))
 center_head=AB@np.linalg.inv(interpolate(*arrays['bottom'],center))@G@interpolate(*arrays['top'],center)
 center_sensor=AB if n=='bottom' else center_head
 vehicle_points=transform(pts,center_sensor)
 ground=horizontal&(np.abs(vehicle_points[:,2])<.08)
 geometry_audit[(n,center,half,near)]={'persistent_points_before_balance':len(pts),'planar_count':int(valid.sum()),'horizontal_fraction_of_planar':float(horizontal[valid].mean()) if valid.any() else None,'ground_band_fraction_of_planar':float(ground[valid].mean()) if valid.any() else None,'ground_band_definition':'horizontal normal |dot(up)|>0.9 and |height in candidate vehicle frame|<0.08m; diagnostic not segmented ground truth'}
 pts=pts[valid&(~horizontal|(idx%4==0))]
 coverage=bool(used_times[0]<=center-half*1e9+120000000 and used_times[-1]>=center+half*1e9-120000000 and np.max(np.diff(used_times))<=150000000)
 return pts,len(frames),coverage
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
 for variant,half,near in ([('1s_pad02',.5,.02)] if (root/'geometry.json').exists() else [('0.5s_pad02',.25,.02),('1s_pad02',.5,.02),('1s_pad06',.5,.06)]):
  source,nb,bc=make_map('bottom',center,half,near);target,nh,hc=make_map('top',center,half,near)
  if min(len(source),len(target))<120:
   results.append({'index':i,'variant':variant,'status':'insufficient_persistent_geometry','source_count':len(source),'target_count':len(target)});continue
  st,sv=spatial_split(source,transform(source,pred));tt,tv=spatial_split(target)
  if min(len(st),len(tt),len(sv),len(tv))<50:continue
  prep=prepare_target(tt);seeds=[pred,np.linalg.inv(AH)@AB]
  for axis in range(3):
   for sign in [-1,1]:
    s=pred.copy();s[axis,3]+=sign*.06;seeds.append(s)
  fits=[register(st,prep,s) for s in seeds];best=min(fits,key=lambda r:r['score']);T=best['T'];similar=[f for f in fits if f['score']<best['score']+.003];spread=max(np.linalg.norm(f['T'][:3,3]-T[:3,3]) for f in similar);angle=max(Rotation.from_matrix(f['T'][:3,:3]@T[:3,:3].T).magnitude()*180/np.pi for f in similar)
  delta=np.linalg.inv(pred)@T;vfit=validation(sv,tv,T);vpred=validation(sv,tv,pred)
  row={k:v for k,v in best.items() if k!='T'};row.update(index=i,variant=variant,time_s=rows[i]['time_s'],T_head_bottom=T.tolist(),T_lio=pred.tolist(),head_frames=nh,bottom_frames=nb,head_points=len(target),bottom_points=len(source),spatial_holdout_fit=vfit,spatial_holdout_lio=vpred,multistart_spread_mm=float(spread*1000),multistart_spread_deg=float(angle),difference_from_lio_mm=float(np.linalg.norm(delta[:3,3])*1000),difference_from_lio_deg=float(Rotation.from_matrix(delta[:3,:3]).magnitude()*180/np.pi),status='computed')
  xp=transform(st,T);target_points,tree,normals,valid=prep;distance,ids=tree.query(xp,workers=1);valid_match=(distance<.20)&valid[ids];xx=xp[valid_match];nn=normals[ids[valid_match]];J=np.c_[np.cross(xx,nn),nn];information=J.T@J/max(len(J),1);row['information']=information.tolist();row['information_eigenvalues']=np.linalg.eigvalsh(information).tolist();row['cad_prior_only']=True
  row['full_symmetric_window']=center-half*1e9>=max(arrays['bottom'][0][0],arrays['top'][0][0]) and center+half*1e9<=min(arrays['bottom'][0][-1],arrays['top'][0][-1]);row['actual_cloud_window_coverage']={'bottom':bc,'top':hc};row['full_symmetric_window']=bool(row['full_symmetric_window'] and bc and hc)
  row['provisional_usable']=bool(row['full_symmetric_window'] and best['rank']==6 and best['condition']<1000 and spread<.02 and angle<1 and vfit['support']>.2 and vfit['rms_m'] is not None and vfit['rms_m']<.03)
  row['geometry']={n:geometry_audit[(n,center,half,near)] for n in ['bottom','top']}
  head_up=H[:3,:3].T@np.array([0,0,1]);row['matched_horizontal_fraction']=float((np.abs(nn@head_up)>.9).mean()) if len(nn) else None
  te,tv=np.linalg.eigh(nn.T@nn/max(len(nn),1));row['translation_information_eigenvalues']=te.tolist();row['weak_translation_direction_head_frame']=tv[:,0].tolist()
  row['rejection_reasons']=[reason for failed,reason in [(not row['full_symmetric_window'],'window_coverage'),(best['rank']!=6 or best['condition']>=1000,'geometry_rank_condition'),(spread>=.02,'translation_seed_sensitivity'),(angle>=1,'rotation_seed_sensitivity'),(vfit['support']<=.2,'holdout_support'),(vfit['rms_m'] is None or vfit['rms_m']>=.03,'holdout_residual')] if failed]
  results.append(row);(out/'results.json').write_text(json.dumps({'samples':results,'dynamic_extrinsic_validated':False,'status':'partial_processing'},indent=2));print(i,variant,'n',len(source),len(target),'val',round(vfit['rms_m'] or 1,3),'diff',round(row['difference_from_lio_mm'],1),'usable',row['provisional_usable'],flush=True)
(out/'results.json').write_text(json.dumps({'samples':results,'dynamic_extrinsic_validated':False,'body_filter':('URDF visual mesh 5mm voxels; dynamic head subtree; filter before 6cm downsampling' if mesh_filter else 'CAD whole-robot conservative box plus margin; head mask uses dynamic LIO candidate; not a mesh self filter'),'mesh_path':a.mesh,'filter_audit':list(filter_audit.values()),'static_filter':'15cm neighborhood multi-frame support, planar surfaces, horizontal balancing','validation':'60cm spatial block holdout; own-LIO deskew/time compensation not independent truth'},indent=2))
