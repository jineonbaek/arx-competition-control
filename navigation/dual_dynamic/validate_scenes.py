"""Same-time held-out cross-LiDAR scene validation; no online TF or fusion output."""
import argparse,json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
from relative import interpolate
from scene_registration import voxel,transform,prepare_target,register,heldout
p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args();root=Path(a.run);out=root/'scene-validation';out.mkdir(exist_ok=True)
D=np.load(root/'relative-diagnostic.npz');times=D['time_ns'];G=D['world_alignment'];base=int(times[0]);indices=set(np.linspace(1,len(times)-2,23).astype(int).tolist());indices.add(int(np.argmax(np.linalg.norm(D['translation_change_m'],axis=1))));indices.add(int(np.argmax(np.linalg.norm(D['rotation_change_deg'],axis=1))));desired=times[sorted(indices)]
arrays={}
for n in ['top','bottom']:
 rows=json.loads((root/'trajectories'/(n+'_odom.json')).read_text());arrays[n]=(np.array([x['t'] for x in rows],np.int64),np.array([x['p'] for x in rows]),np.array([x['q'] for x in rows]))
cache=out/'selected-clouds.npz'
if cache.exists():
 z=np.load(cache);picked={n:{} for n in ['top','bottom']}
 for n in picked:
  for j in range(len(desired)):
   key=f'{n}_{j}'
   if key in z:picked[n][j]=(int(z[key+'_t']),z[key])
else:
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));picked={n:{} for n in ['top','bottom']}
 while reader.has_next():
  topic,data,_=reader.read_next()
  if topic not in ['/dual/top/cloud_body','/dual/bottom/cloud_body']:continue
  name='top' if '/top/' in topic else 'bottom';m=deserialize_message(data,PointCloud2);t=m.header.stamp.sec*10**9+m.header.stamp.nanosec;j=int(np.argmin(np.abs(desired-t)))
  if abs(int(desired[j])-t)>60000000:continue
  if j in picked[name] and abs(picked[name][j][0]-int(desired[j]))<=abs(t-int(desired[j])):continue
  assert m.header.frame_id==name+'_lio_imu',(name,m.header.frame_id)
  offsets={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[c] for c in ['x','y','z']],'itemsize':m.point_step});pts=np.frombuffer(m.data,dtype=dt,count=m.width*m.height);xyz=np.column_stack([pts[c] for c in ['x','y','z']]);rng=np.linalg.norm(xyz,axis=1);xyz=voxel(xyz[(rng>.8)&(rng<8)],.08);picked[name][j]=(t,xyz)
 saved={}
 for n,items in picked.items():
  for j,(t,pts) in items.items():saved[f'{n}_{j}']=pts;saved[f'{n}_{j}_t']=t
 np.savez_compressed(cache,**saved)
print('extraction complete', {n:len(v) for n,v in picked.items()},flush=True)
E=np.eye(4);E[:3,3]=[-.011,-.02329,.04412];L=np.eye(4);L[:3,:3]=[[.9999599541855795,-.006173661982418676,.006478882843571455],[.006240449010145331,.9999270748162661,-.010339334869211488],[-.006414578811244147,.010379351960154099,.999925558345001]];L[:3,3]=[.1756720301648963,-.008414072166182663,.9505154950673719];prior=E@L@np.linalg.inv(E)
results=[]
for j in range(len(desired)):
 if j not in picked['top'] or j not in picked['bottom']:continue
 th,target=picked['top'][j];tb,source=picked['bottom'][j]
 try:
  Bh=interpolate(*arrays['bottom'],th);Bb=interpolate(*arrays['bottom'],tb);Hh=interpolate(*arrays['top'],th)
 except ValueError:continue
 source=transform(source,np.linalg.inv(Bh)@Bb);pred=np.linalg.inv(Hh)@np.linalg.inv(G)@Bh
 rng=np.random.default_rng(123+j);si=rng.permutation(len(source));ti=rng.permutation(len(target));sv=source[si[::3]];tv=target[ti[::3]];st=source[np.delete(si,np.arange(0,len(si),3))];tt=target[np.delete(ti,np.arange(0,len(ti),3))]
 if min(len(st),len(tt))<100:continue
 prep=prepare_target(tt);seeds=[pred,prior]
 for axis in range(3):
  for sign in [-1,1]:
   seed=pred.copy();seed[axis,3]+=sign*.08;seeds.append(seed)
 for sign in [-1,1]:
  seed=pred.copy();seed[:3,:3]=Rotation.from_rotvec([0,0,sign*.05]).as_matrix()@seed[:3,:3];seeds.append(seed)
 fits=[register(st,prep,s) for s in seeds];best=min(fits,key=lambda x:x['score']);T=best['T'];delta=np.linalg.inv(pred)@T
 similar=[f for f in fits if f['score']<best['score']+.003];spread_t=max(np.linalg.norm(f['T'][:3,3]-T[:3,3]) for f in similar);spread_r=max(Rotation.from_matrix(f['T'][:3,:3]@T[:3,:3].T).magnitude()*180/np.pi for f in similar)
 row={k:v for k,v in best.items() if k!='T'};row.update(time_s=(th-base)/1e9,source_time_difference_ms=(tb-th)/1e6,head_points=len(target),bottom_points=len(source),T_head_bottom=T.tolist(),T_lio_prediction=pred.tolist(),heldout_fit=heldout(sv,tv,T),heldout_lio=heldout(sv,tv,pred),difference_from_lio_translation_mm=float(np.linalg.norm(delta[:3,3])*1000),difference_from_lio_rotation_deg=float(Rotation.from_matrix(delta[:3,:3]).magnitude()*180/np.pi),similar_seed_count=len(similar),multistart_spread_mm=float(spread_t*1000),multistart_spread_deg=float(spread_r),gates_provisional=True)
 row['provisional_usable']=bool(row['rank']==6 and row['condition']<1000 and spread_t<.02 and spread_r<1 and row['heldout_fit']['source_within_10cm']>.2)
 results.append(row);(out/'results.json').write_text(json.dumps({'method':'independently deskewed scans; bottom ego-motion compensation to head scan end; robust 6DoF point-to-plane; held-out voxel points; 10 initializations','not_independent_of_deskew_lio':True,'not_dynamic_extrinsic_ground_truth':True,'samples':results},indent=2));print(j,round(row['time_s'],2),'rank',row['rank'],'cond',round(row['condition'],1),'diff',round(row['difference_from_lio_translation_mm'],1),round(row['difference_from_lio_rotation_deg'],2),'usable',row['provisional_usable'],flush=True)
