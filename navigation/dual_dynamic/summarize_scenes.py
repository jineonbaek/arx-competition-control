import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from relative import interpolate
from scene_registration import transform,prepare_target
p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args();root=Path(a.run);out=root/'scene-validation';data=json.loads((out/'results.json').read_text());rows=data['samples'];z=np.load(out/'selected-clouds.npz');D=np.load(root/'relative-diagnostic.npz');base=int(D['time_ns'][0]);arrays={}
for n in ['top','bottom']:
 rr=json.loads((root/'trajectories'/(n+'_odom.json')).read_text());arrays[n]=(np.array([x['t'] for x in rr],np.int64),np.array([x['p'] for x in rr]),np.array([x['q'] for x in rr]))
def plane_validation(source,target,T):
 pts,tree,normals,valid=prepare_target(target);x=transform(source,T);dist,ids=tree.query(x,workers=1);mask=(dist<.30)&valid[ids];res=np.abs(np.sum((x[mask]-pts[ids[mask]])*normals[ids[mask]],axis=1))
 return {'support_fraction':float(mask.mean()),'median_abs_plane_m':float(np.median(res)) if len(res) else None,'rms_plane_m':float(np.sqrt(np.mean(res**2))) if len(res) else None,'support_count':int(mask.sum())}
initial=[r for r in rows if r['time_s']<35 and r['rank']==6 and r['multistart_spread_mm']<20 and r['multistart_spread_deg']<1]
if len(initial)<2:raise RuntimeError('Insufficient stable initial registrations to remove initial gauge error')
Gs=[]
for r in initial:
 t=base+round(r['time_s']*1e9);B=interpolate(*arrays['bottom'],t);H=interpolate(*arrays['top'],t);Gs.append(B@np.linalg.inv(r['T_head_bottom'])@np.linalg.inv(H))
Gs=np.array(Gs);G=np.eye(4);G[:3,3]=Gs[:,:3,3].mean(axis=0);G[:3,:3]=Rotation.from_matrix(Gs[:,:3,:3]).mean().as_matrix()
keys=[k for k in z.files if k.startswith('top_') and k.endswith('_t')]
for r in rows:
 t=base+round(r['time_s']*1e9);key=min(keys,key=lambda k:abs(int(z[k])-t));j=int(key.split('_')[1]);th=int(z[key]);tb=int(z[f'bottom_{j}_t']);source=z[f'bottom_{j}'];target=z[f'top_{j}'];B=interpolate(*arrays['bottom'],th);Bb=interpolate(*arrays['bottom'],tb);H=interpolate(*arrays['top'],th);source=transform(source,np.linalg.inv(B)@Bb);pred=np.linalg.inv(H)@np.linalg.inv(G)@B;T=np.array(r['T_head_bottom']);delta=np.linalg.inv(pred)@T
 r['after_initial_scene_alignment_translation_mm']=float(np.linalg.norm(delta[:3,3])*1000);r['after_initial_scene_alignment_rotation_deg']=float(Rotation.from_matrix(delta[:3,:3]).magnitude()*180/np.pi)
 rng=np.random.default_rng(123+j);si=rng.permutation(len(source));ti=rng.permutation(len(target));sv=source[si[::3]];tv=target[ti[::3]]
 r['heldout_plane_fit']=plane_validation(sv,tv,T);r['heldout_plane_aligned_lio']=plane_validation(sv,tv,pred)
 r['T_scene_aligned_lio_prediction']=pred.tolist()
good=[r for r in rows if r['provisional_usable'] and r['time_s']>=35]
summary={'tested_pairs':len(rows),'provisional_usable_pairs':sum(r['provisional_usable'] for r in rows),'initial_alignment_samples':len(initial),'initial_alignment_translation_spread_mm':(np.ptp(Gs[:,:3,3],axis=0)*1000).tolist(),'initial_alignment_rotation_spread_deg':float(max(Rotation.from_matrix(x[:3,:3]@G[:3,:3].T).magnitude()*180/np.pi for x in Gs)),'non_initial_provisional_usable_pairs':len(good),'aligned_lio_disagreement_percentiles':[50,90,100], 'translation_mm':np.percentile([r['after_initial_scene_alignment_translation_mm'] for r in good],[50,90,100]).tolist(),'rotation_deg':np.percentile([r['after_initial_scene_alignment_rotation_deg'] for r in good],[50,90,100]).tolist(),'heldout_plane_fit_rms_median_m':float(np.median([r['heldout_plane_fit']['rms_plane_m'] for r in good])),'heldout_plane_aligned_lio_rms_median_m':float(np.median([r['heldout_plane_aligned_lio']['rms_plane_m'] for r in good])),'heldout_support_fraction_median':float(np.median([r['heldout_plane_fit']['support_fraction'] for r in good])),'dynamic_extrinsic_validated':False,'limits':['single-scan overlap sparse','initial extrinsic inferred from static scene only','both scans already depend on own LIO deskew','thresholds provisional, not externally calibrated','held-out voxel split shares same physical surfaces; not independent scene or truth']}
data['summary']=summary;data['initial_scene_world_alignment']=G.tolist();(out/'results.json').write_text(json.dumps(data,indent=2));print(json.dumps(summary,indent=2))
