"""Replay existing trajectories into offline flexible-head batch estimator."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'dual_dynamic'))
from relative import interpolate
from flex_core import installation,solve,information_projector
from scene_registration import transform,voxel,prepare_target
from flex_core import body_mask
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--strength',type=float,default=1.);p.add_argument('--variant',default='1s_pad02');p.add_argument('--mesh');a=p.parse_args();root=Path(a.run);out=root/('offline-flex-mesh' if a.mesh else 'offline-flex');out.mkdir(exist_ok=True)
scene_dir=root/('mesh-flex-scenes' if a.mesh else 'cad-flex-scenes')
mesh_filter=None
if a.mesh:
 from mesh_filter import MeshFilter
 mesh_filter=MeshFilter.load(a.mesh)
AB,AH=installation(root);arrays={}
for n in ['top','bottom']:
 rr=json.loads((root/'trajectories'/(n+'_odom.json')).read_text());arrays[n]=(np.array([x['t'] for x in rr],np.int64),np.array([x['p'] for x in rr]),np.array([x['q'] for x in rr]))
start=max(int(v[0][0]) for v in arrays.values());end=min(int(v[0][-1]) for v in arrays.values());ts=np.arange(start,end,200000000,dtype=np.int64);t=(ts-ts[0])/1e9;raw={n:np.array([interpolate(*v,int(tt)) for tt in ts]) for n,v in arrays.items()};baseline=AB@np.linalg.inv(raw['bottom'][0])@raw['bottom']@np.linalg.inv(AB);head=AH@np.linalg.inv(raw['top'][0])@raw['top']
prior_timebase=start if (root/'geometry.json').exists() else int(np.load(root/'relative-diagnostic.npz')['time_ns'][0]);data=json.loads((scene_dir/'results.json').read_text());scene_rows=[s for s in data['samples'] if s['status']=='computed' and s['variant']==a.variant];scenes=[];rejected=[];test_rows=[]
for s in scene_rows:
 idx=int(np.argmin(np.abs(ts-(prior_timebase+round(s['time_s']*1e9)))))
 if s['index']%2:
  if s['provisional_usable']:test_rows.append((idx,s))
  continue
 if not s['provisional_usable']:
  rejected.append({'scene_index':s['index'],'reason':'scene_quality_or_truncated_window'});continue
 sigma=max(.015,s['spatial_holdout_fit']['rms_m']);W,rank=information_projector(np.array(s['information']))
 if rank<3:rejected.append({'scene_index':s['index'],'reason':'insufficient_observable_directions'});continue
 scenes.append({'index':idx,'time':(prior_timebase+round(s['time_s']*1e9)-start)/1e9,'T':np.array(s['T_head_bottom']),'information':np.array(s['information']),'sigma':sigma})
wall=time.monotonic();result=solve(t,baseline,head,scenes,AB,AH,strength=a.strength,max_nfev=35);elapsed=time.monotonic()-wall
suffix=a.variant+'_s'+str(a.strength);np.savez_compressed(out/(suffix+'.npz'),time_ns=ts,baseline=baseline,candidate=result['candidate'],output=result['output'],head_relative=result['relative'],head_relative_velocity=np.gradient(result['relative'][:,:3,3],t,axis=0),fallback=result['fallback'])
def trajectory_stats(T):
 pos=T[:,:3,3];last=t>t[-1]-15;return {'last15s_position_span_mm':(np.ptp(pos[last],axis=0)*1000).tolist(),'max_position_step_m':float(np.linalg.norm(np.diff(pos,axis=0),axis=1).max()),'path_m':float(np.linalg.norm(np.diff(pos,axis=0),axis=1).sum())}
held=[]
for idx,s in test_rows:
 fit=np.array(s['T_head_bottom']);nom=np.linalg.inv(AH)@AB;scene_ns=prior_timebase+round(s['time_s']*1e9)
 rel_at=interpolate(ts,result['relative'][:,:3,3],Rotation.from_matrix(result['relative'][:,:3,:3]).as_quat(),scene_ns,max_gap=.25);joint=np.linalg.inv(rel_at)@AB
 def err(T):
  d=T@np.linalg.inv(fit);return {'translation_mm':float(np.linalg.norm(d[:3,3])*1000),'rotation_deg':float(Rotation.from_matrix(d[:3,:3]).magnitude()*180/np.pi)}
 held.append({'time_s':(scene_ns-start)/1e9,'nearest_node_time_error_s':float(t[idx]-(scene_ns-start)/1e9),'scene_index':s['index'],'nominal_CAD':err(nom),'joint_candidate':err(joint),'final_output_fallback':bool(result['fallback'][idx])})
# Same bottom points, same initial static reference: diagnostic consistency, not absolute accuracy.
z=np.load(scene_dir/'window-clouds.npz');refs=[]
def filtered_bottom(pts,tt):
 if mesh_filter:
  B=AB@np.linalg.inv(raw['bottom'][0])@interpolate(*arrays['bottom'],int(tt))@np.linalg.inv(AB)
  H=AH@np.linalg.inv(raw['top'][0])@interpolate(*arrays['top'],int(tt))
  return voxel(pts[~mesh_filter.mask(pts,AB,np.linalg.inv(B)@H,.02)],.06)
 return pts[~body_mask(pts,AB,.02)]
for i,tt in enumerate(z['bottom_times']):
 if not start<=tt<start+35_000_000_000:continue
 pts=filtered_bottom(z[f'bottom_{i}'],tt);pose=AB@np.linalg.inv(raw['bottom'][0])@interpolate(*arrays['bottom'],int(tt));refs.append(transform(pts,pose))
map_stats={}
if refs:
 ref=voxel(np.vstack(refs),.10);refpts,tree,normals,valid=prepare_target(ref);errs={k:[] for k in ['baseline','candidate','output']};supports=[]
 for idx,s in test_rows:
  if t[idx]<36:continue
  j=int(np.argmin(np.abs(z['bottom_times']-ts[idx])));pts=filtered_bottom(z[f'bottom_{j}'],z['bottom_times'][j]);source_pose=interpolate(*arrays['bottom'],int(z['bottom_times'][j]));anchor_pose=interpolate(*arrays['bottom'],int(ts[idx]));pts=transform(pts,np.linalg.inv(anchor_pose)@source_pose)
  projected={k:transform(pts,(baseline if k=='baseline' else result[k])[idx]@AB) for k in errs};matches={k:tree.query(v,workers=1) for k,v in projected.items()};mask=np.ones(len(pts),bool)
  for dist,ids in matches.values():mask&=(dist<.25)&valid[ids]
  supports.append(int(mask.sum()))
  if mask.sum()<30:continue
  for k,x in projected.items():
   _,ids=matches[k];r=np.abs(np.sum((x[mask]-refpts[ids[mask]])*normals[ids[mask]],axis=1));errs[k].append(float(np.sqrt(np.mean(r*r))))
 map_stats={'evaluated_time_windows':len(errs['baseline']),'median_plane_rms_m':{k:float(np.median(v)) if v else None for k,v in errs.items()},'common_point_support_counts':supports,'caveat':'Reference from same-run initial bottom LIO, hence correlated consistency metric, not external ground truth.'}
correction=np.linalg.inv(baseline)@result['candidate'];delta=np.linalg.norm(correction[:,:3,3],axis=1)*1000;angles=Rotation.from_matrix(correction[:,:3,:3]).magnitude()*180/np.pi;flex=result['relative'];ftrans=flex[:,:3,3]-AH[:3,3];frot=(Rotation.from_matrix(flex[:,:3,:3])*Rotation.from_matrix(AH[:3,:3]).inv()).magnitude()*180/np.pi
report={'version':'offline_loose_coupled_validation_only','run':root.name,'variant':a.variant,'flex_strength':a.strength,'nodes':len(ts),'rate_hz':5,'elapsed_seconds':elapsed,'optimizer':{k:result[k] for k in ['success','status','cost','nfev','accepted_scenes']},'scene_postfit':result['scene_postfit'],'training_scene_times':len(scenes),'rejected_training_scenes':rejected,'held_out_scene_times':len(held),'fallback_fraction':float(result['fallback'].mean()),'baseline':trajectory_stats(baseline),'candidate':trajectory_stats(result['candidate']),'output':trajectory_stats(result['output']),'candidate_correction_mm_percentiles':np.percentile(delta,[50,95,100]).tolist(),'candidate_correction_deg_percentiles':np.percentile(angles,[50,95,100]).tolist(),'flex_translation_from_CAD_mm_percentiles':np.percentile(np.linalg.norm(ftrans,axis=1)*1000,[50,95,100]).tolist(),'flex_rotation_from_CAD_deg_percentiles':np.percentile(frot,[50,95,100]).tolist(),'withheld_relative_pose_comparison':held,'bottom_map_consistency':map_stats,'dynamic_extrinsic_validated':False,'commands_sent':0,'nav2_connected':False,'limitations':['offline batch smoother, not a dual raw-IMU tightly coupled LIO','5Hz states cannot reconstruct head vibration above 2.5Hz','gross scene limits 0.25m/0.35rad and 0.6s freshness are trial design thresholds','LIO correlations and scene reuse not fully modeled; no calibrated covariance claimed','CAD conservative box, not measured articulated mesh','flex rates/acceleration soft priors are design assumptions, require sensitivity checks','Output uses bottom-only outside 0.6s full-rank scene neighborhoods; transitions not approved for online control','No external ground truth; withheld scenes still share LIO deskew','Degeneracy handling applies to cross-sensor scene factors; no independent detection of bottom-only tracking failure']}
report['body_filter']=data['body_filter'];report['mesh_path']=a.mesh
if mesh_filter:
 report['limitations']=[x for x in report['limitations'] if x!='CAD conservative box, not measured articulated mesh']+['URDF zero-pose visual mesh approximated at 5mm; head subtree dynamic but column bending unmodeled','Filter uses independent-LIO dynamic head candidate, not iterative joint mesh/pose optimization']
(out/(suffix+'.json')).write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ['withheld_relative_pose_comparison','limitations','rejected_training_scenes']},indent=2),flush=True)
