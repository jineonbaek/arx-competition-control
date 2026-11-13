"""Numerical evidence only; no online parameter mutation or motion."""
import sys,json,collections
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
p=Path(sys.argv[1]);meta=json.loads((p/'capture.json').read_text());data=np.load(p/'samples.npz');origin=np.array([.185330,-.000003,.230]);sectors=['front','left','rear','right']
def sector(a):return np.floor(((a+np.pi/4)%(2*np.pi))/(np.pi/2)).astype(int)
def quant(a):return np.quantile(a,[0,.5,.95,1]).tolist() if len(a) else []
mesh=np.load(Path(__file__).resolve().parents[1]/'models/lift2s_bottom_upright_20261105/body_voxels.npz');tree=cKDTree(mesh['rigid'])
summary={'commands_sent':0,'capture_errors':meta['errors'],'odom_span_m':meta['odom_span_m'],'stages':{},'scan_effect':{},'maps':{}}
for topic,rows in meta['samples'].items():
 counts=[];near=[];cover=[];points=[];zero_counts=[];framevox=collections.Counter()
 for row in rows:
  a=data[row['key']];a=a[np.isfinite(a).all(axis=1)];zero=np.linalg.norm(a-origin,axis=1)<1e-5;zero_counts.append(int(zero.sum()));a=a[~zero];rel=a-origin;d=np.linalg.norm(a[:,:2],axis=1);angles=np.arctan2(rel[:,1],rel[:,0]);bins=np.floor((angles+np.pi)/(2*np.pi)*720).astype(int).clip(0,719);ids=sector(angles);cover.append([len(set(bins[ids==i])) for i in range(4)]);counts.append(len(a));near.append(np.sum((d<.529)&(a[:,2]>=.03)&(a[:,2]<=1.35)))
  a=a[(d<.8)&(a[:,2]>=.03)&(a[:,2]<=1.35)];points.append(a)
  for v in set(map(tuple,np.floor(a/.04).astype(int))):framevox[v]+=1
 info={'frames':len(rows),'zero_range_return_quantiles':quant(zero_counts),'point_count_quantiles':quant(counts),'points_in_collision_circle_z003_135_quantiles':quant(near),'azimuth_bins_of180_median':dict(zip(sectors,np.median(cover,axis=0).tolist())) if cover else {}}
 if points and any(len(a) for a in points):
  a=np.vstack(points);info['near_point_xyz_quantiles_m']=np.quantile(a,[0,.5,1],axis=0).tolist();info['near_point_mesh_distance_quantiles_m']=quant(tree.query(a)[0]);centers=(np.array(list(framevox))+.5)*.04;weights=np.array(list(framevox.values()));edges=cKDTree(centers).query_pairs(.085,output_type='ndarray');n=len(centers)
  graph=coo_matrix((np.ones(len(edges)),(edges[:,0],edges[:,1])),shape=(n,n));nc,labels=connected_components(graph,directed=False);clusters=[]
  for i in range(nc):
   b=centers[labels==i];w=weights[labels==i]
   if len(b)<2:continue
   clusters.append({'centroid_m':np.average(b,axis=0,weights=w).tolist(),'bounds_m':[b.min(axis=0).tolist(),b.max(axis=0).tolist()],'nearest_xy_radius_m':float(np.linalg.norm(b[:,:2],axis=1).min()),'voxel_count':len(b),'peak_frame_fraction':float(w.max()/max(1,len(rows))),'mesh_distance_median_m':float(np.median(tree.query(b)[0]))})
  info['near_clusters']=sorted(clusters,key=lambda c:c['nearest_xy_radius_m'])[:12]
 summary['stages'][topic]=info
for target in ['/pb/scan','/pb/mapping_scan']:
 raw=meta['scans'][target+'_unchecked'];fin=meta['scans'][target];r=data[raw['key']];f=data[fin['key']];lookup={s:i for i,s in enumerate(raw['stamp_ns'])};pairs=[(lookup[s],i) for i,s in enumerate(fin['stamp_ns']) if s in lookup];rr=r[[a for a,b in pairs]];ff=f[[b for a,b in pairs]];validr=np.isfinite(rr);validf=np.isfinite(ff);angles=fin['angle_min']+np.arange(f.shape[1])*fin['angle_increment'];ss=sector(angles)
 summary['scan_effect'][target]={'paired_frames':len(pairs),'unchecked_finite_quantiles':quant(validr.sum(axis=1)),'final_finite_quantiles':quant(validf.sum(axis=1)),'masked_finite_quantiles':quant((validr&~validf).sum(axis=1)),'by_sector':{sectors[i]:{'unchecked_finite_fraction':float(validr[:,ss==i].mean()),'final_finite_fraction':float(validf[:,ss==i].mean()),'mask_removed_fraction_of_all_bins':float((validr&~validf)[:,ss==i].mean())} for i in range(4)}}
for topic,m in meta['maps'].items():
 a=data[m['key']];res=m['resolution'];ox,oy=m['origin'];t=np.array(m['map_from_body_t']);ix=int(np.floor((t[0]-ox)/res));iy=int(np.floor((t[1]-oy)/res));info={'robot_cell':int(a[iy,ix]) if 0<=iy<a.shape[0] and 0<=ix<a.shape[1] else 'outside','counts':{'free':int(np.sum(a==0)),'unknown':int(np.sum(a<0)),'lethal':int(np.sum(a==100))}}
 yy,xx=np.where(a==100);xy=np.c_[ox+(xx+.5)*res,oy+(yy+.5)*res];dd=np.linalg.norm(xy-t[:2],axis=1);nearidx=np.argsort(dd)[:10];R=np.array(m['map_from_body_R']);body=(np.c_[xy[nearidx],np.full(len(nearidx),t[2])]-t)@R;info['nearest_lethal_distance_m']=dd[nearidx].tolist();info['nearest_lethal_body_xy_m']=body[:,:2].tolist();info['lethal_within_radius0529']=int(np.sum(dd<=.529));summary['maps'][topic]=info
(p/'analysis.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
