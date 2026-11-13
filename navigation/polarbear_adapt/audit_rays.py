import json,sys
from pathlib import Path
import numpy as np
p=Path(sys.argv[1]);m=json.loads((p/'capture.json').read_text());z=np.load(p/'samples.npz');g=m['maps']['/pb/map'];R=np.array(g['map_from_body_R']);t=np.array(g['map_from_body_t']);origin=np.array(g['origin']);res=g['resolution'];sensor=t+R@np.array([.185330,-.000003,.230]);index=np.floor((t[:2]-origin)/res).astype(int);low=origin+index*res;high=low+res;results={}
for topic in ['/pb/scan_unchecked','/pb/scan','/pb/mapping_scan_unchecked','/pb/mapping_scan']:
 s=m['scans'][topic];v=z[s['key']];angles=s['angle_min']+np.arange(v.shape[1])*s['angle_increment'];directions=np.c_[np.cos(angles),np.sin(angles),np.zeros(len(angles))]@R.T;xy=directions[:,:2];safe=np.where(np.abs(xy)<1e-10,1e-10,xy);a=(low-sensor[:2])/safe;b=(high-sensor[:2])/safe;enter=np.maximum(np.minimum(a,b).max(axis=1),0);exit=np.maximum(a,b).min(axis=1);cross=(exit>=enter)&(exit>=0);counts=(np.isfinite(v)&(v>=enter)&cross).sum(axis=1);rear=np.abs(np.abs(angles)-np.pi)<np.deg2rad(15)
 results[topic]={'beams_geometrically_crossing_robot_cell':int(cross.sum()),'valid_crossing_counts_quantiles':np.quantile(counts,[0,.5,.95,1]).tolist(),'rear_30deg_finite_fraction':float(np.isfinite(v[:,rear]).mean())}
a=z[g['key']];si=np.floor((sensor[:2]-origin)/res).astype(int);results['map_cells']={'body':int(a[index[1],index[0]]),'optical_origin':int(a[si[1],si[0]]),'robot_cell_in_map':index.tolist(),'optical_xy_in_map':sensor[:2].tolist(),'note':'ray-cell intersection uses current static pose, <=5.3mm translation span; not full Karto replay'}
(p/'rays.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
