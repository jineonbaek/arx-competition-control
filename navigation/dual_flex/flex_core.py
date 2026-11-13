"""Conservative offline loosely coupled flexible-head smoother. Not an online LIO."""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

def cad_pose(position,rpy=(np.pi,0.,0.)):
 T=np.eye(4);T[:3,:3]=Rotation.from_euler("xyz",rpy).as_matrix();T[:3,3]=position;return T

def installation(run):
 import json
 from pathlib import Path
 config=run/'geometry.json'
 if not config.exists():
  config=Path(__file__).resolve().parents[1]/'config/cad_nominal_20261105.json'
 cad=json.loads(config.read_text())
 return tuple(cad_pose(cad[n]['imu_origin_m'],cad[n]['rpy_rad']) for n in ('bottom','top'))

def body_mask(points, sensor_in_body, padding=.02):
 p=points@sensor_in_body[:3,:3].T+sensor_in_body[:3,3]
 lower=np.array([-.401657,-.380085,.03])-padding;upper=np.array([.422010,.380169,1.268])+padding
 return np.all((p>=lower)&(p<=upper),axis=1)

def information_projector(information,threshold=.02):
 e,v=np.linalg.eigh((information+information.T)*.5);cut=max(float(e.max()),1e-12)*threshold;keep=e>cut
 weights=np.where(keep,np.sqrt(np.maximum(e,0)/max(float(e.max()),1e-12)),0)
 return np.diag(weights)@v.T,int(keep.sum())

def solve(t,baseline,head_measurements,scenes,A_bottom,A_head,strength=1.,max_nfev=30):
 t=np.asarray(t);n=len(t);dt=float(np.median(np.diff(t)));B0=Rotation.from_matrix(baseline[:,:3,:3]);b0=baseline[:,:3,3];Rcad=Rotation.from_matrix(A_head[:3,:3]);pcad=A_head[:3,3]
 Hm=Rotation.from_matrix(head_measurements[:,:3,:3]);hp=head_measurements[:,:3,3];dRm=Hm[:-1].inv()*Hm[1:];dpm=Hm[:-1].inv().apply(np.diff(hp,axis=0))
 init=np.zeros((n,12));D0=np.linalg.inv(baseline)@head_measurements;init[:,6:9]=(Rotation.from_matrix(D0[:,:3,:3])*Rcad.inv()).as_rotvec();init[:,9:12]=D0[:,:3,3]-pcad
 accepted=[]
 nominal=np.linalg.inv(A_head)@A_bottom
 for s in scenes:
  if not 0<=s['index']<n or not np.isfinite(s['T']).all() or not np.isfinite(s['information']).all():continue
  innovation=np.asarray(s['T'])@np.linalg.inv(nominal)
  # Gross-outlier guard only; these are engineering trial limits, not calibrated probabilities.
  if np.linalg.norm(innovation[:3,3])>.25 or Rotation.from_matrix(innovation[:3,:3]).magnitude()>.35:continue
  W,rank=information_projector(np.asarray(s['information']));
  if rank>=3 and np.isfinite(s['T']).all():
   time_s=float(s.get('time',t[s['index']]))
   if not t[0]<=time_s<=t[-1]:continue
   left=min(max(int(np.searchsorted(t,time_s,side='right'))-1,0),n-2);alpha=(time_s-t[left])/(t[left+1]-t[left]);accepted.append((s,W,rank,left,alpha,time_s))
 def decode(x):
  x=x.reshape(n,12);Rb=Rotation.from_rotvec(x[:,:3])*B0;pb=b0+x[:,3:6];Rd=Rotation.from_rotvec(x[:,6:9])*Rcad;pd=pcad+x[:,9:12];Rh=Rb*Rd;ph=pb+Rb.apply(pd);return x,Rb,pb,Rd,pd,Rh,ph
 if not accepted:return {'output':baseline.copy(),'candidate':baseline.copy(),'relative':D0,'fallback':np.ones(n,bool),'success':True,'status':'no_accepted_scene_constraints','accepted_scenes':0,'cost':0.,'nfev':0,'scene_postfit':[]}
 sigB=np.array([.01]*3+[.015]*3);sigH=np.array([.004]*3+[.008]*3);sigAmp=np.array([.15]*3+[.15]*3);sigRate=np.array([.8]*3+[.3]*3);sigAcc=np.array([2.]*3+[1.]*3)
 def scene_error(x,scene):
  s,W,rank,left,alpha,time_s=scene;f=(1-alpha)*x[left,6:12]+alpha*x[left+1,6:12];D=np.eye(4);D[:3,:3]=(Rotation.from_rotvec(f[:3])*Rcad).as_matrix();D[:3,3]=pcad+f[3:];T=np.linalg.inv(D)@A_bottom;delta=T@np.linalg.inv(s['T']);e=np.r_[Rotation.from_matrix(delta[:3,:3]).as_rotvec(),delta[:3,3]]
  return W@e/max(.005,s['sigma'])
 def residual(flat):
  x,Rb,pb,Rd,pd,Rh,ph=decode(flat);rh=(dRm.inv()*(Rh[:-1].inv()*Rh[1:])).as_rotvec();rp=Rh[:-1].inv().apply(np.diff(ph,axis=0))-dpm;f=x[:,6:12]
  parts=[(x[:,:6]/sigB).ravel(),(np.c_[rh,rp]/sigH).ravel(),(f/sigAmp*.1).ravel(),(np.diff(f,axis=0)/dt/sigRate*strength).ravel(),(np.diff(f,n=2,axis=0)/dt**2/sigAcc*strength).ravel(),f[0]/np.array([.01745]*3+[.02]*3)]
  for scene in accepted:parts.append(scene_error(x,scene))
  return np.concatenate(parts)
 count=6*n+6*(n-1)+6*n+6*(n-1)+6*(n-2)+6+6*len(accepted);sparsity=lil_matrix((count,12*n),dtype=int);row=0
 for nodes,length in [([(i,) for i in range(n)],6),([(i,i+1) for i in range(n-1)],6),([(i,) for i in range(n)],6),([(i,i+1) for i in range(n-1)],6),([(i,i+1,i+2) for i in range(n-2)],6),([(0,)],6),([(left,left+1) for _,_,_,left,_,_ in accepted],6)]:
  for ids in nodes:
   for j in ids:sparsity[row:row+length,12*j:12*j+12]=1
   row+=length
 opt=least_squares(residual,init.ravel(),jac_sparsity=sparsity.tocsr(),loss='soft_l1',f_scale=1.,max_nfev=max_nfev,ftol=1e-4,xtol=1e-5,gtol=1e-4)
 x,Rb,pb,Rd,pd,_,_=decode(opt.x);candidate=np.tile(np.eye(4),(n,1,1));candidate[:,:3,:3]=Rb.as_matrix();candidate[:,:3,3]=pb;relative=np.tile(np.eye(4),(n,1,1));relative[:,:3,:3]=Rd.as_matrix();relative[:,:3,3]=pd
 trusted=np.zeros(n,bool)
 postfit=[]
 for scene in accepted:
  s,W,rank,left,alpha,time_s=scene;norm=float(np.linalg.norm(scene_error(x,scene)));passed=norm<=3.*np.sqrt(rank);postfit.append({'evaluation_time':time_s,'left_node':left,'alpha':float(alpha),'rank':rank,'normalized_residual_norm':norm,'consistent':bool(passed)})
  if rank==6 and passed:trusted|=np.abs(t-time_s)<=.6
 trusted &= (np.linalg.norm(x[:,:3],axis=1)<.05)&(np.linalg.norm(x[:,3:6],axis=1)<.05)
 # Non-convergence or no nearby full-rank scene must not be represented as valid fusion.
 if not opt.success:trusted[:]=False
 output=baseline.copy();output[trusted]=candidate[trusted]
 return {'output':output,'candidate':candidate,'relative':relative,'fallback':~trusted,'success':bool(opt.success),'status':str(opt.message),'cost':float(opt.cost),'nfev':int(opt.nfev),'accepted_scenes':len(accepted),'scene_postfit':postfit}
