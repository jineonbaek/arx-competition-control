"""Offline diagnostic analysis, not a chassis fusion estimator."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import savgol_filter,butter,sosfiltfilt
from relative import interpolate,world_alignment,relative_pose
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--cad');a=p.parse_args();root=Path(a.run)
def load(name):return json.loads((root/'trajectories'/(name+'.json')).read_text())
def pose_arrays(rows):return (np.array([r['t'] for r in rows],np.int64),np.array([r['p'] for r in rows]),np.array([r['q'] for r in rows]))
def mean_transform(mats):
 out=np.eye(4);out[:3,3]=mats[:,:3,3].mean(axis=0);out[:3,:3]=Rotation.from_matrix(mats[:,:3,:3]).mean().as_matrix();return out
b=pose_arrays(load('bottom_odom'));h=pose_arrays(load('top_odom'));times=b[0][(b[0]>=h[0][0])&(b[0]<=h[0][-1])];sec=(times-times[0])/1e9
B=np.array([interpolate(*b,int(t)) for t in times]);H=np.array([interpolate(*h,int(t)) for t in times]);initial=sec<10;final=sec>sec[-1]-10
# Estimated static LiDAR relation is ONLY a gauge prior at initialization, never used to merge moving scans.
E=np.eye(4);E[:3,3]=[-.011,-.02329,.04412]
L=np.eye(4);L[:3,:3]=[[.9999599541855795,-.006173661982418676,.006478882843571455],[.006240449010145331,.9999270748162661,-.010339334869211488],[-.006414578811244147,.010379351960154099,.999925558345001]];L[:3,3]=[.1756720301648963,-.008414072166182663,.9505154950673719]
prior=np.linalg.inv(E@L@np.linalg.inv(E))
if a.cad:
 cad=json.loads(Path(a.cad).read_text());AB=np.eye(4);AH=np.eye(4);AB[:3,:3]=AH[:3,:3]=np.diag([1.,-1.,-1.]);AB[:3,3]=cad['bottom']['imu_origin_m'];AH[:3,3]=cad['top']['imu_origin_m'];prior=np.linalg.inv(AB)@AH
G=world_alignment(mean_transform(B[initial]),mean_transform(H[initial]),prior)
rels=np.array([relative_pose(x,y,G) for x,y in zip(B,H)]);r0=mean_transform(rels[initial]);dp=rels[:,:3,3]-r0[:3,3];dr=Rotation.from_matrix(np.array([r0[:3,:3].T@x[:3,:3] for x in rels])).as_rotvec()*180/np.pi
result={'diagnostic_only':True,'dynamic_extrinsic_validated':False,'fusion_enabled':False,'time_alignment':'same-time position interpolation and SLERP, no extrapolation, maximum gap 150ms','initial_alignment':('first 10 seconds mean plus user-confirmed CAD IMU relation as gauge prior only' if a.cad else 'first 10 seconds mean plus old estimated static LiDAR relation as gauge prior only'),'sensors':{}}
for name,M in [('top',H),('bottom',B)]:
 pp=M[:,:3,3];smooth=savgol_filter(pp,11,2,axis=0);vel=np.gradient(smooth,sec,axis=0);rr=Rotation.from_matrix(np.array([M[0,:3,:3].T@x[:3,:3] for x in M])).as_rotvec()*180/np.pi
 result['sensors'][name]={'duration_s':float(sec[-1]),'samples':len(times),'path_length_smoothed_m':float(np.linalg.norm(np.diff(smooth,axis=0),axis=1).sum()),'max_distance_from_initial_m':float(np.linalg.norm(pp-pp[initial].mean(axis=0),axis=1).max()),'end_minus_start_m':(pp[final].mean(axis=0)-pp[initial].mean(axis=0)).tolist(),'endpoint_distance_m':float(np.linalg.norm(pp[final].mean(axis=0)-pp[initial].mean(axis=0))),'max_speed_smoothed_mps':float(np.linalg.norm(vel,axis=1).max()),'max_rotation_from_first_deg':float(np.linalg.norm(rr,axis=1).max()),'initial_position_span_mm':(np.ptp(pp[initial],axis=0)*1000).tolist(),'final_position_span_mm':(np.ptp(pp[final],axis=0)*1000).tolist()}
result['relative_candidate']={'translation_span_mm':(np.ptp(dp,axis=0)*1000).tolist(),'max_translation_change_mm':float(np.linalg.norm(dp,axis=1).max()*1000),'rotation_vector_span_deg':np.ptp(dr,axis=0).tolist(),'max_rotation_change_deg':float(np.linalg.norm(dr,axis=1).max()),'end_minus_start_translation_mm':(dp[final].mean(axis=0)*1000).tolist(),'end_rotation_vector_deg':dr[final].mean(axis=0).tolist(),'initial_translation_span_mm':(np.ptp(dp[initial],axis=0)*1000).tolist(),'final_translation_span_mm':(np.ptp(dp[final],axis=0)*1000).tolist(),'caveat':'Includes independent LIO drift, initial alignment error and real flex; not ground truth dynamic extrinsic.'}
# IMU differential only rotation: initial rigid rotation is approximate and dynamic changes are not corrected here.
hr=load('top_imu');br=load('bottom_imu');tb=np.array([x['t'] for x in br],np.int64);th=np.array([x['t'] for x in hr],np.int64);start=max(tb[0],th[0]);end=min(tb[-1],th[-1]);grid=np.arange(start,end,5000000,dtype=np.int64);s=(grid-start)/1e9
wb=np.column_stack([np.interp((grid-start)/1e9,(tb-start)/1e9,[x['w'][j] for x in br]) for j in range(3)]);wh=np.column_stack([np.interp((grid-start)/1e9,(th-start)/1e9,[x['w'][j] for x in hr]) for j in range(3)])@prior[:3,:3].T
wb-=np.mean(wb[s<10],axis=0);wh-=np.mean(wh[s<10],axis=0);diff=wh-wb
hp=butter(3,2,fs=200,btype='highpass',output='sos');bh=sosfiltfilt(hp,wb,axis=0);hh=sosfiltfilt(hp,wh,axis=0)
result['gyro_diagnostic']={'units':'rad/s','differential_rms_per_axis':np.sqrt(np.mean(diff**2,axis=0)).tolist(),'differential_peak_norm':float(np.linalg.norm(diff,axis=1).max()),'head_highpass_2hz_rms_norm':float(np.sqrt(np.mean(np.sum(hh**2,axis=1)))),'bottom_highpass_2hz_rms_norm':float(np.sqrt(np.mean(np.sum(bh**2,axis=1)))),'caveat':'High-frequency gyro includes sensor noise as well as flex; static noise baseline must be compared.'}
np.savez_compressed(root/'relative-diagnostic.npz',time_ns=times,seconds=sec,relative=rels,translation_change_m=dp,rotation_change_deg=dr,world_alignment=G)
(root/'motion-analysis.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
