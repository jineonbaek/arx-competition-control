"""Read-only D435 body pose using the prewarmed CAD/FoundationPose worker."""
from pathlib import Path
import sys,json,time,uuid,shutil,cv2,numpy as np
r=Path('/home/arx/arx-competition-control');run=Path('/home/arx/arx-place-hold-20261116/reports/d435-visual-level-active')
sys.path[:0]=[str(r/'web/src'),str(r)]
from handeye_column import ColumnModel
from agent.storage import atomic_json
from agent.docking_fit import OBJECT_TO_TOOL
out=run/('estimate-'+str(time.time_ns()));out.mkdir()
frame=out/'frame.npz';shutil.copy2(r/'web/hardware/runtime/grasp-head/frame.npz',frame)
with np.load(frame,allow_pickle=False) as f:d={k:f[k] for k in f.files}
assert str(d['camera_id'])=='d435';rgb=d['rgb'];k=json.loads(str(d['intrinsics']))
model=ColumnModel();tcp=np.array(json.loads((r/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
contact=np.array(json.loads((r/'web/public/cad/battery-grip.json').read_text())['points_object_mm']['tip'])/1000
T=model.wrist_transform(d['joints'],float(d['lift_rad']))@tcp
obj=np.eye(4);obj[:3,:3]=T[:3,:3]@OBJECT_TO_TOOL;obj[:3,3]=T[:3,3]-obj[:3,:3]@contact
cam=np.linalg.inv(d['base_from_color'])@obj
b=np.array(json.loads((r/'config/battery-envelope.json').read_text())['bounds_mm'])/1000
corners=np.array([[x,y,z] for x in b[:,0] for y in b[:,1] for z in b[:,2]])
K=np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1.]])
pix=cv2.projectPoints(corners,cv2.Rodrigues(cam[:3,:3])[0],cam[:3,3],K,np.array(k['coeffs']))[0].reshape(-1,2)
h,w=rgb.shape[:2];low=np.maximum([1,1],np.floor(pix.min(0)-8)).astype(int);high=np.minimum([w-1,h-1],np.ceil(pix.max(0)+8)).astype(int)
roi=[*map(int,low),*map(int,high)];x,y,rx,by=roi
if min(rx-x,by-y)<12:raise ValueError('Held battery outside D435 view')
hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);black=np.zeros((h,w),np.uint8);black[y:by,x:rx]=(hsv[y:by,x:rx,2]<75).astype(np.uint8)
n,labels,stats,centers=cv2.connectedComponentsWithStats(black,8);target=pix.mean(0)
choices=[i for i in range(1,n) if stats[i,cv2.CC_STAT_AREA]>250]
if not choices:raise ValueError('No visible held battery body')
chosen=min(choices,key=lambda i:np.linalg.norm(centers[i]-target));mask=(labels==chosen).astype(np.uint8)*255
contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(mask,contours,-1,255,-1)
ys,xs=np.nonzero(mask);roi=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)]
cv2.imwrite(str(out/'rgb.png'),rgb[:,:,::-1]);cv2.imwrite(str(out/'mask.png'),mask)
vis=rgb.copy();vis[mask>0]=(vis[mask>0]*.5+np.array([0,255,100])*.5).astype(np.uint8);cv2.imwrite(str(out/'mask-overlay.jpg'),vis[:,:,::-1])
request=uuid.uuid4().hex;directory=run/'perception'
args=dict(frame=str(frame),output=str(out/'pose.json'),mask=str(out/'mask.png'),id=str(d['id']),roi=roi,head_frame=None,head_mask=None,head_id=None)
seed_file=run/'pose-seed.json'
if seed_file.exists():
 seed=json.loads(seed_file.read_text())
 if seed['frame_id']==str(d['id']):args['seed_base_from_object']=seed['base_from_object']
atomic_json(directory/'request.json',dict(id=request,operation='estimate',arguments=args))
deadline=time.monotonic()+60
while not (directory/(request+'.json')).exists():
 if time.monotonic()>deadline:raise TimeoutError('Body pose inference timeout')
 time.sleep(.05)
reply=json.loads((directory/(request+'.json')).read_text());print(json.dumps(dict(directory=str(out),reply=reply)),flush=True)
if reply['ok']:
 p=json.loads((out/'pose.json').read_text());print(json.dumps({k:p.get(k) for k in ('quality','registration_s','total_s','rpy_base_deg','center_base_mm','estimation_method')}),flush=True)
 (run/'latest-pose-path.txt').write_text(str(out/'pose.json'))
