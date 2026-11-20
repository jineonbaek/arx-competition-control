"""Read-only D435 body pose using the prewarmed CAD/FoundationPose worker."""
from pathlib import Path
import sys,json,time,uuid,shutil,cv2,numpy as np
r=Path(__file__).resolve().parents[1];run=r/'reports/d435-visual-level-active'
sys.path[:0]=[str(r/'web/src'),str(r)]
from handeye_column import ColumnModel
from agent.storage import atomic_json
from agent.docking_fit import OBJECT_TO_TOOL
from agent.held_d435_mask import held_battery_mask
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
mask,roi=held_battery_mask(rgb,d['xyz'],d['base_from_color'],obj,b*1000,
    projected_center_px=pix.mean(0),min_area=250,margin_mm=10.,saturation_max=50,value_max=160)
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
