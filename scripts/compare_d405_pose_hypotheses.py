"""Compare frozen D405 battery depth to two frozen D435 pose hypotheses; read only."""
import numpy as np,json,cv2,trimesh
from pathlib import Path
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
R=Path('/home/arx/arx-competition-control');W=Path('/home/arx/arx-place-hold-20261116')
with np.load(R/'web/hardware/runtime/grasp-preview/frame.npz',allow_pickle=False) as z:d={k:z[k] for k in z.files}
rgb=d['rgb'];xyz=d['xyz'];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
sel=(hsv[:,:,2]<100)&(xyz[:,:,2]>.06)&(xyz[:,:,2]<.8)
roi=np.zeros(sel.shape,bool);roi[80:200,190:470]=True;sel &= roi
points=xyz[sel]
if len(points)<200:raise ValueError('D405 valid battery depth insufficient: %d'%len(points))
B=d['base_from_color'];world=points@B[:3,:3].T+B[:3,3]
mesh=trimesh.load(str(R/'web/cad/gripper-v2/battery-print.stl'),force='mesh');np.random.seed(42)
tree=cKDTree(mesh.sample(120000)/1000)
poses=[('register',W/'reports/d435-visual-level-active/estimate-1794791174226657939/pose-3d-test.json'),
       ('track',W/'reports/d435-visual-level-active/estimate-1794791345369491507/pose.json')]
results=[]
for name,path in poses:
    T=np.array(json.loads(path.read_text())['T_base_from_object']);local=(world-T[:3,3])@T[:3,:3]
    dist=tree.query(local,workers=1)[0]*1000
    subset=local[::max(1,len(local)//2000)]
    fit=least_squares(lambda delta:tree.query(subset+delta,workers=1)[0],np.zeros(3),bounds=(-.05,.05),loss='soft_l1',f_scale=.005,max_nfev=40,diff_step=1e-3)
    aligned=tree.query(local+fit.x,workers=1)[0]*1000
    results.append(dict(hypothesis=name,median_surface_mm=float(np.median(dist)),p90_surface_mm=float(np.quantile(dist,.9)),under_10mm=float(np.mean(dist<10)),diagnostic_translation_mm=(fit.x*1000).tolist(),aligned_median_mm=float(np.median(aligned)),aligned_p90_mm=float(np.quantile(aligned,.9))))
out=dict(frame_id=str(d['id']),samples=len(points),frame_time=float(d['image_time']),results=results,physical_accuracy_verified=False)
(W/'reports/d405-pose-hypothesis-comparison.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out),flush=True)
