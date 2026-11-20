"""No hardware writes: record stopped frames and inspect fixed-reference matches."""
from pathlib import Path
import sys,json,time,cv2,numpy as np
root=Path('/home/arx/arx-place-hold-20261116');sys.path.insert(0,str(root))
from agent.docking_grip import GripTracker
p=root/'reports/d405-roll-diagnostics';p.mkdir(exist_ok=True)
camera=Path('/home/arx/arx-competition-control/web/public/camera-live');last=-1
for i in range(15):
 for _ in range(100):
  m=json.loads((camera/'d405.json').read_text())
  if m['captured_at']>last:break
  time.sleep(.02)
 last=m['captured_at'];(p/f'frame-{i}.jpg').write_bytes((camera/m['raw_image_file']).read_bytes());time.sleep(.1)
a=cv2.imread(str(root/'reports/native-trial-20261116-042805/before-d405.jpg'))[:,:,::-1]
cv2.imwrite(str(p/'reference.jpg'),a[:,:,::-1]);t=GripTracker();t.bind(a,1.,.8747,now=1.)
ref=t.reference;rows=[]
for i in range(15):
 b=cv2.imread(str(p/f'frame-{i}.jpg'))[:,:,::-1];gray=cv2.cvtColor(b,cv2.COLOR_RGB2GRAY)
 cur,ok,_=cv2.calcOpticalFlowPyrLK(ref['gray'],gray,ref['points'],None,winSize=(21,21),maxLevel=3)
 back,bok,_=cv2.calcOpticalFlowPyrLK(gray,ref['gray'],cur,None,winSize=(21,21),maxLevel=3)
 valid=ok.ravel().astype(bool)&bok.ravel().astype(bool)&(np.linalg.norm(back-ref['points'],axis=2).ravel()<1.)
 mat,inliers=cv2.estimateAffinePartial2D(ref['points'][valid],cur[valid],method=cv2.RANSAC,ransacReprojThreshold=2.)
 rows.append(dict(frame=i,tracking=t.update(b,1.+i*.1,.8747,now=1.+i*.1),
   points=ref['points'].reshape(-1,2).tolist(),current=cur.reshape(-1,2).tolist(),valid=valid.tolist(),
   inlier_ratio=float(inliers.mean()) if inliers is not None else 0.,inliers=inliers.ravel().tolist() if inliers is not None else []))
 # Annotate reference support to distinguish battery interior from moving background.
 drawn=a.copy()
 for point in ref['points'].reshape(-1,2):cv2.circle(drawn,tuple(point.astype(int)),3,(255,0,0),1)
 cv2.imwrite(str(p/'reference-points.jpg'),drawn[:,:,::-1])
(p/'diagnostics.json').write_text(json.dumps(rows,indent=2));print(json.dumps([dict(frame=v['frame'],ratio=v['inlier_ratio'],tracking=v['tracking']) for v in rows]),flush=True)
