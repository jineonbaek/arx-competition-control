"""Read only independent exposures: pose jitter, approach distance and TCP error."""
from pathlib import Path
import json,sys,time,urllib.request,numpy as np
root=Path('/home/arx/arx-place-hold-20261116');sys.path[:0]=[str(root/'web/src'),str(root)]
from cabinet_preview import CabinetPreview
from handeye_column import ColumnModel
from agent.docking_fit import evaluate_fit
from agent.cabinet_preview_geometry import horizontal_plan,opening
class Observer:
 def __init__(self):self.models={s:ColumnModel(side=s) for s in ('left','right')};self.token=object()
 def snapshot(self):return json.load(urllib.request.urlopen('http://127.0.0.1:9983/api/state',timeout=3))
 def motion_valid(self,t):return t is self.token
ctrl=Observer();preview=CabinetPreview(ctrl,None,root=root);rows=[]
tcp=np.array(json.loads((root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
for _ in range(10):
 r=preview.action(dict(camera='d435',model='demo'),include_arm_preview=False,token=ctrl.token)
 s=ctrl.snapshot();T=ctrl.models['right'].wrist_transform(s['arms']['right']['joints'],s['body']['height'])@tcp
 row={k:r.get(k) for k in ('status','reason','image_time','window','table_estimate')}
 if 'window' in r:
  slot=opening(preview.profiles,'demo');contact=preview.grip['points_object_mm']['tip'];fit=evaluate_fit(T,r['window'],slot,preview.battery,contact)
  plan=horizontal_plan(r['window'],slot,preview.battery,tip_point_object_mm=contact,preclearance_mm=55.,max_insertion_depth_mm=5.)
  row.update(fit=fit,tcp_error_mm=(np.array(plan['tcp_pre_base_mm'])-T[:3,3]*1000).tolist(),joints=s['arms']['right']['joints'])
 rows.append(row)
p=root/'reports/cabinet-static-noise.json';p.write_text(json.dumps(rows,indent=2))
print(json.dumps([dict(status=r['status'],center=r.get('window',{}).get('center_base_m'),quad=r.get('window',{}).get('inner_quad_px'),table_top=r.get('table_estimate',{}).get('top_m'),distance=r.get('fit',{}).get('leading_standoff_mm'),tcp_error_mm=r.get('tcp_error_mm')) for r in rows]),flush=True)
