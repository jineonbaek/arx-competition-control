from pathlib import Path
import os,re,json,hashlib
out=Path('/home/arx/arx-competition-control/reports')
skip={'.git','build','install','log','node_modules','datasets','39999','xv','realsense','robomimic','checkpoints','__pycache__'}
pat=re.compile(r'41\.54|gripper.{0,30}(width|mm|scale|offset|open|close)|(?:width|mm|scale|offset).{0,30}gripper|lift.{0,25}(zero|offset|height)|(?:zero|offset).{0,25}lift|零位|零点|夹爪|setZero|zero_offset|home_position|angle_offset',re.I)
hits=[]; manifest=[]
for root in ['/home/arx/LIFT','/home/arx/ROS2_LIFT_Play','/home/arx/docs_lift2','/home/arx/arx-competition-control/repos']:
 for base,ds,fs in os.walk(root):
  ds[:]=[d for d in ds if d not in skip and not d.startswith('.venv')]
  for f in fs:
   p=Path(base)/f
   if not p.is_file():continue
   if p.suffix.lower() not in {'.py','.cpp','.h','.hpp','.yaml','.yml','.json','.xml','.urdf','.xacro','.md','.txt','.sh'} or p.stat().st_size>1000000:continue
   try:t=p.read_text()
   except:continue
   selected=[{'line':i,'text':l[:400]} for i,l in enumerate(t.splitlines(),1) if pat.search(l)]
   if selected:
    hits.append({'path':str(p),'hits':selected});manifest.append({'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(out/'calibration-source-search.json').write_text(json.dumps(hits,ensure_ascii=False,indent=2))
(out/'calibration-source-manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
for h in hits:
 if '/repos/' not in h['path']:print(h['path'],len(h['hits']))
print('FILES',len(hits),'HITS',sum(len(x['hits']) for x in hits))
