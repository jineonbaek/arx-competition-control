"""Camera files -> annotated files. No ROS, CAN, or HTTP control imports."""
import json,time,os,sys,traceback
from pathlib import Path
import cv2
sys.path.insert(0,'/home/arx/arx-competition-control')
from overlay import annotate,label
from depth_direction import direction
ROOT=Path('/home/arx/arx-competition-control/web/public');OUT=ROOT/'camera-overlay';OUT.mkdir(exist_ok=True)
cv2.setNumThreads(1)
def write(path,data):
 temp=path.with_suffix(path.suffix+'.tmp');temp.write_bytes(data);os.replace(temp,path)
while True:
 for camera in ('d435','d405'):
  try:
   m=json.loads((ROOT/'camera-live'/(camera+'.json')).read_text());name=m['raw_image_file']
   if Path(name).name!=name or not name.startswith(camera+'-v-'):raise ValueError('Invalid frame path')
   raw=cv2.imread(str(ROOT/'camera-live'/name))
   if raw is None:raise ValueError('Image missing')
   img,r=annotate(raw[:,:,::-1],camera,captured_at=m['captured_at'],sequence=m['sequence']);age=time.time()-m['captured_at']
   if camera=='d435' and m.get('depth',{}).get('state')=='online':
    name=m['depth'].get('image_file','')
    if Path(name).name==name and name.startswith(camera+'-z-'):
     depth=cv2.imread(str(ROOT/'camera-live'/name),cv2.IMREAD_UNCHANGED)
     if depth is not None:direction(img,r,raw[:,:,::-1],depth,m)
   stale=m['state']!='online' or not 0<=age<=2.
   if stale:label(img,'STALE - DO NOT USE',(12,90),(0,0,255))
   r.update(captured_at=m['captured_at'],updated_at=time.time(),sequence=m['sequence'],stale=stale)
   ok,jpg=cv2.imencode('.jpg',img,[cv2.IMWRITE_JPEG_QUALITY,90]);assert ok
   write(OUT/(camera+'.jpg'),jpg.tobytes());write(OUT/(camera+'.json'),json.dumps(r,ensure_ascii=False).encode())
  except Exception as e:
   write(OUT/(camera+'.json'),json.dumps(dict(camera=camera,stale=True,error=str(e),updated_at=time.time())).encode())
 time.sleep(.25)
