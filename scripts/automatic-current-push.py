#!/usr/bin/env python3
"""Start one owned fixed-axis push, record feedback and both camera sequences."""
import http.cookiejar,json,time,urllib.request,uuid
from pathlib import Path

BASE='http://127.0.0.1:9983'
opener=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
def get(path):return json.load(opener.open(BASE+path,timeout=5))
opener.open(BASE+'/visual-docking.html',timeout=5).read()
epoch=get('/api/state')['command_epoch'];request=uuid.uuid4().hex
data=dict(op='start',request_id=request,expected_epoch=epoch)
response=json.load(opener.open(urllib.request.Request(BASE+'/api/current-axis-push',data=json.dumps(data).encode(),
    headers={'Origin':BASE,'Content-Type':'application/json'}),timeout=10))
run=Path(__file__).resolve().parents[1]/'reports'/response['run_id'];run.mkdir(exist_ok=True)
frames=[];index=0
while True:
    state=get('/api/current-axis-push')
    if state.get('request_id')!=request:raise RuntimeError('Push round changed')
    for camera in ('d405','d435'):
        try:
            meta=get('/camera-live/'+camera+'.json')
            if meta['state']=='online' and 0<=time.time()-meta['captured_at']<1.:
                name='%s-%04d.jpg'%(camera,index)
                (run/name).write_bytes(opener.open(BASE+'/camera-live/'+meta['image_file'],timeout=5).read())
                frames.append(dict(camera=camera,file=name,captured_at=meta['captured_at']))
        except Exception as error:frames.append(dict(camera=camera,error=str(error),time=time.time()))
    (run/'camera-sequence.json').write_text(json.dumps(frames,indent=2))
    print(json.dumps({k:state.get(k) for k in ('status','phase','measured_travel_mm','reason','control_retained')}),flush=True)
    if state['status']!='running':
        # Receipt cleanup is written after the terminal status.
        for _ in range(40):
            if 'ended_at' in state:break
            time.sleep(.1);state=get('/api/current-axis-push')
        print('REPORT',run,flush=True);break
    index+=1;time.sleep(.5)
