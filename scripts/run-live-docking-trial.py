"""One supervised no-release task request; monitor identity, feedback and cameras."""
from pathlib import Path
import json,time,urllib.request,http.cookiejar
base='http://127.0.0.1:9983';root=Path('/home/arx/arx-place-hold-20261116/reports')
out=root/('native-trial-'+time.strftime('%Y%m%d-%H%M%S'));out.mkdir()
op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()));op.open(base+'/',timeout=3).read()
def get(path):return json.load(op.open(base+path,timeout=3))
def post(data):
    return json.load(op.open(urllib.request.Request(base+'/api/visual-docking',json.dumps(data).encode(),{'Origin':base,'Content-Type':'application/json'}),timeout=5))
def pictures(tag):
    for camera in ('d435','d405'):
        m=get('/camera-live/'+camera+'.json');(out/(tag+'-'+camera+'.json')).write_text(json.dumps(m))
        (out/(tag+'-'+camera+'.jpg')).write_bytes(op.open(base+'/camera-live/'+m['image_file'],timeout=3).read())
before=get('/api/state');assert all(before[k] is False for k in ('motion_reserved','competition_reserved','chassis_active'))
(out/'before.json').write_text(json.dumps(before,indent=2));pictures('before')
reply=post(dict(op='start',model='demo'));run=reply['run_id'];(out/'start.json').write_text(json.dumps(reply,indent=2))
print('START',json.dumps(dict(run_id=run,directory=str(out))),flush=True)
rows=[];last_phase=None;last_log=0;last_camera=0;error=None;deadline=time.monotonic()+900
try:
    while time.monotonic()<deadline:
        task=get('/api/visual-docking');state=get('/api/state')
        if task.get('run_id')!=run:raise RuntimeError('Task identity changed')
        if task.get('released') is not False:raise RuntimeError('No-release invariant lost')
        rows.append(dict(time=time.time(),task=task,state=state))
        phase=task['phase']
        if phase!=last_phase:
            print('PHASE',phase,task['status'],task.get('reason',''),flush=True);last_phase=phase
            pictures('phase-'+phase)
        if task['status'] in ('blocked','recovery_required','held_outside','cancelled'):break
        if not all(state['arms'][s]['fresh'] for s in ('left','right')) or not state['body']['fresh']:
            raise RuntimeError('Robot feedback stale')
        if time.monotonic()-last_camera>1:
            for camera in ('d435','d405'):
                m=get('/camera-live/'+camera+'.json')
                if m['state']!='online' or not -.1<=time.time()-m['captured_at']<1.:
                    raise RuntimeError(camera+' camera stale')
            last_camera=time.monotonic()
        if time.monotonic()-last_log>10:
            print('PROGRESS',phase,'gripper',state['arms']['right']['gripper'],'lift',state['body']['height'],flush=True);last_log=time.monotonic()
        time.sleep(.12)
    else:raise RuntimeError('Trial deadline exceeded')
except Exception as e:
    error=repr(e);print('MONITOR_ABORT',error,flush=True)
    try:(out/'cancel.json').write_text(json.dumps(post(dict(op='cancel'))))
    except Exception as cancel_error:(out/'cancel-error.txt').write_text(repr(cancel_error))
    task=get('/api/visual-docking');state=get('/api/state')
finally:
    (out/'observations.json').write_text(json.dumps(rows));(out/'result.json').write_text(json.dumps(dict(task=task,state=state,monitor_error=error),ensure_ascii=False,indent=2))
    try:pictures('after')
    except Exception as e:(out/'image-error.txt').write_text(repr(e))
    print('RESULT',json.dumps(dict(directory=str(out),status=task['status'],phase=task['phase'],reason=task.get('reason'),groups_moved=task.get('groups_moved'),released=task.get('released'),monitor_error=error),ensure_ascii=False),flush=True)
