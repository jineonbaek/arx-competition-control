"""Replace the stopped web observer; never signal hardware drivers."""
from pathlib import Path
import json, time, os, signal, subprocess, hashlib, urllib.request, http.cookiejar
import numpy as np
R=Path('/home/arx/arx-competition-control'); W=Path('/home/arx/arx-place-hold-20261116')
B='http://127.0.0.1:9983'; op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
op.open(B+'/',timeout=4).read()
def get(p):return json.load(op.open(B+p,timeout=4))
def proc(pid):
    try:
        p=Path('/proc')/str(pid);stat=p.joinpath('stat').read_text().rsplit(')',1)[1].split()
        if stat[0]=='Z':return None
        return (stat[19],p.joinpath('cmdline').read_bytes().decode().split('\0'))
    except OSError:return None
owners={}
for p in Path('/proc').glob('[0-9]*'):
    v=proc(p.name)
    if v and any(a.endswith(('/x5_controller_web','/lift_controller_web')) for a in v[1]):owners[int(p.name)]=v
assert len(owners)==3
web=[(int(p.name),proc(p.name)) for p in Path('/proc').glob('[0-9]*')]
web=[v for v in web if v[1] and str(R/'web/src/server.py') in v[1][1]]
assert len(web)==1
task=get('/api/fast-level')
assert task['status']=='held' and task['reason']=='红框前沿深度不符合单一平面' and task['released'] is False and task['control_retained']
rows=[]
for _ in range(12):
    s=get('/api/state');assert all(s['arms'][a]['fresh'] for a in ('left','right')) and s['body']['fresh'] and not s['chassis_active']
    rows.append(s);time.sleep(.1)
assert np.max(np.ptp([s['arms']['right']['joints'] for s in rows],axis=0))<.002
assert np.ptp([s['arms']['right']['gripper'] for s in rows])<.002
assert np.ptp([s['body']['height'] for s in rows])<.002
files={'web/src/fast_visual_level.py':'a49fdfe02ee8a20cb60c053668dd90f6a5ee2ed7767866b239eee83dd913ba13',
       'agent/held_placement_geometry.py':'44fcfb442099535556403e2558ea0e19c14c70b456a61a9544127fde68952610'}
for f,h in files.items():
    assert hashlib.sha256((R/f).read_bytes()).hexdigest()==h
    compile((W/f).read_text(),str(W/f),'exec')
backup=Path('/home/arx/arx-git-backups')/('red-plane-retry-'+time.strftime('%Y%m%d-%H%M%S'));backup.mkdir()
for f in files:
    p=backup/f;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((R/f).read_bytes())
(backup/'before.json').write_text(json.dumps(dict(task=task,state=rows[-1],owners=owners,web=web)))
assert proc(web[0][0])==web[0][1]
os.kill(web[0][0],signal.SIGTERM)
for _ in range(100):
    if proc(web[0][0]) is None:break
    time.sleep(.1)
else:raise RuntimeError('Observer did not exit')
assert {p:proc(p) for p in owners}==owners
for f,h in files.items():
    assert hashlib.sha256((R/f).read_bytes()).hexdigest()==h
    temp=(R/f).with_suffix('.recenter-new');temp.write_bytes((W/f).read_bytes());os.replace(temp,R/f)
env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
child=subprocess.Popen(['bash',str(R/'web/start.sh')],cwd=R,env=env,stdin=subprocess.DEVNULL,stdout=open(backup/'web.log','ab'),stderr=subprocess.STDOUT,start_new_session=True)
(R/'web/hardware/runtime/web.pid').write_text(str(child.pid)+'\n')
for _ in range(100):
    if child.poll() is not None:raise RuntimeError('Observer exited')
    try:
        op.open(B+'/',timeout=2).read();after=get('/api/state')
        if all(after['arms'][a]['fresh'] for a in ('left','right')) and after['body']['fresh']:break
    except OSError:pass
    time.sleep(.2)
else:raise RuntimeError('Feedback unavailable')
assert {p:proc(p) for p in owners}==owners
assert abs(after['arms']['right']['gripper']-rows[-1]['arms']['right']['gripper'])<.02
result=dict(backup=str(backup),web_pid=child.pid,motor_drivers_unchanged=True,no_motion_commands=True,files={f:hashlib.sha256((R/f).read_bytes()).hexdigest() for f in files})
(W/'reports/recenter-deployment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
