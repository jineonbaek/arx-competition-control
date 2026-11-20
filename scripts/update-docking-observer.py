from pathlib import Path
import json,time,hashlib,os,signal,subprocess,http.cookiejar,urllib.request
r=Path('/home/arx/arx-competition-control');w=Path('/home/arx/arx-place-hold-20261116')
expected={'web/src/visual_docking_live.py':'ee76468a3d2fa112ec4aaa5b0c43dab5e4df14ff428038bc3a4fe29fbf65cfbd',
          'web/src/docking_motion.py':'d57de3d0085147d9eb321c91f388f51834ba576970e9687a4e2277d979ea853a'}
op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()));op.open('http://127.0.0.1:9983/',timeout=3).read()
def get(p):return json.load(op.open('http://127.0.0.1:9983'+p,timeout=3))
def proc(pid):
    try:
        p=Path('/proc')/str(pid);s=p.joinpath('stat').read_text().rsplit(')',1)[1].split()
        if s[0]=='Z':return None
        return dict(pid=int(pid),start=s[19],args=p.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0'))
    except FileNotFoundError:return None
s=get('/api/state');assert not any(s[k] for k in ('motion_reserved','competition_reserved','chassis_active'))
task=get('/api/visual-docking')
prior=json.loads((w/'reports/native-trial-20261116-035823/result.json').read_text())['task']
assert prior['status']=='blocked' and prior.get('groups_moved')==[] and prior['released'] is False
assert task['status'] in ('blocked','idle') and not task.get('groups_moved') and task['released'] is False
for name,sha in expected.items():assert hashlib.sha256((r/name).read_bytes()).hexdigest()==sha
owners={i:proc(i) for i in (7272,7589,1359820,6087)};assert all(owners.values())
rows=[proc(p.name) for p in Path('/proc').glob('[0-9]*')];web=[v for v in rows if v and str(r/'web/src/server.py') in v['args']];assert len(web)==1
b=Path('/home/arx/arx-git-backups')/('docking-scene-fix-'+time.strftime('%Y%m%d-%H%M%S'));b.mkdir()
for name in expected:
    target=b/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes((r/name).read_bytes())
    compile((w/name).read_text(),str(w/name),'exec')
(b/'before.json').write_text(json.dumps(dict(task=task,state=s,owners=owners,web=web,expected=expected),indent=2))
assert proc(web[0]['pid'])==web[0];os.kill(web[0]['pid'],signal.SIGTERM)
for _ in range(100):
    if proc(web[0]['pid']) is None:break
    time.sleep(.1)
else:raise RuntimeError('Idle observer failed to exit')
def launch():
    env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    return subprocess.Popen(['bash',str(r/'web/start.sh')],cwd=r,env=env,stdin=subprocess.DEVNULL,stdout=open(b/'web.log','ab'),stderr=subprocess.STDOUT,start_new_session=True)
for name,sha in expected.items():
    assert hashlib.sha256((r/name).read_bytes()).hexdigest()==sha
    tmp=(r/name).with_suffix('.dock-tmp');tmp.write_bytes((w/name).read_bytes());os.replace(tmp,r/name)
child=launch();(r/'web/hardware/runtime/web.pid').write_text(str(child.pid)+'\n')
for _ in range(80):
    if child.poll() is not None:raise RuntimeError('Observer startup failed; preserve hardware, inspect log '+str(b))
    try:
        after=get('/api/state')
        if all(after['arms'][a]['fresh'] for a in ('left','right')) and after['body']['fresh']:break
    except OSError:pass
    time.sleep(.2)
else:raise RuntimeError('Fresh feedback unavailable')
assert {i:proc(i) for i in owners}==owners
op.open('http://127.0.0.1:9983/',timeout=3).read();assert get('/api/visual-docking')['status']=='idle'
result=dict(status='auxiliary_scene_binding_fixed',backup=str(b),web_pid=child.pid,hardware_unchanged=True,no_motion_commands=True,sha256={name:hashlib.sha256((r/name).read_bytes()).hexdigest() for name in expected})
(w/'reports/native-scene-fix-deployment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
