from pathlib import Path
import json,time,hashlib,os,signal,subprocess,urllib.request,http.cookiejar
r=Path('/home/arx/arx-competition-control');w=Path('/home/arx/arx-place-hold-20261116')
expected={'web/src/server.py':'4e0f9dacedd3b7e63c6733deee421d46bac1db786d62214e844c74309fc74139',
 'foundationpose/infer_snapshot.py':'4cfae1dc33a9afad7731ff5d32c041346f696b50f739b2b90f335cb99568d629',
 'web/public/visual-docking.js':'855a1ea37cdb93d9b1930184bb220c5d05c126307af722cef0496320b3fe9f89',
 'web/public/visual-docking.html':'7600a9876875fbcaaff978e51b1b48b2f41f90e4873db29ffa73ad66823449fb',
 'web/src/fast_visual_level.py':None,'agent/visual_battery_level.py':None}
op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()));op.open('http://127.0.0.1:9983/').read()
def get(p):return json.load(op.open('http://127.0.0.1:9983'+p,timeout=3))
def proc(pid):
 try:
  p=Path('/proc')/str(pid);s=p.joinpath('stat').read_text().rsplit(')',1)[1].split()
  if s[0]=='Z':return None
  return dict(pid=int(pid),start=s[19],args=p.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0'))
 except FileNotFoundError:return None
s=get('/api/state');assert not any(s[k] for k in ['motion_reserved','competition_reserved','chassis_active'])
assert all(s['arms'][a]['fresh'] for a in ['left','right']) and s['body']['fresh']
assert s['arms']['right']['gripper']<-.5
owners={i:proc(i) for i in [5894,6048,6201]};assert all(owners.values());web=proc(5101);assert web and str(r/'web/src/server.py') in web['args']
for name,sha in expected.items():
 assert (hashlib.sha256((r/name).read_bytes()).hexdigest() if (r/name).exists() else None)==sha
 if name.endswith('.py'):compile((w/name).read_text(),str(w/name),'exec')
b=Path('/home/arx/arx-git-backups')/('fast-level-'+time.strftime('%Y%m%d-%H%M%S'));b.mkdir()
for name in expected:
 if (r/name).exists():
  p=b/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((r/name).read_bytes())
(b/'before.json').write_text(json.dumps(dict(state=s,owners=owners,web=web,expected=expected),indent=2))
(w/'reports/d435-visual-level-active/stop-model').touch()
assert proc(5101)==web;os.kill(5101,signal.SIGTERM)
for _ in range(100):
 if proc(5101) is None:break
 time.sleep(.1)
else:raise RuntimeError('Idle web observer did not exit')
for name,sha in expected.items():
 assert (hashlib.sha256((r/name).read_bytes()).hexdigest() if (r/name).exists() else None)==sha
 p=(r/name).with_suffix('.level-new');p.write_bytes((w/name).read_bytes());os.replace(p,r/name)
env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
child=subprocess.Popen(['bash',str(r/'web/start.sh')],cwd=r,env=env,stdin=subprocess.DEVNULL,stdout=open(b/'web.log','ab'),stderr=subprocess.STDOUT,start_new_session=True)
(r/'web/hardware/runtime/web.pid').write_text(str(child.pid)+'\n')
for _ in range(100):
 if child.poll() is not None:raise RuntimeError('Web observer startup failed: '+str(b))
 try:
  op.open('http://127.0.0.1:9983/',timeout=3).read();a=get('/api/state')
  if all(a['arms'][k]['fresh'] for k in ['left','right']) and a['body']['fresh']:break
 except OSError:pass
 time.sleep(.2)
else:raise RuntimeError('Fresh robot feedback not restored')
assert {i:proc(i) for i in owners}==owners
assert get('/api/fast-level')['status']=='idle'
result=dict(backup=str(b),web_pid=child.pid,motor_drivers_unchanged=True,no_motion_commands=True,
 sha256={n:hashlib.sha256((r/n).read_bytes()).hexdigest() for n in expected})
(w/'reports/fast-level-deployment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
