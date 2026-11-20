"""One-time audited recovery of the confirmed stopped, outside lift timeout.

Only the web observer is replaced. Hardware controllers are never signalled.
"""
from pathlib import Path
import hashlib,json,time,os,signal,subprocess,urllib.request,http.cookiejar
import numpy as np

r=Path('/home/arx/arx-competition-control');w=Path('/home/arx/arx-place-hold-20261116')
expected={
 'web/src/visual_docking_live.py':'1245294d2e040f06d3d577f35b341c382fbe6bb5c2a9d294e23a00cbc965c321',
 'web/src/docking_motion.py':'a87a72939a35edc9d3aec6f3e12b687c01ed785bf564e80e5bfa218724f077f3',
 'agent/grasp_path.py':'3a94c3dc69c7b11c7988c73a267c6b1a9798aa8f68399b0daacd1196bc510ea8'}
run='ba9ab7e686234fc98a3685a91ebff73b'
op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
def get(p):return json.load(op.open('http://127.0.0.1:9983'+p,timeout=3))
op.open('http://127.0.0.1:9983/',timeout=3).read()
def proc(pid):
 try:
  p=Path('/proc')/str(pid);s=p.joinpath('stat').read_text().rsplit(')',1)[1].split()
  if s[0]=='Z':return None
  return dict(pid=int(pid),start=s[19],args=p.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0'))
 except FileNotFoundError:return None
owners={i:proc(i) for i in (7272,7589,1359820,6087)};assert all(owners.values())
rows=[proc(p.name) for p in Path('/proc').glob('[0-9]*')]
web=[v for v in rows if v and str(r/'web/src/server.py') in v['args']];assert len(web)==1
assert not any(p.read_text().strip()=='visual-docking-live' for p in (Path('/proc')/str(web[0]['pid'])/'task').glob('*/comm'))
task=get('/api/visual-docking')
assert task['run_id']==run and task['status']=='recovery_required' and task['phase']=='align'
assert task['reason']=='Measured lift arrival/rest not confirmed' and task['released'] is False
samples=[]
for _ in range(12):
 s=get('/api/state');assert not s['chassis_active'] and s['body']['fresh'] and all(s['arms'][a]['fresh'] for a in ('left','right'))
 samples.append(s);time.sleep(.1)
assert np.ptp([s['body']['height'] for s in samples])<.002
assert abs(samples[-1]['body']['height']-5.412184238433838)<.02
assert np.max(np.ptp([s['arms']['right']['joints'] for s in samples],axis=0))<.002
assert np.ptp([s['arms']['right']['gripper'] for s in samples])<.002
assert any('Remaining lift target cancelled' in str(v) for v in samples[-1].get('logs',[]))
checked=json.loads((w/'reports/native-live-planning/result.json').read_text())
assert checked['status']=='native_backend_plan_checked' and checked['physical_execution'] is False
assert time.time()-(w/'reports/native-live-planning/result.json').stat().st_mtime<600
assert checked['observation']['fit']['leading_standoff_mm']>100
assert checked['observation']['grip_stable'] is True
assert checked['plan']['strategy']=='arm' and checked['plan']['payload']['high_stage'] is True
driver=checked['source_driver_snapshot_not_execution_authority']
assert driver['gripper_target_raw']==0 and driver['command_epoch']==0
assert max(abs(a-b) for a,b in zip(driver['joint_reference_rad'],samples[-1]['arms']['right']['joints']))<.02
for name,sha in expected.items():
 assert hashlib.sha256((r/name).read_bytes()).hexdigest()==sha
 compile((w/name).read_text(),str(w/name),'exec')
b=Path('/home/arx/arx-git-backups')/('docking-stopped-recovery-'+time.strftime('%Y%m%d-%H%M%S'));b.mkdir()
for name in expected:
 target=b/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes((r/name).read_bytes())
d=r/'web/hardware/runtime/visual-docking';receipt=d/'receipt.json'
assert json.loads(receipt.read_text())['run_id']==run
(b/'receipt.json').write_bytes(receipt.read_bytes())
ack=dict(recovered_run=run,operator_confirmation='已停稳，无下滑或异响，电池夹持稳固',
 evidence=dict(task=task,samples=samples,read_only_plan=checked),
 outcome='Stopped outside confirmed; archive failed receipt; start only a fresh observed task',
 hardware_owners=owners,web=web,time=time.time(),no_replay=True)
(b/'recovery-ack.json').write_text(json.dumps(ack,ensure_ascii=False,indent=2))
assert proc(web[0]['pid'])==web[0];os.kill(web[0]['pid'],signal.SIGTERM)
for _ in range(100):
 if proc(web[0]['pid']) is None:break
 time.sleep(.1)
else:raise RuntimeError('Idle observer did not exit')
assert {i:proc(i) for i in owners}==owners
for name,sha in expected.items():
 assert hashlib.sha256((r/name).read_bytes()).hexdigest()==sha
 tmp=(r/name).with_suffix('.dock-tmp');tmp.write_bytes((w/name).read_bytes());os.replace(tmp,r/name)
# Keep the failed run journal and original receipt; this is an explicit verified
# manual recovery, never an automatic clearing of uncertain physical outcome.
os.replace(receipt,d/('recovered-'+run+'.json'))
(d/('recovery-ack-'+run+'.json')).write_text(json.dumps(ack,ensure_ascii=False,indent=2))
env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
child=subprocess.Popen(['bash',str(r/'web/start.sh')],cwd=r,env=env,stdin=subprocess.DEVNULL,stdout=open(b/'web.log','ab'),stderr=subprocess.STDOUT,start_new_session=True)
(r/'web/hardware/runtime/web.pid').write_text(str(child.pid)+'\n')
for _ in range(100):
 if child.poll() is not None:raise RuntimeError('Observer startup failed; preserve motor drivers: '+str(b))
 try:
  op.open('http://127.0.0.1:9983/',timeout=3).read();s=get('/api/state')
  if all(s['arms'][a]['fresh'] for a in ('left','right')) and s['body']['fresh']:break
 except OSError:pass
 time.sleep(.2)
else:raise RuntimeError('Fresh feedback not restored')
assert {i:proc(i) for i in owners}==owners
assert not s['motion_reserved'] and not s['chassis_active']
assert get('/api/visual-docking')['status']=='idle'
result=dict(status='stopped_outside_recovery_recorded',backup=str(b),web_pid=child.pid,hardware_unchanged=True,no_motion_commands=True,sha256={name:hashlib.sha256((r/name).read_bytes()).hexdigest() for name in expected})
(w/'reports/native-stopped-recovery.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
