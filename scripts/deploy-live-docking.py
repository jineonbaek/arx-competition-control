"""CAS install of observer-only integration; preserves running hardware owners."""
from pathlib import Path
import os,signal,subprocess,time,json,hashlib,shutil,urllib.request,http.cookiejar
r=Path('/home/arx/arx-competition-control');w=Path('/home/arx/arx-place-hold-20261116')
files=['web/src/server.py','web/src/ros_adapter.py','web/src/docking_motion.py','web/src/visual_docking_live.py',
       'agent/docking_fit.py','agent/docking_grip.py','agent/visual_docking.py',
       'web/public/visual-docking.html','web/public/visual-docking.js','web/public/visual-docking.css']
base='http://127.0.0.1:9983';op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
op.open(base+'/',timeout=3).read()
def get(path):return json.load(op.open(base+path,timeout=3))
def ident(pid):
    try:
        p=Path('/proc')/str(pid);s=p.joinpath('stat').read_text().rsplit(')',1)[1].split()
        if s[0]=='Z':return None
        return dict(pid=int(pid),start=s[19],args=p.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0'))
    except FileNotFoundError:return None
def hardware():
    rows=[ident(p.name) for p in Path('/proc').glob('[0-9]*')]
    return [v for v in rows if v and any(Path(a).name in ('x5_controller_web','lift_controller_web','camera_preview.py') for a in v['args'])]
def idle():
    s=get('/api/state')
    assert all(s[k] is False for k in ('motion_reserved','competition_reserved','chassis_active'))
    assert all(s['arms'][side]['fresh'] for side in ('left','right')) and s['body']['fresh']
    for kind in ('cabinet-execution','battery-task','competition-task'):
        row=get('/api/'+kind)
        assert row.get('status') in ('idle','blocked','cancelled','completed','needs_attention','recovery_required')
        assert not row.get('control_owned',False)
    return s
before=idle();owners=hardware();assert len(owners)==4
web=[ident(p.name) for p in Path('/proc').glob('[0-9]*')];web=[p for p in web if p and str(r/'web/src/server.py') in p['args']]
assert len(web)==1
expected={n:hashlib.sha256((r/n).read_bytes()).hexdigest() if (r/n).exists() else None for n in files}
assert expected['web/src/server.py']=='64ae4805306c5f266a6b9ddec988fe1f1ea247fdf1a87be7ce50e531a8f52e2f','Concurrent server source change'
assert expected['web/src/ros_adapter.py']=='9cfdcd1213b37a5f2cbdb9ea28d3f789f6863a97db50801bf8c0d5181827be68','Concurrent adapter source change'
for n in files[2:]:assert expected[n] is None,'Unexpected existing integration file '+n
for n in files:
    if n.endswith('.py'):compile((w/n).read_text(),n,'exec')
b=Path('/home/arx/arx-git-backups')/('live-docking-'+time.strftime('%Y%m%d-%H%M%S'));b.mkdir()
for n in files:
    if (r/n).exists():p=b/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(r/n,p)
(b/'before.json').write_text(json.dumps(dict(state=before,hardware=owners,web=web[0],expected=expected),indent=2))
(b/'git-status.txt').write_bytes(subprocess.check_output(['git','status','--porcelain=v1'],cwd=r))
idle();assert hardware()==owners;old=web[0];assert ident(old['pid'])==old
os.kill(old['pid'],signal.SIGTERM)
for _ in range(100):
    if ident(old['pid']) is None:break
    time.sleep(.1)
else:raise RuntimeError('Idle observer did not exit; files unchanged')
def atom(path,data):
    path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_name(path.name+'.dock-tmp');tmp.write_bytes(data);os.replace(tmp,path)
def launch():
    env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    return subprocess.Popen(['bash',str(r/'web/start.sh')],cwd=r,env=env,stdin=subprocess.DEVNULL,
                            stdout=open(b/'web.log','ab',buffering=0),stderr=subprocess.STDOUT,start_new_session=True)
written=[];new=None
try:
    for n,sha in expected.items():
        p=r/n;assert (hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None)==sha,'Concurrent edit '+n
        atom(p,(w/n).read_bytes());written.append(n)
    new=launch();(r/'web/hardware/runtime/web.pid').write_text(str(new.pid)+'\n')
    for _ in range(80):
        if new.poll() is not None:raise RuntimeError('New observer exited')
        try:
            after=get('/api/state')
            if all(after['arms'][s]['fresh'] for s in ('left','right')) and after['body']['fresh']:break
        except OSError:pass
        time.sleep(.2)
    else:raise RuntimeError('Fresh feedback unavailable')
    op.open(base+'/',timeout=3).read();task=get('/api/visual-docking');assert task['status']=='idle'
    assert hardware()==owners
    result=dict(status='observer_native_docking_installed',backup=str(b),web_pid=new.pid,hardware_owners_unchanged=True,
                no_motion_or_gripper_commands=True,files={n:hashlib.sha256((r/n).read_bytes()).hexdigest() for n in files})
    (w/'reports/native-docking-deployment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
except Exception:
    if new and new.poll() is None:
        new.terminate();new.wait(timeout=10)
    for n in written:
        if hashlib.sha256((r/n).read_bytes()).hexdigest()!=hashlib.sha256((w/n).read_bytes()).hexdigest():raise RuntimeError('Concurrent edit prevents rollback '+n)
        if (b/n).exists():atom(r/n,(b/n).read_bytes())
        else:(r/n).unlink()
    launch();raise
