"""Operator rollback to the pre-migration sources/binary; requires idle chassis.
This restarts the hardware driver and therefore initializes the lift.
"""
from pathlib import Path
import json,os,signal,shutil,subprocess,time,urllib.request
ROOT=Path('/home/arx/arx-competition-control');REPORT=ROOT/'navigation/reports/ros-chassis-20261120'
d=json.load(urllib.request.urlopen('http://127.0.0.1:9983/api/state',timeout=3))
assert not any(d[k] for k in ['chassis_active','motion_reserved','competition_reserved']),'must be idle'
procs={}
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  a=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\0') if x]
  for kind,needle in [('server',str(ROOT/'web/src/server.py')),('body',str(ROOT/'web/hardware/build/lift_controller_web'))]:
   if needle in a:procs[kind]=(int(p.name),dict(x.decode().split('=',1) for x in (p/'environ').read_bytes().split(b'\0') if x))
 except (OSError,UnicodeError):pass
assert len(procs)==2
subprocess.run(['systemctl','--user','stop','arx-gamepad-teleop.service'],check=True)
for kind in ['server']:
 pid=procs[kind][0];os.kill(pid,signal.SIGTERM);end=time.monotonic()+15
 while Path('/proc',str(pid)).exists() and time.monotonic()<end:time.sleep(.1)
 assert not Path('/proc',str(pid)).exists()
subprocess.run(['systemctl','--user','disable','--now','arx-chassis-scheduler.service'],check=True)
pid=procs['body'][0];os.kill(pid,signal.SIGTERM);end=time.monotonic()+15
while Path('/proc',str(pid)).exists() and time.monotonic()<end:time.sleep(.1)
assert not Path('/proc',str(pid)).exists()
for f in json.loads((REPORT/'candidate/chassis_runtime/deploy-manifest.json').read_text()):shutil.copy2(REPORT/'before'/f,ROOT/f)
shutil.copy2(REPORT/'before/lift_controller_web',ROOT/'web/hardware/build/lift_controller_web')
for kind,script in [('body','web/hardware/start-body.sh'),('server','web/start.sh')]:
 with (REPORT/('rollback-'+kind+'.log')).open('ab') as log:
  p=subprocess.Popen(['/bin/bash',str(ROOT/script)],cwd=ROOT,env=procs[kind][1],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);print(kind,p.pid)
subprocess.run(['systemctl','--user','start','arx-gamepad-teleop.service'],check=True)
print('Restored previous files and binary; check lift calibration and /api/state before use.')
