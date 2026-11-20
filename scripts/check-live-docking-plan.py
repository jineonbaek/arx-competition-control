"""Read-only real-image check of native backend planning; all dispatch is forbidden."""
from pathlib import Path
import sys,json,time,threading,subprocess
from types import SimpleNamespace
import urllib.request
ROOT=Path('/home/arx/arx-place-hold-20261116');RUNTIME=Path('/home/arx/arx-competition-control')
sys.path[:0]=[str(ROOT/'web/src'),str(ROOT)]
import yaml
from handeye_column import ColumnModel
from cabinet_preview import CabinetPreview
from visual_docking_live import LiveDockBackend
command='source /home/arx/arx-competition-control/env.sh; export ROS_DOMAIN_ID=191 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; timeout 5 ros2 topic echo /web/right/trajectory_status --once --full-length'
raw=subprocess.check_output(['bash','-c',command],text=True)
status=json.loads(yaml.safe_load(raw.split('---')[0])['data'])
class Observer:
    def __init__(self):
        self.config=json.loads((RUNTIME/'web/config/control.json').read_text());self.models={s:ColumnModel(side=s) for s in ('left','right')}
        self.adapter=SimpleNamespace(trajectory_status=lambda side:status);self.clock=time.monotonic;self.token=object()
    def snapshot(self):return json.load(urllib.request.urlopen('http://127.0.0.1:9983/api/state',timeout=3))
    def motion_valid(self,token):return token is self.token
    def submit(self,*a,**kw):raise AssertionError('READ ONLY: dispatch forbidden')
    def start_trajectory(self,*a,**kw):raise AssertionError('READ ONLY: trajectory forbidden')
ctrl=Observer();preview=CabinetPreview(ctrl,None,root=ROOT);out=ROOT/'reports/native-live-planning';out.mkdir(exist_ok=True)
events=[];backend=None
try:
    backend=LiveDockBackend(ctrl,preview,ctrl.token,None,'demo',threading.Event(),lambda **e:events.append(e))
    observation=backend.observe();plan=backend.plan('insert' if observation['at_entry'] else 'align',observation)
    result=dict(status='native_backend_plan_checked',physical_execution=False,observation=observation,plan=plan,
                source_driver_snapshot_not_execution_authority=status,events=events)
except Exception as error:
    result=dict(status='native_backend_plan_rejected',physical_execution=False,reason=repr(error),events=events)
    if backend is not None:result.update(cached_plan=backend.cached,window=getattr(backend,'window',None),table=getattr(backend,'table',None),initial=backend.initial)
finally:
    if backend is not None:backend.close()
(out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps({k:result[k] for k in ('status','physical_execution','reason') if k in result},ensure_ascii=False),flush=True)
