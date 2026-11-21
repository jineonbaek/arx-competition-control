"""Read-only network/LIO checks and explicitly confirmed, sensor-only initialization."""
import fcntl,json,os,subprocess,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
_cache={}
def enabled_ips():
 import sys
 sys.path.insert(0,str(ROOT/'navigation/desktop'))
 from lidar_ingress import state
 flags=state();return ['192.168.1.3']+(['192.168.1.164'] if flags['head'] else [])
def check_network():
 try:
  info=json.loads(subprocess.run(['ip','-j','-4','addr','show','dev','enp130s0'],capture_output=True,text=True,timeout=2,check=True).stdout)[0]
  if 'UP' not in info['flags']:return '雷达网口 enp130s0 未启用'
  if not any(x.get('local')=='192.168.1.50' for x in info.get('addr_info',[])):return '雷达网口缺少 192.168.1.50 地址'
  for ip in enabled_ips():
   route=json.loads(subprocess.run(['ip','-j','route','get',ip],capture_output=True,text=True,timeout=2,check=True).stdout)
   if not route or route[0].get('dev')!='enp130s0':return '雷达路由错误：'+ip
   if subprocess.run(['ping','-I','enp130s0','-c','1','-W','1',ip],capture_output=True,timeout=2).returncode:return '雷达不可达：'+ip
  return ''
 except (OSError,ValueError,IndexError,KeyError,subprocess.SubprocessError) as e:return '雷达网络检查失败：'+str(e)
def cached(key,fn):
 old=_cache.get(key)
 if old and time.monotonic()-old[0]<4:return old[1]
 result=fn();_cache[key]=(time.monotonic(),result);return result
def network_status():return cached('network',check_network)
def lio_processes():
 pids=[]
 for p in Path('/proc').glob('[0-9]*/cmdline'):
  try:
   a=p.read_bytes().decode().split('\0')
   if a and Path(a[0]).name=='fastlio_mapping':pids.append(int(p.parent.name))
  except (OSError,UnicodeError):pass
 return pids
def _lio_health(timeout=4):
 try:
  p=subprocess.run(['bash','-c','source "$1/navigation/sensor_env.sh"; export ROS_DOMAIN_ID=192; exec python3 "$1/navigation/desktop/lidar_health.py" --stage lio --timeout "$2"','arx-lio-check',str(ROOT),str(timeout)],capture_output=True,text=True,timeout=timeout+6)
  d=json.loads(p.stdout.strip().splitlines()[-1]);return '' if p.returncode==0 and d.get('ready') else 'FAST-LIO未就绪：'+ '、'.join(d.get('missing',[]))
 except (OSError,ValueError,IndexError,subprocess.SubprocessError) as e:return 'FAST-LIO检查失败：'+str(e)
def lio_status():
 found=lio_processes()
 if not found:return 'FAST-LIO未启动；请确认底盘及雷达/IMU静止后，点击初始化 FAST-LIO'
 if len(found)>1:return '存在多个 FAST-LIO 进程，请先核对；未自动停止'
 return cached('lio',_lio_health)
def robot_idle():
 try:
  with urllib.request.urlopen('http://127.0.0.1:9983/api/state',timeout=3) as r:s=json.load(r)
  return s.get('body',{}).get('fresh') is True and s.get('chassis_scheduler',{}).get('ready') is True and s.get('nav_sdk_ready') is True and not any(s.get(k,True) for k in ['chassis_active','competition_reserved'])
 except (OSError,ValueError):return False
def ensure_lio(stationary_confirmed=False):
 found=lio_processes()
 if found:
  fault=lio_status()
  return '' if not fault else fault+'；保留原融合/地图会话，请在导航入口恢复'
 if not stationary_confirmed:return '需要确认底盘及雷达/IMU静止；不会自动初始化 FAST-LIO'
 if not robot_idle():return '控制状态不可确认或任务仍在运行；未初始化 FAST-LIO'
 fault=network_status()
 if fault:return fault
 runtime=Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))
 try:
  with (runtime/('arx-navigation-'+str(os.getuid())+'.lock')).open('a') as lock:
   fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
   if lio_processes():return '已有融合进程启动，未重复初始化'
   if not robot_idle():return '任务状态已变化，未初始化'
   from lidar_services import lidar_status
   raw=lidar_status()
   if raw:return raw
   with InitLease() as lease:
    env=dict(os.environ,ARX_LIO_INIT_CLIENT=lease.session.client_id,ARX_LIO_INIT_INSTANCE=lease.session.instance)
    p=subprocess.run(['bash',str(ROOT/'navigation/single_bottom/start_mapping.sh'),'--odometry-confirmed'],cwd=ROOT/'navigation',env=env,capture_output=True,text=True,timeout=100)
    log=ROOT/'web/hardware/runtime/launcher-lio.log';log.parent.mkdir(parents=True,exist_ok=True)
    with log.open('a') as f:f.write(time.strftime('%F %T\n')+p.stdout+p.stderr)
    if p.returncode:return 'FAST-LIO初始化失败，请查看 launcher-lio.log；未停止已有硬件'
    _cache.pop('lio',None);fault=_lio_health(35)
   return '初始化期间底盘保护租约失效，请核对融合状态' if lease.lost else fault

 except BlockingIOError:return '导航/雷达操作正在进行，未并发初始化'
 except (OSError,subprocess.SubprocessError) as e:return 'FAST-LIO初始化失败：'+str(e)

class InitLease:
 def __enter__(self):
  import sys,threading
  sys.path.insert(0,str(ROOT/'chassis_runtime'))
  from arx_chassis.client import Session
  self.session=Session('manual');self.session.manage('reserve');self.lost=False;self.stop=threading.Event()
  def beat():
   while not self.stop.wait(.3):
    try:self.session.transport.manage('heartbeat',client_id=self.session.client_id,instance=self.session.instance)
    except Exception:self.lost=True;return
  self.thread=threading.Thread(target=beat,daemon=True);self.thread.start();return self
 def __exit__(self,*args):
  self.stop.set();self.thread.join(2)
  try:self.session.transport.manage('release',client_id=self.session.client_id,instance=self.session.instance)
  except Exception:self.lost=True
