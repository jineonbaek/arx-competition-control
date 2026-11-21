"""Raw sensor startup only. Existing LIO and navigation sessions are never stopped."""
import fcntl,json,os,subprocess,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
_lock=threading.RLock();_cached=None;_cached_at=0.

def drivers():
 result=[]
 for p in Path('/proc').glob('[0-9]*/cmdline'):
  try:
   a=p.read_bytes().decode().rstrip('\0').split('\0')
   if a and Path(a[0]).name=='livox_ros_driver2_node':result.append(a)
  except (OSError,UnicodeError):pass
 return result

def probe():
 global _cached,_cached_at
 with _lock:
  if _cached is not None and time.monotonic()-_cached_at<4:return _cached
  try:
   p=subprocess.run(['bash','-c','source "$1/navigation/sensor_env.sh"; export ROS_DOMAIN_ID=192; exec python3 "$1/navigation/desktop/lidar_health.py" --timeout 4','arx-lidar-check',str(ROOT)],capture_output=True,text=True,timeout=10)
   data=json.loads(p.stdout.strip().splitlines()[-1])
   fault='' if p.returncode==0 and data.get('ready') is True else '雷达数据未就绪：'+ '、'.join(data.get('missing',[]))
  except (OSError,ValueError,IndexError,subprocess.SubprocessError) as e:fault='雷达自检失败：'+str(e)
  _cached,_cached_at=fault,time.monotonic();return fault

def lidar_status():
 found=drivers()
 if len(found)>1:return '存在多个雷达驱动，请核对重复进程'
 if not found:return '雷达驱动未启动'
 if not any('single_bottom_driver' in x for x in found[0]):return '存在其他雷达驱动，未覆盖其配置'
 return probe()

def ensure_lidars():
 global _cached
 with _lock:
  found=drivers()
  if len(found)>1:return '存在多个雷达驱动，未重复初始化；请先核对进程'
  if found:
   if not any('single_bottom_driver' in x for x in found[0]):return '存在其他雷达驱动，未覆盖其配置'
   return probe()
  runtime=Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))
  try:
   with (runtime/('arx-navigation-'+str(os.getuid())+'.lock')).open('a') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Same lock as navigation startup; recheck after taking ownership.
    if drivers():return probe()
    result=subprocess.run(['bash',str(ROOT/'navigation/single_bottom/start_mapping.sh'),'--driver-only'],cwd=ROOT/'navigation',capture_output=True,text=True,timeout=55)
    log=ROOT/'web/hardware/runtime/launcher-lidars.log';log.parent.mkdir(parents=True,exist_ok=True)
    with log.open('a') as f:f.write(time.strftime('%F %T\n')+result.stdout+result.stderr)
    if result.returncode:return '雷达初始化失败，请查看 launcher-lidars.log；未启动融合或导航'
    _cached=None
    return probe()
  except BlockingIOError:return '雷达/导航操作正在进行，未并发初始化'
  except (OSError,subprocess.SubprocessError) as e:return '雷达初始化失败：'+str(e)
