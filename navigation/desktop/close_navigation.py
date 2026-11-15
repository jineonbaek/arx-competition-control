"""End exactly the viewer's navigation session; normal discard never deletes files."""
import errno,fcntl,json,os,subprocess,sys,time,urllib.error,urllib.request
from pathlib import Path


def lock_path():
 return Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))/('arx-navigation-'+str(os.getuid())+'.lock')


def handle_close(root,expected,backend,choice=None):
 root=Path(root).resolve();active=root/'reports/active-polarbear-adapt.txt'
 try:
  with open(lock_path(),'a') as lock:
   try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
   except BlockingIOError:return 0  # A standard launch/switch owns this close.
   if not expected or not active.exists() or active.read_text().strip()!=expected:return 0
   run=Path(expected).resolve()
   if run.parent!=root/'reports' or not (run/'launch.pid').is_file() or Path(expected).is_symlink():return 10
   mode=(run/'desktop-mode').read_text().strip() if (run/'desktop-mode').exists() else 'legacy-mapping'
   if mode not in ('mapping','mapping-navigation','legacy-mapping','navigation'):return 10
   if not backend.idle():backend.error('底盘仍在运动、任务占有控制权或状态无法确认；请停车后再关闭。');return 10
   if choice is None:choice='discard' if mode=='navigation' else backend.choose(mode)
   if choice not in ('save','discard'):return 10
   # Window may have been closed while another actor requested control.
   if not backend.idle():backend.error('控制状态已变化，保留当前会话。');return 10
   if choice=='save' and mode!='navigation' and not backend.save():
    backend.error('地图保存失败，保留当前会话并重新打开RViz。');return 10
   if not backend.idle():backend.error('保存期间控制权已变化，保留当前会话。');return 10
   (run/'drive-ready').unlink(missing_ok=True)
   if not backend.stop(run):
    backend.error('会话未完全退出，未丢弃地图；目标接收保持关闭，请检查日志。');return 10
   (run/'rviz-exit.json').write_text(json.dumps(dict(choice=choice,mode=mode,time_ns=time.time_ns(),historical_files_deleted=False)))
   if active.exists() and active.read_text().strip()==expected:active.unlink()
   return 0
 except (OSError,ValueError,subprocess.SubprocessError) as exc:
  backend.error('退出未完成，保留当前会话：'+str(exc));return 10


class Backend:
 def __init__(self,root):
  self.root=root;self.env=os.environ.copy();self.env.pop('ARX_ADAPT_RUN',None)
  self.http=urllib.request.build_opener(urllib.request.ProxyHandler({}))
 def idle(self):
  # Retry the SAME full control-state predicates. Never replace a failed
  # check with the weaker historical chassis_active-only helper.
  s=None
  for attempt in range(2):
   try:
    with self.http.open('http://127.0.0.1:9983/api/state',timeout=2) as reply:s=json.load(reply)
    break
   except (OSError,ValueError):pass
  if not isinstance(s,dict) or s.get('chassis_active',True) or s.get('motion_reserved',False) or s.get('capabilities',{}).get('chassis'):return False
  try:
   with self.http.open('http://127.0.0.1:9985/v1/state',timeout=1) as reply:task=json.load(reply).get('task')
   if task and task.get('status')!='acknowledged':return False
  except urllib.error.URLError as exc:
   if not isinstance(exc.reason,ConnectionRefusedError) and getattr(exc.reason,'errno',None)!=errno.ECONNREFUSED:return False
  except (OSError,ValueError):return False
  return True
 def choose(self,mode):
  result=subprocess.run(['zenity','--question','--title=结束本轮建图','--text=是否保存本轮地图？\n不保存将结束本轮建图并丢弃未保存内容。以前已保存的地图不受影响。','--ok-label=保存地图并退出','--cancel-label=继续建图','--extra-button=不保存地图并退出'],text=True,stdout=subprocess.PIPE)
  # Zenity 4 returns 1 for an extra button as well as for Cancel.
  # The exact printed label distinguishes explicit discard from cancellation.
  if result.returncode in (0,1) and result.stdout.strip()=='不保存地图并退出':return 'discard'
  if result.returncode:return 'keep'
  return 'save'
 def save(self):
  return subprocess.run(['bash','desktop/save_navigation_map.sh'],cwd=self.root,env=self.env,stdin=subprocess.DEVNULL).returncode==0
 def stop(self,run):
  # Nonempty second argument is the existing explicit skip-autosave path.
  return subprocess.run(['bash','-c','source sensor_env.sh; source desktop/session.sh; stop_session "$1" discard','close_navigation',str(run)],cwd=self.root,env=self.env,stdin=subprocess.DEVNULL).returncode==0
 def error(self,text):
  print(text,file=sys.stderr,flush=True)
  if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):subprocess.run(['zenity','--error','--title=导航未退出','--text='+text])


if __name__=='__main__':
 root=Path(__file__).resolve().parents[1]
 raise SystemExit(handle_close(root,sys.argv[1] if len(sys.argv)>1 else '',Backend(root)))
