"""Owned T2 recovery: right arm first, then chassis; neither signals task success."""
import copy,json,math,os,select,subprocess,threading,time
from pathlib import Path
import numpy as np
class T2TimeoutRecovery:
 def __init__(self,module,root):self.module=module;self.root=Path(root);self.runs={};self.lock=threading.RLock()
 def capture_goal(self):return self.module.placement.get_preobservation_goal()
 def _guard(self,context,run):
  if context['cancelled'].is_set() or run['cancel'].is_set() or not context['controller'].motion_valid(context['token']):raise ValueError('T2超时恢复已取消或控制权失效')
 def start(self,context,visit,goal=None):
  if not isinstance(visit,dict) or visit.get('point_id')!='point_2':raise ValueError('没有本轮T2归位目标')
  pose=np.asarray(visit.get('goal_pose'),float)
  if pose.shape!=(3,) or not np.isfinite(pose).all():raise ValueError('T2归位坐标无效')
  with self.lock:
   key=context['request_id']
   if key in self.runs:raise ValueError('本轮恢复已启动')
   run=dict(status='running',phase='right_arm_return',reason='',right_arm_ready=False,chassis_ready=False,cancel=threading.Event(),worker=None,arm_id=None,child=None,transport=None)
   self.runs[key]=run;thread=threading.Thread(target=self._run,args=(context,run,copy.deepcopy(visit),copy.deepcopy(goal)),daemon=True);run['thread']=thread;thread.start()
  return self.inspect(context)
 def inspect(self,context):
  run=self.runs.get(context['request_id'])
  if run is None:return dict(status='unknown',reason='没有本轮恢复记录')
  with self.lock:return {k:copy.deepcopy(v) for k,v in run.items() if k in ('status','phase','reason','right_arm_ready','chassis_ready','arm_receipt','chassis_receipt')}
 def _run(self,context,run,visit,goal):
  backend=self.module.placement;ctrl=context['controller']
  try:
   self._guard(context,run)
   original=self.module.runs.get(context['request_id'],{});owns=original.get('arm_run_id') and backend.snapshot().get('run_id')==original['arm_run_id']
   history=backend.motion.snapshot_paths() if owns and callable(getattr(backend.motion,'snapshot_paths',None)) else []
   if owns and backend.snapshot().get('returned') is True:history=[]
   if owns and getattr(backend,'t2_observation_q',None) is not None:goal=dict(goal or backend.get_preobservation_goal(),joints_rad=np.asarray(backend.t2_observation_q).tolist())
   if goal is None:goal=backend.get_preobservation_goal()
   expected=getattr(backend,'driver',None) if owns else None
   result=backend.start_preobservation_owned(context['token'],goal,history,expected_driver=expected);run['arm_id']=result['run_id']
   while True:
    self._guard(context,run);result=backend.snapshot()
    if result.get('run_id')!=run['arm_id']:raise ValueError('右臂恢复身份已改变')
    if result.get('status')=='complete' and result.get('preobservation_ready') is True:
     if backend.thread:backend.thread.join(3.)
     if backend.thread and backend.thread.is_alive():raise ValueError('右臂恢复仍未清理')
     break
    if result.get('status')!='running':raise ValueError(result.get('reason') or '右臂预观察点恢复未确认')
    run['cancel'].wait(.05)
   run.update(right_arm_ready=True,arm_receipt=result,phase='chassis_return')
   self._guard(context,run);target=np.asarray(goal['joints_rad'],float)
   def hold_arm():
    self._guard(context,run);state=ctrl.snapshot()
    if not state['arms']['right']['fresh'] or np.max(abs(np.asarray(state['arms']['right']['joints'])-target))>np.deg2rad(2.):raise ValueError('右臂预观察点保持未确认')
   receipt=self._return_chassis(context,run,visit['goal_pose'],hold_arm)
   hold_arm();run.update(status='completed',phase='ready_to_resume',chassis_ready=True,chassis_receipt=receipt)
  except Exception as e:
   try:
    if run.get('arm_id') and backend.snapshot().get('run_id')==run['arm_id']:backend.action(dict(op='cancel'))
   except Exception:pass
   run.update(status='failed',reason=str(e))
 def _return_chassis(self,context,run,goal,guard):
  from navigation.polarbear_adapt.chassis_command import WebChassis
  env=os.environ.copy();env['ROS_DOMAIN_ID']='192';env['PYTHONPATH']=str(self.root/'navigation')+os.pathsep+env.get('PYTHONPATH','')
  worker=self.root/'navigation/polarbear_adapt/t2_owned_return.py'
  child=subprocess.Popen(['/usr/bin/python3',str(worker),json.dumps(goal)],env=env,cwd=self.root/'navigation',stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1);run['child']=child
  transport=WebChassis(submit=lambda p:context['controller'].submit(p,context['token']));transport.sdk_units=transport.native_sdk=True;run['transport']=transport
  deadline=time.monotonic()+90.
  try:
   while time.monotonic()<deadline:
    guard();child.stdin.write('{"action":"step"}\n');child.stdin.flush();end=time.monotonic()+1.
    while not select.select([child.stdout],[],[],.05)[0]:
     guard()
     if child.poll() is not None or time.monotonic()>end:raise ValueError('T2回位定位控制通道未就绪')
    result=json.loads(child.stdout.readline());guard();cmd=np.asarray(result['command'],float)
    if cmd.shape!=(3,) or not np.isfinite(cmd).all() or np.linalg.norm(cmd[:2])>.225001 or abs(cmd[2])>.800001:raise ValueError('T2回位低速指令无效')
    if any(cmd) and not 0<=time.time_ns()-int(result['source_ns'])<=1_000_000_000:raise ValueError('T2回位权威车位已过期')
    transport.send(cmd)
    if result['status']=='completed':return result
    if result['status']=='failed':raise ValueError(result.get('reason') or 'T2回位失败')
    run['cancel'].wait(.03)
   raise ValueError('T2回位90秒内未确认')
  finally:
   try:transport.end_output()
   finally:
    if child.poll() is None:child.terminate()
    try:child.communicate(timeout=3.)
    except subprocess.TimeoutExpired:child.kill();child.communicate()
    run['child']=None;run['transport']=None
 def stop(self,context):
  run=self.runs.get(context['request_id'])
  if run is None:return dict(stopped=True)
  run['cancel'].set();backend=self.module.placement
  if run.get('arm_id') and backend.snapshot().get('run_id')==run['arm_id']:backend.action(dict(op='cancel'))
  run['thread'].join(4.)
  worker=getattr(backend,'thread',None)
  return dict(stopped=not run['thread'].is_alive() and not (run.get('arm_id') and worker and worker.is_alive()))
