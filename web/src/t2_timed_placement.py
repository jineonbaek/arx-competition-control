"""One real T2 attempt, then confirmed recovery and an explicit timeout outcome."""
import copy,threading,time
from navigation.polarbear_adapt.task_deadline import T2_TIMEOUT_S
class TimedT2Placement:
 navigation=False;recoveries={}
 def __init__(self,placement_module,navigation,root,*,recovery=None,clock=time.monotonic):
  self.placement_module=placement_module;self.nav_session=navigation;self.root=root;self.clock=clock;self.recovery=recovery;self.runs={};self.lock=threading.RLock()
 def __getattr__(self,name):return getattr(self.placement_module,name)
 def readiness(self):return self.placement_module.readiness()
 def _alive(self,context):return not context['cancelled'].is_set() and context['controller'].motion_valid(context['token'])
 def start(self,context):
  with self.lock:
   key=context['request_id']
   if key in self.runs:raise ValueError('T2本轮已启动，不能重放放置')
   run=dict(start=self.clock(),visit=copy.deepcopy(getattr(self.nav_session,'visit',None)),phase='placing',stopped=False,result=None,raw=None)
   self.runs[key]=run
   if self.recovery is None:
    from t2_timeout_recovery import T2TimeoutRecovery
    self.recovery=T2TimeoutRecovery(self.placement_module,self.root)
   try:
    capture=getattr(self.recovery,'capture_goal',None);run['goal']=capture() if callable(capture) else None
   except Exception as e:run['goal_error']=str(e);run['goal']=None
   try:run['raw']=self.placement_module.start(context)
   except Exception as e:run['raw']=dict(status='failed',reason=str(e))
   return self.inspect(context)
 def _stopped(self,context,run):
  if not run['stopped']:
   try:run['stopped']=self.placement_module.stop(context).get('stopped') is True
   except Exception as e:run['stop_error']=str(e)
  return run['stopped']
 def inspect(self,context):
  with self.lock:
   run=self.runs.get(context['request_id'])
   if run is None:return dict(status='unknown',reason='没有本轮T2计时记录')
   if not self._alive(context):
    self.stop(context);return dict(status='stopped',reason='操作员停止或任务控制权失效')
   if run['result'] is not None:return copy.deepcopy(run['result'])
   elapsed=self.clock()-run['start'];expired=elapsed>=T2_TIMEOUT_S
   if run['phase']=='placing':
    raw=run['raw'];run['raw']=None
    if raw is None:
     try:raw=self.placement_module.inspect(context)
     except Exception as e:raw=dict(status='failed',reason=str(e))
    if raw.get('status')=='completed' and (not expired or isinstance(raw.get('completed_at_monotonic'),(float,int)) and raw['completed_at_monotonic']<=run['start']+T2_TIMEOUT_S):run['result']=copy.deepcopy(raw);return raw
    if raw.get('status') not in ('running','accepted'):
     run.update(phase='waiting_timeout',failure=raw.get('reason') or 'T2未完成');self._stopped(context,run)
   if not expired:
    return dict(status='running',timeout_s=T2_TIMEOUT_S,elapsed_s=elapsed,timeout_phase=run['phase'],reason=(run.get('failure','')+'；等待原180秒期限，不重复放置') if run['phase']=='waiting_timeout' else '')
   if not self._stopped(context,run):return dict(status='running',timeout_phase='stopping',reason='T2已超时，等待原动作停止确认',elapsed_s=elapsed)
   if run['phase']!='recovering':
    run['phase']='recovering'
    try:result=self.recovery.start(context,run['visit'],run.get('goal'))
    except Exception as e:return dict(status='failed',reason='T2超时归位未启动：'+str(e))
   else:result=self.recovery.inspect(context)
   if result.get('status')=='completed':
    if result.get('right_arm_ready') is not True or result.get('chassis_ready') is not True:return dict(status='failed',reason='T2超时归位缺少右臂或底盘到位回执')
    run['result']=dict(status='completed',outcome='timed_out',placement_confirmed=False,timeout_s=T2_TIMEOUT_S,elapsed_s=elapsed,recovery=copy.deepcopy(result),reason='T2超过180秒，已停止并归位，跳过放置后继续导航')
    return copy.deepcopy(run['result'])
   return dict(result,timeout_phase=result.get('phase','recovering'),elapsed_s=elapsed)
 def stop(self,context):
  with self.lock:
   run=self.runs.get(context['request_id'])
   if run is None:return dict(stopped=True)
   a=self._stopped(context,run)
   try:b=self.recovery.stop(context).get('stopped') is True if run['phase']=='recovering' else True
   except Exception:b=False
   return dict(stopped=a and b)
 def resume(self,context):raise ValueError('T2不能重复放置，请结束本轮后重新开始')
