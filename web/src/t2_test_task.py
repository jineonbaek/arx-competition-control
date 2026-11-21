"""Real point2-only integration; reuse competition ownership and placement."""
from competition_task import CompetitionTask
from navigation.polarbear_adapt.t2_scope import t2_pair_indices

class T2ArrivalStage:
 navigation=True;recoveries={}
 def __init__(self,session):self.session=session
 def start(self,context):self.session.begin(context);return dict(status='running')
 def inspect(self,context):return self.session.arrival(context,'point_2')
 def stop(self,context):return self.session.stop(context)
 def resume(self,context):raise ValueError('请结束原任务后重新开始T2实测，不重放放置')

class T2ExitStage(T2ArrivalStage):
 def start(self,context):self.session.complete_route(context);return dict(status='completed')
 def inspect(self,context):return dict(status='completed' if not self.session.active else 'running')

class T2TestTask(CompetitionTask):
 stages=(('navigate_t2','T2前一点 → T2，导航并停稳'),
         ('place','执行 T2 真实放置任务'),
         ('navigate_exit','放置完成后继续剩余导航，至路线终点结束'))
 def _context(self,s,stage,record):return super()._context(s,stage,record)|dict(route_scope='t2_test')
 def _prepare_run(self,s):
  place=self.modules.get('place')
  if getattr(place,'preview_only',False):raise ValueError('T2实测需要真实放置后端')
  if callable(getattr(place,'readiness',None)):
   ready=place.readiness()
   if not ready.get('reference_valid'):raise ValueError(ready.get('reason') or '柜口参考无效')
  if s['index']==0 and self.navigation:
   self.navigation.prepare(self._context(s,'navigate_t2',dict(request_id=s['run_id']+':navigate_t2')))
 def _recover(self,s,stage,module,record,failure):raise failure
 def snapshot(self,client=None):
  result=super().snapshot(client)
  ps=((result.get('navigation') or {}).get('route') or {}).get('definition',{}).get('points',[])
  try:
   entry,end=t2_pair_indices(ps)
   result['target']=(f'第{entry+1}点 → 第{end+1}点（T2真实任务）→ 第{len(ps)}点（路线终点）'
                     if end+1<len(ps) else f'第{entry+1}点 → 第{end+1}点（T2真实任务，已是路线终点）')
  except ValueError:
   result['target']='T2前一点 → T2真实任务 → 继续剩余导航至路线终点'
  result['scope']='t2_test';result['can_continue']=False
  place=self.modules.get('place')
  if callable(getattr(place,'readiness',None)):
   ready=place.readiness()
   if not ready.get('reference_valid'):
    result['can_start']=False
    if result['status']=='idle':result['reason']=ready.get('reason','柜口参考无效')
  return result
 def action(self,client,data):
  if data.get('action')=='continue':raise ValueError('T2实测不重放已执行动作；结束原任务后重新开始')
  return super().action(client,data)


def make_t2_navigation_task(controller,root,navigation,placement):
 """Keep real placement behind T2 arrival and the normal handoff receipt."""
 return T2TestTask(controller,root,navigation=navigation,modules={
  'navigate_t2':T2ArrivalStage(navigation),
  'place':placement,
  'navigate_exit':T2ExitStage(navigation)})
