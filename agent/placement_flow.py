"""Existing RunStore/BatteryTask-compatible durable fixed flow; no import effects."""
import copy
import time
from threading import Lock

PHASES=('held','head','preplace','wrist','correct','reobserve','descend','release','withdraw','verify')

class PlacementFlow:
    def __init__(self,store,backend):
        self.store,self.backend=store,backend
        if not hasattr(store,'placement_lock'):store.placement_lock=Lock()
        self.flow=store.state.setdefault('placement_flow',dict(status='running',phase='held',index=0,
            inflight=False,placement_confirmed=False,side='left'))
        store.state['battery_flow']=self.flow
        store.save()

    def _finish(self,status,reason=''):
        self.flow.update(status=status,reason=reason,placement_confirmed=status=='completed',
            recovery_required=status!='completed' and self.flow.get('motion_started',False))
        self.store.save();self.store.event('placement_flow',copy.deepcopy(self.flow))
        return copy.deepcopy(self.flow)

    def run(self):
        if not self.store.placement_lock.acquire(False):return dict(status='running',placement_confirmed=False)
        try:
            if self.flow['status']!='running':return copy.deepcopy(self.flow)
            # Unknown physical outcomes require an operator; they are never replayed.
            if self.flow.get('inflight') or self.flow.get('motion_started') and self.flow.get('index',0)>0:
                self.backend.stop();return self._finish('recovery_required','重启时存在未确认阶段，不重放动作或松爪')
            for i in range(self.flow.get('index',0),len(PHASES)):
                name=PHASES[i]
                if self.store.cancelled:
                    self.backend.stop();return self._finish('cancelled','已请求停止，未追加松爪动作')
                if name in ('preplace','correct','descend','release','withdraw'):self.flow['motion_started']=True
                self.flow.update(phase=name,index=i,inflight=True);self.store.save()
                self.store.event('placement_phase',dict(phase=name,status='started'))
                try:
                    result=self.backend.call(name)
                    if self.store.cancelled:raise ValueError('任务已取消')
                    if not isinstance(result,dict) or result.get('status')!='confirmed':raise ValueError('阶段未获得确认回执')
                except Exception as e:
                    self.backend.stop()
                    return self._finish('cancelled' if self.store.cancelled else 'needs_attention',str(e)[:300])
                self.store.state['calls']['fixed_place_'+str(i)]={'result':dict(ok=True,data=result)}
                self.store.event('placement_phase',dict(phase=name,status='confirmed',result=result))
                self.flow.update(index=i+1,inflight=False);self.store.save()
            return self._finish('completed')
        finally:self.store.placement_lock.release()
