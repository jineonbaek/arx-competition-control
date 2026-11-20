"""Adapt the colleague's route and task protocol; no navigation algorithm or grasp changes."""
import copy
import threading
import time
import uuid
from urllib.error import HTTPError,URLError


class NavigationStage:
    navigation=True
    recoveries={}
    def __init__(self,session,point):self.session,self.point=session,point
    def start(self,context):
        if self.point=='point_1':self.session.begin(context)
        else:self.session.complete_visit(context)
        return dict(status='running')
    def inspect(self,context):return self.session.arrival(context,self.point)
    def stop(self,context):return self.session.stop(context)
    def resume(self,context):
        raise ValueError('导航路线已取消；请在导航端核对剩余路线，不重放原路线或已完成抓取')


class NavigationSession:
    def __init__(self,controller,route,client=None):
        self.controller,self.route=controller,route
        self.lock=threading.RLock();self.active=False;self.error=''
        self.client=client
        if self.client is None:
            try:
                from navigation.polarbear_adapt.task_client import TaskClient
                self.client=TaskClient()
            except ImportError:self.error='导航同学的 task_client 尚未安装，导航接入不可用'
        self.latest={};self.visit=None;self.mission=None;self.events={}
        self.stop_event=threading.Event();self.worker=None;self.start_requested=False
        self.clear_requested=False

    def stage(self,point):return NavigationStage(self,point)

    def snapshot(self):
        view=self.route.snapshot().get('route') or {}
        with self.lock:
            state=copy.deepcopy(self.latest);active=self.active;error=self.error
        if not active and self.client is not None:
            try:state=self.client.state()
            except Exception as exc:error=str(exc)
        return dict(route=view,handoff=state,registered=active,error=error)

    def begin(self,context):
        if self.active:raise ValueError('原导航任务尚未清理，不能重启路线')
        if self.client is None:raise ValueError(self.error)
        view=self.route.snapshot().get('route') or {};definition=view.get('definition') or {}
        if not view.get('available') or not view.get('status','').startswith('待开始'):
            raise ValueError('请先在导航端准备待开始的比赛路线；不会接管已运行的路线')
        points=[p for p in definition.get('points',[]) if p.get('type')=='task']
        if [p.get('point_id') for p in points]!=['point_1','point_2']:
            raise ValueError('本场固定取物、放置流程需要按顺序设置地点1和地点2')
        state=self.client.state();previous=state.get('task')
        if previous and previous['status']!='acknowledged':raise ValueError('导航上一次到访尚未确认结束')
        registration=self.client.register(['point_1','point_2'])
        self.lease_seconds=registration['lease_seconds']
        self.context=context;self.route_id=definition['route_id'];self.map_id=definition['map_id']
        self.points={p['point_id']:p['id'] for p in points}
        self.baseline_visit=(previous or {}).get('visit_id')
        self.start_id=context['run_id']+':route-start';self.clear_id=context['run_id']+':route-clear'
        self.events={};self.visit=None;self.mission=None;self.latest=state;self.error=''
        self.stop_event=threading.Event();self.active=True;self.clear_requested=False
        self.start_requested=False
        self.worker=threading.Thread(target=self._heartbeat,args=(self.stop_event,),daemon=True,name='competition-navigation')
        self.worker.start()
        self.route.command('start',self.start_id,self.route_id)
        self.start_requested=True;self.command_started=time.monotonic()

    def _heartbeat(self,stop_event):
        deadline=time.monotonic()+self.lease_seconds
        while not stop_event.is_set():
            try:
                self.client.heartbeat();state=self.client.state()
                task=state.get('task')
                if task and task['status']!='acknowledged':self.client.task()
                if stop_event.is_set():return
                with self.lock:self.latest=state;self.error=''
                deadline=time.monotonic()+self.lease_seconds
                if task and task['status']=='cancelling' and not self.clear_requested:
                    self.context['cancelled'].set()
            except Exception as exc:
                if stop_event.is_set():return
                if time.monotonic()>=deadline:
                    with self.lock:self.error='导航通信/保活失败：'+str(exc)
                # The same client instance can renew its own expired lease.
                # Registration never replaces an active visit or another valid worker.
                try:self.client.register(['point_1','point_2'])
                except Exception:pass
            stop_event.wait(.2)

    def check(self):
        if self.active and self.error:return self.error
        if self.active:
            task=self.latest.get('task') or {}
            if task.get('status')=='needs_attention':return '导航交接暂停：'+str(task.get('error'))
        return None

    def await_channel(self,context):
        while self.active and self.check():
            if context['cancelled'].wait(.1):raise ValueError('整场停止，不自动恢复交接')
        if not self.active or context['cancelled'].is_set():raise ValueError('原导航任务已结束')

    def _receipt(self,ident):
        receipt=self.route.receipt(ident)
        return receipt

    def arrival(self,context,point):
        waiting=self.check()
        if not self.active:return dict(status='unknown',reason='原导航会话已停止，不能重放')
        if waiting:return dict(status='running',reason=waiting)
        receipt=self._receipt(self.start_id)
        if not receipt or receipt['status']!='published':return dict(status='running',reason='停车等待原路线启动回执，不重复启动')
        route=self.route.snapshot().get('route') or {};status=route.get('status','')
        if not route.get('available'):return dict(status='running',reason='等待导航执行端/观察通道恢复')
        if (route.get('definition') or {}).get('route_id')!=self.route_id:
            raise ValueError('本轮路线已被替换，停止推进作业')
        drive_failed=route.get('drive_received',0)>=self.command_started and route.get('drive_status','').startswith('stopped:')
        if status.startswith(('开始被拒绝','已取消','全部到达并停稳')) or drive_failed:
            return dict(status='failed',reason=status+' '+route.get('drive_status',''))
        with self.lock:task=copy.deepcopy(self.latest.get('task'))
        if not task or task.get('visit_id')==self.baseline_visit:return dict(status='running',reason=status)
        if point=='point_2' and self.visit and task['visit_id']==self.visit['visit_id']:
            return dict(status='running',reason='等待导航确认交接并到达地点2')
        if (task.get('point_id')!=point or task.get('route_point_id')!=self.points[point]
                or task.get('map_id')!=self.map_id or self.mission and task.get('mission_id')!=self.mission):
            raise ValueError('导航到访地点、路线或本轮任务身份不匹配')
        if task['status'] not in ('notified','running'):raise ValueError('导航到访尚未有效停稳或已中止')
        if self.controller.snapshot()['chassis_active']:return dict(status='running',reason='等待底盘运动占用归还')
        self.visit=task;self.mission=task['mission_id']
        return dict(status='completed',arrival=task)

    def _event(self,event,**values):
        visit=self.visit
        if not visit:raise ValueError('没有本轮到访，不能发送作业回执')
        key=(visit['visit_id'],event)
        if key not in self.events:
            self.events[key]=(str(uuid.uuid5(uuid.NAMESPACE_URL,self.context['run_id']+':'+':'.join(key))),values)
        ident,original=self.events[key]
        if event=='cancelled':
            # The operator cancellation flag must not prohibit its own receipt.
            for attempt in range(2):
                try:return self.client.event(visit,event,request_id=ident,**original)
                except (TimeoutError,URLError) as error:
                    if isinstance(error,HTTPError) or attempt:raise
        while self.active:
            self.await_channel(self.context)
            try:return self.client.event(visit,event,request_id=ident,**original)
            except (TimeoutError,URLError,OSError) as error:
                with self.lock:self.error='等待原交接回执：'+str(error)
                if self.context['cancelled'].wait(.2):raise ValueError('整场停止，不重放交接')
        raise ValueError('原导航任务已结束')

    def before_work(self,context,stage):
        self.await_channel(context)
        expected={'pick':'point_1','place':'point_2'}.get(stage)
        if not expected:return
        if not self.visit or self.visit['point_id']!=expected:raise ValueError('当前作业缺少匹配的导航到点通知')
        if not self.controller.motion_valid(context['token']):raise ValueError('作业机械臂保护令牌尚未就绪')
        self._event('accept')

    def complete_visit(self,context):
        while True:
            self.await_channel(context)
            robot=self.controller.snapshot()
            if not robot['chassis_active'] and not robot.get('motion_reserved',False):break
            if context['cancelled'].wait(.1):raise ValueError('整场停止，不自动恢复交接')
        self._event('complete',control_released=True,navigation_ready=True)

    def complete_route(self,context):
        self.complete_visit(context)
        while not context['cancelled'].wait(.1):
            if self.check():continue
            task=self.latest.get('task') or {};view=self.route.snapshot().get('route') or {}
            if task.get('status')=='acknowledged' and task.get('outcome')=='completed' and view.get('status','').startswith('全部到达并停稳'):
                self.close();return
            if not view.get('available') or view.get('drive_status','').startswith('stopped:'):
                continue # Keep the acknowledged work; never replay it to repair a view outage.
        raise ValueError('整场停止，不自动完成剩余路线')

    def stop(self,context):
        if not self.active:return dict(stopped=True)
        self.controller.stop_competition_navigation(context['competition_token'])
        if not self.start_requested:self.close();return dict(stopped=True)
        self.clear_requested=True
        self.route.command('clear',self.clear_id)
        end=time.monotonic()+3
        while time.monotonic()<end:
            receipt=self.route.receipt(self.clear_id);view=self.route.snapshot().get('route') or {}
            if receipt and receipt['status']=='rejected':raise ValueError(receipt.get('reason') or '导航取消未确认')
            state=self.client.state();task=state.get('task') or {}
            robot=self.controller.snapshot()
            if task.get('status')=='cancelling' and not robot['chassis_active']:
                self.visit=task;self._event('cancelled',control_released=True)
            if receipt and receipt['status']=='published' and view.get('status','').startswith('已取消'):
                self.close();return dict(stopped=True)
            time.sleep(.05)
        raise ValueError('导航路线取消尚未确认，保留整场占用供重试停止')

    def close(self):
        self.stop_event.set()
        if self.worker and self.worker is not threading.current_thread():self.worker.join(1)
        self.active=False
