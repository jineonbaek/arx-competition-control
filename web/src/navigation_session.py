"""Adapt the colleague's route and task protocol; no navigation algorithm or grasp changes."""
import copy
import threading
import time
import uuid
from urllib.error import HTTPError,URLError
from navigation.polarbear_adapt.t2_scope import t2_pair_indices


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
        self.clear_requested=False;self.prepared=False

    def stage(self,point):return NavigationStage(self,point)

    def snapshot(self):
        view=self.route.snapshot().get('route') or {}
        with self.lock:
            state=copy.deepcopy(self.latest);active=self.active;error=self.error
        if not active and self.client is not None:
            try:state=self.client.state()
            except Exception as exc:error=str(exc)
        return dict(route=view,handoff=state,registered=active,error=error)

    def validate_start_route(self,scope='competition'):
        if self.active and not self.prepared:raise ValueError('原导航任务尚未清理，不能重启路线')
        if self.client is None:raise ValueError(self.error)
        view=self.route.snapshot().get('route') or {};definition=view.get('definition') or {}
        status=view.get('status','').removeprefix('T2单独实测；')
        if scope=='t2_test':status=status.removeprefix('测试模式（无真实作业）；')
        if not view.get('available') or not (status.startswith('待开始') or scope=='t2_test' and status.startswith(('已取消','全部到达并停稳'))):
            raise ValueError('请先在导航端准备待开始的比赛路线；不会接管已运行的路线')
        if scope!='t2_test' and any(p.get('test_task_id') for p in definition.get('points',[])):
            raise ValueError('当前为测试模式，请在RViz开始路线；不启动真实抓取或放置')
        points=[p for p in definition.get('points',[]) if p.get('type')=='task']
        if scope=='t2_test':
            ps=definition.get('points',[])
            _,end=t2_pair_indices(ps)
            points=[ps[end]]
        elif scope!='competition' or [p.get('point_id') for p in points]!=['point_1','point_2']:
            raise ValueError('比赛路线必须按顺序包含T1任务点和T2任务点，请核对点类型；不会自动补造坐标')
        try:state=self.client.state()
        except Exception as exc:raise ValueError('导航交接服务未就绪，未开始机构准备：'+str(exc)) from exc
        live=state.get('route')
        if live is None:raise ValueError('导航交接服务尚无可核对的路线状态')
        if (live.get('route_id')!=definition.get('route_id') or live.get('map_id')!=definition.get('map_id')
                or live.get('running') or (scope=='competition' and not live.get('competition_startable_draft'))
                or (scope=='t2_test' and live.get('state') not in ('draft','cancelled','completed'))):
            raise ValueError('导航交接服务的当前路线身份或可启动状态不一致，未准备机构')
        if state.get('worker_ready') and not (self.active and self.prepared):
            try:self.client.heartbeat()  # Reuse our own still-valid two-second lease only.
            except Exception as exc:raise ValueError('已有有效worker且无法确认本实例登记，未准备机构') from exc
        previous=state.get('task')
        if previous and previous['status']!='acknowledged':raise ValueError('导航上一次到访尚未确认结束')
        return definition,points,state

    def prepare(self,context):
        if self.active:raise ValueError('原导航任务尚未清理')
        scope=context.get('route_scope','competition')
        definition,points,state=self.validate_start_route(scope);previous=state.get('task')
        registration=self.client.register([p['point_id'] for p in points])
        self.scope=scope
        self.lease_seconds=registration['lease_seconds']
        self.context=context;self.route_id=definition['route_id'];self.map_id=definition['map_id']
        self.points={p['point_id']:p['id'] for p in points}
        self.baseline_visit=(previous or {}).get('visit_id')
        self.start_id=context['run_id']+':route-start';self.clear_id=context['run_id']+':route-clear'
        self.events={};self.visit=None;self.mission=None;self.latest=state;self.error=''
        self.stop_event=threading.Event();self.active=True;self.clear_requested=False
        self.start_requested=False;self.prepared=True;self.prepared_definition=copy.deepcopy(definition)
        self.worker=threading.Thread(target=self._heartbeat,args=(self.stop_event,),daemon=True,name='competition-navigation')
        self.worker.start()
    def begin(self,context):
        with self.lock:
            if not self.active:self.prepare(context)
            if not self.prepared or self.context['run_id']!=context['run_id']:raise ValueError('准备中的导航任务身份不一致')
            # Prove that our registration remains valid after any arm preparation.
            self.client.heartbeat()
            definition,points,state=self.validate_start_route(self.scope)
            if definition!=self.prepared_definition:raise ValueError('机构准备期间路线已改变，请重新检查')
            if context['cancelled'].is_set() or not self.active:raise ValueError('本次实测已停止')
            self.prepared=False;self.start_requested=True
            if self.scope=='t2_test':self.route.command('start_t2',self.start_id,self.route_id,expected_definition=self.prepared_definition)
            else:self.route.command('start',self.start_id,self.route_id)
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
                    from agent.recovery_watch import report_fault
                    report_fault('导航通信',self.error,run=self.context['run_id'])
                # The same client instance can renew its own expired lease.
                # Registration never replaces an active visit or another valid worker.
                try:self.client.register(list(self.points))
                except Exception:pass
            stop_event.wait(.2)

    def check(self):
        if self.active and self.error:return self.error
        if self.active:
            task=self.latest.get('task') or {}
            if task.get('status')=='needs_attention':return '导航交接暂停：'+str(task.get('error'))
        return None

    def await_channel(self,context):
        from agent.recovery_watch import RetryWatch
        watch=RetryWatch(context['run_id'],'导航交接通信',max_attempts=450,max_s=45.)
        timeout=(context.get('t2_timed_out') is True and self.visit and self.visit.get('point_id')=='point_2')
        while self.active and (self.error if timeout else self.check()):
            watch.fail(self.error if timeout else self.check())
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
        t2=getattr(self,'scope',None)=='t2_test'
        if receipt and receipt['status']=='rejected':return dict(status='failed',reason=receipt.get('reason','路线启动被拒绝'))
        if not receipt or receipt['status']!=('started' if t2 else 'published'):return dict(status='running',reason='停车等待原路线启动回执，不重复启动')
        if t2:self.mission=receipt['mission_id']
        route=self.route.snapshot().get('route') or {};status=route.get('status','').removeprefix('T2单独实测；')
        if not route.get('available'):return dict(status='running',reason='等待导航执行端/观察通道恢复')
        if (route.get('definition') or {}).get('route_id')!=self.route_id:
            raise ValueError('本轮路线已被替换，停止推进作业')
        drive_failed=route.get('drive_received',0)>=self.command_started and route.get('drive_status','').startswith('stopped:')
        terminal=receipt.get('state') in ('cancelled','completed','needs_attention','manual_override') if t2 else status.startswith(('开始被拒绝','已取消','全部到达并停稳')) or drive_failed
        if terminal:
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
        if event in ('cancelled','fail'):
            # The operator cancellation flag must not prohibit its own receipt.
            for attempt in range(2):
                try:return self.client.event(visit,event,request_id=ident,**original)
                except (TimeoutError,URLError) as error:
                    if isinstance(error,HTTPError) or attempt:raise
        from agent.recovery_watch import RetryWatch
        watch=RetryWatch(self.context['run_id'],'导航交接回执',max_attempts=225,max_s=45.)
        while self.active:
            self.await_channel(dict(self.context,t2_timed_out=True) if event=='timeout' else self.context)
            try:return self.client.event(visit,event,request_id=ident,**original)
            except (TimeoutError,URLError,OSError) as error:
                with self.lock:self.error='等待原交接回执：'+str(error)
                watch.fail(self.error)
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
        from agent.recovery_watch import RetryWatch
        watch=RetryWatch(context['run_id'],'交接控制权归还',max_attempts=450,max_s=45.)
        while True:
            self.await_channel(context)
            robot=self.controller.snapshot()
            if not robot['chassis_active'] and not robot.get('motion_reserved',False):break
            watch.fail('底盘或机械臂占用尚未归还')
            if context['cancelled'].wait(.1):raise ValueError('整场停止，不自动恢复交接')
        if context.get('t2_timed_out'):
            if not self.visit or self.visit['point_id']!='point_2':raise ValueError('超时跳过仅适用于本轮T2')
            self._event('timeout',control_released=True,navigation_ready=True,error='t2_task_timeout_180s')
        else:self._event('complete',control_released=True,navigation_ready=True)

    def complete_route(self,context):
        self.complete_visit(context)
        from agent.recovery_watch import RetryWatch
        watch=RetryWatch(context['run_id'],'导航结束回执',max_attempts=450,max_s=45.)
        while not context['cancelled'].wait(.1):
            if self.check():
                watch.fail(self.check());continue
            task=self.latest.get('task') or {};view=self.route.snapshot().get('route') or {}
            receipt=self.route.receipt(self.start_id) or {}
            ended=receipt.get('status')=='started' and receipt.get('mission_id')==self.mission and receipt.get('state')=='completed' if self.scope=='t2_test' else view.get('status','').startswith('全部到达并停稳')
            if task.get('status')=='acknowledged' and task.get('outcome')==('timed_out' if context.get('t2_timed_out') else 'completed') and ended:
                self.close();return
            if not view.get('available') or view.get('drive_status','').startswith('stopped:'):
                watch.fail('导航结束通道不可用，保留已完成作业回执')
                continue # Keep the acknowledged work; never replay it to repair a view outage.
            if ended:watch.fail('路线已完成但作业完成回执未确认')
            else:watch.reset()
        raise ValueError('整场停止，不自动完成剩余路线')

    def finish_forward(self,context,*,clock=time.monotonic,transport_factory=None):
        """Terminal body-forward action; scoped ownership is kept until output ends."""
        if transport_factory is None:
            from navigation.polarbear_adapt.chassis_command import WebChassis
            transport_factory=WebChassis
        ctrl=context['controller'];token=context['token']
        transport=transport_factory(submit=lambda payload:ctrl.submit(payload,token))
        transport.sdk_units=transport.native_sdk=True
        end=clock()+4.
        try:
            while clock()<end:
                if context['cancelled'].is_set() or not ctrl.motion_valid(token):raise ValueError('末点前进已停止或控制权失效')
                state=ctrl.snapshot()
                if not state['body']['fresh'] or not state.get('nav_sdk_ready',False):raise ValueError('末点前进底盘反馈未就绪')
                remaining=end-clock()
                if remaining<=0:break
                transport.send((.225,0.,0.),ttl_s=min(.15,remaining))
                if context['cancelled'].wait(min(.04,remaining)):raise ValueError('操作员停止末点前进')
        finally:transport.end_output()

    def stop(self,context):
        with self.lock:
            if not self.active:return dict(stopped=True)
            if self.prepared and not self.start_requested:self.close();return dict(stopped=True)
            self.controller.stop_competition_navigation(context['competition_token'])
            if not self.start_requested:self.close();return dict(stopped=True)
            self.clear_requested=True
            if self.scope=='t2_test':self.route.command('cancel',self.clear_id,self.route_id,start_request_id=self.start_id)
            else:self.route.command('clear',self.clear_id)
            end=time.monotonic()+3
            while time.monotonic()<end:
                receipt=self.route.receipt(self.clear_id);view=self.route.snapshot().get('route') or {}
                if receipt and receipt['status']=='rejected':raise ValueError(receipt.get('reason') or '导航取消未确认')
                state=self.client.state();task=state.get('task') or {}
                robot=self.controller.snapshot()
                started=self.route.receipt(self.start_id) or {}
                own_mission=self.mission or started.get('mission_id')
                if task.get('status')=='cancelling' and (self.scope!='t2_test' or task.get('mission_id')==own_mission) and not robot['chassis_active']:
                    self.visit=task;self._event('cancelled',control_released=True)
                t2_done=self.scope=='t2_test' and receipt and receipt['status']=='cancelled' and (not task or task.get('mission_id')!=own_mission or task.get('status')=='acknowledged')
                if t2_done or receipt and receipt['status']=='published' and view.get('status','').removeprefix('T2单独实测；').startswith('已取消'):
                    self.close();return dict(stopped=True)
                time.sleep(.05)
            raise ValueError('导航路线取消尚未确认，保留整场占用供重试停止')

    def close(self):
        self.stop_event.set()
        if self.worker and self.worker is not threading.current_thread():self.worker.join(1)
        self.active=False;self.prepared=False
