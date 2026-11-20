"""Single fixed-flow task owner for the web process; construction is inert."""
import copy
import json
import math
import threading
from pathlib import Path

from agent.storage import RunStore
from agent.perception_session import PerceptionSession
from control import Rejected


class BatteryTask:
    def __init__(self, root, controller, workflow_factory, *, depth_control=None, close_timeout_s=2.):
        if not callable(workflow_factory) or close_timeout_s <= 0:
            raise ValueError('Workflow factory and bounded close timeout required')
        self.root=Path(root)
        self.controller=controller
        self.workflow_factory=workflow_factory
        self.depth_control=depth_control
        self.close_timeout_s=float(close_timeout_s)
        self.task_settings=copy.deepcopy(getattr(workflow_factory,'task_settings',{}))
        self.lock=threading.RLock()
        self.active=None
        self.prepared=None
        self.requests={}

    def _restore_depth(self,state):
        previous=state.get('depth_previous')
        if self.depth_control is None or previous is None:return
        state['depth_previous']=None
        for camera,enabled in previous.items():
            try:self.depth_control.request({'camera':camera,'enabled':enabled})
            except Exception as error:self.controller.log(camera+' depth restore failed: '+str(error),'warning')

    @staticmethod
    def _request(value):
        if not isinstance(value,str) or not 1 <= len(value) <= 128:
            raise ValueError('request_id must be a bounded string')
        return value

    @staticmethod
    def _logs(store):
        try:lines=(store.path/'events.jsonl').read_text(encoding='utf-8').splitlines()[-100:]
        except OSError:return []
        output=[]
        for line in lines:
            try:
                record=json.loads(line)
                if isinstance(record,dict):output.append(record)
            except (ValueError,TypeError):pass
        return output

    @staticmethod
    def _task_settings(data):
        target=data.get('lift_target_rad');observer_z=data.get('observer_z_mm')
        if (type(target) not in (int,float) or not math.isfinite(target) or not 1<=target<=18
                or type(observer_z) not in (int,float) or not math.isfinite(observer_z)
                or not 0<=observer_z<=1500):
            raise ValueError('升降目标须为 1–18 rad，预观察 Z 须为 0–1500 mm')
        return {'lift_target_rad':float(target),'lift_acceptance_rad':float(math.floor(target)),
                'observer_z_mm':float(observer_z)}

    @staticmethod
    def _vision_mode(value):
        if value not in ('d405','d435'):
            raise ValueError('视觉模式须为 D405 或 D435 Plan B')
        return value


    def _view(self, client):
        state=self.active
        epoch=self.controller.snapshot()['command_epoch']
        if state is None:
            return {'status':'idle','available':True,'command_epoch':epoch,
                    'recovery_required':False,'phase':'idle','mission_stage':'IDLE','logs':[],
                    'settings':copy.deepcopy(self.task_settings)}
        return {'status':state['status'],'available':state['status']!='running'
                    and not state['recovery_required'],
                'command_epoch':epoch,'run_id':state['run_id'],
                'reason':state['reason'],'flow':copy.deepcopy(state['flow']),
                'recovery_required':state['recovery_required'],
                'phase':state['store'].state.get('battery_flow',{}).get('phase','starting'),
                'mission_stage':state['store'].state.get('mission',{}).get('stage','IDLE'),
                'settings':copy.deepcopy(state['store'].state.get('battery_task_settings',{})),
                'logs':self._logs(state['store'])}

    def snapshot(self, client):
        with self.lock:return self._view(client)

    def prewarm(self,cancelled):
        """Prepare the competition grasp worker without capturing or inferring."""
        with self.lock:
            if self.prepared is not None:return
            try:
                store=RunStore.create(self.root)
                store.state['battery_task_settings']=copy.deepcopy(self.task_settings);store.save()
                session=PerceptionSession(store.path);store.perception_session=session
                prepared={'store':store,'session':session,'worker':None}
                self.prepared=prepared
            except Exception as error:
                self.controller.log('视觉模型预热未启动，抓取时按原逻辑加载：'+str(error)[:200],'warning')
                return
            def load():
                try:
                    result=session.prewarm(lambda:cancelled.is_set() or store.cancelled)
                    store.event('vision_prewarm',result)
                except Exception as error:
                    session.close()
                    if not store.closed:
                        store.event('vision_prewarm',{'status':'unavailable','reason':str(error)[:200]})
                    if not cancelled.is_set():
                        self.controller.log('视觉模型预热未完成，抓取时按原逻辑加载：'+str(error)[:200],'warning')
            prepared['worker']=threading.Thread(target=load,daemon=True,name='battery-vision-prewarm')
            prepared['worker'].start()

    def discard_prewarm(self):
        with self.lock:
            prepared=self.prepared;self.prepared=None
        if prepared is None:return
        (prepared['store'].path/'cancel.flag').touch(mode=0o600)
        prepared['session'].close()
        prepared['worker'].join(3.)
        prepared['store'].close()

    def result(self, after=''):
        """Read published artifacts from the same run as the shared task."""
        with self.lock:
            run=(self.active['store'].path if self.active else
                 max(self.root.glob('run-*/state.json'),key=lambda p:p.stat().st_mtime,
                     default=None))
        if run is None:return {'run_id':None,'revision':'idle','observations':{}}
        if run.name=='state.json':run=run.parent
        run=run.resolve();state_path=run/'state.json'
        revision=run.name+':'+str(state_path.stat().st_mtime_ns)
        if revision==after:return {'run_id':run.name,'revision':revision,'unchanged':True}
        state=json.loads(state_path.read_text())
        def read(relative):
            if not isinstance(relative,str):return None
            path=(run/relative).resolve()
            if not path.is_relative_to(run):raise ValueError('任务结果路径无效')
            try:return json.loads(path.read_text())
            except (OSError,ValueError):return None
        data=read(state.get('grasp_preview')) or {}
        data.update(run_id=run.name,revision=revision,observations={})
        for camera,ident in state.get('latest_battery_observations',{}).items():
            value=read(state.get('battery_observations',{}).get(ident))
            if value:data['observations'][camera]=value
        data['coarse_alignment']=read(state.get('coarse_alignment'))
        data['pick_plan']=read(state.get('battery_pick_plan'))
        data['flow']=state.get('battery_flow',{})
        data['tool_results']=[{'name':call['result'].get('tool'),
            'ok':call['result'].get('ok'),'error':call['result'].get('error'),
            'data':{k:v for k,v in call['result'].get('data',{}).items()
                    if k in ('status','location','message','plan_id','result_file','image_time','execution_receipt',
                             'execution_status','execution_id','stage','extract_stage','carry_stage','extracted',
                             'extraction_dx_mm','chassis_retreat_mm','carry_ready','target_wrist_base_mm',
                             'target_transport_joints_deg')}}
            for key,call in state.get('calls',{}).items()
            if key.startswith('fixed_') and not key.startswith('fixed_poll_') and 'result' in call]
        receipts={}
        for journal in (run/'executions').glob('*.json'):
            receipt=(read(str(journal.relative_to(run))) or {}).get('receipt',{})
            receipts[receipt.get('execution_id')]=receipt
        for tool in data['tool_results']:
            original=tool['data'].get('execution_receipt',{})
            receipt=receipts.get(original.get('execution_id'))
            if receipt:
                tool['data']['execution_receipt']=receipt
                if receipt.get('status')=='completed':tool.update(ok=True,error=None)
        return data

    def action(self, client, data):
        if not isinstance(client,str) or not client or not isinstance(data,dict):
            raise ValueError('Valid web session and JSON object required')
        action=data.get('action')
        request=self._request(data.get('request_id'))
        with self.lock:
            cached=self.requests.get((client,request))
            if cached is not None:return copy.deepcopy(cached)
            if action=='start':
                if (data.get('vision_mode',self.task_settings.get('vision_mode','d405'))=='plan_f'
                        and ('lift_target_rad' in data or 'observer_z_mm' in data)):
                    # Plan F has no observer-Z move; retain configuration, ignoring this unused input.
                    data=dict(data,observer_z_mm=self.task_settings['observer_z_mm'])
                required={'action','request_id','expected_epoch','environment_clear'}
                settings={'lift_target_rad','observer_z_mm'}
                optional=settings|{'vision_mode'}
                if (not required<=set(data) or not set(data)<=required|optional
                        or (bool(set(data)&settings) and not settings<=set(data))
                        or type(data['expected_epoch']) is not int
                        or data['environment_clear'] is not True):
                    raise ValueError('start requires expected_epoch, confirmed environment_clear and both fixed-point values')
                values=copy.deepcopy(self.task_settings)
                if settings<=set(data):values.update(self._task_settings(data))
                if 'vision_mode' in data:
                    values['vision_mode']=self._vision_mode(data['vision_mode'])
                else:
                    values.setdefault('vision_mode','d405')
                result=self._start(client,data['expected_epoch'],data['environment_clear'],values)
            elif action=='cancel':
                if set(data)!= {'action','request_id'}:
                    raise ValueError('cancel accepts request_id only')
                result=self._cancel(client)
            else:raise ValueError('Unknown battery task action')
            self.requests[(client,request)]=copy.deepcopy(result)
            return result

    def start_owned(self, token, environment_clear):
        """Competition-only entry; the parent retains ownership after this subtask."""
        with self.lock:
            if not self.controller.motion_valid(token):
                raise Rejected('整场任务控制权无效')
            return self._start('competition', None, environment_clear,
                               copy.deepcopy(self.task_settings), token=token,use_prepared=True)

    def _start(self, client, expected_epoch, environment_clear, settings, *, token=None,use_prepared=False):
        if self.active is not None:
            if self.active['status']=='running' or self.active['recovery_required']:
                raise Rejected('已有自动任务或待恢复任务')
            if self.active['store'] is not None and not self.active['store'].closed:
                self.active['store'].close()
        prepared=self.prepared if use_prepared else None
        if prepared is not None:self.prepared=None
        owns_motion=token is None
        if owns_motion:token=self.controller.reserve_motion(expected_epoch)
        store=prepared['store'] if prepared is not None else None;depth_previous=None
        try:
            if self.depth_control is not None:
                cameras=self.depth_control.snapshot()['cameras']
                required_cameras=('d435',) if settings.get('vision_mode')=='d435' else ('d435','d405')
                depth_previous={camera:cameras[camera]['requested'] for camera in required_cameras}
                for camera in depth_previous:self.depth_control.request({'camera':camera,'enabled':True})
            if store is None:store=RunStore.create(self.root)
            if settings:
                store.state['battery_task_settings']=copy.deepcopy(settings)
                store.save()
            flow=self.workflow_factory(store,self.controller,token,environment_clear)
        except Exception:
            if depth_previous is not None:self._restore_depth({'depth_previous':depth_previous})
            if store is not None:
                session=getattr(store,'perception_session',None)
                if session is not None:session.close()
                store.close()
            if owns_motion and self.controller.motion_valid(token):self.controller.release_motion(token)
            raise
        self.active={'client':client,'run_id':store.state['run_id'],'status':'running',
                     'reason':'','flow':None,'recovery_required':False,
                     'store':store,'token':token,'worker':None,'depth_previous':depth_previous,
                     'owns_motion':owns_motion,'runner':flow}
        worker=threading.Thread(target=self._run,args=(self.active,flow),
                                daemon=True,name='battery-fixed-flow')
        self.active['worker']=worker
        worker.start()
        return self._view(client)

    def _cancel(self, client):
        if self.active is None:
            raise Rejected('当前没有自动任务')
        (self.active['store'].path/'cancel.flag').touch(mode=0o600)
        return self._view(client)

    def _run(self, state, flow):
        try:
            result=flow.run()
            if not isinstance(result,dict):raise ValueError('Workflow returned invalid result')
        except Exception as error:
            result={'status':'needs_attention','reason':str(error)[:300]}
        self._restore_depth(state)
        with self.lock:
            state['flow']=copy.deepcopy(result)
            state['status']=str(result.get('status') or 'needs_attention')
            state['reason']=str(result.get('reason') or '')
            if not state['owns_motion']:
                state['store'].event('battery_subtask_finished',dict(status=state['status'],
                    released_control=False,reason=state['reason']))
                state['store'].close()
                return
            try:self.controller.release_motion(state['token'])
            except Rejected:
                state['recovery_required']=True
                state['store'].event('battery_recovery',dict(verified_stopped=False,
                    released_control=False,reason=state['reason']))
                self.controller.log('自动任务轨迹尚未清理，继续保留控制权')
            else:
                pending=state['store'].state.pop('pending',None)
                if pending is not None:
                    state['store'].state['recovered_pending']=pending
                    state['store'].save()
                state['store'].event('battery_recovery',dict(verified_stopped=False,
                    released_control=True,reason=state['reason']))
                self.controller.log('自动任务已退出，控制权已返还 9983')
            state['store'].close()

    def close(self):
        self.discard_prewarm()
        with self.lock:
            state=self.active
            if state is None:return
            worker=state.get('worker')
            if worker is not None and worker.is_alive():
                (state['store'].path/'cancel.flag').touch(mode=0o600)
        if worker is not None and worker.is_alive():worker.join(self.close_timeout_s)
        if worker is None or not worker.is_alive():self._restore_depth(state)
