"""Single fixed-flow task owner for the web process; construction is inert."""
import copy
import json
import threading
from pathlib import Path

from agent.storage import RunStore
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
        self.lock=threading.RLock()
        self.active=None
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

    def _view(self, client):
        state=self.active
        epoch=self.controller.snapshot()['command_epoch']
        if state is None:
            return {'status':'idle','available':True,'command_epoch':epoch,
                    'recovery_required':False,'phase':'idle','mission_stage':'IDLE','logs':[]}
        return {'status':state['status'],'available':state['status']!='running'
                    and not state['recovery_required'],
                'command_epoch':epoch,'run_id':state['run_id'],
                'reason':state['reason'],'flow':copy.deepcopy(state['flow']),
                'recovery_required':state['recovery_required'],
                'phase':state['store'].state.get('battery_flow',{}).get('phase','starting'),
                'mission_stage':state['store'].state.get('mission',{}).get('stage','IDLE'),
                'logs':self._logs(state['store'])}

    def snapshot(self, client):
        with self.lock:return self._view(client)

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
                    if k in ('status','location','message','plan_id','result_file','image_time','execution_receipt')}}
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
                required={'action','request_id','expected_epoch','environment_clear'}
                if (set(data)!=required or type(data['expected_epoch']) is not int
                        or data['environment_clear'] is not True):
                    raise ValueError('start requires expected_epoch and confirmed environment_clear')
                result=self._start(client,data['expected_epoch'],data['environment_clear'])
            elif action=='cancel':
                if set(data)!= {'action','request_id'}:
                    raise ValueError('cancel accepts request_id only')
                result=self._cancel(client)
            else:raise ValueError('Unknown battery task action')
            self.requests[(client,request)]=copy.deepcopy(result)
            return result

    def _start(self, client, expected_epoch, environment_clear):
        if self.active is not None:
            if self.active['status']=='running' or self.active['recovery_required']:
                raise Rejected('已有自动任务或待恢复任务')
            if self.active['store'] is not None and not self.active['store'].closed:
                self.active['store'].close()
        token=self.controller.reserve_motion(expected_epoch)
        store=None;depth_previous=None
        try:
            if self.depth_control is not None:
                cameras=self.depth_control.snapshot()['cameras']
                depth_previous={camera:cameras[camera]['requested'] for camera in ('d435','d405')}
                for camera in depth_previous:self.depth_control.request({'camera':camera,'enabled':True})
            store=RunStore.create(self.root)
            flow=self.workflow_factory(store,self.controller,token,environment_clear)
        except Exception:
            if depth_previous is not None:self._restore_depth({'depth_previous':depth_previous})
            if store is not None:store.close()
            if self.controller.motion_valid(token):self.controller.release_motion(token)
            raise
        self.active={'client':client,'run_id':store.state['run_id'],'status':'running',
                     'reason':'','flow':None,'recovery_required':False,
                     'store':store,'token':token,'worker':None,'depth_previous':depth_previous}
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
        with self.lock:
            state=self.active
            if state is None:return
            worker=state.get('worker')
            if worker is not None and worker.is_alive():
                (state['store'].path/'cancel.flag').touch(mode=0o600)
        if worker is not None and worker.is_alive():worker.join(self.close_timeout_s)
        if worker is None or not worker.is_alive():self._restore_depth(state)
