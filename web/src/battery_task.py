"""Single fixed-flow task owner for the web process; construction is inert."""
import copy
import json
import math
from numbers import Real
import threading
import time
from pathlib import Path

from agent.motion_feedback import ArrivalMonitor
from agent.storage import RunStore,atomic_json
from agent.perception_session import PerceptionSession
from control import Rejected
from agent.battery_references import REFERENCES
from agent import fast_pick


FAST_HANDLE_QWEN_TASK=(
    '当前是D405已到预抓点的近景。请以提供的图2、图4电池局部参考为外观依据，'
    '在CURRENT实时图像中找到无人机电池外露端部的一体式刚性拉手；主要匹配拉手，不要只凭长方体电池外形选择。'
    '参考中的拉手是紧贴电池端面的宽厚条桥，两端与电池端部连接，条桥与端面之间形成可供夹指进入的凹槽或开口。'
    '结合条桥、槽口和相连端面的整体结构判断，不能把任意长条、凸起或矩形板都当作拉手。'
    '优先选择紧凑地包含拉手及相连电池端面的候选；电池主体可以被遮挡或超出画面，不要求看到完整电池。'
    '拉手和画面可能旋转或倾斜，不限定横向、竖向、画面中心或固定像素位置。'
    '近处两侧的大型夹板、夹指及其黑色长条内衬是机器人夹爪，必须排除；'
    '无人机机臂、起落架、螺旋桨、黄色机身边缘、空电池槽和地面也不是目标。'
    '不要依赖小圆孔、颜色、标签、文字或打印纹理，不要求看到任何孔位。'
    '参考图只用于比较结构，候选和位置必须以CURRENT为准。'
    '候选仅包含夹爪内衬、机架或无法确认拉手连接关系时不要勉强选择；'
    '存在疑似目标但无法可靠区分时返回 ambiguous，明确没有可见电池拉手或相连端部时返回 not_found。')


DEFAULT_RECOGNITION_PROFILES={
    'handle':{
        'id':'handle','name':'把手方案（当前）',
        'dino_prompt':'battery pull handle . recessed battery handle .',
        'qwen_task':(
            '找到安装在无人机电池槽中的可拆卸比赛电池。\n\n'
            '当前D435视角通常只能看到电池外露端部和前端提手，长方体电池主体大部分被无人机机身遮挡。'
            '因此不要要求看到完整电池外壳、侧面卡扣、电源连接器、标签或其他孔位。\n\n'
            '主要依据电池前端提手判断：提手与电池外端面连成一体，具有宽横向、圆角矩形的提手开口或提手槽，'
            '提手后方连接被无人机机身遮挡的长方体电池主体。目标位于无人机电池槽口处，而不是地面或无人机机架上。\n\n'
            '优先选择同时包含提手和相连电池端面的候选。若候选只覆盖提手，但其位置明确位于电池槽口并与电池端面连接，也可以选择。\n\n'
            '必须排除无人机起落架、机臂、螺旋桨、空电池槽、地面标定板、桌面，以及画面下方的机器人夹爪和机械臂。'
            '忽略颜色、明暗、材质、标签、条码和打印纹理差异。\n\n'
            '画面前景左右两侧的大型夹板、夹指和黑色内衬是机器人夹爪，即使带开口也不能选择。'
            '优先结合目标与前方无人机电池槽口的连接关系判断，不要求严格位于画面中心。'
            '如果只能确认夹爪或多个候选无法可靠区分，返回 ambiguous；没有电池提手或电池外端部则返回 not_found。')},
    'battery':{
        'id':'battery','name':'完整电池方案（历史）',
        'reference_images':{'first':['d405-01','d405-03'],'reobserve':['d405-02','d405-04']},
        'dino_prompt':'a rectangular drone battery pack with a protruding handle at one end, partially inserted into a drone.',
        'qwen_task':(
            '找到前方无人机电池槽口处已安装的可拆卸比赛电池。'
            '电池主体可能被无人机遮挡，通常只能看到近似矩形或圆角矩形的外露端面及部分侧边，主体向机身内部延伸。'
            '结合它与机身、电池槽口的连接关系判断，不要求看到完整电池。'
            '若能看清与端面相连的提手，可作为辅助依据；未清楚看到提手，不应单独据此排除电池。'
            '不要依赖小圆孔、颜色、文字标签或打印纹理。'
            '画面近处两侧的大型夹板、夹指及黑色内衬属于机器人夹爪，必须排除；'
            '即使呈长方形、带凹槽或开口，也不能选择。机臂、起落架、整架无人机、空槽口和地面标定板同样不是目标。'
            '相机接近后画面可能旋转，不要求电池固定横放、竖放或严格位于画面中心。'
            '参考图是D405成功夹持任务中的电池局部实拍，用于比较外观；位置与选择必须以CURRENT实时场景为准。'
            '无法可靠确认时返回 ambiguous；明确没有可见电池时返回 not_found，不要强行选择最相似的物体。')},

}


class BatteryTask:
    def __init__(self, root, controller, workflow_factory, *, depth_control=None, plan_c_path=None,
                 recognition_profiles_path=None,close_timeout_s=2.,fast_start_path=None,fast_dock=None):
        if not callable(workflow_factory) or close_timeout_s <= 0:
            raise ValueError('Workflow factory and bounded close timeout required')
        self.root=Path(root)
        self.controller=controller
        self.workflow_factory=workflow_factory
        self.depth_control=depth_control
        self.fast_start_path = Path(fast_start_path) if fast_start_path else None
        self.fast_dock = fast_dock
        self.close_timeout_s=float(close_timeout_s)
        self.task_settings=copy.deepcopy(getattr(workflow_factory,'task_settings',{}))
        self.plan_c_path=None if plan_c_path is None else Path(plan_c_path)
        self.plan_c=self._load_plan_c()
        self.recognition_profiles_path=(None if recognition_profiles_path is None
                                        else Path(recognition_profiles_path))
        self.recognition_profiles,self.selected_recognition_profile_id=self._load_recognition_profiles()
        self.task_settings['recognition_profile']=copy.deepcopy(
            self.recognition_profiles[self.selected_recognition_profile_id])
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
    def _lift_settings(target):
        if type(target) not in (int,float) or not math.isfinite(target) or not 1<=target<=18:
            raise ValueError('升降目标须为 1–18 rad')
        return {'lift_target_rad':float(target),'lift_acceptance_rad':float(math.floor(target))}

    @classmethod
    def _task_settings(cls,data):
        observer_z=data.get('observer_z_mm')
        if (type(observer_z) not in (int,float) or not math.isfinite(observer_z)
                or not 0<=observer_z<=1500):
            raise ValueError('升降目标须为 1–18 rad，预观察 Z 须为 0–1500 mm')
        return cls._lift_settings(data.get('lift_target_rad'))|{'observer_z_mm':float(observer_z)}

    @staticmethod
    def _plan_c_joints(value):
        if (not isinstance(value,list) or len(value)!=6
                or any(isinstance(v,bool) or not isinstance(v,Real) or not math.isfinite(v) for v in value)):
            raise ValueError('Plan C 点位须为六轴有限关节反馈')
        return [float(v) for v in value]

    def _load_plan_c(self):
        profile={'lift_target_rad':self.task_settings.get('lift_target_rad'),
                 'lift_acceptance_rad':self.task_settings.get('lift_acceptance_rad'),
                 'pregrasp_joints_rad':None,'grasp_joints_rad':None}
        if self.plan_c_path is None or not self.plan_c_path.exists():return profile
        try:
            saved=json.loads(self.plan_c_path.read_text(encoding='utf-8'))
            profile.update(self._lift_settings(saved.get('lift_target_rad')))
            for key in ('pregrasp_joints_rad','grasp_joints_rad'):
                if saved.get(key) is not None:profile[key]=self._plan_c_joints(saved[key])
            return profile
        except (OSError,ValueError,TypeError) as error:
            raise ValueError('Plan C 已保存点位无效：'+str(error)) from error

    def _save_plan_c(self):
        if self.plan_c_path is not None:
            self.plan_c_path.parent.mkdir(parents=True,exist_ok=True)
            atomic_json(self.plan_c_path,self.plan_c)

    @staticmethod
    def _recognition_profile(value):
        if (not isinstance(value,dict) or not {'id','name','dino_prompt','qwen_task'}<=set(value)
                or set(value)-{'id','name','dino_prompt','qwen_task','reference_images'}):
            raise ValueError('识别方案须包含 id、名称、DINO 提示词和 Qwen 任务')
        limits={'id':80,'name':80,'dino_prompt':1000,'qwen_task':1800}
        clean={}
        for key,limit in limits.items():
            text=value[key]
            if not isinstance(text,str) or not text.strip() or len(text)>limit:
                raise ValueError('识别方案字段为空或过长：'+key)
            clean[key]=text.strip()
        if any(not (c.isalnum() or c in '-_') for c in clean['id']):
            raise ValueError('识别方案 id 仅允许字母、数字、横线和下划线')
        if 'reference_images' in value:
            refs=value['reference_images']
            if (not isinstance(refs,dict) or set(refs)!={'first','reobserve'}
                    or any(not isinstance(ids,list) or not ids or len(ids)>len(REFERENCES)
                           or any(not isinstance(v,str) or v not in REFERENCES for v in ids)
                           or len(set(ids))!=len(ids) for ids in refs.values())):
                raise ValueError('参考图须为已有D405图号，分别设置首次和再次观察')
            clean['reference_images']=copy.deepcopy(refs)
        return clean

    def _load_recognition_profiles(self):
        profiles=copy.deepcopy(DEFAULT_RECOGNITION_PROFILES);selected='handle'
        if self.recognition_profiles_path is None or not self.recognition_profiles_path.exists():
            return profiles,selected
        try:
            saved=json.loads(self.recognition_profiles_path.read_text(encoding='utf-8'))
            if not isinstance(saved,dict) or not isinstance(saved.get('custom_profiles',[]),list):
                raise ValueError('识别方案文件格式无效')
            for value in saved.get('custom_profiles',[]):
                profile=self._recognition_profile(value)
                if not profile['id'].startswith('custom-'):
                    raise ValueError('自定义识别方案 id 无效')
                profiles[profile['id']]=profile
            if saved.get('selected') in profiles:selected=saved['selected']
            return profiles,selected
        except (OSError,ValueError,TypeError) as error:
            raise ValueError('已保存识别方案无效：'+str(error)) from error

    def _save_recognition_profiles(self):
        if self.recognition_profiles_path is None:return
        self.recognition_profiles_path.parent.mkdir(parents=True,exist_ok=True)
        atomic_json(self.recognition_profiles_path,{
            'selected':self.selected_recognition_profile_id,
            'custom_profiles':[copy.deepcopy(profile) for identity,profile in self.recognition_profiles.items()
                               if identity not in DEFAULT_RECOGNITION_PROFILES]})

    def _select_recognition_profile(self,identity):
        if not isinstance(identity,str) or identity not in self.recognition_profiles:
            raise ValueError('未知识别提示词方案')
        self.selected_recognition_profile_id=identity
        self.task_settings['recognition_profile']=copy.deepcopy(self.recognition_profiles[identity])
        self._save_recognition_profiles()

    def _settings_view(self,settings=None):
        value=copy.deepcopy(self.task_settings if settings is None else settings)
        if self.fast_start_path is not None:
            value['fast_start'] = self._fast_reference()
            value['fast_dock_policy'] = getattr(self.fast_dock, 'policy', None)
        value['fast_pregrasp_joints_deg']=fast_pick.load_pregrasp()
        value['plan_c']=copy.deepcopy(self.plan_c)
        value['reference_catalog']=[dict(id=k,label=v['label'],url='/grasp-reference/'+k+'.png') for k,v in REFERENCES.items()]
        value['recognition_profiles']=[copy.deepcopy(profile) for profile in self.recognition_profiles.values()]
        value['selected_recognition_profile_id']=self.selected_recognition_profile_id
        return value

    def _fast_reference(self):
        try:
            return json.loads(self.fast_start_path.read_text())
        except FileNotFoundError:
            return None

    @staticmethod
    def _vision_mode(value):
        if value not in ('d405','d435','plan_f','fixed'):
            raise ValueError('抓取模式须为 D405、D435 Plan B、Plan F 或 Plan C 固定点')
        return value


    def _view(self, client):
        state=self.active
        epoch=self.controller.snapshot()['command_epoch']
        if state is None:
            return {'status':'idle','available':True,'command_epoch':epoch,
                    'recovery_required':False,'phase':'idle','mission_stage':'IDLE','logs':[],
                    'settings':self._settings_view()}
        return {'status':state['status'],'available':state['status']!='running'
                    and not state['recovery_required'],
                'command_epoch':epoch,'run_id':state['run_id'],
                'reason':state['reason'],'flow':copy.deepcopy(state['flow']),
                'recovery_required':state['recovery_required'],
                'phase':state['store'].state.get('battery_flow',{}).get('phase','starting'),
                'mission_stage':state['store'].state.get('mission',{}).get('stage','IDLE'),
                'grasp_recovery':{k:copy.deepcopy(v) for k,v in state['store'].state.get('grasp_recovery',{}).items()
                                  if k in ('point2','attempt','status','last_recovery')},
                'settings':self._settings_view(state['store'].state.get('battery_task_settings',{})),
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
        head=state.get('battery_flow',{}).get('head')
        value=read(state.get('battery_observations',{}).get(head))
        if value:data['observations']['head']=value
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
            if action in ('start','start_fast'):
                if action=='start_fast':data=dict(data,vision_mode='plan_f')
                if (data.get('vision_mode',self.task_settings.get('vision_mode','d405'))=='plan_f'
                        and ('lift_target_rad' in data or 'observer_z_mm' in data)):
                    # Plan F has no observer-Z move; retain configuration, ignoring this unused input.
                    data=dict(data,observer_z_mm=self.task_settings['observer_z_mm'])
                required={'action','request_id','expected_epoch','environment_clear'}
                settings={'lift_target_rad','observer_z_mm'}
                optional=settings|{'vision_mode','plan_c_lift_target_rad','recognition_profile'}
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
                if 'recognition_profile' in data:
                    values['recognition_profile']=self._recognition_profile(data['recognition_profile'])
                if 'plan_c_lift_target_rad' in data:
                    self.plan_c.update(self._lift_settings(data['plan_c_lift_target_rad']))
                    self._save_plan_c()
                if values['vision_mode']=='fixed':
                    values['plan_c']=copy.deepcopy(self.plan_c)
                if (values['vision_mode']=='fixed'
                        and any(self.plan_c.get(key) is None
                                for key in ('pregrasp_joints_rad','grasp_joints_rad'))):
                    raise ValueError('Plan C 请先记录预夹取点和夹取点')
                if action=='start_fast':values['pickup_flow']='fast_plan_f'
                result=self._start(client,data['expected_epoch'],data['environment_clear'],values)
            elif action=='record_fast_pregrasp':
                if set(data)!={'action','request_id'}:
                    raise ValueError('记录快模式预抓点仅需要请求编号')
                arm=self.controller.snapshot().get('arms',{}).get('right',{})
                if arm.get('fresh') is not True:
                    raise ValueError('记录快模式预抓点需要新鲜右臂反馈')
                joints=fast_pick.pregrasp_joints(arm.get('joints'))
                fast_pick.FAST_PREGRASP_PATH.parent.mkdir(parents=True,exist_ok=True)
                atomic_json(fast_pick.FAST_PREGRASP_PATH,[math.degrees(v) for v in joints])
                result=self._view(client)
            elif action=='record_fast_start':
                if self.fast_start_path is None:
                    raise ValueError('快速抓取未配置')
                if self.active and self.active['status'] == 'running':
                    raise Rejected('抓取任务正在运行')
                from fast_grasp_start import capture
                from navigation_view import read_snapshot
                if action == 'record_fast_start':
                    if set(data) != {'action', 'request_id'}:
                        raise ValueError('记录起点仅需要请求编号')
                    robot = self.controller.snapshot()
                    if robot.get('motion_reserved') or robot.get('competition_reserved'):
                        raise Rejected('机器人正被其他任务使用')
                    reference = capture(robot, read_snapshot())
                    atomic_json(self.fast_start_path, reference)
                    result = self._view(client)
            elif action=='select_recognition_profile':
                if set(data)!= {'action','request_id','profile_id'}:
                    raise ValueError('select_recognition_profile requires profile_id')
                if self.active is not None and self.active['status']=='running':
                    raise Rejected('自动任务运行中，不能切换识别方案')
                self._select_recognition_profile(data['profile_id']);result=self._view(client)
            elif action=='save_recognition_profile':
                if set(data)!= {'action','request_id','profile'}:
                    raise ValueError('save_recognition_profile requires profile')
                if self.active is not None and self.active['status']=='running':
                    raise Rejected('自动任务运行中，不能修改识别方案')
                profile=self._recognition_profile(data['profile'])
                if not profile['id'].startswith('custom-'):
                    raise ValueError('内置识别方案不可覆盖，请另存为自定义方案')
                self.recognition_profiles[profile['id']]=profile
                self._select_recognition_profile(profile['id']);result=self._view(client)
            elif action=='delete_recognition_profile':
                if set(data)!= {'action','request_id','profile_id'}:
                    raise ValueError('delete_recognition_profile requires profile_id')
                if self.active is not None and self.active['status']=='running':
                    raise Rejected('自动任务运行中，不能删除识别方案')
                identity=data['profile_id']
                if not isinstance(identity,str) or not identity.startswith('custom-') or identity not in self.recognition_profiles:
                    raise ValueError('只能删除已保存的自定义识别方案')
                del self.recognition_profiles[identity]
                if self.selected_recognition_profile_id==identity:self.selected_recognition_profile_id='handle'
                self.task_settings['recognition_profile']=copy.deepcopy(
                    self.recognition_profiles[self.selected_recognition_profile_id])
                self._save_recognition_profiles();result=self._view(client)
            elif action=='record_plan_c':
                if set(data)!={'action','request_id','point','lift_target_rad'} or data['point'] not in ('pregrasp','grasp'):
                    raise ValueError('record_plan_c requires pregrasp/grasp and lift_target_rad')
                if self.active is not None and self.active['status']=='running':
                    raise Rejected('自动任务运行中，不能记录点位')
                arm=self.controller.snapshot().get('arms',{}).get('right',{})
                if arm.get('fresh') is not True:
                    raise ValueError('记录 Plan C 点位需要新鲜右臂反馈')
                self.plan_c.update(self._lift_settings(data['lift_target_rad']))
                self.plan_c[data['point']+'_joints_rad']=self._plan_c_joints(arm.get('joints'))
                self._save_plan_c()
                result=self._view(client)
            elif action=='cancel':
                if set(data)!= {'action','request_id'}:
                    raise ValueError('cancel accepts request_id only')
                result=self._cancel(client)
            else:raise ValueError('Unknown battery task action')
            self.requests[(client,request)]=copy.deepcopy(result)
            return result

    def start_owned(self, token, environment_clear, *, fast_pregrasp_at=None):
        """Competition-only entry; the parent retains ownership after this subtask."""
        with self.lock:
            if not self.controller.motion_valid(token):
                raise Rejected('整场任务控制权无效')
            settings=copy.deepcopy(self.task_settings)
            if fast_pregrasp_at is not None:
                settings.update(vision_mode='plan_f',pickup_flow='competition_fast',
                                fast_pregrasp_completed_at=fast_pregrasp_at)
            return self._start('competition', None, environment_clear,
                               settings, token=token,use_prepared=True)

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
                mode=settings.get('vision_mode')
                required_cameras=(() if mode=='fixed' else ('d435',) if mode=='d435'
                                  else ('d405',) if mode=='plan_f' else ('d435','d405'))
                depth_previous={camera:cameras[camera]['requested'] for camera in required_cameras}
                for camera in depth_previous:self.depth_control.request({'camera':camera,'enabled':True})
            if store is None:store=RunStore.create(self.root)
            if settings:
                store.state['battery_task_settings']=copy.deepcopy(settings)
                if settings.get('pickup_flow') in ('fast_plan_f','competition_fast'):
                    profile=store.state['battery_task_settings']['recognition_profile']
                    profile.update(id='fast-handle-near',name='快模式 · 图2/4把手近景',
                        qwen_task=FAST_HANDLE_QWEN_TASK,
                        reference_images={'first':['d405-02','d405-04'],
                                          'reobserve':['d405-02','d405-04']})
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

    def _prepare_plan_f_lift(self, state, flow):
        settings=state['store'].state.get('battery_task_settings',{})
        if settings.get('vision_mode')!='plan_f':return
        policy=copy.deepcopy(flow.tools.backends.policies['coarse']['lift_policy'])
        policy['settle_s']=min(policy['settle_s'],.05)
        target=settings['lift_target_rad'];acceptance=settings['lift_acceptance_rad']
        monitor=ArrivalMonitor([acceptance],started_at=self.controller.clock(),
            minimum=[acceptance],**policy)
        snapshot=self.controller.snapshot();body=snapshot.get('body',{})
        if body.get('fresh') is not True or body.get('height',-1)<acceptance:
            prestarted=self.controller.take_prestarted_lift(state['token'],target)
            if not prestarted:
                result=self.controller.submit({'type':'lift','target':target},state['token'])
                if result.get('ok') is not True:raise ValueError('Plan F lift submission not confirmed')
            moving=True
        else:moving=False
        while True:
            snapshot=self.controller.snapshot();body=snapshot.get('body',{});now=self.controller.clock()
            ready=(self.controller.motion_valid(state['token'])
                   and snapshot.get('chassis_active') is False
                   and snapshot.get('capabilities',{}).get('lift')==''
                   and body.get('fresh') is True)
            result=monitor.update([body.get('height')],body.get('received'),now,
                ready=ready,cancelled=state['store'].cancelled)
            if result['arrived']:
                if moving:self.controller.submit({'type':'lift.stop'},state['token'])
                return
            if result['stop_required']:
                if moving and self.controller.motion_valid(state['token']):
                    self.controller.submit({'type':'lift.stop'},state['token'])
                raise ValueError('Plan F lift preparation failed: '+result['status'])
            time.sleep(.02)

    def _run(self, state, flow):
        from agent.codex_recovery import session
        with session(state['run_id'],self.controller,lambda:not state['store'].cancelled and self.controller.motion_valid(state['token']),
                     lambda event,kind:state['store'].event('codex_advice',dict(event=event,kind=kind))):
            return self._run_task(state, flow)

    def _run_task(self, state, flow):
        try:
            store = state['store']
            if store.state.get('battery_task_settings', {}).get('pickup_flow') == 'fast':
                if store.state.get('fast_dock'):
                    raise ValueError('本轮已有贴靠记录，不能重复前进')
                dock = dict(run_id=state['run_id'], status='approaching')
                store.state['fast_dock'] = dock
                store.save()
                store.event('fast_dock', dict(stage='底盘贴靠平台'))
                receipt = self.fast_dock(self.controller, state['token'], store)
                if receipt.get('status') != 'stopped':
                    raise ValueError('底盘停车未确认')
                dock.update(receipt)
                store.save()
                store.event('fast_dock', dict(stage='底盘已停稳，前往快模式预抓取点', **receipt))
                from agent.fast_pick import prepare_pregrasp
                prepare_pregrasp(flow)
            else:
                self._prepare_plan_f_lift(state,flow)
                settings=store.state.get('battery_task_settings',{})
                if settings.get('pickup_flow')=='competition_fast':
                    from agent.fast_pick import enter_pregrasp_refinement
                    enter_pregrasp_refinement(flow,settings['fast_pregrasp_completed_at'])
            result=flow.run()
            if not isinstance(result,dict):raise ValueError('Workflow returned invalid result')
        except Exception as error:
            result={'status':'needs_attention','reason':str(error)[:300]}
        if result.get('status') not in ('completed','cancelled'):
            from agent.recovery_watch import report_fault
            report_fault('抓取',result.get('reason') or result.get('status'),run=state['run_id'])
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
