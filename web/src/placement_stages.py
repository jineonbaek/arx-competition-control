"""Independent right-arm placement controls; observations and edits never move."""
import copy,json,math,threading,time,uuid
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation
from agent.grasp_path import TimedJointPath,plan_correction
from agent.visual_battery_level import motion_limits
from agent.storage import atomic_json
from docking_motion import DockingMotion
from placement_clamp import close_for_placement,require_closed_target
from fast_visual_level import MAX_ARM_SEGMENT_S,split_plan
from placement_stage_geometry import cabinet_observation,height_reading,z_target,battery_front
from d405_alignment_overlay import write_horizontal_overlay
from insertion_observation import capture_insertion

def _number(value):return type(value) in (int,float) and math.isfinite(value)
def _sequence(state):return state.get('chassis_scheduler',{}).get('output_seq')

def time_preobserve(start,goal,model,height,tcp,limits,guard):
    """Retain the checked straight joint path and retime each rest-to-rest segment."""
    points=np.array([start,goal])
    reach=model.reach+np.linalg.norm(tcp[:3,3])
    while True:
        guard()
        path=TimedJointPath(points,reach,tcp_position=lambda q:(model.wrist_transform(q,height)@tcp)[:3,3],**limits)
        long_segments=np.diff(path.times)>MAX_ARM_SEGMENT_S
        if not np.any(long_segments):
            return path
        refined=[points[0]]
        for a,b,too_long in zip(points,points[1:],long_segments):
            if too_long:
                refined.append((a+b)/2)
            refined.append(b)
        points=np.array(refined)

def require_residual_improvement(previous,current,tolerance,step):
    if current>tolerance and current>=previous-min(.15,step*.20):
        raise ValueError('D405 位移后误差未改善，已保持；可检查画面或单独微调 Z')

def check_tool_arrival(before,actual,target):
    error=float(np.linalg.norm(actual[:3,3]-target[:3,3])*1000)
    angle=float(np.degrees(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude()))
    vector=(target[:3,3]-before[:3,3])*1000;length=float(np.linalg.norm(vector))
    progress=float((actual[:3,3]-before[:3,3])@(vector/length))*1000 if length else 0.
    if error>2.5 or angle>1. or (length>.2 and progress<max(.05,min(.25,length*.25))):
        raise ValueError('末端未产生所要求的位移或未保持朝向')
    return error,angle

class PlacementStages:
    def __init__(self,controller,capture,push,root):
        self.ctrl=controller;self.capture=capture;self.push=push;self.root=Path(root)
        self.lock=threading.RLock();self.thread=None;self.motion=None;self.requests={}
        self.measure_lock=threading.Lock()
        self.cancel=threading.Event();self.shutdown=threading.Event();self.observer=None;self.observing=False
        self.geometry=None;self.geometry_binding=None;self.observation_error='尚未开启 D405 实时观测'
        self.run=None;self.manual_anchor=None;self.state=dict(status='idle',phase='idle',operation=None,timeline=[])
        self.settings_path=self.root/'web/hardware/runtime/placement-stages/settings.json'
        self.tcp=np.asarray(json.loads((self.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
        if hasattr(push,'observer') and push.observer is None:
            push.observer=lambda reference:capture_insertion(self.capture,self.ctrl,self.tcp,reference)
        q=[0.]*6;source='尚未记录'
        try:
            route=json.loads((self.root/'config/taught-placement.json').read_text())
            path=self.root/route['d405_preobservation_waypoint'];point=json.loads(path.read_text())
            q=np.degrees(point['joints_rad']).tolist();source='原 D405 预观测点'
        except (OSError,ValueError,KeyError):pass
        self.settings=dict(preobserve_joints_deg=q,preobserve_source=source,preobserve_lift_rad=0.,
            cabinet_z_mm=630.,cabinet_height_provisional=True,target_z_mm=630.,z_reference='visual',
            correction_axes='z',y_target_offset_mm=0.,correction_step_mm=3.,tolerance_mm=1.,
            insert_distance_mm=150.,insert_speed_mm_s=8.)
        self.config_error=''
        if self.settings_path.exists():
            try:
                saved=json.loads(self.settings_path.read_text())
                if saved.get('settings_version',1)<2:saved['settings']['insert_distance_mm']=150.
                self.settings=self._validate_settings(saved['settings'])
                self.manual_anchor=saved.get('manual_anchor')
            except (ValueError,OSError,KeyError,TypeError) as e:self.config_error='保存参数读取失败：'+str(e)

    def _driver(self):return self.ctrl.adapter.trajectory_status('right') or {}
    def _tool(self,state):return self.ctrl.models['right'].wrist_transform(state['arms']['right']['joints'],state['body']['height'])@self.tcp
    def _validate_settings(self,data):
        if not isinstance(data,dict) or set(data)!=set(self.settings):raise ValueError('参数字段不完整')
        q=data['preobserve_joints_deg'];limits=self.ctrl.config['arms']['right']['joint_limits_deg']
        if not isinstance(q,list) or len(q)!=6 or any(not _number(v) or not lo<=v<=hi for v,(lo,hi) in zip(q,limits)):
            raise ValueError('预观测关节点无效或超限')
        ranges={'cabinet_z_mm':(1,3000),'target_z_mm':(1,3000),'preobserve_lift_rad':(0,18),
                'y_target_offset_mm':(-100,100),'correction_step_mm':(.5,5.),'tolerance_mm':(.5,5.),
                'insert_distance_mm':(100,180),'insert_speed_mm_s':(1,30)}
        for k,(lo,hi) in ranges.items():
            if not _number(data[k]) or not lo<=data[k]<=hi:raise ValueError('参数超范围：'+k)
        if (type(data['cabinet_height_provisional']) is not bool or data['z_reference'] not in ('visual','manual','model')
            or data['correction_axes'] not in ('z','y','yz') or not isinstance(data['preobserve_source'],str)
            or len(data['preobserve_source'])>128):raise ValueError('高度来源或修正轴无效')
        return copy.deepcopy(data)

    def _persist(self,settings=None,manual_anchor=None):
        settings=copy.deepcopy(self.settings if settings is None else settings)
        anchor=copy.deepcopy(self.manual_anchor if manual_anchor is None else manual_anchor)
        self.settings_path.parent.mkdir(parents=True,exist_ok=True)
        atomic_json(self.settings_path,dict(settings_version=2,settings=settings,manual_anchor=anchor))
        self.settings=settings;self.manual_anchor=anchor
    def _bound(self,record,state):
        return bool(record and not state.get('chassis_active') and record['base_sequence']==_sequence(state)
                    and record['driver_instance']==self._driver().get('driver_instance'))
    def _manual_height(self,state,tool):
        if not self._bound(self.manual_anchor,state):raise ValueError('尚无有效手动 Z 标定；底盘移动后请重标')
        value=float(tool[2,3]*1000+self.manual_anchor['offset_mm'])
        return dict(tcp_z_mm=value,model_tcp_z_mm=float(tool[2,3]*1000),
            relative_cabinet_z_mm=value-self.settings['cabinet_z_mm'],visual_z_correction_mm=self.manual_anchor['offset_mm'],
            source='manual_translation_calibration',up_base=[0.,0.,1.])
    def _visual_height(self,state,tool):
        if not self._bound(self.geometry_binding,state):raise ValueError('D405 高度参考失效；底盘或驱动状态已改变')
        return height_reading(tool,self.geometry,self.settings['cabinet_z_mm'],now=time.time())
    def _height(self,state,reference=None):
        tool=self._tool(state);model=float(tool[2,3]*1000)
        if not state['arms']['right']['fresh'] or not state['body']['fresh']:
            return dict(tcp_z_mm=None,model_tcp_z_mm=None,source='unavailable',reason='机械臂或升降反馈过期')
        sources=[reference] if reference else ['visual','manual']
        reason=self.observation_error
        for source in sources:
            try:
                if source=='visual':return self._visual_height(state,tool)
                if source=='manual':return self._manual_height(state,tool)
                if source=='model':return dict(tcp_z_mm=model,model_tcp_z_mm=model,relative_cabinet_z_mm=model-self.settings['cabinet_z_mm'],
                    visual_z_correction_mm=None,source='model_uncompensated',up_base=[0.,0.,1.])
            except (ValueError,KeyError,TypeError) as e:reason=str(e)
        return dict(tcp_z_mm=None,model_tcp_z_mm=model,source='unavailable',reason=reason)

    def snapshot(self):
        with self.lock:
            s=self.ctrl.snapshot();value=copy.deepcopy(self.state)
            delegated_active=False
            if self.state.get('operation')=='insert' and self.state.get('delegate_stop_pending'):
                worker=getattr(self.push,'thread',None)
                delegated_active=bool(worker and worker.is_alive())
            value.update(settings=copy.deepcopy(self.settings),observing=self.observing,
                observation_error=self.observation_error,geometry=copy.deepcopy(self.geometry),
                command_epoch=s['command_epoch'],running=bool(self.thread and self.thread.is_alive()) or delegated_active,
                can_start=not (self.thread and self.thread.is_alive()) and not delegated_active and not any(s.get(k) for k in ('motion_reserved','competition_reserved','chassis_active')),
                config_error=self.config_error,manual_anchor_valid=self._bound(self.manual_anchor,s))
            try:
                value['height']=self._height(s);value['selected_height']=self._height(s,self.settings['z_reference'])
                tool=self._tool(s);value['current_tcp_mm']=(tool[:3,3]*1000).tolist()
                value['current_joints_deg']=np.degrees(s['arms']['right']['joints']).tolist()
                q=np.radians(self.settings['preobserve_joints_deg'])
                pre=self.ctrl.models['right'].wrist_transform(q,s['body']['height'])@self.tcp
                value['preobserve_tcp_mm']=(pre[:3,3]*1000).tolist()
                value['preobserve_tool_rpy_deg']=Rotation.from_matrix(pre[:3,:3]).as_euler('xyz',degrees=True).tolist()
                value['lift_rad']=s['body']['height'];value['lift_model_contribution_mm']=s['body']['height']/41.54*1000
            except (ValueError,KeyError,TypeError) as e:value['height']=dict(tcp_z_mm=None,model_tcp_z_mm=None,source='unavailable',reason=str(e))
            if value.get('active_phase'):
                value['active_phase']['duration_s']=time.time()-value['active_phase']['started_at']
                value['active_phase']['cumulative_duration_s']=time.time()-value['started_at']
            return value

    def _save(self,**values):
        with self.lock:
            self.state.update(values)
            if self.run:
                try:atomic_json(self.run/'receipt.json',self.state)
                except OSError as e:self.state['recording_error']=str(e)

    def action(self,data):
        if not isinstance(data,dict):raise ValueError('无效请求')
        op=data.get('op')
        with self.lock:
            if op in ('observe_on','observe_off') and set(data)=={'op'}:
                self.observing=op=='observe_on'
                if self.observing and not (self.observer and self.observer.is_alive()):
                    self.observer=threading.Thread(target=self._observe_loop,daemon=True,name='placement-height-observer');self.observer.start()
                return self.snapshot()
            if op=='cancel' and set(data)=={'op','request_id'}:
                if data['request_id']!=self.state.get('request_id'):raise ValueError('停止请求不属于当前阶段')
                self.cancel.set()
                if self.state.get('operation')=='insert':
                    try:self.push.action({'op':'cancel','request_id':data['request_id']})
                    except ValueError:pass  # The child validates ownership itself.
                return self.snapshot()
            if op=='run' and data.get('request_id') in self.requests:
                if self.requests[data['request_id']]!=data:raise ValueError('同一请求编号不能改变内容')
                return self.snapshot()
            if self.thread and self.thread.is_alive():raise ValueError('当前阶段尚未结束；参数不会在动作中改变')
            if op=='save_settings' and set(data)=={'op','settings'}:
                self._persist(settings=self._validate_settings(data['settings']));self.config_error='';return self.snapshot()
            if op=='record_preobserve' and set(data)=={'op'}:
                s=self.ctrl.snapshot()
                if not s['arms']['right']['fresh'] or not s['body']['fresh']:raise ValueError('当前位姿反馈过期')
                settings=copy.deepcopy(self.settings);settings.update(preobserve_joints_deg=np.degrees(s['arms']['right']['joints']).tolist(),
                    preobserve_source='当前实测点',preobserve_lift_rad=s['body']['height'])
                self._persist(settings=self._validate_settings(settings));return self.snapshot()
            if op=='save_cartesian' and set(data)=={'op','xyz_mm','rpy_deg'}:
                xyz=np.asarray(data['xyz_mm'],float);rpy=np.asarray(data['rpy_deg'],float)
                if xyz.shape!=(3,) or rpy.shape!=(3,) or not np.isfinite(np.r_[xyz,rpy]).all():raise ValueError('预观测 TCP 位姿无效')
                s=self.ctrl.snapshot();T=np.eye(4);T[:3,3]=xyz/1000;T[:3,:3]=Rotation.from_euler('xyz',rpy,degrees=True).as_matrix()
                wrist=T@np.linalg.inv(self.tcp);q=self.ctrl.models['right'].inverse_wrist(np.radians(self.settings['preobserve_joints_deg']),s['body']['height'],
                    wrist[:3,3]*1000,Rotation.from_matrix(wrist[:3,:3]).as_euler('xyz',degrees=True),self.ctrl.config['arms']['right']['joint_limits_deg'])
                settings=copy.deepcopy(self.settings);settings.update(preobserve_joints_deg=np.degrees(q).tolist(),preobserve_source='自定义 TCP 逆解点',preobserve_lift_rad=s['body']['height'])
                self._persist(settings=self._validate_settings(settings));return self.snapshot()
            if op=='calibrate_z' and set(data)=={'op','known_tcp_z_mm'}:
                if not _number(data['known_tcp_z_mm']) or not 1<=data['known_tcp_z_mm']<=3000:raise ValueError('实际 TCP 高度无效')
                s=self.ctrl.snapshot()
                if s['chassis_active'] or not s['arms']['right']['fresh'] or not s['body']['fresh']:raise ValueError('请在底盘停稳且反馈在线时标定')
                anchor=dict(offset_mm=float(data['known_tcp_z_mm']-self._tool(s)[2,3]*1000),
                    known_tcp_z_mm=data['known_tcp_z_mm'],base_sequence=_sequence(s),driver_instance=self._driver().get('driver_instance'),captured_at=time.time())
                self._persist(manual_anchor=anchor);return self.snapshot()
            fields={'op','phase','request_id','expected_epoch'}
            if op!='run' or not fields<=set(data) or set(data)-fields not in (set(),{'delta_z_mm'}):raise ValueError('仅支持独立阶段及 Z 调整')
            if data['phase'] not in ('preobserve','correct','insert','z'):raise ValueError('未知阶段')
            if not isinstance(data['request_id'],str) or not 1<=len(data['request_id'])<=128 or type(data['expected_epoch']) is not int:raise ValueError('需要本次请求编号和控制代次')
            if 'delta_z_mm' in data and (data['phase']!='z' or not _number(data['delta_z_mm']) or not 0<abs(data['delta_z_mm'])<=20):raise ValueError('Z 步进范围为 ±20 mm')
            request=data['request_id']
            if request in self.requests:
                if self.requests[request]!=data:raise ValueError('同一请求编号不能改变内容')
                return self.snapshot()
            s=self.ctrl.snapshot()
            if s['command_epoch']!=data['expected_epoch'] or any(s.get(k) for k in ('motion_reserved','competition_reserved','chassis_active')):
                raise ValueError('其他控制正在执行或状态已变化')
            run=self.root/'reports'/('placement-stage-'+uuid.uuid4().hex);run.mkdir(parents=True)
            token=None if data['phase']=='insert' else self.ctrl.reserve_motion(data['expected_epoch'])
            self.cancel=threading.Event();self.motion=None;self.run=run
            self.requests[request]=copy.deepcopy(data)
            self.state=dict(status='running',phase=data['phase'],operation=data['phase'],request_id=request,run_id=self.run.name,
                started_at=time.time(),settings=copy.deepcopy(self.settings),initial=s,timeline=[],released=False)
            self.thread=threading.Thread(target=self._run,args=(copy.deepcopy(data),token),daemon=True,name='placement-stage')
            try:self.thread.start()
            except Exception:
                if token:self.ctrl.release_motion(token)
                self.state.update(status='blocked',reason='阶段线程未能启动',phase='hold')
                raise
            return self.snapshot()

    def _capture(self):
        with self.measure_lock:return self._capture_locked()

    def _capture_locked(self):
        initial=self.ctrl.snapshot();view=self.capture.action({'op':'capture'})
        with np.load(self.capture.cache/'frame.npz',allow_pickle=False) as f:frame={k:f[k] for k in f.files}
        if str(frame['id'])!=view['id']:raise ValueError('D405 本轮配对帧已被替换')
        current=self.ctrl.snapshot()
        if current['command_epoch']!=initial['command_epoch'] or current['chassis_active'] or _sequence(current)!=_sequence(initial):raise ValueError('D405 曝光期间控制状态或底盘发生变化')
        geometry=cabinet_observation(frame)
        with self.lock:
            self.geometry=geometry;self.geometry_binding=dict(base_sequence=_sequence(current),driver_instance=self._driver().get('driver_instance'))
            self.observation_error=''
        if self.run and threading.current_thread() is self.thread:
            try:
                name='d405-'+geometry['frame_id']
                if not cv2.imwrite(str(self.run/(name+'.jpg')),frame['rgb'][:,:,::-1]):raise OSError('D405 图像保存失败')
                atomic_json(self.run/(name+'.json'),geometry)
            except OSError as e:self._save(recording_error=str(e))
        try:
            write_horizontal_overlay(frame,dict(status='partial',reason='D405 仓口高度观测',frame_id=str(frame['id']),
                image_time=float(frame['image_time']),calibration_sha256=str(frame['calibration_sha256'])),self.root/'web/public/camera-overlay')
        except (OSError,ValueError):pass
        return frame,geometry

    def _observe_loop(self):
        while not self.shutdown.is_set():
            if not self.observing:return
            try:
                if not (self.thread and self.thread.is_alive()):self._capture()
            except Exception as e:
                with self.lock:self.observation_error=str(e)
            self.shutdown.wait(.8)

    def _timed(self,name,call,planned=None):
        started=time.time();self._save(active_phase=dict(phase=name,started_at=started,planned_duration_s=planned,status='running'))
        try:result=call();status='ok';return result
        except Exception:status='error';raise
        finally:
            row=dict(phase=name,planned_duration_s=planned,duration_s=time.time()-started,
                     cumulative_duration_s=time.time()-self.state['started_at'],status=status)
            self._save(timeline=self.state['timeline']+[row],active_phase=None)

    def _run(self,data,token):
        moving=False;stopped=True;line=None;advance=[0.,time.monotonic()];delegate_started=False
        try:
            initial=self.ctrl.snapshot();height=initial['body']['height'];raw=initial['arms']['right']['gripper'];driver=self._driver()
            clamp=None
            def guard(state=None,now=None):
                s=self.ctrl.snapshot() if state is None else state
                if self.cancel.is_set():raise ValueError('阶段已停止')
                if token is not None and not self.ctrl.motion_valid(token):raise ValueError('阶段控制权已中断')
                if s['chassis_active'] or not s['arms']['right']['fresh'] or not s['body']['fresh']:raise ValueError('底盘运动或位姿反馈失效')
                if abs(s['body']['height']-height)>.02 or (clamp and abs(s['arms']['right']['gripper']-raw)>.15):raise ValueError('升降或夹持状态发生变化')
                current_driver=self._driver()
                if clamp:require_closed_target(current_driver)
                if current_driver.get('driver_instance')!=driver.get('driver_instance'):raise ValueError('右臂驱动发生变化')
                if line is not None and self.motion and self.motion.active=='arm':
                    start,axis,length,rotation=line;T=self._tool(s);delta=(T[:3,3]-start)*1000;progress=float(delta@axis)
                    angle=np.degrees(Rotation.from_matrix(rotation.T@T[:3,:3]).magnitude())
                    if np.linalg.norm(delta-progress*axis)>2.5 or angle>1.:raise ValueError('末端偏离直线或固定朝向')
                    if progress>advance[0]+.25:advance[:]=[progress,time.monotonic()]
                    if self._driver().get('status')!='running':advance[1]=time.monotonic()
                    elif progress<length-1.5 and time.monotonic()-advance[1]>2.:raise ValueError('末端停止前进，可能接触障碍')
                return True
            guard();settings=self.state['settings'];phase=data['phase']
            if phase=='insert':
                guard();s=self.ctrl.snapshot()
                if s['command_epoch']!=data['expected_epoch']:raise ValueError('入仓前控制状态已变化')
                command=dict(op='start',request_id=data['request_id'],expected_epoch=s['command_epoch'],
                    verify_insertion=True,cabinet_depth_mm=settings['insert_distance_mm'],speed_mm_s=settings['insert_speed_mm_s'])
                self.push.action(command);delegate_started=True
                while True:
                    if self.cancel.is_set():self.push.action({'op':'cancel','request_id':data['request_id']})
                    value=self.push.snapshot()
                    if value.get('request_id')!=data['request_id']:raise ValueError('入仓任务已被替换')
                    self._save(push=value,phase='insert',measured_travel_mm=value.get('measured_travel_mm'))
                    if value['status']!='running' and 'ended_at' in value:break
                    self.shutdown.wait(.1)
                delegate_started=False
                self._save(status=value['status'],phase='hold',reason=value.get('reason',''),
                    timeline=self.state['timeline']+[dict(phase='insert',planned_duration_s=value.get('planned_duration_s'),
                    duration_s=value.get('actual_total_s'),cumulative_duration_s=time.time()-self.state['started_at'],status=value['status'])])
                return
            clamp=self._timed('clamp',lambda:close_for_placement(self.ctrl,token,driver.get('driver_instance'),guard))
            raw=clamp['feedback_raw'];self._save(clamp=clamp)
            guard();initial=self.ctrl.snapshot()
            limits=motion_limits(json.loads((self.root/'config/battery-execution.json').read_text()),self.ctrl.config)
            model=self.ctrl.models['right'];self.motion=DockingMotion(self.ctrl,token,guard=guard,require_lift_settle=False,timeout_s=60.)
            def execute(points,times,label):
                nonlocal moving,stopped
                segments=split_plan(points,times)
                guard();s=self.ctrl.snapshot();d=self._driver();moving=True;stopped=False
                result=self._timed(label,lambda:self.motion.arm(segments,lambda:guard(),
                    expected_start=dict(joints=s['arms']['right']['joints'],height=height),
                    expected_driver={k:d[k] for k in ('driver_instance','command_epoch')}),float(times[-1]))
                stopped=True;return result
            def move_tool(target,label):
                nonlocal line
                s=self.ctrl.snapshot();q=np.asarray(s['arms']['right']['joints']);before=self._tool(s)
                if np.linalg.norm(target[:3,3]-before[:3,3])<.0002:
                    self._timed(label,lambda:None,0.);return
                path=plan_correction(model,q,height,self.tcp,target,self.ctrl.config['arms']['right']['joint_limits_deg'],
                    timing_limits=dict(limits,tcp_speed_mm_s=15.,tcp_acceleration_mm_s2=10.))
                atomic_json(self.run/(label+'-'+str(len(self.state['timeline']))+'-plan.json'),path)
                vector=target[:3,3]-before[:3,3];length=float(np.linalg.norm(vector))
                line=(before[:3,3].copy(),vector/length,length*1000,before[:3,:3].copy());advance[:]=[0.,time.monotonic()]
                end=execute(path['joint_waypoints_rad'],path['timing']['time_from_start_s'],label)
                line=None
                actual=self._tool(end);error,angle=check_tool_arrival(before,actual,target)
                self._save(endpoint_error_mm=error,endpoint_rotation_error_deg=angle)
            if phase=='preobserve':
                q=np.asarray(initial['arms']['right']['joints']);goal=np.radians(settings['preobserve_joints_deg'])
                if np.max(abs(q-goal))>np.radians(.1):
                    if not model.check(q,goal,height)['clear']:raise ValueError('预观测路径穿过机械臂基座')
                    path=time_preobserve(q,goal,model,height,self.tcp,limits,guard)
                    end=execute(path.q.tolist(),path.times.tolist(),'preobserve')
                    if np.max(abs(np.asarray(end['arms']['right']['joints'])-goal))>np.radians(2.):raise ValueError('预观测关节点未到达')
                else:self._timed('preobserve',lambda:None,0.)
                guard();self._save(status='completed',phase='hold',reason='已到预观测点；等待你选择下一步')
            elif phase=='z':
                s=self.ctrl.snapshot();reading=self._height(s,settings['z_reference'])
                if reading['tcp_z_mm'] is None:raise ValueError(reading.get('reason','高度依据不可用'))
                delta=data.get('delta_z_mm',settings['target_z_mm']-reading['tcp_z_mm'])
                target=z_target(self._tool(s),reading['up_base'],delta)
                move_tool(target,'z');self._save(status='completed',phase='hold',reason='Z 调整完成；X/Y 和末端朝向保持')
            elif phase=='correct':
                attempt=0;last_norm=None;last_step=0.
                while True:
                    guard();frame,geometry=self._timed('d405_observe',self._capture);guard();attempt+=1
                    s=self.ctrl.snapshot();T=self._tool(s);reading=self._visual_height(s,T)
                    error_z=settings['target_z_mm']-reading['tcp_z_mm'] if 'z' in settings['correction_axes'] else 0.
                    error_y=0.;dy=0.
                    if 'y' in settings['correction_axes']:
                        if geometry['lateral_anchor_base_m'] is None:raise ValueError('左右相对内沿未全部可见；可单独修正 Z')
                        body=battery_front(frame,geometry);axis=np.asarray(geometry['lateral_axis_base'])
                        observed=float(axis@(np.asarray(body['point_base_m'])-geometry['lateral_anchor_base_m']))*1000
                        error_y=settings['y_target_offset_mm']-observed
                        dy=float(np.clip(error_y,-settings['correction_step_mm'],settings['correction_step_mm']))/axis[1]
                    self._save(d405_attempt=attempt,d405_error_z_mm=error_z,d405_error_y_mm=error_y,
                               d405_frame_id=geometry['frame_id'],height=reading)
                    norm=max(abs(error_z),abs(error_y))
                    if last_norm is not None:require_residual_improvement(last_norm,norm,settings['tolerance_mm'],last_step)
                    if norm<=settings['tolerance_mm']:break
                    target=T.copy();target[1,3]+=dy/1000
                    dz=float(np.clip(error_z,-settings['correction_step_mm'],settings['correction_step_mm']))
                    target=z_target(target,geometry['up_base'],dz-np.asarray(geometry['up_base'])[1]*dy)
                    last_norm=norm;last_step=float(np.linalg.norm(target[:3,3]-T[:3,3])*1000)
                    move_tool(target,'d405_correct')
                self._save(status='completed',phase='hold',reason='D405 已收敛；等待你选择入仓，不自动触发')
        except Exception as e:
            if delegate_started:
                try:
                    self.push.action({'op':'cancel','request_id':data['request_id']})
                    worker=getattr(self.push,'thread',None)
                    if worker:worker.join(5.)
                    self._save(delegate_stop_pending=bool(worker and worker.is_alive()),push=self.push.snapshot())
                except Exception as stop_error:self._save(delegate_stop_pending=True,stop_error=str(stop_error))
            if self.motion:
                try:self.motion.hold()
                except Exception as stop_error:self._save(stop_error=str(stop_error))
            if moving:
                history=[];deadline=time.monotonic()+3.
                while time.monotonic()<deadline:
                    s=self.ctrl.snapshot();a=s['arms']['right'];d=self._driver()
                    if a['fresh'] and d.get('status') in ('idle','failed','reference_complete'):
                        history.append((a['received'],np.asarray(a['joints'])))
                        history=[v for v in history if history[-1][0]-v[0]<=.6]
                        if len(history)>=3 and history[-1][0]-history[0][0]>=.25 and np.max(np.ptp([v[1] for v in history],axis=0))<.002:
                            stopped=True;break
                    self.shutdown.wait(.08)
            self._save(status='held' if moving else 'blocked',phase='hold',reason=str(e))
        finally:
            if token is not None and stopped:
                try:self.ctrl.release_motion(token)
                except Exception as e:stopped=False;self._save(cleanup_error=str(e))
            self._save(ended_at=time.time(),total_duration_s=time.time()-self.state['started_at'],
                control_retained=bool(token and not stopped),final=self.ctrl.snapshot(),
                trajectory_handoffs=self.motion.handoffs if self.motion else [])

    def close(self):
        self.observing=False;self.cancel.set();self.shutdown.set()
        if self.state.get('operation')=='insert' and self.thread and self.thread.is_alive():
            try:self.push.action({'op':'cancel','request_id':self.state['request_id']})
            except ValueError:pass
        if self.thread:self.thread.join(3.)
        if self.observer:self.observer.join(2.)
