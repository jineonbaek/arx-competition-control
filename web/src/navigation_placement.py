from agent.insertion_verification import require_verified_receipt
"""Join real point_2 arrival, D435 chassis admission and visually confirmed battery seating."""
import copy
import math
import json
from pathlib import Path
import threading
import time

import numpy as np

from agent.storage import atomic_json
from agent.taught_placement import load_taught_route
from agent.visual_chassis_correction import load_chassis_policy, run_visual_chassis_workflow
from placement_reference_store import load_active_reference


class NavigationPlacement:
    recoveries = {}
    preview_only = False

    def __init__(self, controller, cabinet, placement, navigation, root, *, clock=time.time,lidar_locked=False):
        self.ctrl, self.cabinet, self.placement, self.navigation = controller, cabinet, placement, navigation
        self.lidar_locked=lidar_locked
        self.root=Path(root);self.clock=clock;self.lock=threading.RLock();self.runs={};self.visits=set()

    @property
    def reference_root(self):return self.root/'web/hardware/runtime/placement-references'

    def readiness(self):
        try:
            if self.lidar_locked:
                from seated_reference import geometry_hashes
                standard=json.loads((self.root/'config/seated-reference.json').read_text())
                valid=standard['geometry_sha256']==geometry_hashes(self.root)
                return dict(connected=True,reference_valid=valid,reference_id=standard['reference_id'],
                            admission_source='lidar_point_2',reason='' if valid else '坐底标准与当前标定不符')
            record=load_active_reference(self.reference_root)
            sensor=record.get('capture',{}).get('d435') or {}
            center=np.asarray(sensor.get('center_base_m'),float)
            axis=np.asarray(sensor.get('inside_axis_base'),float)
            valid=(sensor.get('valid') is True and
                   center.shape==(3,) and axis.shape==(3,) and
                   np.isfinite(center).all() and np.isfinite(axis).all() and np.linalg.norm(axis[:2])>.01)
            return dict(connected=True,reference_valid=bool(valid),reference_id=record['reference_id'],
                        reason='' if valid else '柜口参考无效，请在确认的柜前位置记录当前位置和画面')
        except (ValueError,KeyError,OSError) as error:
            return dict(connected=True,reference_valid=False,reason=str(error))

    def start(self, context):
        with self.lock:
            visit=copy.deepcopy(self.navigation.visit)
            if (not self.navigation.active or not visit or visit.get('point_id')!='point_2'
                    or visit.get('mission_id')!=self.navigation.mission):
                raise ValueError('放置缺少本轮地点2停车交接')
            if not self.ctrl.motion_valid(context['token']):raise ValueError('放置控制权失效')
            identity=(visit['mission_id'],visit['visit_id'])
            if identity in self.visits:raise ValueError('本轮导航到访已处理，不能重复推入或松爪')
            result=self._launch(context,'lidar_stop' if self.lidar_locked else 'visual_chassis',visit_id=visit['visit_id'])
            self.visits.add(identity)
            return result

    def _launch(self,context,phase,**fields):
        # Called with self.lock held, so a request is recorded before the worker can finish.
        request=context['request_id']
        if request in self.runs:raise ValueError('本轮放置已启动，不能重复执行')
        if any(r['thread'].is_alive() for r in self.runs.values()):raise ValueError('已有放置任务')
        run=dict(status='running',phase=phase,reason='',run_id=request,
                 stop=threading.Event(),arm_run_id=None,**fields)
        thread=threading.Thread(target=self._run,args=(context,run),daemon=True,name='t2-placement')
        run['thread']=thread
        self.runs[request]=run
        thread.start()
        return self.inspect(context)

    def inspect(self,context):
        with self.lock:
            r=self.runs.get(context['request_id'])
            if r is None:return dict(status='unknown',reason='没有本轮放置记录')
            return {k:copy.deepcopy(v) for k,v in r.items() if k not in ('thread','stop')}

    def _check_owner(self,context,run):
        if context['cancelled'].is_set() or run['stop'].is_set():raise ValueError('放置已停止')
        if not self.ctrl.motion_valid(context['token']):raise ValueError('放置控制权失效')

    def _check(self,context,run):
        self._check_owner(context,run)
        visit=self.navigation.visit or {}
        if visit.get('visit_id')!=run['visit_id']:raise ValueError('导航交接身份已变化')
        error=self.navigation.check()
        if error:raise ValueError(error)

    def _observe(self,context,run):
        self._check(context,run)
        result=self.cabinet.action({'camera':'d435','model':'demo'},include_arm_preview=False,token=context['token'])
        if result.get('status')!='localized':raise ValueError(result.get('reason') or 'D435 仓口未定位')
        window=result['window']
        if window.get('pose_ambiguous'):raise ValueError('D435 仓口朝向存在歧义，不能修正底盘')
        return dict(d435=dict(valid=True,captured_at=result['image_time'],
                    center_base_m=window['center_base_m'],inside_axis_base=window['inside_axis_base']))

    def _pose(self):
        from navigation_view import read_snapshot
        pose=read_snapshot().get('pose') or {}
        if (pose.get('valid') is not True or pose.get('age_s',100)>0.5 or
                not all(isinstance(pose.get(k),(int,float)) and math.isfinite(pose[k]) for k in ('x','y','yaw'))):
            raise ValueError('底盘实测定位不可用')
        return {k:float(pose[k]) for k in ('x','y','yaw')}

    def _stop_chassis(self,context,run):
        self.ctrl.end_chassis_output(context['token'])
        start=None;deadline=time.monotonic()+3.;stable_since=time.monotonic()
        while time.monotonic()<deadline:
            self._check(context,run)
            state=self.ctrl.snapshot()
            try:pose=self._pose()
            except ValueError:
                start=None;run['stop'].wait(.05);continue
            if start is None:start=pose;stable_since=time.monotonic()
            if state['chassis_active']:raise ValueError('底盘存在其他输出，不能交接机械臂')
            if math.hypot(pose['x']-start['x'],pose['y']-start['y'])>.003 or abs(math.atan2(math.sin(pose['yaw']-start['yaw']),math.cos(pose['yaw']-start['yaw'])))>.01:
                start=pose;stable_since=time.monotonic()
            elif time.monotonic()-stable_since>=.7:
                return dict(stopped=True,pose=pose)
            run['stop'].wait(.05)
        raise ValueError('底盘实测停稳未确认')

    def _chassis_step(self,step,context,run):
        from navigation.polarbear_adapt.chassis_command import WebChassis
        self._check(context,run);start=self._pose()
        c,s=math.cos(start['yaw']),math.sin(start['yaw'])
        target=np.array([start['x']+c*step['forward_m']-s*step['lateral_m'],
                         start['y']+s*step['forward_m']+c*step['lateral_m'],
                         start['yaw']+math.radians(step['yaw_deg'])])
        transport=WebChassis(submit=lambda command:self.ctrl.submit(command,context['token']))
        deadline=time.monotonic()+8.
        try:
            while time.monotonic()<deadline:
                self._check(context,run);pose=self._pose();yaw=pose['yaw']
                delta=target-np.array([pose['x'],pose['y'],yaw]);delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
                if np.linalg.norm(delta[:2])<.003 and abs(delta[2])<math.radians(.4):break
                if math.hypot(pose['x']-start['x'],pose['y']-start['y'])>.065:raise ValueError('底盘单次修正实测位移超限')
                if abs(math.atan2(math.sin(yaw-start['yaw']),math.cos(yaw-start['yaw'])))>math.radians(3.):raise ValueError('底盘单次修正实测角度超限')
                body=np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])@delta[:2]
                velocity=body*3.;speed=np.linalg.norm(velocity)
                if speed>.10:velocity*=.10/speed
                transport.send([*velocity,float(np.clip(delta[2]*2.,-.10,.10))])
                run['stop'].wait(.05)
            else:raise ValueError('底盘修正未到位')
        finally:transport.end_output()
        self._stop_chassis(context,run)

    def _admit(self,context,run):
        if self.lidar_locked:
            stopped=self._stop_chassis(context,run)
            if stopped.get('stopped') is not True:raise ValueError('雷达地点 2 尚未停稳')
            receipt=dict(status='complete',outcome='locked_admitted',admission_source='lidar_point_2',
                         point_id='point_2',mission_id=self.navigation.mission,captured_at=self.clock(),
                         chassis_pose=stopped['pose'],chassis_locked=True,chassis_stopped=True,admission_valid=True)
        else:
            reference=load_active_reference(self.reference_root)['capture']
            if (reference.get('d435') or {}).get('valid') is not True:raise ValueError('柜口参考无效，请先记录确认的柜前画面')
            policy=load_chassis_policy(self.root/'config/visual-chassis-placement.json')
            path=self.root/'reports/visual-chassis-active/handoff.json';path.parent.mkdir(parents=True,exist_ok=True)
            receipt=run_visual_chassis_workflow(reference,policy,
                observe=lambda:self._observe(context,run),
                command_chassis=lambda step:self._chassis_step(step,context,run),
                stop_chassis=lambda:self._stop_chassis(context,run),
                write_handoff=lambda value:atomic_json(path,value|dict(navigation_visit_id=run['visit_id'])),
                clock=self.clock)
            if receipt['outcome']!='locked_admitted':raise ValueError(receipt['reason'])
        return receipt|dict(navigation_visit_id=run['visit_id'])

    def _run(self,context,run):
        try:
            self._check(context,run)
            route=load_taught_route(self.root/'config/taught-placement.json')
            receipt=self._admit(context,run)
            self._check(context,run)
            result=self.placement.start_owned(context['token'],receipt)
            run.update(phase='arm_placement',arm_run_id=result['run_id'])
            while True:
                self._check(context,run);result=self.placement.snapshot()
                with self.lock:
                    run['arm_progress']={k:result[k] for k in ('phase','d405_live','d405_attempt','d405_seat_attempt',
                        'd405_reason','reason','measured_travel_mm','released','returned') if k in result}
                if result.get('run_id')!=run['arm_run_id']:raise ValueError('放置执行身份已变化')
                if result.get('status')=='complete':
                    outside=result.get('handle_outside_mm')
                    if (result.get('released') is not True or result.get('returned') is not True or
                            not require_verified_receipt(result,route.full_depth_mm) or
                            abs(result['verified_depth_mm']-route.full_depth_mm)>2. or
                            type(outside) not in (float,int) or not math.isfinite(outside) or outside<=0.):
                        raise ValueError('缺少本体坐底、把手在外、松爪和返回回执')
                    if self.placement.thread:
                        self.placement.thread.join(3.)
                        if self.placement.thread.is_alive():raise ValueError('放置执行清理未结束')
                    run.update(status='completed',completed_at_monotonic=time.monotonic(),phase='returned',placement_confirmed=True,reason='',placement=result);return
                if result.get('status') not in ('running','idle'):raise ValueError(result.get('reason') or '放置未成功')
                run['stop'].wait(.1)
        except Exception as error:
            if run.get('arm_run_id') and self.placement.snapshot().get('run_id')==run['arm_run_id']:
                self.placement.action({'op':'cancel'})
                if self.placement.thread:self.placement.thread.join(3.)
            run.update(status='stopped' if context['cancelled'].is_set() or run['stop'].is_set() else 'failed',reason=str(error))
            try:self.ctrl.end_chassis_output(context['token'])
            except Exception as stop_error:run['stop_error']=str(stop_error)
            if run['status']=='failed' and self.navigation and self.navigation.active:
                try:self.navigation._event('fail',error=str(error)[:300])
                except Exception as receipt_error:run['receipt_error']=str(receipt_error)

    def stop(self,context):
        run=self.runs.get(context['request_id'])
        if run is None:return dict(stopped=True)
        run['stop'].set()
        owns_arm=run.get('arm_run_id') and self.placement.snapshot().get('run_id')==run['arm_run_id']
        if owns_arm:self.placement.action({'op':'cancel'})
        self.ctrl.end_chassis_output(context['token'])
        run['thread'].join(4.)
        worker=getattr(self.placement,'thread',None)
        return dict(stopped=not run['thread'].is_alive() and not (owns_arm and worker and worker.is_alive()))

    def resume(self,context):raise ValueError('需核实原放置状态，不能重复推入或松爪')

class CurrentPositionPlacement(NavigationPlacement):
    """Use the standalone T2 controller lease to run the normal D405 placement."""

    def __init__(self,controller,placement,root):
        super().__init__(controller,None,placement,None,root,lidar_locked=True)

    def readiness(self):
        return super().readiness() | dict(admission_source='t2_current_position')

    def _stationary(self):
        if self.ctrl.snapshot()['chassis_active']:
            raise ValueError('底盘仍有运动输出')

    def start(self,context):
        with self.lock:
            if context.get('route_scope')!='t2_test' or not self.ctrl.motion_valid(context['token']):
                raise ValueError('缺少本轮 T2 单测的机械臂控制权')
            self._stationary()
            return self._launch(context,'current_position')

    def _check(self,context,run):
        self._check_owner(context,run)
        self._stationary()

    def _admit(self,context,run):
        self._check(context,run)
        return dict(status='complete',outcome='locked_admitted',admission_source='t2_current_position',
                    test_run_id=context['run_id'],captured_at=self.clock(),chassis_locked=True,
                    chassis_stopped=True,admission_valid=True)
