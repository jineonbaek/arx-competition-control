from agent.insertion_verification import require_verified_receipt
"""Independent fixed-base visual placement; never registers or acknowledges navigation."""
import copy
import math
from pathlib import Path
import threading
import time

from agent.taught_placement import load_taught_route
from agent.visual_chassis_correction import (
    compare_observations, finish_visual_chassis_workflow, load_chassis_policy, plan_chassis_step,
)
from placement_reference_store import load_active_reference


class PlacementDebug:
    def __init__(self, controller, cabinet, placement, root, *, clock=time.time):
        self.ctrl,self.cabinet,self.placement=controller,cabinet,placement
        self.root=Path(root);self.clock=clock;self.lock=threading.RLock();self.thread=None
        self.cancel=threading.Event();self.requests={};self.token=None;self.arm_run_id=None
        self.state=dict(status='idle',phase='idle',run_id=None,reason='可独立调试，无需导航到点通知')

    def snapshot(self):
        with self.lock:
            robot=self.ctrl.snapshot();running=bool(self.thread and self.thread.is_alive())
            value=copy.deepcopy(self.state)
            value.update(running=running,mode='standalone_debug',moves_chassis=False,
                sends_navigation_receipt=False,command_epoch=robot['command_epoch'],
                can_start=not running and not any(robot.get(k) for k in ('motion_reserved','competition_reserved','chassis_active')))
            if self.arm_run_id:
                arm=self.placement.snapshot()
                if arm.get('run_id')==self.arm_run_id:value['placement']=arm
            return value

    def action(self,client,data):
        if not isinstance(data,dict):raise ValueError('无效调试请求')
        op=data.get('op');request=data.get('request_id')
        if not isinstance(request,str) or not 1<=len(request)<=128:raise ValueError('需要本轮请求编号')
        if op=='cancel':
            if set(data)!={'op','request_id'}:raise ValueError('无效停止请求')
            with self.lock:
                if request!=self.state.get('run_id'):raise ValueError('停止请求不属于当前调试轮次')
                if self.thread and self.thread.is_alive():
                    self.cancel.set()
                    if self.arm_run_id and self.placement.snapshot().get('run_id')==self.arm_run_id:
                        self.placement.action({'op':'cancel'})
                return self.snapshot()
        fields={'op','request_id'} if op=='observe' else {'op','request_id','expected_epoch','environment_clear','base_fixed_confirmed'}
        if op not in ('observe','place') or set(data)!=fields:raise ValueError('仅支持只定位、固定底盘放置和停止')
        with self.lock:
            old=self.requests.get(request)
            if old:
                if old!=(client,data):raise ValueError('同一请求编号不能改变内容')
                return self.snapshot()
            robot=self.ctrl.snapshot()
            if self.thread and self.thread.is_alive():raise ValueError('本轮视觉调试仍在进行')
            if any(robot.get(k) for k in ('motion_reserved','competition_reserved','chassis_active')):
                raise ValueError('请先结束其他任务并将底盘停稳')
            if op=='place' and (data['environment_clear'] is not True or data['base_fixed_confirmed'] is not True):
                raise ValueError('请确认底盘已停稳并固定、放置通道已清空')
            token=self.ctrl.reserve_motion(data['expected_epoch']) if op=='place' else None
            self.token=token;self.arm_run_id=None;self.cancel=threading.Event()
            self.state=dict(status='running',phase='locate',run_id=request,operation=op,
                            reason='正在读取 D435 仓口位置',started_at=self.clock(),image=None)
            self.requests[request]=(client,copy.deepcopy(data))
            self.thread=threading.Thread(target=self._run,args=(op,token,request),daemon=True,name='placement-debug')
            try:self.thread.start()
            except Exception:
                if token is not None:self.ctrl.release_motion(token)
                self.state.update(status='failed',reason='调试线程启动失败');raise
            return self.snapshot()

    def _check(self,token):
        if self.cancel.is_set():raise ValueError('本轮调试已停止')
        robot=self.ctrl.snapshot()
        if robot.get('chassis_active'):raise ValueError('底盘出现运动输出，视觉调试已停止')
        if token is not None and not self.ctrl.motion_valid(token):raise ValueError('本轮调试控制权失效')

    def _run(self,op,token,request):
        started=self.clock();retained=False
        try:
            self._check(token)
            result=self.cabinet.action({'camera':'d435','model':'demo'},include_arm_preview=False,token=token)
            self._check(token)
            with self.lock:self.state.update(image=result.get('image'),observation=result)
            if result.get('status')!='localized':raise ValueError(result.get('reason') or 'D435 本轮未定位到仓口')
            if result['window'].get('pose_ambiguous'):raise ValueError('D435 仓口朝向存在歧义')
            window=result['window'];current={'d435':dict(valid=True,captured_at=result['image_time'],
                center_base_m=window['center_base_m'],inside_axis_base=window['inside_axis_base'])}
            try:
                record=load_active_reference(self.root/'web/hardware/runtime/placement-references')
                policy=load_chassis_policy(self.root/'config/visual-chassis-placement.json')
                residual=compare_observations(record['capture'],current,policy,now=self.clock())
                step=plan_chassis_step(residual,policy)
                with self.lock:self.state.update(residual=residual,aligned=step['action']=='lock_admitted')
            except ValueError as error:
                if op=='place':raise
                with self.lock:self.state.update(reference_reason=str(error),aligned=False)
                step=None
            if op=='observe':
                with self.lock:self.state.update(status='observed',phase='located',reason='本轮定位完成；未移动底盘或机械臂')
                return
            if step['action']!='lock_admitted':raise ValueError('当前柜口偏差仍需调整，请先用导航或手动底盘对位，再重新定位')
            # The operator explicitly confirms fixed, stopped base. Fresh paired
            # cabinet capture additionally requires no active chassis output. No
            # navigation arrival, map pose or navigation receipt is synthesized.
            handoff=finish_visual_chassis_workflow(admitted=True,reason='manual_fixed_base_and_fresh_visual_alignment',
                chassis_pose={'source':'operator_confirmed_fixed_base','manual_debug':True},now=self.clock())
            with self.lock:
                self._check(token)
                arm=self.placement.start_owned(token,handoff)
                self.arm_run_id=arm['run_id'];self.state.update(phase='arm_placement',reason='正在执行机械臂放置')
            while True:
                self._check(token);arm=self.placement.snapshot()
                if arm.get('run_id')!=self.arm_run_id:raise ValueError('机械臂调试轮次发生变化')
                if arm.get('status')=='complete_unverified' and arm.get('released') is True and arm.get('returned') is True:
                    if self.placement.thread:
                        self.placement.thread.join(3.)
                        if self.placement.thread.is_alive():raise ValueError('未确认入仓的退出清理尚未结束')
                    with self.lock:self.state.update(status='completed_unverified',phase='returned',placement_confirmed=False,reason='入仓未确认；已松爪退出，本轮结束')
                    break
                if arm.get('status')=='complete':
                    route=load_taught_route(self.root/'config/taught-placement.json')
                    if not arm.get('released') or not require_verified_receipt(arm,route.full_depth_mm):raise ValueError('缺少完整推入和松爪回执')
                    if self.placement.thread:
                        self.placement.thread.join(3.)
                        if self.placement.thread.is_alive():raise ValueError('机械臂任务仍在清理')
                    with self.lock:self.state.update(status='completed',phase='returned',reason='放置并返回完成；未向导航发送回执')
                    break
                if arm.get('status') not in ('running','idle'):raise ValueError(arm.get('reason') or '机械臂放置未完成')
                self.cancel.wait(.1)
        except Exception as error:
            cleanup_error=None
            if self.arm_run_id and self.placement.snapshot().get('run_id')==self.arm_run_id:
                try:self.placement.action({'op':'cancel'})
                except Exception as stop_error:cleanup_error=str(stop_error)
                finally:
                    worker=self.placement.thread
                    if worker:
                        try:worker.join(3.)
                        except Exception as join_error:cleanup_error=str(join_error)
                        try:retained=worker.is_alive()
                        except Exception:retained=True
                    elif cleanup_error:retained=True
            with self.lock:
                self.state.update(status='cancelled' if self.cancel.is_set() else 'failed',reason=str(error))
                if cleanup_error:self.state.update(cleanup_reason=cleanup_error)
        finally:
            if token is not None and not retained:
                try:self.ctrl.release_motion(token)
                except Exception as error:
                    retained=True
                    with self.lock:self.state.update(status='recovery_required',reason=str(error))
            with self.lock:self.state.update(control_retained=retained,duration_s=self.clock()-started)

    def close(self):
        with self.lock:
            run=self.state.get('run_id');running=bool(self.thread and self.thread.is_alive())
        if running:
            self.action('shutdown',{'op':'cancel','request_id':run});self.thread.join(4.)
