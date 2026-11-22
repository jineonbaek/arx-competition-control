"""Finish placement after the saved route endpoint and its eight-second push."""
import copy
import math
import threading
import time
import numpy as np
from placement_policy import CHASSIS_FORWARD_SDK, CHASSIS_FORWARD_SECONDS, CHASSIS_COMPLETION_BASIS


def special_target(definition):
    points=definition.get('points',[])
    indices=[i for i,p in enumerate(points) if p.get('point_id')=='point_2' and p.get('type')=='task']
    if len(indices)!=1 or indices[0]>=len(points)-1 or points[-1].get('type')!='special':
        raise ValueError('底盘入仓需要 T2 后已有的特殊终点；不会新建或改写点位')
    target=points[-1];pose=np.asarray(target.get('pose'),float)
    if pose.shape!=(3,) or not np.isfinite(pose).all():raise ValueError('底盘入仓特殊点坐标无效')
    return dict(index=len(points),id=target['id'],pose=pose.tolist())


class ChassisReleaseModule:
    navigation=False
    recoveries={}

    def __init__(self,placement):
        self.placement=placement;self.ctrl=placement.ctrl;self.backend=placement.placement
        self.runs={};self.lock=threading.RLock()

    def _prepared(self,context):
        prepared=self.placement.runs.get(context['run_id']+':place',{})
        if (prepared.get('placement') or {}).get('preobservation_ready') is not True:
            raise ValueError('本轮缺少底盘入仓的机械臂保持记录')
        return prepared

    def _feedback_guard(self,prepared,*,closed=True):
        state=self.ctrl.snapshot();arm=state['arms']['right']
        if not arm['fresh'] or not state['body']['fresh']:raise ValueError('底盘放置反馈过期')
        driver=self.ctrl.adapter.trajectory_status('right')
        if (not driver or driver.get('driver_instance')!=prepared['chassis_driver_instance']
                or not driver.get('ready') or not driver.get('enabled')):raise ValueError('底盘放置期间右臂驱动发生变化')
        if closed:
            from fast_visual_level import require_gripper_retained
            require_gripper_retained(arm['gripper'],prepared['chassis_gripper_raw'])
        return state

    def hold(self,context):
        if context['cancelled'].is_set():raise ValueError('底盘放置已停止')
        self._feedback_guard(self._prepared(context)['placement'])

    def forward(self,context,call):
        self.hold(context)
        self.backend.timeline.finish('ok')
        return self.backend._timed('chassis_forward',call,planned_duration_s=CHASSIS_FORWARD_SECONDS)

    def start(self,context):
        receipt=context.get('forward_receipt') or {}
        if (receipt.get('status')!='completed' or receipt.get('output_ended') is not True
                or type(receipt.get('duration_s')) not in (int,float)
                or not math.isfinite(receipt['duration_s']) or receipt['duration_s']<CHASSIS_FORWARD_SECONDS
                or receipt.get('sdk_forward')!=CHASSIS_FORWARD_SDK):
            raise ValueError(f'缺少特殊点后完整 {CHASSIS_FORWARD_SECONDS:g} 秒前进及停车回执，保持夹持')
        original=self._prepared(context)
        # The live preparation run also owns a Thread and Event. Only the
        # recorded placement and retreat data belong in the release snapshot.
        prepared=copy.deepcopy({key:original[key] for key in ('placement','chassis_start_pose')})
        with self.lock:
            key=context['request_id']
            if key in self.runs:raise ValueError('本轮底盘放置已处理，不能重复松爪或退出')
            run=dict(status='running',phase='release',reason='',cancel=threading.Event(),released=False,
                     returned=False,child=None,transport=None,motion=None,receipt=copy.deepcopy(receipt))
            self.runs[key]=run
            thread=threading.Thread(target=self._run,args=(context,run,prepared),daemon=True,name='chassis-placement-return')
            run['thread']=thread;thread.start()
        return self.inspect(context)

    def _guard(self,context,run,prepared):
        if (context['cancelled'].is_set() or run['cancel'].is_set()
                or not self.ctrl.motion_valid(context['token'])):raise ValueError('底盘放置已停止或控制权失效')
        return self._feedback_guard(prepared,closed=not run['released'])

    def _phase(self,run,name,call):
        run['phase']=name
        timed=getattr(self.backend,'_timed',None)
        return timed(name,call) if callable(timed) else call()

    def _run(self,context,run,original):
        prepared=original['placement']
        try:
            self.backend.save(status='running',phase='release',reason=f'底盘 {CHASSIS_FORWARD_SECONDS:g} 秒推进已结束；松爪退出')
            def release():
                state=self._guard(context,run,prepared)
                if state['chassis_active']:raise ValueError('底盘输出尚未结束，保持夹持')
                target=float(self.ctrl.config['arms']['right']['gripper_open_raw'])
                def preflight(state,now):
                    self._guard(context,run,prepared)
                    if state['chassis_active']:raise ValueError('松爪时底盘仍有输出')
                    return True
                reply=self.ctrl.submit(dict(type='gripper',arm='right',position=target),context['token'],preflight=preflight)
                if reply.get('ok') is not True:raise ValueError('松爪指令未确认')
                run['released']=True;self.backend.save(released=True)
                deadline=time.monotonic()+6.
                while time.monotonic()<deadline:
                    state=self._guard(context,run,prepared)
                    if abs(state['arms']['right']['gripper']-target)<=.35:return
                    run['cancel'].wait(.05)
                raise ValueError('夹爪张开反馈未到位')
            self._phase(run,'release',release)
            guard=lambda:self._guard(context,run,prepared)
            goal=original['chassis_start_pose']
            goal=[goal['x'],goal['y'],goal['yaw']]
            run['chassis_retreat']=self._phase(run,'chassis_retreat',lambda:self._retreat(context,run,goal,guard))
            self._phase(run,'arm_return',lambda:self._arm_return(context,run,prepared))
            run.update(status='completed',phase='complete',returned=True,insertion_depth_verified=False,
                       completion_basis=CHASSIS_COMPLETION_BASIS,reason='底盘入仓、松爪、退回 T2 并完成机械臂返回')
            timeline=getattr(self.backend,'timeline',None)
            if timeline:timeline.complete()
            self.backend.save(status='complete',phase='complete',released=True,returned=True,insertion=True,
                              insertion_depth_verified=False,completion_basis=run['completion_basis'],reason=run['reason'],
                              **(timeline.snapshot() if timeline else {}))
            parent=self.placement.runs[context['run_id']+':place']
            parent.update(placement_pending=False,phase='returned',completion_basis=run['completion_basis'])
        except Exception as error:
            run.update(status='stopped' if context['cancelled'].is_set() or run['cancel'].is_set() else 'failed',reason=str(error))
            self.backend.save(status='held',phase=run['phase'],released=run['released'],returned=False,reason=str(error))
        finally:
            try:self.ctrl.end_chassis_output(context['token'])
            except Exception:pass

    def _retreat(self,context,run,goal,guard):
        from t2_timeout_recovery import T2TimeoutRecovery
        return T2TimeoutRecovery(self.placement,self.placement.root)._return_chassis(context,run,goal,guard)

    def _arm_return(self,context,run,prepared):
        from docking_motion import DockingMotion
        from fast_visual_level import return_after_release,reverse_reached_route,require_joint_arrival
        guard=lambda *args:self._guard(context,run,prepared)
        motion=DockingMotion(self.ctrl,context['token'],guard=guard,timeout_s=180.,lift_tolerance_rad=.35,require_lift_settle=False,check_lift=False)
        run['motion']=motion
        try:
            return_after_release(self.ctrl,motion,self.ctrl.models['right'],self.backend.tcp,
                reverse_reached_route(prepared['chassis_return_route_rad']),self.backend.limits,guard,
                prepared['chassis_driver_instance'],self.backend.timeline.plan)
            require_joint_arrival(self.ctrl.snapshot()['arms']['right']['joints'],prepared['chassis_initial_joints_rad'],2.)
        except Exception:
            motion.hold();raise

    def inspect(self,context):
        with self.lock:
            run=self.runs.get(context['request_id'])
            if run is None:return dict(status='unknown',reason='没有本轮底盘松爪退出记录')
            return {k:copy.deepcopy(v) for k,v in run.items() if k not in ('thread','cancel','child','transport','motion')}

    def stop(self,context):
        run=self.runs.get(context['request_id'])
        if run is None:return dict(stopped=True)
        run['cancel'].set()
        if run.get('motion'):run['motion'].hold()
        self.ctrl.end_chassis_output(context['token']);run['thread'].join(4.)
        return dict(stopped=not run['thread'].is_alive())

    def resume(self,context):raise ValueError('核实本轮底盘和夹爪状态，不重放入仓动作')
