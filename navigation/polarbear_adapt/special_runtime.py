"""Special straight mode inside the existing driver; no second sender."""
import json,time
import numpy as np
import rclpy
from urllib.error import URLError,HTTPError
from tf2_ros import TransformException
from std_msgs.msg import Bool,String
from terminal_forward import TerminalForward
from through_task import ThroughTaskChain
from mppi_trial.stationary import StopWindow
from special_segments import StraightChain,RADIUS
from special_segments import body_delta,wrap
from mppi_trial.recovery import InputRecoveryWindow
from recovery_budget import paused_budget
from task_approach import TaskApproach,TaskStopReview
from mppi_trial.speed_profile import SLOW_SDK,cap_sdk

class SlowTestChain(StraightChain):
    def step(self,*args,**kwargs):
        result=super().step(*args,**kwargs)
        result.command=cap_sdk(result.command,'slow')
        return result


class ThroughTestChain(StraightChain):
    """Ordinary pass points keep cruise; only the test task uses slow approach."""
    def __init__(self,mp,od,points):
        super().__init__(mp,od,points,continuous=True,terminal_precision=False)
        self.approach=None
    def step(self,mp,od,stamp,now,dt,previous):
        if self.approach is not None:
            self.approach.previous_sample=self.previous_sample
            result=self.approach.step(mp,od,stamp,now,dt,previous)
            self.previous_sample=self.approach.previous_sample;self.measured=self.approach.measured
            return result
        result=super().step(mp,od,stamp,now,dt,previous)
        if self.index<len(self.goals)-1:return result
        self.approach=SlowTestChain(mp,od,[self.points[-1]])
        approach=self.approach.step(mp,od,stamp,now,dt,result.command)
        self.previous_sample=self.approach.previous_sample;self.measured=self.approach.measured
        approach.passed_ids=result.passed_ids
        return approach

def transient_input(error):
    return (isinstance(error,HTTPError) and error.code in (502,503,504) or
            isinstance(error,TransformException) or
            isinstance(error,(TimeoutError,URLError)) and not isinstance(error,HTTPError) or
            str(error).startswith(('special_sensor_stale','bottom_obstacles_stale','dual_origin_mapping_stale',
                'special_pose_authority_invalid','special_pose_stale','special_odometry_gap',
                'special_source_repeated','special_cycle_overrun','special_console_unready','special_console_busy_or_unready',
                'task_worker_unavailable','previous_output_end_unconfirmed','motion_pose_stale_or_invalid','motion_pose_changed_during_proof')))

@paused_budget
def wait_special_ready(n,epoch,chain=None):
    n.status('special_waiting_for_inputs');n.waypoints.show('特殊段自动恢复：制动、核验连续新定位与停稳，保留点序')
    stop=StopWindow(after=n.now());map_stop=StopWindow(after=n.now());window=InputRecoveryWindow()
    anchor=None if chain is None else chain.previous_sample
    first_check=True;unproven_motion=False;last_reason=None;reason_at=0.;previous_recovery=None
    stop_stamp=None;stop_signature=None;stable=False;map_stable=False
    def reset():
        return StopWindow(after=n.now()),StopWindow(after=n.now()),InputRecoveryWindow()
    while rclpy.ok():
        try:
            n.current_intent(epoch)
            braked=n.soft_brake() if hasattr(n,'soft_brake') else (n.brake() is None)
            with n.transport.client.open(n.transport.origin+'/api/state',timeout=.3) as reply:s=json.load(reply)
            if (s.get('chassis_scheduler') or {}).get('maintenance'):raise ValueError('maintenance_active')
            if not braked or not s['execution_enabled'] or not s['body']['fresh'] or not s.get('nav_sdk_ready',False) or s['capabilities']['chassis'] or s.get('chassis_active',False):raise ValueError('special_console_unready')
            mp,od,source=n.special_poses()
            if previous_recovery is not None:
                previous_stamp,previous_m,previous_o=previous_recovery;dt=source-previous_stamp
                rm,ro=body_delta(previous_m,mp),body_delta(previous_o,od)
                coherent=(np.linalg.norm(rm[:2]-ro[:2])<=.04 and abs(wrap(rm[2]-ro[2]))<=.04)
                if dt<0 or (coherent and (max(np.linalg.norm(rm[:2]),np.linalg.norm(ro[:2]))>max(.06,.90*dt) or max(abs(rm[2]),abs(ro[2]))>max(.08,1.2*dt))):
                    unproven_motion=True;first_check=False
            if previous_recovery is None or source!=previous_recovery[0]:previous_recovery=(source,mp.copy(),od.copy())
            if anchor is not None:
                old_stamp,old_m,old_o=anchor;dm,do=body_delta(old_m,mp),body_delta(old_o,od)
                mismatch=float(np.linalg.norm(dm[:2]-do[:2]));yaw_error=abs(wrap(dm[2]-do[2]))
                travel=max(float(np.linalg.norm(dm[:2])),float(np.linalg.norm(do[:2])))
                elapsed=max(0.,source-old_stamp)
                # A stopped sample includes commanded travel and braking after
                # the last consumed pose. Test agreement and the elapsed-time
                # motion envelope, not a fixed 10cm total displacement.
                if first_check and mismatch<=.04 and yaw_error<=.04:
                    unproven_motion=(travel>max(.10,.90*elapsed+.03) or max(abs(dm[2]),abs(do[2]))>max(.15,1.2*elapsed+.03))
                    first_check=False
                if unproven_motion and travel<=.04 and max(abs(dm[2]),abs(do[2]))<=.04:unproven_motion=False
                if mismatch>.04 or yaw_error>.04 or unproven_motion:
                    reason=f'special_recovery_waiting_alignment map_odom_m={mismatch:.4f} yaw_rad={yaw_error:.4f} travel_m={travel:.4f} source_gap_s={elapsed:.3f}'
                    if reason!=last_reason and time.monotonic()-reason_at>2.:
                        n.status(reason);n.waypoints.show('自动恢复核验中：坐标连续性未通过，保持制动并等待有效定位')
                        last_reason=reason;reason_at=time.monotonic()
                    stop,map_stop,window=reset();stop_stamp=None;stop_signature=None;stable=map_stable=False;n.spin(.05);continue
            signature=tuple(mp)+tuple(od)
            # Only identical repeated batches may retain the last verdict.
            # A revised pose under the same stamp invalidates stopped proof.
            if source==stop_stamp and signature!=stop_signature:
                stop,map_stop,window=reset();stop_stamp=None;stop_signature=None;stable=map_stable=False
            if source!=stop_stamp:
                stable=stop.add(source,od);map_stable=map_stop.add(source,mp)
                stop_stamp=source;stop_signature=signature
            stamps=tuple((n.sensor[t].sec,n.sensor[t].nanosec) for t in ('/pb/scan','/localization/odometry'))
            revisions=stamps+((int(source*1e9),),(int(source*1e9),))
            if window.observe(time.monotonic(),stable and map_stable,revisions):
                n.current_intent(epoch);n.validate_motion_proof(source)
                if chain is not None:chain.previous_sample=(source,mp.copy(),od.copy());chain.measured=np.zeros(3)
                n.status('special_inputs_restored');n.waypoints.show('新数据与停稳验证通过，自动继续原特殊段')
                return mp,od,source
        except Exception as exc:
            if not transient_input(exc):raise
            stop,map_stop,window=reset();stop_stamp=None;stop_signature=None;stable=map_stable=False
        n.spin(.05)
    raise ValueError('ros_shutdown')

def cancel_mppi(driver):
    pending=list(driver.mppi_requests);handles=[]
    if driver.mppi_handle is not None:handles.append(driver.mppi_handle)
    cancelled=set();results={};deadline=time.monotonic()+2.
    while True:
        unresolved=False
        for f in pending:
            if not f.done():unresolved=True;continue
            h=f.result()
            if h is not None and h.accepted and h not in handles:handles.append(h)
        for h in handles:
            if h.status in (4,5,6):continue
            if h.status not in (1,2,3):unresolved=True;continue
            if id(h) not in cancelled:
                h.cancel_goal_async();results[id(h)]=h.get_result_async();cancelled.add(id(h))
            if not results[id(h)].done():unresolved=True
            elif results[id(h)].result().status not in (4,5,6):raise ValueError('special_old_mppi_not_terminal')
        if not unresolved:break
        if time.monotonic()>deadline:raise ValueError('special_old_mppi_cancel_timeout')
        driver.current_intent(driver.seen);driver.spin(.025)
    driver.mppi_requests=[];driver.mppi_handle=None

def run_special(n):
    epoch=n.seen
    if hasattr(n,'wait_worker_ready'):n.wait_worker_ready(epoch)
    n.current_intent(epoch)
    if not n.transport.native_sdk:raise ValueError('special_native_sdk_required')
    if n.waypoints.blocking():raise ValueError('special_task_owns_chassis')
    if hasattr(n,'wait_controller_quiet'):n.wait_controller_quiet(epoch)
    else:cancel_mppi(n)
    n.escape.clear()
    def console():
        with n.transport.client.open(n.transport.origin+'/api/state',timeout=.3) as r:return json.load(r)
    try:
        state=console()
        if (not state['execution_enabled'] or not state['body']['fresh'] or not state.get('nav_sdk_ready',False) or state['capabilities']['chassis']):raise ValueError('special_console_busy_or_unready')
        if (state.get('chassis_scheduler') or {}).get('maintenance'):raise ValueError('maintenance_active')
        mp,od,source=n.special_poses()
    except Exception as exc:
        if not transient_input(exc):raise
        mp,od,source=wait_special_ready(n,epoch)
    session=getattr(n,'special_session',None)
    if session is None or session['epoch']!=epoch:
        entry=n.waypoints.route.direct_source
        if entry is None or np.linalg.norm(mp[:2]-np.asarray(entry['pose'][:2]))>RADIUS:raise ValueError('special_entry_moved')
        task=getattr(n.waypoints.route,'mode','direct')=='task_direct'
        points=n.waypoints.route.direct_points();test_point=points[-1].get('test_task_id')
        # Test T2 shares parking geometry/policy, while its route marker stays
        # special: no worker ownership, real task handoff or T1 fixed backoff.
        oriented=task or test_point=='point_2'
        control_point=dict(points[-1],type='task',point_id='point_2') if test_point=='point_2' else points[-1]
        arrival=n.waypoints.runtime.t2_policy(control_point) if oriented and control_point.get('point_id')=='point_2' else None
        if oriented:
            chain=(ThroughTaskChain(mp,od,points[:-1]+[control_point],arrival_policy=arrival) if len(points)>1 else TaskApproach(mp,od,[control_point],arrival_policy=arrival))
        elif test_point:chain=(ThroughTestChain if len(points)>1 else SlowTestChain)(mp,od,points)
        else:chain=StraightChain(mp,od,points,continuous=True,terminal_precision=False)
        review=TaskStopReview(control_point,arrival_policy=arrival) if oriented else None
        chain.previous_sample=(source,mp.copy(),od.copy())
        session=dict(terminal_forward=None,epoch=epoch,task=task,oriented=oriented,control_point=control_point,points=points,test_point=test_point,chain=chain,review=review,stage='approach')
        n.special_session=session
    else:
        task=session['task'];oriented=session['oriented'];control_point=session['control_point'];points=session['points'];test_point=session['test_point'];chain=session['chain'];review=session['review']
    suspended=session.pop('terminal_suspended_at',None)
    if suspended is not None and session['terminal_forward'] is not None:
        session['terminal_forward'].end+=time.monotonic()-suspended
    if session['stage']=='backoff':
        from route_backoff_runtime import run_backoff
        run_backoff(n,epoch,chain);return 'arrived_stopped'
    if not task and not test_point:chain.reset_start_assist()
    prior=np.zeros(3);last_source=source;last_tick=time.monotonic();stop=None;stop_stamp=None;stop_started=0.
    progress_pose=od.copy();progress_at=last_tick
    n.status('task_direct_running' if task else 'test_task_direct_running' if test_point else 'special_straight_running');n.waypoints.show((f'测试任务点：慢速SDK上限{SLOW_SDK:g}，T1后退／T2不后退，忽略代价地图' if test_point else f'任务点直线末段：慢速SDK上限{SLOW_SDK:g}，位置与方向闭环，忽略代价地图' if task else '特殊直线段：忽略代价地图，保留急停/定位保护'))
    def brake():
        if n.active:n.brake()
    def recover():
        nonlocal mp,od,source,prior,last_source,last_tick,stop,stop_stamp,progress_pose,progress_at
        was_stopping=stop is not None
        brake();waiting=time.monotonic();mp,od,source=wait_special_ready(n,epoch,chain)
        if session['terminal_forward'] is not None:session['terminal_forward'].end+=time.monotonic()-waiting
        if not task and not test_point:chain.reset_start_assist()
        if was_stopping and not oriented:chain.terminal_correction=True
        prior=np.zeros(3);last_source=source;last_tick=time.monotonic();stop=None;stop_stamp=None
        progress_pose=od.copy();progress_at=last_tick
    def terminal_permit(state):
        route=n.waypoints.route
        value=state.get('terminal_forward')
        if value and value.get('session_active'):
            if (value.get('route_id')!=route.route_id or value.get('mission_id')!=route.mission_id
                    or value.get('point_id')!=points[-1]['id']):raise ValueError('terminal_forward_identity_mismatch')
            return value
        # Standalone RViz test routes have no arm work or web task owner.
        if any(p.get('type')=='task' for p in route.points):raise ValueError('terminal_forward_web_guard_missing')
        return dict(route_id=route.route_id,mission_id=route.mission_id,point_id=points[-1]['id'],
                    authorized=True,sdk_forward=.225,duration_s=4.)
    try:
        while rclpy.ok():
            tick=time.monotonic();dt=tick-last_tick;last_tick=tick
            if n.waypoints.blocking():n.active=False;raise ValueError('special_task_owns_chassis')
            try:
                n.current_intent(epoch)
                state=console()
                if (state.get('chassis_scheduler') or {}).get('maintenance'):raise ValueError('maintenance_active')
                if not state['execution_enabled'] or not state['body']['fresh'] or not state.get('nav_sdk_ready',False) or state['capabilities']['chassis']:raise ValueError('special_console_unready')
                mp,od,source=n.special_poses()
                if dt>.30:raise ValueError('special_cycle_overrun')
            except Exception as exc:
                if not transient_input(exc):raise
                recover();continue
            forward=session['terminal_forward']
            if forward is not None and stop is None:
                prior=np.asarray(forward.command(terminal_permit(state),time.monotonic()))
                if not np.any(prior):
                    brake();stop=StopWindow(after=n.now());stop_started=time.monotonic();stop_stamp=None
                    n.status('terminal_forward_verifying_stopped')
            if source!=last_source and forward is None:
                try:step=chain.step(mp,od,source,n.now(),max(.001,dt),prior)
                except Exception as exc:
                    if not transient_input(exc):raise
                    recover();continue
                last_source=source
                for ident in step.passed_ids:n.waypoints.route.advance_direct(ident);n.waypoints.show()
                prior=np.asarray(step.command)
                if step.terminal_near and stop is None and not oriented and not test_point:
                    route=n.waypoints.route
                    if route.target_index!=route._execution_limit()-1:raise ValueError('special_continuous_successor_not_grouped')
                    session['terminal_forward']=TerminalForward(terminal_permit(state),time.monotonic())
                    prior=np.asarray(session['terminal_forward'].command(terminal_permit(state),time.monotonic()))
                    n.status('special_terminal_forward_continuous')
                elif step.terminal_near and stop is None:
                    # Persist before a stop command can fail and unwind outward.
                    if not oriented:chain.terminal_correction=True
                    brake();prior=np.zeros(3);stop=StopWindow(after=n.now());stop_started=time.monotonic();stop_stamp=None
                    n.status('task_verifying_stopped' if oriented else 'special_verifying_stopped')
            if stop is not None:
                # Keep the stop latched through coast; no moving reverse chase.
                if not oriented or control_point.get('point_id')=='point_2':n.transport.send((0.,0.,0.))
                if source!=stop_stamp:
                    stop_stamp=source
                    stationary=stop.add(source,od)
                    if oriented and (stationary or control_point.get('point_id')!='point_2' and time.monotonic()-stop_started>4.):
                        try:n.current_intent(epoch);n.validate_motion_proof(source)
                        except Exception as exc:
                            if not transient_input(exc):raise
                            recover();continue
                        verdict=review.check(mp,stationary)
                        if verdict in ('arrived','t1_fallback','t2_fallback'):
                            if verdict=='t1_fallback':n.status('arrived_stopped_t1_fallback task_direct')
                            if verdict=='t2_fallback':n.status('arrived_stopped_t2_fallback '+('task_direct' if task else 'test_task_direct'))
                            return 'arrived_stopped'
                        if verdict=='park':
                            n.pause_terminal_correction(epoch,'task_direct_outside_arrival_tolerance_or_not_stationary')
                            review=TaskStopReview(control_point,arrival_policy=chain.arrival_policy);session['review']=review
                        stop=None;prior=np.zeros(3);progress_pose=od.copy();progress_at=time.monotonic()
                        n.status('task_terminal_correction');continue
                    if not oriented and stationary:
                        if session['terminal_forward'] is not None:
                            receipt=session['terminal_forward'].receipt(time.monotonic())
                            n.waypoints.terminal_forward_pub.publish(String(data=json.dumps(receipt)))
                            return 'arrived_stopped'
                        if np.linalg.norm(od[:2]-chain.target)>RADIUS:
                            chain.terminal_correction=True
                            stop=None;prior=np.zeros(3);progress_pose=od.copy();progress_at=time.monotonic()
                            n.status('special_terminal_correction');continue
                        if test_point=='point_1':
                            session['stage']='backoff'
                            from route_backoff_runtime import run_backoff
                            run_backoff(n,epoch,chain)
                        return 'arrived_stopped'
            else:
                if tick-progress_at>=3.:
                    if np.linalg.norm(od[:2]-progress_pose[:2])<.006 and abs(od[2]-progress_pose[2])<.015:
                        if oriented and np.linalg.norm(mp[:2]-chain.goal[:2])<=RADIUS:
                            brake();prior=np.zeros(3);stop=StopWindow(after=n.now());stop_started=time.monotonic();stop_stamp=None
                            n.status('task_verifying_stopped');continue
                        if not task and not test_point and session['terminal_forward'] is None and np.linalg.norm(od[:2]-chain.target)>.5:
                            floor=chain.request_start_assist()
                            n.status(f'special_start_assist_sdk {floor:.2f}')
                        else:n.status('special_no_progress_observed')
                        # No-progress never brakes or restarts the segment.
                        # Fresh-source/control/arrival checks still gate output.
                    progress_pose=od.copy();progress_at=tick
                if time.monotonic()-tick>.20:
                    recover();continue
                if any(abs(v)>1e-9 for v in prior):
                    try:
                        n.current_intent(epoch)
                        n.validate_motion_proof(source)
                    except Exception as exc:
                        if not transient_input(exc):raise
                        recover();continue
                    n.intent.publish(Bool(data=True));n.active=True
                    if session['terminal_forward'] is not None:
                        remaining=session['terminal_forward'].end-time.monotonic()
                        n.transport.send(prior,ttl_s=min(.15,max(.001,remaining)))
                    else:n.transport.send(prior)
                else:brake()
            n.spin(max(.001,.05-(time.monotonic()-tick)))
        raise ValueError('ros_shutdown')
    finally:
        n.brake()
        if session['terminal_forward'] is not None:session['terminal_suspended_at']=time.monotonic()
