"""Special straight mode inside the existing driver; no second sender."""
import json,time
import numpy as np
import rclpy
from urllib.error import URLError,HTTPError
from tf2_ros import TransformException
from std_msgs.msg import Bool
from mppi_trial.stationary import StopWindow
from special_segments import StraightChain,RADIUS
from special_segments import body_delta
from mppi_trial.recovery import InputRecoveryWindow
from recovery_budget import paused_budget
from task_approach import TaskApproach,TaskStopReview

def transient_input(error):
    return (isinstance(error,TransformException) or
            isinstance(error,(TimeoutError,URLError)) and not isinstance(error,HTTPError) or
            str(error).startswith(('special_sensor_stale','bottom_obstacles_stale','dual_origin_mapping_stale',
                'special_pose_authority_invalid','special_pose_stale','special_odometry_gap',
                'special_source_repeated','special_cycle_overrun','special_console_unready','special_console_busy_or_unready',
                'task_worker_unavailable','motion_pose_stale_or_invalid','motion_pose_changed_during_proof')))

@paused_budget
def wait_special_ready(n,epoch,chain=None):
    n.brake();n.status('special_waiting_for_inputs');n.waypoints.show('特殊段停车等待新数据，保留入口与进度')
    stop=StopWindow(after=n.now());window=InputRecoveryWindow()
    while rclpy.ok():
        try:
            n.current_intent(epoch)
            with n.transport.client.open(n.transport.origin+'/api/state',timeout=.3) as reply:s=json.load(reply)
            if (s.get('chassis_scheduler') or {}).get('maintenance'):raise ValueError('maintenance_active')
            if not s['execution_enabled'] or not s['body']['fresh'] or not s.get('nav_sdk_ready',False) or s['capabilities']['chassis']:raise ValueError('special_console_unready')
            mp,od,source=n.special_poses()
            if chain is not None and chain.previous_sample is not None:
                _,old_m,old_o=chain.previous_sample;dm,do=body_delta(old_m,mp),body_delta(old_o,od)
                if max(np.linalg.norm(dm[:2]),np.linalg.norm(do[:2]))>.10 or max(abs(dm[2]),abs(do[2]))>.15 or np.linalg.norm(dm[:2]-do[:2])>.04 or abs(dm[2]-do[2])>.04:
                    raise ValueError('special_localization_jump')
            stable=stop.add(source,od)
            stamps=tuple((n.sensor[t].sec,n.sensor[t].nanosec) for t in ('/pb/scan','/localization/odometry'))
            revisions=stamps+((int(source*1e9),),(int(source*1e9),))
            if window.observe(time.monotonic(),stable,revisions):
                if chain is not None:
                    chain.previous_sample=(source,mp.copy(),od.copy());chain.measured=np.zeros(3)
                return mp,od,source
        except Exception as exc:
            if not transient_input(exc):raise
            window.observe(time.monotonic(),False,());stop=StopWindow(after=n.now())
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
    entry=n.waypoints.route.direct_source
    if entry is None or np.linalg.norm(mp[:2]-np.asarray(entry['pose'][:2]))>RADIUS:raise ValueError('special_entry_moved')
    task=getattr(n.waypoints.route,'mode','direct')=='task_direct'
    points=n.waypoints.route.direct_points()
    chain=(TaskApproach if task else StraightChain)(mp,od,points)
    review=TaskStopReview(points[0]) if task else None
    chain.previous_sample=(source,mp.copy(),od.copy())
    prior=np.zeros(3);last_source=source;last_tick=time.monotonic();stop=None;stop_stamp=None;stop_started=0.
    progress_pose=od.copy();progress_at=last_tick;deadline=last_tick+480
    n.status('task_direct_running' if task else 'special_straight_running');n.waypoints.show('任务点直线末段：位置与方向闭环，忽略代价地图' if task else '特殊直线段：忽略代价地图，保留急停/定位保护')
    def brake():
        if n.active:n.brake()
    def recover():
        nonlocal mp,od,source,prior,last_source,last_tick,stop,stop_stamp,progress_pose,progress_at,deadline
        brake();waiting=time.monotonic();mp,od,source=wait_special_ready(n,epoch,chain)
        deadline+=time.monotonic()-waiting
        prior=np.zeros(3);last_source=source;last_tick=time.monotonic();stop=None;stop_stamp=None
        progress_pose=od.copy();progress_at=last_tick
    try:
        while time.monotonic()<deadline:
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
            if source!=last_source:
                try:step=chain.step(mp,od,source,n.now(),max(.001,dt),prior)
                except Exception as exc:
                    if not transient_input(exc):raise
                    recover();continue
                last_source=source
                for ident in step.passed_ids:n.waypoints.route.advance_direct(ident);n.waypoints.show()
                prior=np.asarray(step.command)
                if step.terminal_near and stop is None:
                    brake();prior=np.zeros(3);stop=StopWindow(after=n.now());stop_started=time.monotonic();stop_stamp=None
                    n.status('task_verifying_stopped' if task else 'special_verifying_stopped')
            if stop is not None:
                if not task and np.linalg.norm(od[:2]-chain.target)>RADIUS:
                    stop=None;prior=np.zeros(3);progress_pose=od.copy();progress_at=time.monotonic()
                    n.status('special_terminal_correction');continue
                if source!=stop_stamp:
                    stop_stamp=source
                    stationary=stop.add(source,od)
                    if task and (stationary or time.monotonic()-stop_started>4.):
                        try:n.current_intent(epoch);n.validate_motion_proof(source)
                        except Exception as exc:
                            if not transient_input(exc):raise
                            recover();continue
                        verdict=review.check(mp,stationary)
                        if verdict in ('arrived','t1_fallback'):
                            if verdict=='t1_fallback':n.status('arrived_stopped_t1_fallback task_direct')
                            return 'arrived_stopped'
                        if verdict=='park':n.pause_terminal_correction(epoch,'task_direct_outside_arrival_tolerance_or_not_stationary')
                        stop=None;prior=np.zeros(3);progress_pose=od.copy();progress_at=time.monotonic()
                        n.status('task_terminal_correction');continue
                    if not task and stationary:return 'arrived_stopped'
            else:
                if tick-progress_at>=3.:
                    if np.linalg.norm(od[:2]-progress_pose[:2])<.006 and abs(od[2]-progress_pose[2])<.015:
                        if task and np.linalg.norm(mp[:2]-chain.goal[:2])<=RADIUS:
                            brake();prior=np.zeros(3);stop=StopWindow(after=n.now());stop_started=time.monotonic();stop_stamp=None
                            n.status('task_verifying_stopped');continue
                        raise ValueError('special_no_progress')
                    progress_pose=od.copy();progress_at=tick
                if time.monotonic()-tick>.20:raise ValueError('special_cycle_overrun')
                if any(abs(v)>1e-9 for v in prior):
                    try:
                        n.current_intent(epoch)
                        if task:n.validate_motion_proof(source)
                    except Exception as exc:
                        if not transient_input(exc):raise
                        recover();continue
                    n.intent.publish(Bool(data=True));n.active=True;n.transport.send(prior)
                else:brake()
            n.spin(max(.001,.05-(time.monotonic()-tick)))
        raise ValueError('special_timeout')
    finally:n.brake()
