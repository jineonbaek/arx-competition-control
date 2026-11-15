"""Special straight mode inside the existing driver; no second sender."""
import json,time
import numpy as np
from std_msgs.msg import Bool
from mppi_trial.stationary import StopWindow
from special_segments import StraightChain,RADIUS

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
            if h.status not in (1,2,3):continue
            if id(h) not in cancelled:
                h.cancel_goal_async();results[id(h)]=h.get_result_async();cancelled.add(id(h))
            if not results[id(h)].done():unresolved=True
            elif results[id(h)].result().status not in (4,5,6):raise ValueError('special_old_mppi_not_terminal')
        if not unresolved:break
        if time.monotonic()>deadline:raise ValueError('special_old_mppi_cancel_timeout')
        driver.current_intent(driver.seen);driver.spin(.025)
    driver.mppi_requests=[];driver.mppi_handle=None

def run_special(n):
    epoch=n.seen;n.current_intent(epoch)
    if not n.transport.native_sdk:raise ValueError('special_native_sdk_required')
    if n.waypoints.blocking():raise ValueError('special_task_owns_chassis')
    cancel_mppi(n);n.escape.clear()
    own=0
    def console():
        with n.transport.client.open(n.transport.origin+'/api/state',timeout=.3) as r:return json.load(r)
    state=console()
    if (not state['execution_enabled'] or not state['body']['fresh'] or state['chassis_active'] or state.get('motion_reserved',False) or not state.get('nav_sdk_ready',False) or state['capabilities']['chassis']):raise ValueError('special_console_busy_or_unready')
    baseline=state.get('command_epoch')
    if type(baseline) is not int:raise ValueError('special_missing_control_epoch')
    mp,od,source=n.special_poses()
    entry=n.waypoints.route.direct_source
    if entry is None or np.linalg.norm(mp[:2]-np.asarray(entry['pose'][:2]))>RADIUS:raise ValueError('special_entry_moved')
    chain=StraightChain(mp,od,n.waypoints.route.direct_points())
    prior=np.zeros(3);last_source=None;last_tick=time.monotonic();stop=None;stop_stamp=None
    progress_pose=od.copy();progress_at=last_tick;deadline=last_tick+480
    n.status('special_straight_running');n.waypoints.show('特殊直线段：忽略代价地图，保留急停/定位保护')
    def brake():
        nonlocal own
        if n.active:n.brake();own+=1
    try:
        while time.monotonic()<deadline:
            tick=time.monotonic();dt=tick-last_tick;last_tick=tick;n.current_intent(epoch)
            if n.waypoints.blocking():n.active=False;raise ValueError('special_task_owns_chassis')
            state=console()
            if state.get('command_epoch')!=baseline+own or state.get('motion_reserved',False):
                # Do not reacquire or send a stop into another owner's operation.
                # Existing SDK lease/deadman remains authoritative.
                n.active=False;raise ValueError('special_external_control_change')
            if not state['execution_enabled'] or not state['body']['fresh'] or not state.get('nav_sdk_ready',False) or state['capabilities']['chassis']:raise ValueError('special_console_unready')
            mp,od,source=n.special_poses()
            if dt>.30:raise ValueError('special_cycle_overrun')
            if source!=last_source:
                step=chain.step(mp,od,source,n.now(),max(.001,dt),prior);last_source=source
                for ident in step.passed_ids:n.waypoints.route.advance_direct(ident);n.waypoints.show()
                prior=np.asarray(step.command)
                if step.terminal_near and stop is None:
                    brake();prior=np.zeros(3);stop=StopWindow(after=n.now());n.status('special_verifying_stopped')
            if stop is not None:
                if np.linalg.norm(od[:2]-chain.target)>RADIUS:raise ValueError('special_outside_arrival_radius')
                if source!=stop_stamp:
                    stop_stamp=source
                    if stop.add(source,od):return 'arrived_stopped'
            else:
                if tick-progress_at>=3.:
                    if np.linalg.norm(od[:2]-progress_pose[:2])<.006 and abs(od[2]-progress_pose[2])<.015:raise ValueError('special_no_progress')
                    progress_pose=od.copy();progress_at=tick
                if time.monotonic()-tick>.20:raise ValueError('special_cycle_overrun')
                if any(abs(v)>1e-9 for v in prior):
                    n.current_intent(epoch);n.intent.publish(Bool(data=True));n.active=True;n.transport.send(prior);own+=1
                else:brake()
            n.spin(max(.001,.05-(time.monotonic()-tick)))
        raise ValueError('special_timeout')
    finally:brake()
