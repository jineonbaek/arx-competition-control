"""One measured test-T1 reverse; recover the same origin and stopping latch."""
import time,json
import numpy as np
import rclpy
from std_msgs.msg import Bool
from mppi_trial.stationary import StopWindow
from route_test_mode import Backoff

def run_backoff(n,epoch,chain):
    from special_runtime import transient_input,wait_special_ready
    mp,od,source=n.special_poses()
    state=getattr(chain,'backoff_state',None)
    if state is None:
        state=dict(policy=Backoff(od),complete=False);chain.backoff_state=state
        n.status('test_t1_backoff');n.waypoints.show('测试T1：后退目标10cm，上限20cm，不纠偏')
    policy=state['policy']
    if state['complete']:return policy.progress
    last_source=source;began=time.monotonic();stop=None;command=(0.,0.,0.)
    try:
        while rclpy.ok():
            tick=time.monotonic()
            try:
                n.current_intent(epoch)
                with n.transport.client.open(n.transport.origin+'/api/state',timeout=.3) as reply:state_console=json.load(reply)
                if (not state_console['execution_enabled'] or not state_console['body']['fresh'] or not state_console.get('nav_sdk_ready',False)
                    or state_console['capabilities']['chassis'] or (state_console.get('chassis_scheduler') or {}).get('maintenance')):
                    raise ValueError('test_backoff_console_unready')
                mp,od,source=n.special_poses()
                if source!=last_source:
                    chain.sample(mp,od,source,n.now());last_source=source;command=policy.command(od)
                    if policy.stopping and stop is None:
                        n.brake();stop=StopWindow(after=n.now());n.status('test_t1_backoff_verifying_stopped')
                    if stop is not None and stop.add(source,od):
                        n.validate_motion_proof(source);state['complete']=True
                        n.status('test_t1_backoff_complete '+str(round(policy.progress,4)));return policy.progress
                if time.monotonic()-tick>.20:raise ValueError('test_backoff_cycle_overrun')
                if time.monotonic()-began>=12.:raise ValueError('test_backoff_progress_recheck')
                n.current_intent(epoch);n.validate_motion_proof(source)
                if stop is None and any(command):
                    n.intent.publish(Bool(data=True));n.active=True;n.transport.send(command)
                else:n.brake()
                n.spin(max(.001,.05-(time.monotonic()-tick)))
            except Exception as exc:
                recoverable=(transient_input(exc) or str(exc).startswith(('test_backoff_console_unready','test_backoff_cycle_overrun','test_backoff_progress_recheck','special_localization_jump')))
                if not recoverable:raise
                n.brake();n.status('test_t1_backoff_recovering '+str(exc))
                mp,od,source=wait_special_ready(n,epoch,chain)
                # Keep the original origin and sticky stopping flag. In
                # particular, a target already reached can never reverse again.
                command=policy.command(od);last_source=source;began=time.monotonic()
                stop=StopWindow(after=n.now()) if policy.stopping else None
        raise ValueError('ros_shutdown')
    finally:n.brake()
