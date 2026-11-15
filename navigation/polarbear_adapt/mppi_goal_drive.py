"""RViz/waypoint goals -> audited Smac path -> native Omni MPPI -> guarded SDK."""
import fcntl
import json
import math
import os
import signal
import time
import copy
import hashlib

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from tf2_ros import TransformException

from goal_drive import Driver
from axis_trial import body_delta
from continuous_track import swept_poses
from mppi_trial.session_policy import changed_route, valid_effort, slew_effort, CommandEpoch, OwnedGoals, waitable_startup
from mppi_trial.trial_policy import ReadinessHold, speed_factors
from mppi_trial.stationary import StopWindow
from rclpy.time import Time
from follow_preview import fresh
from path_audit import heading
from mppi_trial.path_retention import safe_suffix
from mppi_trial.path_sampling import sampled_route
from mppi_trial.recovery import recoverable, RecoveryWindow, upcoming_route, input_unavailable, InputRecoveryWindow
from mission_route import path_covers_vias
from via_tracking import controller_route
from tracking_handoff import live_prefix
from escape_mode import EscapeMode
from async_path_audit import AsyncAuditor, NonblockingGrid, AuditPending
from executor_tf import TFReceiver


class MppiDriver(Driver):
    def __init__(self):
        super().__init__()
        self.listener.unregister()
        self.tf_receiver=TFReceiver(self.tf,self.context,use_sim_time=bool(self.get_parameter("use_sim_time").value))
        self.tf_diagnostic_at=0.
        self.transport.native_sdk=True
        self.mppi_received={}
        self.mppi_handle=None
        self.previous_candidate_route=None
        self.retention_failure=None
        self.recovery=None
        self.recovery_after=0.
        self.recovery_route=None
        self.recovery_blocker=None
        self.resume_pending=False
        self.escape=EscapeMode(self)
        self._special_execution=False
        self.mppi_requests=[]
        self.path_auditor=AsyncAuditor()
        self.pending_audit_path=None
        self.last_candidate_stamp=-math.inf
        self.candidate_owner=None;self.candidate_newest_ns=-1;self.candidate_digest=None
        self.active_controller_route=None
        self.audit_retry=self.create_timer(.01,self.retry_audit_path)
        ns=os.getenv('ARX_MPPI_NAMESPACE','/pb/mppi')
        self.mppi_client=ActionClient(self,FollowPath,os.getenv('ARX_MPPI_ACTION','/follow_path'))
        for key,kind,suffix in [('sdk',TwistStamped,'sdk_preview'),('status',String,'preview_status'),('inputs',Bool,'inputs_ready')]:
            self.create_subscription(kind,ns+'/'+suffix,lambda msg,k=key:self.mppi_received.update({k:(time.monotonic(),msg)}),1)

    def destroy_node(self):
        if hasattr(self,'tf_receiver'):self.tf_receiver.close()
        if hasattr(self,'path_auditor'):self.path_auditor.close()
        if hasattr(self,'escape'):self.escape.clear()
        if hasattr(self,'waypoints'):self.waypoints.close()
        return super().destroy_node()

    def spin(self,seconds):
        super().spin(seconds)
        # A busy iteration may leave only one callback slot. Consume a bounded
        # set of already-ready callbacks before using cached input next time.
        # All snapshot writes remain on this original ROS thread.
        deadline=time.monotonic()+.005
        for _ in range(16):
            if time.monotonic()>=deadline or not rclpy.ok():break
            rclpy.spin_once(self,timeout_sec=0.)

    def tf_diagnostic(self,frame,reason,source_stamp,elapsed):
        tick=time.monotonic()
        if tick-self.tf_diagnostic_at<1.:return
        self.tf_diagnostic_at=tick
        value=self.tf_receiver.diagnostic()
        value.update(frame=frame,reason=reason,lookup_ms=elapsed*1000,
                     source_age_ms=None if source_stamp is None else (self.now()-source_stamp)*1000,
                     sensor_receipt_gaps_ms={k:(tick-v)*1000 for k,v in self.sensor_received.items()})
        self.get_logger().warning('tf_read_diagnostic '+json.dumps(value))

    def pose(self,frame,now):
        # Preserve the concurrent TF-only receiver and diagnostics, while all
        # motion consumers use the same authoritative source-stamped batch.
        began=time.monotonic()
        try:return super().pose(frame,now)
        except ValueError:
            m=self.pose_snapshot
            source=None if m is None else m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            self.tf_diagnostic(frame,'motion_authority_invalid',source,time.monotonic()-began)
            raise

    def grid(self,message,frame):
        super().grid(message,frame)
        if frame in self.grids and hasattr(self,'path_auditor'):
            stamp,grid=self.grids[frame]
            self.grids[frame]=(stamp,NonblockingGrid(grid,self.path_auditor,frame,lambda:self.seq))

    def retry_audit_path(self):
        item=self.pending_audit_path
        if item is None:return
        epoch,message=item
        t=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if epoch!=self.seq or self.target is None or not 0<=self.now()-t<3.:
            self.pending_audit_path=None;return
        if self.stale_inputs():return
        self.pending_audit_path=None
        self.receive(message)

    def plan_changed(self,message):
        if self._special_execution:self.status('special_mode_goal_rejected');self.waypoints.show('特殊直线段执行中，请先取消路线');return
        stamp=message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        if stamp<=self.waypoints.accept_after_ns:
            self.status('pre_handoff_goal_rejected');return
        if self.waypoints.blocking():
            self.status('task_owns_chassis_goal_rejected');return
        return super().plan_changed(message)

    def cancel(self,why):
        if self._special_execution and why in ('replan_blocked','terminal_sweep_blocked','path_withdrawn'):return
        if self._special_execution:
            self._special_execution=False;self.waypoints.accept_after_ns=self.get_clock().now().nanoseconds
        if why in ('replan_blocked','terminal_sweep_blocked','path_withdrawn'):
            if self.retain_valid_route(why):return
            if self.retention_failure and not (recoverable(self.retention_failure) or input_unavailable(self.retention_failure)):
                why='stopped: '+self.retention_failure
            else:
                self.hold_for_replan(self.retention_failure or why)
                return
        if hasattr(self,'escape'):self.escape.clear()
        self.pending_audit_path=None
        if self.mppi_handle is not None and self.mppi_handle.status in (1,2,3):
            self.mppi_handle.cancel_goal_async()
        self.recovery=None
        self.resume_pending=False
        self.recovery_route=self.recovery_blocker=None
        self.active_controller_route=None
        super().cancel(why)

    def accept_goal(self,message):
        if self._special_execution:self.status('special_mode_goal_rejected');self.waypoints.show('特殊直线段执行中，请先取消路线');return
        stamp=message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        if stamp<=self.waypoints.accept_after_ns:
            self.status('pre_handoff_goal_rejected');return
        if self.waypoints.blocking():
            self.status('task_owns_chassis_goal_rejected');return
        previous=self.seq
        super().accept_goal(message)
        if self.seq!=previous:
            self.pending_audit_path=None
            self.last_candidate_stamp=-math.inf
            self.candidate_owner=None;self.candidate_newest_ns=-1;self.candidate_digest=None
            self.active_controller_route=None
            self.escape.reset()
            self.recovery=None
            self.resume_pending=False
            self.recovery_route=self.recovery_blocker=None

    def hold_for_replan(self,reason):
        self.brake()
        self.resume_pending=False
        if self.mppi_handle is not None and self.mppi_handle.status in (1,2,3):
            self.mppi_handle.cancel_goal_async()
        if self.recovery is None:
            previous=self.route if self.route is not None else self.previous_candidate_route
            self.recovery_route=None if previous is None else previous.copy()
        self.route=None
        if self.recovery is None:
            self.recovery=RecoveryWindow(time.monotonic())
            self.recovery_after=self.now()
            self.status('waiting_for_same_goal_replan '+reason)
            self.waypoints.show('等待安全路径')

    def current_intent(self,epoch):
        if self.seq!=epoch or self.target is None:raise ValueError('goal_changed')
        if self.ready_file is not None and not self.ready_file.is_file():
            raise ValueError('execution_gate_closed')

    def remember_mppi_request(self,future):
        retained=[]
        for f in self.mppi_requests:
            if not f.done():retained.append(f);continue
            try:h=f.result()
            except Exception:retained.append(f);continue
            if h is not None and h.accepted and h.status in (1,2,3):retained.append(f)
        self.mppi_requests=retained+[future]

    def recovery_console(self):
        with self.transport.client.open(self.transport.origin+'/api/state',timeout=.3) as reply:
            state=json.load(reply)
        if (not state['execution_enabled'] or not state['body']['fresh'] or
                state['chassis_active'] or state.get('motion_reserved',False) or
                not state.get('nav_sdk_ready',False) or state['capabilities']['chassis']):
            raise ValueError('console_not_ready_or_busy')
        return state.get('command_epoch')

    def wait_for_replan(self,epoch):
        # No goal/finished messages are emitted here: the existing Smac timer
        # continues planning to the same semantic target and stamp.
        window=self.recovery
        stopped=StopWindow(after=self.now())
        stop_stamp=-math.inf;stationary=False
        console_epoch=self.recovery_console();console_at=time.monotonic()
        paused=False
        while time.monotonic()-self.started<480:
            tick=time.monotonic();self.current_intent(epoch);self.brake()
            if tick-console_at>=.25:
                now_epoch=self.recovery_console();console_at=tick
                if console_epoch is not None and now_epoch!=console_epoch:
                    raise ValueError('external_command_during_replan')
            if window.paused(tick) and not paused:
                self.status('paused_waiting_for_safe_path')
                self.waypoints.show('暂停等待安全路径');paused=True
            if self.escape.maybe_run(epoch):
                self.route=None;self.recovery_after=self.now();window=RecoveryWindow(time.monotonic());self.recovery=window
                stopped=StopWindow(after=self.now());stationary=False;stop_stamp=-math.inf
            safe=False;revisions=(0,0)
            if stale:=self.stale_inputs():
                raise ValueError('sensor_or_costmap_stale '+','.join(stale))
            else:
                try:
                    pose=self.checked();odom=self.pose('odom',self.now())
                    _,odom,source_stamp=self.motion_poses()
                    if source_stamp!=stop_stamp:
                        stationary=stopped.add(source_stamp,odom);stop_stamp=source_stamp
                    if (self.route is not None and self.recovery_after<self.route_time<=self.now()
                            and self.now()-self.route_time<3.):
                        route=np.vstack((pose,self.route))
                        if np.linalg.norm(self.route[0,:2]-pose[:2])>.15:
                            raise ValueError('replan_blocked')
                        global_grid=self.grids['map'][1];local_grid=self.grids['odom'][1]
                        prefix=upcoming_route(route,global_grid.radius)
                        yaw=pose[2]-odom[2];c,s=math.cos(yaw),math.sin(yaw)
                        local=prefix.copy();local[:,:2]=(prefix[:,:2]-pose[:2])@np.array([[c,-s],[s,c]])+odom[:2]
                        local[:,2]-=yaw
                        safe=(stationary
                              and not global_grid.along(self.route)['first']
                              and not global_grid.along([pose,self.route[0]])['first']
                              and not local_grid.along(local)['first'])
                        revisions=tuple(m.sec*1000000000+m.nanosec for m,_ in (self.grids['map'],self.grids['odom']))
                except (ValueError,TransformException) as exc:
                    if isinstance(exc,TransformException) or not recoverable(str(exc)):raise
            if window.observe(tick,safe,revisions):
                self.current_intent(epoch)
                if self.recovery_console()!=console_epoch:
                    raise ValueError('external_command_during_replan')
                self.recovery=None
                self.recovery_route=self.recovery_blocker=None
                self.resume_pending=True
                self.status('checking_replanned_mppi_sweep');self.waypoints.show('复核MPPI扫掠')
                return
            self.spin(.025)
        raise ValueError('goal_timeout')

    def wait_for_inputs(self,epoch,reason):
        # A data outage is a parked state, not a completed/cancelled mission.
        # No controller result or sparse exception survives this boundary.
        self.brake();self.escape.clear();self.pending_audit_path=None
        if input_unavailable(self.escape.failed or ''):self.escape.failed=None
        if self.mppi_handle is not None and self.mppi_handle.status in (1,2,3):
            self.mppi_handle.cancel_goal_async()
        began=time.monotonic();window=InputRecoveryWindow()
        console_epoch=self.recovery_console();console_at=began
        self.status('waiting_for_inputs '+reason)
        self.waypoints.show('数据超时，停车等待')
        while rclpy.ok():
            tick=time.monotonic();self.current_intent(epoch);self.brake()
            if tick-console_at>=.25:
                if self.recovery_console()!=console_epoch:
                    raise ValueError('external_command_during_input_wait')
                console_at=tick
            ready=False;revisions=()
            try:
                # Occupancy is checked by the subsequent fresh replan and sweep.
                # TF, source freshness, SLAM jumps and all non-occupancy guards stay.
                self.checked(occupancy=False)
                at,inputs=self.mppi_received.get('inputs',(0,None))
                ready=(inputs is not None and inputs.data and tick-at<=.15)
                if ready:
                    stamps=[self.sensor[topic] for topic in ('/pb/scan','/localization/odometry')]
                    stamps.extend(self.grids[frame][0] for frame in ('map','odom'))
                    revisions=tuple((stamp.sec,stamp.nanosec) for stamp in stamps)
            except TransformException:
                pass
            except ValueError as exc:
                if not input_unavailable(str(exc)):raise
            if window.observe(tick,ready,revisions):
                self.current_intent(epoch)
                if self.recovery_console()!=console_epoch:
                    raise ValueError('external_command_during_input_wait')
                self.started+=time.monotonic()-began
                self.status('inputs_restored_waiting_fresh_plan')
                return
            self.spin(.025)
        raise ValueError('ros_shutdown')

    def retain_valid_route(self,reason):
        self.retention_failure=None
        if self.recovery is not None:return False
        previous=self.previous_candidate_route
        if previous is None or self.target is None:return False
        if np.linalg.norm(previous[-1,:2]-self.target[:2])>1e-6 or abs(math.atan2(math.sin(previous[-1,2]-self.target[2]),math.cos(previous[-1,2]-self.target[2])))>1e-6:return False
        started=time.monotonic()
        try:
            pose=self.checked()
            _,op,_=self.motion_poses()
            live_prefix(previous,pose,op,self.grids['map'][1],self.grids['odom'][1],self.waypoints.route.vias())
            if self.stale_inputs() or time.monotonic()-started>.20:return False
        except Exception as exc:
            self.retention_failure=str(exc)
            return False
        # Revalidate with fresh data without replacing/restarting the accepted geometry.
        self.route=previous
        self.status('retained_revalidated_path_after_'+reason)
        return True

    def receive(self,message):
        if self._special_execution:return
        if hasattr(self,'escape') and self.escape.busy:return
        if self.target is None:return
        ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec;t=ns*1e-9
        # Reject foreign/stale candidates before they can affect tracking,
        # pending work, freshness, via constraints or the SLAM alignment gate.
        if message.header.frame_id!='map' or not self.request<=t<=self.now() or self.now()-t>=3.:return
        pts=np.array([[p.pose.position.x,p.pose.position.y,heading(p.pose.orientation)] for p in message.poses])
        if len(pts) and (not np.isfinite(pts).all() or np.linalg.norm(pts[-1,:2]-self.target[:2])>=.06):return
        if self.candidate_owner!=self.seq:
            self.candidate_owner=self.seq;self.candidate_newest_ns=-1;self.candidate_digest=None
        digest=hashlib.sha256(pts.tobytes()).digest()
        if ns<self.candidate_newest_ns or (ns==self.candidate_newest_ns and self.candidate_digest not in (None,digest)):return
        if len(pts) and not path_covers_vias(pts,self.waypoints.route.vias()):
            self.previous_candidate_route=self.route
            try:
                if not self.retain_valid_route('via_candidate_rejected'):
                    self.hold_for_replan('replan_blocked via_candidate_missing')
            finally:self.previous_candidate_route=None
            return
        self.candidate_newest_ns=ns;self.candidate_digest=digest
        # This current valid candidate supersedes earlier pending geometry.
        self.pending_audit_path=None
        if len(pts):self.last_candidate_stamp=max(self.last_candidate_stamp,t)
        if len(pts) and (stale:=self.stale_inputs()):
            self.pending_audit_path=(self.seq,copy.deepcopy(message))
            # Input freshness is checked by the output loop. A briefly stale
            # candidate alone must not cancel the accepted native action.
            return
        # A near-goal planner withdrawal must not cancel an accepted controller result.
        # Current/goal footprints are still checked; distance alone never means arrival.
        if not message.poses and self.target is not None and self.mppi_handle is not None:
            try:
                delta=body_delta(self.pose('map',self.now()),self.target)
                if np.linalg.norm(delta[:2])<=.045 and abs(delta[2])<=.09:return
            except Exception:pass
        self.previous_candidate_route=self.route
        try:
            super().receive(message)
            if self.route is not None and self.waypoints.phase in ('后台校验新路径，沿安全原路径跟踪','后台校验首条路径'):
                self.waypoints.phase=None;self.waypoints.show()
            if self.route is not None and (stale:=self.stale_inputs()):
                self.pending_audit_path=(self.seq,copy.deepcopy(message))
            # Ignore malformed/stale input without consuming stateful checked() faults
            # or refreshing the old path's certification timestamp.
            if self.route is None and self.target is not None and self.recovery is None:
                self.route=self.previous_candidate_route
        except AuditPending:
            self.pending_audit_path=(self.seq,copy.deepcopy(message))
            # Pending is not a collision result. Restore accepted geometry and
            # its controller; the next command must freshly certify the complete
            # near-horizon body sweep in both maps before any SDK send.
            self.route=self.previous_candidate_route
            label='后台校验新路径，沿安全原路径跟踪' if self.route is not None else '后台校验首条路径'
            if self.waypoints.phase!=label:
                self.status('auditing_candidate_while_tracking' if self.route is not None else 'auditing_initial_path')
                self.waypoints.show(label)
        finally:self.previous_candidate_route=None

    def validate_active_prefix(self,pose,odom):
        if self.active_controller_route is None:raise ValueError('replan_blocked no_active_controller_geometry')
        return live_prefix(self.active_controller_route,pose,odom,self.grids['map'][1],self.grids['odom'][1],[])

    def make_request(self,route):
        path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        clipped=controller_route(route,self.waypoints.route.vias())
        if self.waypoints.route.vias() and self.grids['map'][1].along(clipped[-2:])['first']:
            raise ValueError('replan_blocked via_connector_blocked')
        for x,y,yaw in sampled_route(clipped):
            pose=PoseStamped();pose.header=path.header
            pose.pose.position.x,pose.pose.position.y=float(x),float(y)
            pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
            path.poses.append(pose)
        return FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id='goal_checker',progress_checker_id='progress_checker')

    def verify_arrival(self):
        self.escape.restore_for_arrival()
        if self.waypoints.route.vias():raise ValueError('replan_blocked')
        self.brake()
        window=StopWindow(after=self.now())
        deadline=time.monotonic()+4.
        while time.monotonic()<deadline:
            pose=self.checked()
            _,odom,source_stamp=self.motion_poses()
            if window.add(source_stamp,odom):
                error=body_delta(pose,self.target)
                if np.linalg.norm(error[:2])>.045 or abs(error[2])>.09:
                    raise ValueError(f'outside_arrival_tolerance xy={np.linalg.norm(error[:2]):.4f} yaw={error[2]:.4f}')
                return 'arrived_stopped'
            self.spin(.05)
        raise ValueError('not_stationary')

    def run_goal(self):
        if self._special_execution:
            from special_runtime import run_special
            return run_special(self)
        epoch=self.seen
        try:
            while True:
                self.current_intent(epoch)
                try:
                    if self.escape.maybe_run(epoch):self.hold_for_replan('escape_completed')
                    if self.recovery is not None:self.wait_for_replan(epoch)
                    return self.track_route()
                except (ValueError,TransformException) as exc:
                    self.current_intent(epoch)
                    reason='tf_missing' if isinstance(exc,TransformException) else str(exc)
                    if input_unavailable(reason):
                        self.wait_for_inputs(epoch,reason)
                        self.recovery=None;self.route=None
                        self.hold_for_replan('inputs_restored')
                        continue
                    if not recoverable(str(exc)):raise
                    self.hold_for_replan(str(exc))
        finally:self.escape.clear()

    def track_route(self):
        if not self.transport.native_sdk:raise ValueError('native_sdk_required')
        if not self.mppi_client.wait_for_server(timeout_sec=2.):raise ValueError('mppi_unavailable')
        pending=result=None
        pending_at=0.
        local_handle=None
        accepted_route=None
        tested_route_stamp=None
        epoch=self.seen
        hold=ReadinessHold()
        prior=measured=np.zeros(3)
        sample_pose=None
        last_tick=time.monotonic()
        last_consumed=0.
        startup=time.monotonic()
        owned=OwnedGoals()
        command_epoch=CommandEpoch()
        generation=0
        input_wait_reason=None
        pending_via_id=accepted_via_id=None
        pending_geometry=None
        self.active_controller_route=None
        def accept(future):
            handle=future.result()
            owned.remember(handle)
            if handle.accepted:
                if self.seq!=epoch:handle.cancel_goal_async()
        try:
            while time.monotonic()-self.started<480:
                tick=time.monotonic();dt=tick-last_tick;last_tick=tick
                if self.active and dt>.30:raise ValueError('control_cycle_overrun')
                self.current_intent(epoch)
                if self.recovery is not None:raise ValueError('replan_blocked')
                at,inputs=self.mppi_received.get('inputs',(0,None))
                ready=not self.stale_inputs() and inputs is not None and inputs.data and tick-at<=.15
                if not ready:
                    self.brake();prior=np.zeros(3)
                    if local_handle is None and tick-startup<3.:
                        self.spin(.025);continue
                    hold.update(False,tick)
                    self.spin(.025);continue
                try:
                    pose=self.checked()
                    _,odom,source_stamp=self.motion_poses()
                except (TransformException,ValueError) as exc:
                    reason='tf_missing' if isinstance(exc,TransformException) else str(exc)
                    if waitable_startup(reason,tick-startup,local_handle is not None):
                        self.brake();prior=np.zeros(3);self.spin(.025);continue
                    if not input_unavailable(reason):raise
                    self.brake();prior=np.zeros(3);hold.update(False,tick)
                    if input_wait_reason is None:
                        self.status('waiting_for_fresh_input '+reason)
                        self.waypoints.show('定位短断，停车等待新帧')
                    input_wait_reason=reason
                    self.spin(.025);continue
                self.waypoints.observe(pose,odom,source_stamp)
                if sample_pose is None:sample_pose,sample_at=odom,tick
                if tick-sample_at>=.25:
                    measured=body_delta(sample_pose,odom)/(tick-sample_at)
                    sample_pose,sample_at=odom,tick
                if pending is not None and pending.done():
                    local_handle=pending.result();pending=None
                    if not local_handle.accepted:raise ValueError('mppi_rejected_path')
                    self.mppi_handle=local_handle
                    accepted_via_id=pending_via_id
                    self.active_controller_route=pending_geometry.copy()
                    command_epoch.begin(generation,self.now())
                    result=local_handle.get_result_async()
                if result is not None and result.done() and pending is None:
                    outcome=result.result()
                    if outcome.status!=4 or outcome.result.error_code:
                        raise ValueError(f'mppi_failed status={outcome.status} code={outcome.result.error_code} {outcome.result.error_msg}')
                    if accepted_via_id is None:return self.verify_arrival()
                    if any(v['id']==accepted_via_id for v in self.waypoints.route.vias()):
                        raise ValueError('replan_blocked via_not_observed')
                    # A controller subgoal is not semantic arrival. Advance in
                    # this same loop without a programmed stop/stationary phase.
                    local_handle=result=None;accepted_route=None;tested_route_stamp=None;self.active_controller_route=None
                near=np.linalg.norm(body_delta(pose,self.target)[:2])<=.045
                if self.route is None or not 0<=self.now()-max(self.route_time,self.last_candidate_stamp)<3.:
                    if not (near and local_handle is not None):
                        self.brake();prior=np.zeros(3)
                        if tick-startup>10.:raise ValueError('no_current_path')
                        self.spin(.025);continue
                elif pending is None and (self.route_time,tuple(v['id'] for v in self.waypoints.route.vias()))!=tested_route_stamp:
                    tested_route_stamp=(self.route_time,tuple(v['id'] for v in self.waypoints.route.vias()))
                    clipped=controller_route(self.route,self.waypoints.route.vias())
                    if changed_route(accepted_route,clipped):
                        accepted_route=clipped.copy()
                        pending_geometry=clipped.copy()
                        pending_via_id=self.waypoints.route.vias()[0]['id'] if self.waypoints.route.vias() else None
                        generation+=1
                        pending=self.mppi_client.send_goal_async(self.make_request(self.route),
                            feedback_callback=lambda _,token=generation:command_epoch.feedback(token,self.now()))
                        self.remember_mppi_request(pending)
                        pending.add_done_callback(accept)
                        pending_at=tick
                if pending is not None and tick-pending_at>2.:raise ValueError('mppi_accept_timeout')
                if local_handle is None:
                    self.spin(.025);continue
                self.validate_active_prefix(pose,odom)
                at,sdk=self.mppi_received.get('sdk',(0,None))
                st,status=self.mppi_received.get('status',(0,None))
                fresh=(sdk is not None and status is not None and status.data=='preview_ready'
                       and tick-min(at,st)<=.15 and command_epoch.allows(
                           sdk.header.stamp.sec+sdk.header.stamp.nanosec*1e-9,self.now()))
                if not fresh:
                    self.brake();prior=np.zeros(3)
                    hold.update(False,tick);self.spin(.025);continue
                if at<=last_consumed:self.spin(.01);continue
                last_consumed=at
                v=sdk.twist;command=np.array([v.linear.x,v.linear.y,v.angular.z])
                command=self.escape.before_command(command)
                if not valid_effort(command):raise ValueError('invalid_sdk_effort')
                xy,yaw=speed_factors(measured);command[:2]*=xy;command[2]*=yaw
                command=np.array(slew_effort(prior,command,dt))
                command=self.escape.before_command(command)
                # Position is within the stateful goal checker's tolerance.
                # Brake translation while yaw finishes; measured-motion sweeps
                # and the independent final position check still apply.
                if near:command[:2]=0.
                for frame,current in [('map',pose),('odom',odom)]:
                    for sweep in swept_poses(current,command,measured):
                        if issue:=self.grids[frame][1].along(sweep)['first']:
                            self.recovery_blocker=(frame,issue['first_cell'])
                            raise ValueError(f'{frame}_sweep_blocked {issue}')
                if self.stale_inputs() or time.monotonic()-tick>.10:
                    self.brake();prior=np.zeros(3)
                    hold.update(False,time.monotonic());self.spin(.025);continue
                self.current_intent(epoch)
                try:self.validate_motion_proof(source_stamp)
                except ValueError as exc:
                    if not input_unavailable(str(exc)):raise
                    self.brake();prior=np.zeros(3);hold.update(False,time.monotonic())
                    if input_wait_reason is None:
                        self.status('waiting_for_fresh_input '+str(exc))
                        self.waypoints.show('定位短断，停车等待新帧')
                    input_wait_reason=str(exc)
                    self.spin(.025);continue
                if max(abs(command))>1e-6:
                    # No status/marker publication between this proof and SDK.
                    self.intent.publish(Bool(data=True));self.active=True
                    self.transport.send(command)
                else:self.brake()
                # Reset only after the final proof and output, never merely
                # because preview inputs were fresh before a costly sweep.
                hold.update(True,time.monotonic())
                if self.resume_pending:
                    self.resume_pending=False;self.status('following_replanned_path')
                    self.waypoints.phase=None;self.waypoints.show()
                if input_wait_reason is not None:
                    input_wait_reason=None;self.status('following_retained_path_after_fresh_input')
                    self.waypoints.phase=None;self.waypoints.show()
                prior=command
                self.spin(max(.001,.05-(time.monotonic()-tick)))
            raise ValueError('goal_timeout')
        finally:
            try:self.brake()
            finally:
                owned.close()
                self.mppi_handle=None
                self.active_controller_route=None

    def changed(self,message):
        if self._special_execution:return
        return super().changed(message)


if __name__=='__main__':
    if os.getenv('ROS_DOMAIN_ID')!='192' or os.getenv('ARX_REPLAY','0')=='1':
        raise SystemExit('Live NUC navigation domain required')
    lock=open('/tmp/arx-pb-goal-drive.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init();node=MppiDriver()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:node.serve()
    finally:node.brake();node.destroy_node();rclpy.try_shutdown()
