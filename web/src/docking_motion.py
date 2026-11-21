"""Native existing-controller ports. No SDK, driver lifecycle or gripper commands."""
from contextlib import contextmanager
import time
import numpy as np
from insertion_contact import InsertionContact
from navigation.pose_freshness import MOTION_MAX_AGE_NS


@contextmanager
def record_elapsed(detail, name, clock=time.monotonic):
    if detail is None:
        yield
        return
    started = clock()
    try:
        yield
    finally:
        detail[name] = detail.get(name, 0.) + (clock() - started)


class ArmStartChanged(ValueError):
    """Rejected in preflight, before this segment was handed to the arm."""


class DockPoseReader:
    """Reuse authoritative navigation source/costmap checks for a bounded goal."""
    def __init__(self,delta):
        from agent.chassis_retreat import _PoseReader
        self.reader=_PoseReader();self.delta=np.asarray(delta,float)
    def read(self):
        r=self.reader
        for _ in range(8):r.executor.spin_once(timeout_sec=0.)
        if not r.node.received_pose:return None
        mp,op,source=r.node.motion_poses()
        if r.node.target is None:
            c,s=np.cos(mp[2]),np.sin(mp[2]);d=self.delta
            r.node.target=mp+np.r_[np.array([[c,-s],[s,c]])@d[:2],d[2]]
        r.node.checked()
        if r.previous_pose is not None and source>r.previous_pose[1]:
            r.measured=r.body_delta(r.previous_pose[0],op)/(source-r.previous_pose[1])
        r.previous_pose=(op.copy(),source)
        return mp,op,source
    def now(self):return self.reader.now()
    def check_motion(self,command,source):return self.reader.check_motion(command,source)
    def close(self):self.reader.close()


class DockingMotion:
    def __init__(self,controller,token,*,guard,sleep=time.sleep,base_factory=DockPoseReader,
                 base_client=None,timeout_s=30.,lift_tolerance_rad=.02,require_lift_settle=True,check_lift=True):
        if not np.isfinite(lift_tolerance_rad) or not .02<=lift_tolerance_rad<=.35:raise ValueError('Invalid bounded lift tolerance')
        self.check_lift=bool(check_lift)
        self.lift_tolerance=lift_tolerance_rad
        self.require_lift_settle=bool(require_lift_settle)
        self.ctrl=controller;self.token=token;self.guard=guard;self.sleep=sleep
        self.base_factory=base_factory;self.base_client=base_client;self.timeout=timeout_s
        self.active=None;self.reader=None;self.transport=None
        self.expected_aux_driver=None
        self.handoffs=[]
        self.arm_handle=None
        self.contact_stop_pending=False

    def _binding(self,state,start,driver):
        if start is not None and (max(abs(a-b) for a,b in zip(state['arms']['right']['joints'],start['joints']))>np.radians(.1)
                or self.check_lift and abs(state['body']['height']-start['height'])>.02):raise ValueError('Planned auxiliary start changed')
        if driver is not None:
            current=self.ctrl.adapter.trajectory_status('right')
            if current is None or any(current.get(k)!=driver.get(k) for k in ('driver_instance','command_epoch')):
                raise ValueError('Holding arm driver changed during auxiliary movement')

    def _check(self,cancel):
        cancel()
        if not self.ctrl.motion_valid(self.token) or self.guard(self.ctrl.snapshot(),self.ctrl.clock()) is not True:
            raise ValueError('Motion ownership, feedback or scene changed')
        if self.active in ('lift','base'):self._binding(self.ctrl.snapshot(),None,self.expected_aux_driver)

    def _rest(self,group,after,cancel,*,target=None,settled_when=None):
        # Arm reference is already complete; lift acknowledgement only accepts
        # its target, so lift still needs the full bounded travel deadline.
        deadline=self.ctrl.clock()+(self.timeout if group=='lift' else min(8.,self.timeout));history=[];last_stamp=after
        while self.ctrl.clock()<deadline:
            self._check(cancel);s=self.ctrl.snapshot()
            row=s['arms']['right'] if group=='arm' else s['body']
            value=np.asarray(row['joints'] if group=='arm' else [row['height']],float)
            stamp=row['received']
            if not row['fresh'] or not np.isfinite(value).all():raise ValueError('Feedback unavailable during settling')
            if stamp>last_stamp:
                last_stamp=stamp;history.append((stamp,value.copy()))
                history=[v for v in history if stamp-v[0]<=.6]
                arrived=target is None or abs(value[0]-target)<=self.lift_tolerance
                if (arrived and len(history)>=3 and stamp-history[0][0]>=.25
                        and np.max(np.ptp([v for _,v in history],axis=0))<=.002
                        and (settled_when is None or settled_when(s))):return s
            self.sleep(.08)
        raise ValueError('Measured '+group+' arrival/rest not confirmed')

    def settled_arm(self,guard):
        return self._rest('arm',self.ctrl.clock(),guard)

    def arm(self,segments,cancel,*,expected_start=None,expected_driver=None,geometry_preflight=None,rail_start_check=None,rail_path=False,timing_detail=None,settled_when=None):
        if self.contact_stop_pending:raise ValueError('上次阻塞停止未确认，不能启动新轨迹')
        self.arm_handle=None
        self._check(cancel)
        start=expected_start;expected=expected_driver
        try:
            for plan in segments:
                self._check(cancel)
                # New drivers expose the exact goal supporting the load. Anchor
                # each segment there, including already-compensated plans, and
                # recheck shifted joint limits/arm clearance before dispatch.
                driver=self.ctrl.adapter.trajectory_status('right')
                if driver and 'joint_reference_rad' in driver:
                    from agent.held_placement_geometry import loaded_trajectory_plan
                    state=self.ctrl.snapshot();q=state['arms']['right']['joints'];h=state['body']['height']
                    plan,offset=loaded_trajectory_plan(plan,q,driver,self.ctrl.config['arms']['right']['joint_limits_deg'])
                    points=plan['joint_waypoints_rad'];model=self.ctrl.models['right']
                    with record_elapsed(timing_detail, 'loaded_check', self.ctrl.clock):
                        chain = [q] + points
                        if rail_path and hasattr(model, 'check_rail_path'):
                            clear = model.check_rail_path(chain, h)
                        else:
                            clear = all(model.check(a,b,h)['clear'] for a,b in zip(chain,points))
                        if not clear:
                            raise ValueError('Loaded trajectory arm sweep rejected')
                    self.handoffs.append(dict(driver_instance=driver['driver_instance'],command_epoch=driver['command_epoch'],
                        measured_joints_rad=list(q),held_reference_rad=driver['joint_reference_rad'],
                        first_command_rad=points[0],constant_shift_rad=offset.tolist()))
                    self._check(cancel)
                def dispatch_guard(state,now):
                    if start is not None:
                        if self.check_lift and abs(state['body']['height']-start['height'])>.02:
                            raise ArmStartChanged('Planned lift height changed at dispatch boundary')
                        if max(abs(a-b) for a,b in zip(state['arms']['right']['joints'],start['joints']))>np.radians(.1):
                            if rail_start_check is None:
                                raise ArmStartChanged('Planned arm start changed at dispatch boundary')
                            rail_start_check(state,start,plan['joint_waypoints_rad'][0])
                    if geometry_preflight is not None:geometry_preflight(state)
                    allowed=self.guard(state,now)
                    if allowed is True:self.active='arm'
                    return allowed
                with record_elapsed(timing_detail, 'dispatch', self.ctrl.clock):
                    handle=self.ctrl.start_trajectory('right',plan,self.token,preflight=dispatch_guard,expected_driver=expected,
                        require_lift_settle=self.require_lift_settle)
                self.arm_handle=dict(handle)
                if expected is not None:
                    expected=dict(driver_instance=handle['driver_instance'],command_epoch=handle['command_epoch']+1)
                with record_elapsed(timing_detail, 'reference', self.ctrl.clock):
                    end=self.ctrl.clock()+float(plan['time_from_start_s'][-1])+5.
                    while self.ctrl.clock()<end:
                        self._check(cancel)
                        if self.ctrl.poll_trajectory(self.token,preflight=self.guard)['status']=='reference_complete':break
                        self.sleep(.02)
                    else:raise ValueError('Arm reference completion timed out')
                with record_elapsed(timing_detail, 'settle', self.ctrl.clock):
                    settled=self._rest('arm',self.ctrl.clock(),cancel,settled_when=settled_when)
                start=dict(joints=settled['arms']['right']['joints'],height=settled['body']['height'])
            self.completed_driver=expected
            self.active=None
            return self.ctrl.snapshot()
        except InsertionContact as contact:
            self.confirm_contact_stop(contact)
            raise
        except Exception:self.hold();raise

    def confirm_contact_stop(self, contact):
        """Intentional stop, with the existing owner retained for visual checking."""
        # This latch outlives hold() and task exception cleanup. Neither stable
        # encoders nor merely sending stop can return an uncertain stop's owner.
        self.contact_stop_pending=True
        self.ctrl.begin_contact_stop(self.token)
        before=self.ctrl.adapter.trajectory_status('right')
        handle=self.arm_handle
        if (self.active!='arm' or not before or not handle or not self.ctrl.motion_valid(self.token)
                or any(before.get(k)!=handle.get(k) for k in ('driver_instance','request_id'))
                or before.get('command_epoch')!=handle['command_epoch']+1
                or before.get('status') not in ('running','reference_complete')
                or before.get('reference_from_trajectory') is not True
                or not before.get('ready') or not before.get('enabled')):
            raise ValueError('阻塞停止不再属于本轮有效轨迹，保持夹持')
        def stopped_status(state):
            # A lease/tracking fault may race the stop request. Its reason is
            # preserved by the driver and must never authorize seating release.
            return bool(state and state.get('driver_instance')==handle['driver_instance']
                and state.get('request_id')==handle['request_id']
                and state.get('command_epoch')==before['command_epoch']+1
                and state.get('reference_from_trajectory') is False
                and state.get('ready') and state.get('enabled')
                and (state.get('status')=='failed' and state.get('reason')=='manual_stop'
                     or state.get('status')=='reference_complete' and not state.get('reason')))
        self.hold()
        future=getattr(self.ctrl.adapter,'pending_arm_stops',{}).get('right')
        if future is None:
            raise ValueError('阻塞停止回执不可用')
        deadline=self.ctrl.clock()+3.
        while self.ctrl.clock()<deadline:
            self._check(lambda:None)
            state=self.ctrl.adapter.trajectory_status('right')
            if not state or state.get('driver_instance')!=before.get('driver_instance'):
                raise ValueError('阻塞停止期间驱动反馈变化')
            if future.done():
                try:accepted=future.result().success is True
                except Exception as error:raise ValueError('阻塞停止请求失败') from error
                if not accepted:raise ValueError('驱动拒绝阻塞停止')
                if state.get('status')=='failed' and state.get('reason')!='manual_stop':
                    raise ValueError('阻塞停止期间发生驱动故障，保持夹持')
                if stopped_status(state):
                    settled=self._rest('arm',self.ctrl.clock(),lambda:None)
                    self._check(lambda:None)
                    end=self.ctrl.adapter.trajectory_status('right')
                    if not stopped_status(end):
                        raise ValueError('阻塞停止后控制参考发生变化')
                    self.ctrl.complete_contact_stop(self.token)
                    self.completed_driver={k:end[k] for k in ('driver_instance','command_epoch')}
                    contact.evidence.update(stop_confirmed=True,reference_reset=True,
                        stop_confirmed_at=time.time(),stopped_joint_rad=settled['arms']['right']['joints'],
                        **self.completed_driver)
                    self.contact_stop_pending=False
                    return settled
            self.sleep(.02)
        raise ValueError('阻塞停止尚未确认，保持夹持')

    def lift(self,target,cancel,*,expected_start=None,expected_driver=None):
        if type(target) not in (int,float) or not np.isfinite(target) or not 0<=target<=18:
            raise ValueError('Lift target outside 0..18rad')
        self._check(cancel);self.expected_aux_driver=expected_driver
        try:
            def dispatch_guard(state,now):
                self._binding(state,expected_start,expected_driver)
                allowed=self.guard(state,now)
                if allowed is True:self.active='lift'
                return allowed
            reply=self.ctrl.submit(dict(type='lift',target=float(target)),self.token,preflight=dispatch_guard)
            if reply.get('ok') is not True:raise ValueError('Lift preflight did not dispatch target')
            future=self.ctrl.adapter.pending_lift;deadline=self.ctrl.clock()+2.
            while future is not None and not future.done() and self.ctrl.clock()<deadline:
                self._check(cancel);self.sleep(.02)
            if future is None or not future.done():raise ValueError('Lift command receipt unknown')
            result=future.result()
            if not result or not result.results or not all(r.successful for r in result.results):raise ValueError('Lift driver rejected target')
            self._rest('lift',self.ctrl.clock(),cancel,target=target)
            self.active=None;return self.ctrl.snapshot()
        except Exception:self.hold();raise

    def base(self,delta,cancel,*,expected_start=None,expected_driver=None):
        d=np.asarray(delta,float)
        if d.shape!=(3,) or not np.isfinite(d).all() or np.linalg.norm(d[:2])>.05 or abs(d[2])>.03:
            raise ValueError('Base step exceeds 50mm/0.03rad bound')
        if self.base_client is None:raise ValueError('Exclusive chassis identity required')
        from navigation.polarbear_adapt.chassis_command import WebChassis
        from navigation.polarbear_adapt.mppi_trial.stationary import StopWindow
        self._check(cancel);self.expected_aux_driver=expected_driver;self.active='base';self.reader=self.base_factory(d)
        sent=[False]
        def publish(command):
            # Stop output remains available on stale feedback or cancellation.
            if command['type']=='chassis.end_output':return self.ctrl.submit(command,self.base_client)
            with self.ctrl.lock:
                self._check(cancel)
                if not sent[0]:self._binding(self.ctrl.snapshot(),expected_start,expected_driver)
                result=self.ctrl.submit(command,self.base_client)
                if result.get('ok') is True:sent[0]=True
                return result
        self.transport=WebChassis(submit=publish)
        self.transport.sdk_units=True;self.transport.native_sdk=True
        from agent.pose_wait import PoseWait,pose_unavailable
        pose_wait=PoseWait(self.transport)
        deadline=self.ctrl.clock()+self.timeout;goal=None;last_source=None;stopping=None;previous=np.zeros(3)
        try:
            while self.ctrl.clock()<deadline or pose_wait.waiting:
                tick=self.ctrl.clock();was_waiting=pose_wait.waiting;resumed_at=None
                try:
                    self._check(cancel);sample=self.reader.read()
                    if sample is None:
                        pose_wait.begin(sent[0]);pose_wait.hold();self.sleep(.05);continue
                    _,pose,source=sample;pose=np.asarray(pose,float)
                    if not np.isfinite(np.r_[pose,source]).all() or not 0<=self.reader.now()-source<=MOTION_MAX_AGE_NS/1e9:raise ValueError('Base pose stale')
                    if pose_wait.waiting:
                        if not pose_wait.ready(sample,self.reader.now(),allow_motion=stopping is None):
                            self.sleep(.05);continue
                        if stopping is not None and pose_wait.window is not None:stopping=pose_wait.window
                        previous=np.zeros(3);last_source=None;resumed_at=self.ctrl.clock()
                    if last_source is not None and source<=last_source:self.sleep(.05);continue
                    if goal is None:
                        c,s=np.cos(pose[2]),np.sin(pose[2]);goal=pose+np.r_[np.array([[c,-s],[s,c]])@d[:2],d[2]]
                    dt=.05 if last_source is None else source-last_source;last_source=source
                    e=goal-pose;e[2]=np.arctan2(np.sin(e[2]),np.cos(e[2]))
                    c,s=np.cos(pose[2]),np.sin(pose[2]);xy=np.array([[c,s],[-s,c]])@e[:2]
                    if stopping is not None:
                        if np.linalg.norm(e[:2])>.008 or abs(e[2])>.02:raise ValueError('Base drifted outside measured stopping bound')
                        if stopping.add(source,pose):self.active=None;return self.ctrl.snapshot()
                        self.sleep(.05);continue
                    if np.linalg.norm(e[:2])<.005 and abs(e[2])<.01:
                        self.transport.end_output();stopping=StopWindow(after=self.reader.now());self.sleep(.05);continue
                    command=np.r_[xy*.8,e[2]*.5]
                    command[:2]*=min(1.,.02/max(np.linalg.norm(command[:2]),1e-9));command[2]=np.clip(command[2],-.02,.02)
                    command=np.clip(command,previous-.04*min(dt,.1),previous+.04*min(dt,.1))
                    self.reader.check_motion(command,source);self._check(cancel)
                    self.transport.send(command);previous=command;self.sleep(.05)
                except Exception as exc:
                    if not pose_unavailable(exc):raise
                    pose_wait.begin(sent[0]);pose_wait.hold();self.sleep(.05)
                finally:
                    if was_waiting or pose_wait.waiting:
                        deadline+=max(0.,(self.ctrl.clock() if resumed_at is None else resumed_at)-tick)
            raise ValueError('Base measured goal timed out')
        except Exception:self.hold();raise
        finally:
            if self.transport is not None:self.transport.end_output()
            if self.reader is not None:self.reader.close()
            self.reader=self.transport=None

    def hold(self):
        # No stop is sent after successful measured completion: preserve its
        # already-issued loaded joint reference and clamp target.
        if self.active=='arm':self.ctrl.submit(dict(type='arm.stop',arm='right'),self.token)
        elif self.active=='lift':self.ctrl.submit(dict(type='lift.stop'),self.token)
        elif self.active=='base' and self.transport is not None:self.transport.end_output()
        self.active=None
