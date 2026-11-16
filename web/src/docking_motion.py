"""Native existing-controller ports. No SDK, driver lifecycle or gripper commands."""
import time
import numpy as np


class DockingMotion:
    def __init__(self,controller,token,*,guard,sleep=time.sleep,timeout_s=30.,lift_tolerance_rad=.02,require_lift_settle=True):
        if not np.isfinite(lift_tolerance_rad) or not .02<=lift_tolerance_rad<=.35:raise ValueError('Invalid bounded lift tolerance')
        self.lift_tolerance=lift_tolerance_rad
        self.require_lift_settle=bool(require_lift_settle)
        self.ctrl=controller;self.token=token;self.guard=guard;self.sleep=sleep
        self.timeout=timeout_s;self.active=None
        self.expected_aux_driver=None

    def _binding(self,state,start,driver):
        if start is not None and (max(abs(a-b) for a,b in zip(state['arms']['right']['joints'],start['joints']))>np.radians(.1)
                or abs(state['body']['height']-start['height'])>.02):raise ValueError('Planned auxiliary start changed')
        if driver is not None:
            current=self.ctrl.adapter.trajectory_status('right')
            if current is None or any(current.get(k)!=driver.get(k) for k in ('driver_instance','command_epoch')):
                raise ValueError('Holding arm driver changed during auxiliary movement')

    def _check(self,cancel):
        cancel()
        if not self.ctrl.motion_valid(self.token) or self.guard(self.ctrl.snapshot(),self.ctrl.clock()) is not True:
            raise ValueError('Motion ownership, feedback or scene changed')
        if self.active=='lift':self._binding(self.ctrl.snapshot(),None,self.expected_aux_driver)

    def _rest(self,group,after,cancel,*,target=None):
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
                        and np.max(np.ptp([v for _,v in history],axis=0))<=.002):return s
            self.sleep(.08)
        raise ValueError('Measured '+group+' arrival/rest not confirmed')

    def arm(self,segments,cancel,*,expected_start=None,expected_driver=None):
        self._check(cancel)
        start=expected_start;expected=expected_driver
        try:
            for plan in segments:
                self._check(cancel)
                def dispatch_guard(state,now):
                    if start is not None:
                        if (max(abs(a-b) for a,b in zip(state['arms']['right']['joints'],start['joints']))>np.radians(.1)
                                or abs(state['body']['height']-start['height'])>.02):
                            raise ValueError('Planned arm start changed at dispatch boundary')
                    allowed=self.guard(state,now)
                    if allowed is True:self.active='arm'
                    return allowed
                handle=self.ctrl.start_trajectory('right',plan,self.token,preflight=dispatch_guard,expected_driver=expected,
                    require_lift_settle=self.require_lift_settle)
                if expected is not None:
                    expected=dict(driver_instance=handle['driver_instance'],command_epoch=handle['command_epoch']+1)
                end=self.ctrl.clock()+float(plan['time_from_start_s'][-1])+5.
                while self.ctrl.clock()<end:
                    self._check(cancel)
                    if self.ctrl.poll_trajectory(self.token,preflight=self.guard)['status']=='reference_complete':break
                    self.sleep(.02)
                else:raise ValueError('Arm reference completion timed out')
                settled=self._rest('arm',self.ctrl.clock(),cancel)
                start=dict(joints=settled['arms']['right']['joints'],height=settled['body']['height'])
            self.completed_driver=expected
            self.active=None
            return self.ctrl.snapshot()
        except Exception:self.hold();raise

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

    def hold(self):
        # No stop is sent after successful measured completion: preserve its
        # already-issued loaded joint reference and clamp target.
        if self.active=='arm':self.ctrl.submit(dict(type='arm.stop',arm='right'),self.token)
        elif self.active=='lift':self.ctrl.submit(dict(type='lift.stop'),self.token)
        self.active=None
