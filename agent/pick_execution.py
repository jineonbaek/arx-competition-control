"""Insertion arrival -> one close request; no task thread or hardware adapter setup."""
from threading import Lock


class PickCloseStep:
    """Bind PickMonitor to an already reserved Controller, after insertion is issued.

    The caller owns path validation, polling and cleanup of all participating axes.
    This step stops only the right arm on failure and never releases task ownership.
    """
    def __init__(self, controller, token, monitor):
        if not controller.motion_valid(token):
            raise ValueError('Valid motion ownership required')
        self.controller = controller
        self.token = token
        self.monitor = monitor
        self.stop_request = None
        self.lock = Lock()

    def _ready(self, snapshot):
        return (self.controller.motion_valid(self.token)
                and snapshot.get('capabilities', {}).get('right:gripper') == '')

    def _stop(self, receipt):
        if receipt['stop_required'] and self.stop_request is None:
            # A failed stop cannot make the interrupted task resumable.
            self.stop_request = 'failed'
            try:
                self.controller.interrupt_motion(self.token)
                result = self.controller.submit({'type': 'arm.stop', 'arm': 'right'}, self.token)
                if result.get('ok') is True:
                    self.stop_request = 'sent_unverified'
            except Exception as error:
                self.controller.log('抓取停止请求失败：' + str(error), 'warning')
        return {**receipt, 'stop_request': self.stop_request}

    def poll(self, *, cancelled=False):
        with self.lock:
            if self.monitor.failure:
                return self._stop(self.monitor.abort(self.monitor.failure))
            try:
                snapshot = self.controller.snapshot()
                receipt = self.monitor.update(snapshot, self.controller.clock(),
                    ready=self._ready(snapshot), cancelled=cancelled)
                if receipt['status'] == 'ready_to_close':
                    def preflight(fresh, now):
                        nonlocal receipt
                        receipt = self.monitor.start_closing(fresh, now, ready=self._ready(fresh))
                        return receipt['close_permitted']
                    result = self.controller.submit({'type': 'gripper', 'arm': 'right',
                        'position': self.monitor.gripper_target}, self.token, preflight=preflight)
                    if result.get('ok') is not True and result.get('deferred') is not True:
                        receipt = self.monitor.abort('execution_unknown')
                    # Manual stop may arrive while the driver confirms outside its lock.
                    if not self.controller.motion_valid(self.token):
                        receipt = self.monitor.abort('interrupted')
            except Exception as error:
                self.controller.log('抓取阶段中止：' + str(error), 'warning')
                receipt = self.monitor.abort('execution_unknown')
            return self._stop(receipt)


class PickTrajectoryStep(PickCloseStep):
    """Single checked insertion followed by measured settling and closure.

    Preflight is supplied by the deterministic planner, never by Agent arguments.
    It must recheck scene validity, unchanged lift/base, TCP version and clearance.
    """
    def __init__(self,controller,token,monitor,plan,*,preflight):
        import copy
        import numpy as np
        from scipy.spatial.transform import Rotation
        from .grasp_geometry import rigid
        super().__init__(controller,token,monitor)
        if not callable(preflight) or monitor.failure or monitor.stage!='inserting' or monitor.last_sample is not None:
            raise ValueError('Fresh monitor and deterministic preflight required')
        self.plan=copy.deepcopy(plan)
        try:
            q=np.asarray(self.plan['joint_waypoints_rad'][-1],float)
            if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('Invalid endpoint')
            end=rigid(monitor.model.wrist_transform(q,monitor.feedback_policy['expected_lift_rad'])@monitor.tcp)
            error=np.r_[(end[:3,3]-monitor.target[:3,3])*1000,
                np.degrees(Rotation.from_matrix(monitor.target[:3,:3].T@end[:3,:3]).as_rotvec())]
            if np.any(np.abs(error)>monitor.arm.tolerance):raise ValueError('Trajectory and measured-arrival target differ')
        except (KeyError,TypeError,IndexError) as error:
            raise ValueError('Insertion trajectory endpoint required') from error
        self.preflight=preflight
        self.started=False
        self.reference_complete=False

    def _ready(self,snapshot):
        return (super()._ready(snapshot)
                and self.preflight(snapshot,self.controller.clock()) is True)

    def start(self):
        with self.lock:
            if self.started or self.monitor.failure:raise ValueError('Insertion cannot be replayed or resumed')
            self.started=True
            try:
                self.controller.start_trajectory('right',self.plan,self.token,preflight=self.preflight)
                return dict(status='awaiting_trajectory',close_permitted=False,
                            stop_required=False,grasp_confirmed=False)
            except Exception as error:
                self.controller.log('前伸轨迹未完成确认：'+str(error),'warning')
                return self._stop(self.monitor.abort('execution_unknown'))

    def cancel(self):
        with self.lock:
            return self._stop(self.monitor.abort('cancelled'))

    def poll(self,*,cancelled=False):
        with self.lock:
            if not self.started:raise ValueError('Insertion has not been started')
            if self.monitor.failure:
                return self._stop(self.monitor.abort(self.monitor.failure))
            if cancelled:
                return self._stop(self.monitor.abort('cancelled'))
            if not self.reference_complete:
                try:
                    receipt=self.controller.poll_trajectory(self.token,preflight=self.preflight)
                    if receipt['status']!='reference_complete':
                        return dict(status='trajectory_running' if receipt['status']=='running' else receipt['status'],
                                    close_permitted=False,stop_required=False,grasp_confirmed=False)
                    self.monitor.begin_settling(self.controller.clock())
                    self.reference_complete=True
                except Exception as error:
                    self.controller.log('前伸轨迹中止：'+str(error),'warning')
                    return self._stop(self.monitor.abort('execution_unknown'))
        return super().poll(cancelled=cancelled)
