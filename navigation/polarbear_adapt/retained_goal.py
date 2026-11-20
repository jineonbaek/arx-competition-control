"""Goal-preserving fault handling. Movement still uses the original proof path."""
import re,time
from urllib.error import HTTPError,URLError
import rclpy
from mppi_trial.recovery import recoverable,input_unavailable
from recovery_budget import paused_budget

def recovery_kind(error):
    reason=str(error);prefix=reason.split(' ',1)[0]
    if input_unavailable(reason):return 'inputs'
    if recoverable(reason) or prefix in ('mppi_unavailable','mppi_rejected_path','mppi_accept_timeout','control_cycle_overrun'):
        return 'replan'
    if prefix=='mppi_failed':
        code=re.search(r'\bcode=(\d+)\b',reason)
        return 'replan' if code and 102<=int(code[1])<=107 else 'park'
    if prefix=='console_not_ready_or_busy':return 'console'
    if prefix=='task_worker_unavailable':return 'worker'
    if isinstance(error,HTTPError) and error.code in (502,503,504):return 'console'
    if isinstance(error,(TimeoutError,ConnectionError,URLError)) and not isinstance(error,HTTPError):return 'console'
    return 'park'

class RetainedGoal:
    def same_intent(self,epoch):
        if self.seq!=epoch or self.target is None:raise ValueError('goal_changed')

    def soft_brake(self):
        try:self.brake();return True
        except Exception as exc:
            self.get_logger().warning('Waiting for scoped brake confirmation: '+str(exc),throttle_duration_sec=3.)
            return False

    def park_navigation(self,epoch,reason,status='paused_navigation',phase=None):
        self.soft_brake();self.escape.clear()
        self.parked_reason=reason
        self.status(status+' '+reason)
        self.waypoints.show(phase or ('停车保留任务，等待人工处理：'+reason[:160]))
        while rclpy.ok():
            self.same_intent(epoch);self.soft_brake()
            try:self.spin(.1)
            except Exception as exc:
                self.same_intent(epoch)
                self.get_logger().warning('Parked navigation callback: '+str(exc),throttle_duration_sec=3.)
        raise ValueError('ros_shutdown')

    @paused_budget
    def wait_controller_quiet(self,epoch):
        # A timeout is not proof that an old request was never accepted.
        # Keep all futures until terminal/cancelled, including late acceptance.
        from special_runtime import cancel_mppi
        while rclpy.ok():
            self.current_intent(epoch);self.soft_brake()
            try:cancel_mppi(self);return
            except ValueError as exc:
                if str(exc)!='special_old_mppi_cancel_timeout':raise
                self.status('waiting_for_controller_quiescence')
                self.waypoints.show('停车等待旧控制器结束，保留原任务')
                self.spin(.1)
        raise ValueError('ros_shutdown')

    @paused_budget
    def wait_console_ready(self,epoch):
        while rclpy.ok():
            self.current_intent(epoch);self.soft_brake()
            try:self.recovery_console();return
            except Exception as exc:
                if str(exc)=='maintenance_active':raise
                self.get_logger().warning('Waiting for console readiness: '+str(exc),throttle_duration_sec=3.)
            self.spin(.2)
        raise ValueError('ros_shutdown')

    @paused_budget
    def wait_worker_ready(self,epoch):
        while rclpy.ok():
            self.same_intent(epoch);self.soft_brake()
            if self.ready_file is not None and not self.ready_file.is_file():raise ValueError('execution_gate_closed')
            if self.task_worker_ready():return
            self.spin(.1)
        raise ValueError('ros_shutdown')

    @paused_budget
    def recover_pending_fault(self,epoch):
        kind,reason=self.pending_fault
        if time.monotonic()<self.retry_after:self.spin(.05);return False
        self.soft_brake()
        if kind=='inputs':self.wait_for_inputs(epoch,reason);reason='inputs_restored'
        if kind=='console':self.wait_console_ready(epoch)
        if kind=='worker':self.wait_worker_ready(epoch)
        self.wait_controller_quiet(epoch)
        self.recovery=None
        self.hold_for_replan(reason)
        self.pending_fault=None
        return True
