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
