"""Task-local pause for unavailable pose, retaining the original bounded goal."""
from navigation.polarbear_adapt.mppi_trial.stationary import StopWindow
from navigation.polarbear_adapt.chassis_command import output_unavailable
from urllib.error import URLError

def pose_unavailable(error):
    reason=str(error)
    return (reason in ('motion_pose_stale_or_invalid','motion_pose_changed_during_proof',
                       'sensor_or_costmap_stale /localization/odometry',
                       'special_pose_stale','special_invalid_pose','定位 TF 过期','Base pose stale')
            or reason.startswith('special_pose_authority_invalid'))

class PoseWait:
    def __init__(self,transport):
        self.transport=transport;self.waiting=False;self.ended=False;self.moved=False;self.window=None;self.end_error=None
    def begin(self,moved):
        if not self.waiting:
            self.waiting=True;self.ended=False
        # Every new unavailability revokes accumulated stationary evidence.
        self.window=None
        self.moved=self.moved or bool(moved)
    def hold(self):
        if self.ended:return True
        try:self.transport.end_output();self.ended=True;self.end_error=None;return True
        except Exception as exc:
            if not (output_unavailable(exc) or isinstance(exc,(TimeoutError,ConnectionError,URLError))
                    or str(exc)=='ROS scheduler rejected chassis command'):raise
            self.end_error=str(exc);return False
    def ready(self,sample,now,*,allow_motion):
        if not self.waiting:return True
        if not self.hold():return False
        if self.moved:
            if self.window is None:self.window=StopWindow(after=now)
            if not self.window.add(sample[2],sample[1]):return False
        if allow_motion:self.transport.begin_task()
        self.waiting=False;self.moved=False;return True
