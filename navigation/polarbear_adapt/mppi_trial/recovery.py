"""Obstacle recovery and parked input recovery; actuator guards stay terminal."""
import math
import numpy as np

RECOVERABLE=frozenset(('map_sweep_blocked','odom_sweep_blocked','map_footprint_blocked',
 'odom_footprint_blocked','goal_blocked','replan_blocked','terminal_sweep_blocked',
 'path_withdrawn','no_current_path','escape_new_obstacle','escape_deeper_overlap',
 'escape_overlap_reentry','escape_no_progress','escape_exception_changed',
 'escape_lease_unavailable','escape_heading_changed','path_audit_pending'))

def recoverable(reason):
    return reason.split(' ',1)[0] in RECOVERABLE

def input_unavailable(reason):
    return (reason=='定位 TF 过期' or reason.startswith('input outage exceeded ') or
            reason.split(' ',1)[0] in (
                'sensor_or_costmap_stale','bottom_obstacles_stale',
                'dual_origin_mapping_stale','tf_missing','motion_pose_stale_or_invalid','motion_pose_changed_during_proof',
                'escape_inputs_stale','escape_reference_stale','escape_restore_stale','previous_output_end_unconfirmed'))

class InputRecoveryWindow:
    """Require a continuous fresh interval and advancing sources, never a timeout."""
    def __init__(self):self.since=None;self.updates=None
    def observe(self,now,ready,revisions=()):
        if not ready:
            self.since=None;self.updates=None;return False
        if self.since is None:
            self.since=now;self.updates=[set() for _ in revisions]
        for seen,revision in zip(self.updates,revisions):seen.add(revision)
        return (now-self.since>=.35 and len(self.updates)==4
                and all(len(seen)>=2 for seen in self.updates))

class RecoveryWindow:
    def __init__(self,started):
        self.started=started;self.since=None;self.global_updates=set();self.local_updates=set()
    def paused(self,now):return now-self.started>=30.
    def observe(self,now,safe,revisions):
        if not safe:
            self.since=None;self.global_updates.clear();self.local_updates.clear();return False
        if self.since is None:self.since=now
        self.global_updates.add(revisions[0]);self.local_updates.add(revisions[1])
        return (now-self.since>=.35 and len(self.global_updates)>=2 and len(self.local_updates)>=2)

def upcoming_route(route,radius,distance=.6):
    route=np.asarray(route,float);result=[route[0].copy()];left=distance
    for target in route[1:]:
        prior=result[-1];yaw=math.atan2(math.sin(target[2]-prior[2]),math.cos(target[2]-prior[2]))
        delta=np.r_[target[:2]-prior[:2],yaw]
        travel=np.linalg.norm(delta[:2])+radius*abs(yaw)
        if travel>left:
            result.append(prior+delta*(left/travel));break
        result.append(prior+delta);left-=travel
        if left<=0:break
    return np.asarray(result)
