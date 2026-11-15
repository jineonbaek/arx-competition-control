"""Temporal D405 holding evidence. Offline only, with no actuator/camera transport."""
import copy
import hashlib
import json
import math
import re
import numpy as np


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def validate_identity(identity):
    keys={'reference_id','camera','serial','handeye_sha256','tcp_sha256','battery_sha256','grasp_id'}
    if (not isinstance(identity,dict) or set(identity)!=keys or identity['camera']!='d405'
            or any(not isinstance(v,str) or not v for v in identity.values())
            or any(not re.fullmatch('[0-9a-f]{64}',identity[k]) for k in ('handeye_sha256','tcp_sha256','battery_sha256'))):
        raise ValueError('A bound D405 grasp-reference identity is required')
    return copy.deepcopy(identity)


class HeldBatteryMonitor:
    def __init__(self,identity,*,max_age_s=.5,min_samples=3,settle_s=.2,
                 max_slip_px=3.,max_rotation_deg=2.,gripper_range=(-1.5,-.3)):
        self.identity=validate_identity(identity)
        if (any(not finite(v) or v<=0 for v in (max_age_s,settle_s,max_slip_px,max_rotation_deg))
                or type(min_samples) is not int or min_samples<3 or settle_s>=max_age_s
                or len(gripper_range)!=2 or not all(finite(v) for v in gripper_range) or gripper_range[0]>=gripper_range[1]):
            raise ValueError('Invalid finite holding policy')
        self.max_age=max_age_s;self.min_samples=min_samples;self.settle=settle_s
        self.max_slip=max_slip_px;self.max_rotation=max_rotation_deg;self.gripper_range=tuple(gripper_range)
        self.history=[];self.seen={};self.last_stamp=-math.inf;self.last_now=-math.inf;self.slip_latched=False;self.reference_sha256=None

    def update(self,observation,feedback,*,now,carry_certificate=None,phase='carrying'):
        r=dict(mode='offline',executable=False,identity=copy.deepcopy(self.identity),presence='unknown',engagement='unknown',
            pose='unobservable',stability='unknown',holding_eligibility='unknown',grip_feedback='unavailable',
            metric_pose=None,placement_confirmed=False,reason='',valid_until=now if finite(now) else 0.,frame_id=None)
        def fail(reason,blocked=False):
            self.history=[];r.update(reason=reason,holding_eligibility='blocked' if blocked else 'unknown');return r
        try:
            if not finite(now) or now<self.last_now:return fail('Invalid or backward observer clock')
            self.last_now=now
            if not isinstance(observation,dict) or observation.get('identity')!=self.identity:return fail('Reference/calibration/grasp identity changed')
            stamp=observation['captured_at'];frame=observation['frame_id']
            if not finite(stamp) or not isinstance(frame,str) or not frame or not 0<=now-stamp<=self.max_age or stamp<self.last_stamp:
                return fail('Stale, future or backward D405 observation')
            r.update(frame_id=frame,captured_at=stamp,valid_until=stamp+self.max_age)
            encoded=json.dumps(observation,sort_keys=True,allow_nan=False)
            repeated=frame in self.seen
            if repeated and self.seen[frame]!=encoded:return fail('Same frame identity has conflicting observations')
            if not repeated:
                if stamp<=self.last_stamp:return fail('Different frame without increasing exposure timestamp')
                self.seen[frame]=encoded;self.last_stamp=stamp
            if phase in ('release_requested','released_verified'):
                r.update(presence=observation.get('presence','unknown') if observation.get('critical_visible') is True else 'unknown',
                    holding_eligibility='not_required',reason='Release phase; opening/empty view is not transport slip or placement completion')
                return r
            if observation.get('critical_visible') is not True:return fail('Critical grip region is unobservable')
            presence=observation.get('presence','unknown');engagement=observation.get('engagement','unknown');pose=observation.get('pose','unobservable')
            r.update(presence=presence,engagement=engagement,pose=pose,
                observable_axes=copy.deepcopy(observation.get('observable_axes',[])))
            if phase not in ('carrying','outside','inserting','release_requested'):
                return fail('Holding phase unknown or release outcome unresolved')
            if presence=='absent':return fail('Visibly empty grip',True)
            if presence!='present':return fail('Battery presence is unknown')
            delta=np.asarray(observation['image_delta'],float);sigma=observation['uncertainty_px']
            if delta.shape!=(3,) or not np.isfinite(delta).all() or not finite(sigma) or sigma<0:return fail('Invalid image-plane displacement bound')
            r.update(image_delta=delta.tolist(),uncertainty_px=sigma)
            for _,old,_ in self.history:
                if np.linalg.norm(delta[:2]-old[:2])-2*sigma>self.max_slip or abs(delta[2]-old[2])>self.max_rotation:
                    self.slip_latched=True
            if self.slip_latched:
                r['stability']='slip_suspected';return fail('Relative slip is latched; a new independently qualified grasp is required',True)
            if engagement=='visibly_incorrect' or pose=='misaligned':return fail('Visual grip region or pose is incorrect',True)
            if engagement!='visually_consistent' or pose!='matched':return fail('Grip/pose ambiguous or unobservable')
            if not isinstance(feedback,dict) or feedback.get('fresh') is not True or not finite(feedback.get('captured_at')) or not finite(feedback.get('position_raw')) or abs(feedback['captured_at']-stamp)>.2:
                return fail('Missing fresh paired gripper feedback')
            if not self.gripper_range[0]<=feedback['position_raw']<=self.gripper_range[1]:
                r['grip_feedback']='conflicting';return fail('Visual and raw gripper feedback conflict',True)
            r['grip_feedback']='consistent'
            if not repeated:self.history.append((stamp,delta.copy(),sigma))
            self.history=[v for v in self.history if stamp-v[0]<=self.max_age]
            stable=(len(self.history)>=self.min_samples and stamp-self.history[0][0]>=self.settle
                and max(np.linalg.norm(delta[:2]-v[:2])+sigma+s for _,v,s in self.history)<=self.max_slip
                and max(abs(delta[2]-v[2]) for _,v,_ in self.history)<=self.max_rotation)
            if stable:r['stability']='stable_observed'
            c=carry_certificate
            qualified=(isinstance(c,dict) and c.get('scope')=='simulation' and c.get('reference_id')==self.identity['reference_id']
                and c.get('grasp_id')==self.identity['grasp_id'] and c.get('unobserved_axes_bounded') is True and c.get('load_validated') is True
                and c.get('identity_sha256')==hashlib.sha256(json.dumps(self.identity,sort_keys=True,separators=(',',':')).encode()).hexdigest()
                and isinstance(observation.get('reference_sha256'),str) and re.fullmatch('[a-f0-9]{64}',observation['reference_sha256'])
                and c.get('reference_sha256')==observation['reference_sha256']
                and finite(c.get('issued_at')) and finite(c.get('expires_at')) and c['issued_at']<=now<c['expires_at'])
            if qualified:
                if self.reference_sha256 is not None and self.reference_sha256!=observation['reference_sha256']:
                    return fail('A new reference cannot silently rebaseline the same grasp',True)
                self.reference_sha256=observation['reference_sha256']
            if stable and qualified:
                r.update(holding_eligibility='eligible',reason='Simulation-scoped holding evidence consistent',
                         valid_until=min(r['valid_until'],c['expires_at']))
            else:r['reason']='Waiting for independent stable frames' if not stable else 'Load/unobserved-axis evidence missing; RGB cannot measure force or full pose'
            return r
        except (KeyError,TypeError,ValueError,OverflowError):return fail('Malformed held-battery evidence')
