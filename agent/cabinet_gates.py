"""Phase-specific evidence gates. Decisions are simulation-only, never motor permission."""
from dataclasses import dataclass,field
import copy
import hashlib
import json
import re
from .held_battery import finite


def identity_hash(identity):
    return hashlib.sha256(json.dumps(identity,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class Context:
    run_id:str
    boot_id:str
    generation:int
    asset_hash:str
    held_identity_hash:str
    engagement:str='outside_verified'
    release:str='held_known'
    manual_latched:bool=False
    state_seq:int=0


@dataclass(frozen=True)
class Evidence:
    gate:str
    status:str
    issued_at:float
    expires_at:float
    run_id:str
    boot_id:str
    generation:int
    asset_hash:str
    scope:str='simulation'
    actions:tuple=()
    details:dict=field(default_factory=dict)
    state_seq:int=0


@dataclass(frozen=True)
class Decision:
    action:str
    allowed:bool
    required:tuple
    blocked:dict
    context:Context
    issued_at:float
    valid_until:float
    mode:str='offline'
    executable:bool=False

    def as_dict(self):
        from dataclasses import asdict
        return asdict(self)


def gates(*numbers):return tuple(f'G{n:02d}' for n in numbers)


REQUIRED={
    'inspect':(), 'stop':(),
    'observe':gates(4,5,6),
    'search':gates(1,2,3,4,6,7,8,9,13,14,18,19,20),
    'base_align':gates(1,2,3,4,5,6,7,8,9,13,14,18,19,20),
    'lift_align':gates(1,2,3,4,6,8,9,10,13,14,18,19,20),
    'outside_align':gates(1,2,3,4,5,6,8,9,11,13,14,18,19,20),
    'insert':gates(1,2,3,4,5,6,8,9,11,12,13,14,18,19,20),
    'retreat':gates(1,2,3,4,6,8,9,11,12,13,14,18,19,20),
    'release':gates(1,2,3,4,6,9,12,13,14,15,16,19,20),
    'release_progress':gates(1,2,3,4,6,13,15,16,19,20),
    'withdraw_empty':gates(1,2,3,4,6,8,11,13,14,16,18,19,20),
    'complete':gates(17,19),
}


class Gatekeeper:
    def evaluate(self,action,evidence,context,*,now,held=None):
        if action not in REQUIRED:raise ValueError('Unknown autonomy action')
        required=REQUIRED[action];blocked={};expiry=now if finite(now) else 0.
        # Diagnostics and stop requests do not depend on movement gates or fresh sensors.
        if action in ('inspect','stop'):return Decision(action,True,required,{},context,expiry,expiry)
        try:
            if (not isinstance(context,Context) or not context.run_id or not context.boot_id or type(context.generation) is not int
                    or context.generation<0 or type(context.state_seq) is not int or context.state_seq<0 or not re.fullmatch('[a-f0-9]{64}',context.asset_hash)
                    or not re.fullmatch('[a-f0-9]{64}',context.held_identity_hash) or not finite(now)):
                raise ValueError('Context identity/clock invalid')
            if context.engagement not in ('outside_verified','disengaged_verified','may_be_inserted','inside_verified','released_verified','unknown'):
                raise ValueError('Unknown physical engagement')
            if context.release not in ('held_known','release_requested','released_verified','unknown'):raise ValueError('Unknown release state')
            if action in ('search','base_align','lift_align','outside_align') and context.engagement not in ('outside_verified','disengaged_verified'):
                blocked['engagement']='Battery may still be inserted; base/lift/re-alignment forbidden'
            if context.release=='unknown' and action not in ('observe',):blocked['release']='Release outcome unknown; do not replay or select an empty-grip branch'
            if context.manual_latched and action!='observe':blocked['G20']='Manual stop/takeover is latched; explicit new resume required'
            if action in ('search','base_align','lift_align','outside_align','insert','retreat','release') and context.release!='held_known':
                blocked['release']='Carrying motion/release cannot run during or after requested release'
            if action in ('release','release_progress') and context.engagement!='inside_verified':blocked['engagement']='Insertion/support not yet independently verified'
            if action in ('withdraw_empty','complete') and context.release!='released_verified':blocked['release']='Release has not been verified'
            if (action=='withdraw_empty' and isinstance(held,dict) and held.get('presence')=='present'
                    and identity_hash(held.get('identity'))==context.held_identity_hash
                    and finite(held.get('captured_at')) and finite(held.get('valid_until')) and held['captured_at']<=now<held['valid_until']):
                blocked['G16']='Fresh visual battery-in-grip evidence conflicts with empty-grip withdrawal certificate'
            expiry=now+2.
            for gate in required:
                if gate=='G09':
                    valid=(isinstance(held,dict) and held.get('mode')=='offline' and held.get('executable') is False
                        and held.get('holding_eligibility')=='eligible' and identity_hash(held.get('identity'))==context.held_identity_hash
                        and finite(held.get('captured_at')) and finite(held.get('valid_until'))
                        and held['captured_at']<=now<held['valid_until'])
                    if not valid:blocked[gate]='D405 holding evidence missing, stale, conflicting or unqualified'
                    else:expiry=min(expiry,held['valid_until'])
                    continue
                row=evidence.get(gate)
                if not isinstance(row,Evidence):blocked[gate]='Required evidence missing';continue
                if (row.gate!=gate or row.status!='verified' or row.scope!='simulation'
                        or not finite(row.issued_at) or not finite(row.expires_at) or not row.issued_at<=now<row.expires_at
                        or row.run_id!=context.run_id or row.boot_id!=context.boot_id or type(row.generation) is not int
                        or row.generation!=context.generation or type(row.state_seq) is not int or row.state_seq!=context.state_seq or row.asset_hash!=context.asset_hash
                        or row.actions and action not in row.actions):
                    blocked[gate]='Evidence status, scope, time or run/generation/assets mismatch';continue
                expiry=min(expiry,row.expires_at);d=copy.deepcopy(row.details)
                if not isinstance(d,dict):blocked[gate]='Invalid evidence detail';continue
                if gate=='G07' and not all(d.get(k) is True for k in ('si_mapping_verified','braking_verified','odometry_control_verified')):
                    blocked[gate]='SDK→SI mapping, braking or control odometry is unverified'
                if gate=='G08' and not all(d.get(k) is True for k in ('normal_clear','stopping_clear','coverage_complete')):
                    blocked[gate]='Normal/stopping envelope or obstacle coverage is unverified'
                if gate=='G12':
                    values=[d.get(k) for k in ('clearance_mm','error_upper_mm','reserve_mm')]
                    if (d.get('bound_kind')!='validated_end_to_end_bound' or not all(finite(v) for v in values)
                            or min(values)<0 or values[0]<=0 or values[1]+values[2]>=values[0]):
                        blocked[gate]='No verified end-to-end error bound strictly inside aperture clearance'
                    if action=='retreat' and not all(d.get(k) is True for k in ('retreat_verified','contact_free','target_static','current_pose_known')):
                        blocked[gate]='Independent retreat/current-pose/contact-free evidence missing'
                if gate=='G13' and not all(d.get(k) is True for k in ('actual_velocity_known','fresh_post_stop')):
                    blocked[gate]='Measured rest is not proven by fresh post-stop samples'
                if gate=='G15' and not all(d.get(k) is True for k in ('support_verified','insertion_depth_verified')):
                    blocked[gate]='Support and actual insertion depth not independently verified'
                if gate=='G16' and action=='withdraw_empty' and not all(d.get(k) is True for k in ('empty_grip_verified','release_verified')):
                    blocked[gate]='Empty grip and release result must be verified before withdrawal'
                if gate=='G17' and not all(d.get(k) is True for k in ('battery_left_in_slot','stable_verified')):
                    blocked[gate]='Stable battery left in slot not verified'
        except (ValueError,TypeError,AttributeError,KeyError):blocked['context']='Invalid/malformed evidence context'
        return Decision(action,not blocked,required,blocked,context,now if finite(now) else 0.,expiry)
