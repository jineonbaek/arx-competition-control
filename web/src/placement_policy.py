"""Explicit T2 release policy; legacy/manual visual entry keeps its own checks."""
import json
from pathlib import Path

VISUAL_SEATED = 'visual_seated'
LIDAR_FIXED_STROKE = 'lidar_fixed_stroke'
LIDAR_SPECIAL_POINT = 'lidar_special_point'
CHASSIS_FORWARD_SECONDS = 8.
CHASSIS_FORWARD_SDK = .3
CHASSIS_COMPLETION_BASIS = f'saved_special_point_plus_{CHASSIS_FORWARD_SECONDS:g}s_chassis'


def terminal_forward_seconds(chassis_insertion=False):
    return CHASSIS_FORWARD_SECONDS if chassis_insertion is True else 4.


def terminal_forward_sdk(chassis_insertion=False):
    return CHASSIS_FORWARD_SDK if chassis_insertion is True else .225


def load_placement_mode(root):
    path=Path(root)/'config/t2-placement.json'
    if not path.exists():return VISUAL_SEATED
    policy=json.loads(path.read_text())
    if policy.get('version')!=1 or policy.get('mode') not in (VISUAL_SEATED,LIDAR_FIXED_STROKE,LIDAR_SPECIAL_POINT):
        raise ValueError('T2 放置模式配置无效')
    return policy['mode']


def distance_receipt_complete(result):
    import math
    travel=result.get('measured_travel_mm')
    return (result.get('completion_basis')=='measured_axial_travel'
            and result.get('seat_verification_enabled') is False
            and result.get('release_ready') is True
            and result.get('minimum_travel_reached') is True
            and type(travel) in (int,float) and math.isfinite(travel) and travel+1e-9>=150.)
