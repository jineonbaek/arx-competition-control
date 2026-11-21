"""Save the map-frame start and joint feedback together for the fast pickup."""
import math
from time import time


def finite(values, length):
    return (isinstance(values, (list, tuple)) and len(values) == length and
            all(type(value) in (int, float) and math.isfinite(value) for value in values))


def map_pose(view):
    pose = view.get('pose') or {}
    coordinates = [pose.get(key) for key in ('x', 'y', 'yaw')]
    route = (view.get('route') or {}).get('definition') or {}
    if (not pose.get('valid') or pose.get('frame') != 'map' or not route.get('map_id')
            or not finite(coordinates, 3)):
        raise ValueError('雷达地图位姿无效，无法记录或使用出发点')
    return route['map_id'], coordinates


def capture(robot, view, captured_at=None):
    map_id, pose = map_pose(view)
    body = robot.get('body') or {}
    if robot.get('chassis_active') is not False or not body.get('fresh') or not finite([body.get('height')], 1):
        raise ValueError('记录出发点需要底盘停止输出和有效的升降反馈')
    arms = {}
    for side in ('left', 'right'):
        arm = robot.get('arms', {}).get(side) or {}
        if not arm.get('fresh') or not finite(arm.get('joints'), 6) or not finite([arm.get('gripper')], 1):
            raise ValueError('缺少机械臂实时关节或夹爪反馈')
        arms[side] = {key: arm.get(key) for key in ('joints', 'gripper', 'base_wrist', 'timestamp')}
    return dict(captured_at=time() if captured_at is None else captured_at,
                frame='map', map_id=map_id, pose=pose, pose_age_s=view['pose']['age_s'],
                lift_rad=body['height'], arms=arms)


def check_start(reference, view):
    map_id, actual = map_pose(view)
    target = reference['pose']
    if map_id != reference['map_id'] or not finite(target, 3):
        raise ValueError('出发点与当前雷达地图不匹配')
    yaw = actual[2] - target[2]
    if math.dist(actual[:2], target[:2]) > .07 or abs(math.atan2(math.sin(yaw), math.cos(yaw))) > .09:
        raise ValueError('当前底盘未在已记录的出发点，请先定位到起点')
