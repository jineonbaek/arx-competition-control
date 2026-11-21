"""Depth-backed battery leading edge relative to an observed cabinet mouth."""
import copy
from pathlib import Path

from agent.held_insertion_path import ENTRY_ANGLE_DEG
import numpy as np

from agent.docking_fit import OBJECT_TO_TOOL
from agent.insertion_verification import UnstableInsertionFrame
from d405_left_edges import _left_battery, battery_geometry, insertion_mouth, paired_geometry
from placement_stage_geometry import battery_front

ROOT = Path(__file__).resolve().parents[2]


def partial_front(points, pixels, tool, bounds, contact):
    # A depth step along the side may belong to a shoulder anywhere along the
    # battery. Only use a resolved transverse edge at the distal end of the CAD
    # envelope. The reported position always comes from measured depth points.
    obj_mm = (points - tool[:3, 3]) @ (tool[:3, :3] @ OBJECT_TO_TOOL) * 1000 + contact
    nose = np.abs(obj_mm[:, 2] - bounds[0, 2]) <= 10.
    points, pixels, local = points[nose], pixels[nose], obj_mm[nose]
    if (len(points) < 12 or np.ptp(local[:, 1]) < 18.
            or np.ptp(np.quantile(local[:, 2], [.1, .9])) > 4.):
        raise ValueError('电池左外沿可见，但前端横棱的深度不足，无法测量仓外间距')
    # Retain the central part of the cross edge so an occluded corner cannot
    # silently become the tracked front point after the push.
    center = (bounds[0, 1] + bounds[1, 1]) / 2
    if local[:, 1].min() > center-6. or local[:, 1].max() < center+6.:
        raise ValueError('电池前端横棱中段被遮挡')
    middle = np.abs(local[:, 1] - center) <= 5.
    if middle.sum() < 3:
        raise ValueError('电池前端横棱中段深度不足')
    return dict(point_base_m=np.median(points[middle], axis=0).tolist(),
                support_pixels=len(points),quad_px=None)


def measure_insertion(frame, tool, reference=None):
    tool = np.asarray(tool, float)
    if (tool.shape != (4, 4) or not np.isfinite(tool).all()
            or not np.allclose(tool[:3,:3].T @ tool[:3,:3], np.eye(3), atol=1e-5)):
        raise ValueError('入仓末端位姿无效')
    if str(frame['camera_id']) != 'd405':
        raise ValueError('入仓校验需要 D405 配对深度')
    rgb, xyz, transform, rs, intr = paired_geometry(frame)
    if reference is None:
        cabinet = insertion_mouth(rgb, xyz, transform, rs, intr)
        expected = None
    else:
        if str(frame['calibration_sha256']) != reference['calibration_sha256']:
            raise ValueError('入仓校验期间相机标定发生变化')
        cabinet = copy.deepcopy(reference['cabinet'])
        expected = tool[:3,:3] @ reference['front_point_tool_m'] + tool[:3,3]
    axis = np.asarray(cabinet['inside_axis_base'])
    up = np.asarray(cabinet['up_base'])
    # Accept a pre-existing reference recorded with a full visible opening.
    mouth = np.asarray(cabinet['mouth_anchor_base_m'] if 'mouth_anchor_base_m' in cabinet else cabinet['height_anchor_base_m'])
    if (not np.isfinite(np.r_[axis, up, mouth]).all() or abs(np.linalg.norm(axis)-1)>1e-6
            or abs(np.linalg.norm(up)-1)>1e-6 or abs(axis@up)>1e-6):
        raise ValueError('实测仓口入口轴无效')
    angle = float(np.degrees(np.arccos(np.clip(axis @ tool[:3,0], -1., 1.))))
    if angle>ENTRY_ANGLE_DEG+1e-9:
        raise ValueError(f'末端与实测入口轴夹角 {angle:.2f}°，超过 {ENTRY_ANGLE_DEG:g}°')
    tracking = dict(axis_base=axis.tolist(),mouth_base_m=mouth.tolist(),
                    expected_front_base_m=None if expected is None else expected.tolist(),
                    quad_px=None if reference is None else reference['quad_px'])
    method = reference.get('front_method', 'four_edges') if reference else 'four_edges'
    bounds, contact = battery_geometry(ROOT)
    if method == 'four_edges':
        try:
            front = battery_front(frame, cabinet, outside_range_mm=(-210.,300.), insertion_tracking=tracking)
            obj = (np.asarray(front['point_base_m'])-tool[:3,3]) @ (tool[:3,:3] @ OBJECT_TO_TOOL)*1000 + contact
            if (not np.isfinite(obj).all() or abs(obj[2]-bounds[0,2])>10.
                    or np.any(obj[:2] < bounds[0,:2]-10.) or np.any(obj[:2] > bounds[1,:2]+10.)):
                raise ValueError('四棱候选和夹持电池的实物前端不符')
        except ValueError:
            if reference is not None:
                raise ValueError('新深度棱与原电池前端不一致或已被遮挡')
            method = 'partial_distal_edge'
    if method == 'partial_distal_edge':
        try:
            side = _left_battery(rgb, xyz, transform, tool, bounds, contact, intr)
            front = partial_front(*side, tool, bounds, contact)
        except ValueError as error:
            if reference is not None:
                raise
            raise UnstableInsertionFrame('D405 未取得电池前端的有效深度，无法确定轴向距离', frame) from error
    point = np.asarray(front['point_base_m'])
    if expected is not None:
        delta = (point - expected)*1000
        along = float(delta @ axis)
        if abs(along)>30. or np.linalg.norm(delta-along*axis)>10.:
            raise ValueError('新深度棱与夹持电池不一致，不能把背景当作入仓证据')
    depth = float((point - mouth) @ axis)*1000
    return dict(frame_id=str(frame['id']), image_time=float(frame['image_time']),
                calibration_sha256=str(frame['calibration_sha256']), depth_mm=depth,
                axis_base=axis.tolist(), up_base=up.tolist(), support_pixels=front['support_pixels'],
                front_point_tool_m=(tool[:3,:3].T @ (point-tool[:3,3])).tolist(),
                cabinet=cabinet, quad_px=front['quad_px'], initial_left_clearance_mm=None,
                axis_error_deg=angle, front_method=method,
                method='fresh_depth_supported_battery_leading_edge_vs_measured_mouth', physical_accuracy_verified=False)


def capture_insertion(capture,controller,tcp,reference=None):
    view=capture.action({'op':'capture'})
    with np.load(capture.cache/'frame.npz',allow_pickle=False) as stored:frame={k:stored[k] for k in stored.files}
    if str(frame['id'])!=view['id']:raise ValueError('入仓校验 RGB-D 帧被替换')
    tool=controller.models['right'].wrist_transform(frame['joints'],float(frame['lift_rad']))@tcp
    return measure_insertion(frame,tool,reference)
