"""Depth-backed battery leading edge relative to an observed cabinet mouth."""
import copy,json
import numpy as np
from placement_stage_geometry import cabinet_observation,battery_front


def measure_insertion(frame,tool,reference=None):
    if str(frame['camera_id'])!='d405':raise ValueError('入仓校验需要 D405 配对深度')
    cabinet=copy.deepcopy(reference['cabinet']) if reference else cabinet_observation(frame)
    if reference and str(frame['calibration_sha256'])!=reference['calibration_sha256']:
        raise ValueError('入仓校验期间相机标定发生变化')
    T=np.asarray(tool,float)
    expected=None if reference is None else T[:3,:3]@np.asarray(reference['front_point_tool_m'])+T[:3,3]
    tracking=dict(axis_base=cabinet['inside_axis_base'],mouth_base_m=cabinet['height_anchor_base_m'],
                  expected_front_base_m=None if expected is None else expected.tolist(),quad_px=None if reference is None else reference['quad_px'])
    front=battery_front(frame,cabinet,outside_range_mm=(-210.,300.),insertion_tracking=tracking)
    point=np.asarray(front['point_base_m']);axis=np.asarray(cabinet['inside_axis_base']);up=np.asarray(cabinet['up_base'])
    T=np.asarray(tool,float)
    if (T.shape!=(4,4) or not np.isfinite(T).all() or abs(np.linalg.norm(axis)-1)>1e-6
        or abs(axis@up)>1e-6 or axis@T[:3,0]<np.cos(np.radians(3.))):
        raise ValueError('电池朝向与实测仓口入口轴不一致')
    point_tool=T[:3,:3].T@(point-T[:3,3])
    if reference:
        expected=T[:3,:3]@np.asarray(reference['front_point_tool_m'])+T[:3,3]
        delta=(point-expected)*1000;along=float(delta@axis)
        if abs(along)>30. or np.linalg.norm(delta-along*axis)>10.:
            raise ValueError('新深度棱与夹持电池不一致，不能把背景当作入仓证据')
    depth=float((point-np.asarray(cabinet['height_anchor_base_m']))@axis)*1000
    return dict(frame_id=str(frame['id']),image_time=float(frame['image_time']),
                calibration_sha256=str(frame['calibration_sha256']),depth_mm=depth,
                axis_base=axis.tolist(),up_base=up.tolist(),support_pixels=front['support_pixels'],
                front_point_tool_m=point_tool.tolist(),cabinet=cabinet,quad_px=front['quad_px'],
                method='fresh_depth_supported_battery_leading_edge_vs_measured_mouth',physical_accuracy_verified=False)


def capture_insertion(capture,controller,tcp,reference=None):
    view=capture.action({'op':'capture'})
    with np.load(capture.cache/'frame.npz',allow_pickle=False) as stored:frame={k:stored[k] for k in stored.files}
    if str(frame['id'])!=view['id']:raise ValueError('入仓校验 RGB-D 帧被替换')
    tool=controller.models['right'].wrist_transform(frame['joints'],float(frame['lift_rad']))@tcp
    return measure_insertion(frame,tool,reference)
