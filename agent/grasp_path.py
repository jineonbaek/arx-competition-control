"""Offline insertion IK with an explicit calibrated tool transform; no command transport."""
import numpy as np
from scipy.spatial.transform import Rotation


def plan_insert(model, start_q, lift_rad, pre_mm, end_mm, base_tool_rotation, wrist_from_tool, limits_deg):
    if wrist_from_tool is None:
        return dict(status='tcp_pending',joint_waypoints_rad=None,wrist_pre_mm=None,wrist_end_mm=None,
                    executable=False,reason='TCP 待用户提供，未计算腕部轨迹')
    from .grasp_geometry import rigid
    tcp=rigid(wrist_from_tool)
    orientation=np.eye(4);orientation[:3,:3]=np.asarray(base_tool_rotation,float)
    rigid(orientation)
    pre,end,q,limits=map(lambda x:np.asarray(x,float),(pre_mm,end_mm,start_q,limits_deg))
    if (pre.shape!=(3,) or end.shape!=(3,) or q.shape!=(6,) or limits.shape!=(6,2)
            or not all(np.isfinite(v).all() for v in (pre,end,q,limits))
            or not np.isfinite(lift_rad) or not 0<=lift_rad<=18
            or np.any(limits[:,0]>=limits[:,1])):
        raise ValueError('Invalid insertion inputs')
    delta=end-pre
    if not 0<delta[0]<=200 or not np.allclose(delta[1:],0,atol=1e-6):
        raise ValueError('Insertion must be constant Y/Z, positive X, at most 200 mm')
    if np.linalg.norm(tcp[:3,3])>1:raise ValueError('TCP uses metres; offset exceeds tool size bound')
    wrist_rotation=orientation[:3,:3]@tcp[:3,:3].T
    rpy=Rotation.from_matrix(wrist_rotation).as_euler('xyz',degrees=True)
    points=np.linspace(pre,end,int(np.ceil(delta[0]/2))+1)
    wrists=points-(wrist_rotation@tcp[:3,3])*1000
    # Approach is a separate joint-space proposal; only the insertion below has a straightness bound.
    first=np.asarray(model.inverse_wrist(q,lift_rad,wrists[0],rpy,limits))
    waypoints=[first];position_bound=0.;angle_error=0.;clearances=[]
    clearances.append(model.check(q,first,lift_rad))
    for previous_point,target,target_wrist in zip(points[:-1],points[1:],wrists[1:]):
        a=waypoints[-1]
        b=np.asarray(model.inverse_wrist(a,lift_rad,target_wrist,rpy,limits))
        dq=b-a
        if np.max(np.abs(dq))>np.radians(5):
            raise ValueError('Insertion IK branch jump or singular configuration')
        clearance=model.check(a,b,lift_rad)
        clearances.append(clearance)
        endpoint_errors=[]
        for joints,point in ((a,previous_point),(b,target)):
            T=model.wrist_transform(joints,lift_rad)@tcp
            endpoint_errors.append(np.linalg.norm(T[:3,3]*1000-point))
        # For linear joint interpolation, ||p''|| <= reach * sum(|dq|)^2; chord error <= bound/8.
        curvature=(model.reach+np.linalg.norm(tcp[:3,3]))*np.abs(dq).sum()**2/8*1000
        bound=max(endpoint_errors)+curvature
        position_bound=max(position_bound,float(bound))
        if bound>1:raise ValueError('Joint interpolation cannot keep TCP within 1 mm of insertion line')
        count=max(2,int(np.ceil(np.max(np.abs(dq))/np.radians(.1)))+1)
        for t in np.linspace(0,1,count):
            actual=(model.wrist_transform(a+t*dq,lift_rad)@tcp)[:3,:3]
            error=np.degrees(Rotation.from_matrix(orientation[:3,:3].T@actual).magnitude())
            angle_error=max(angle_error,float(error))
        if angle_error>.3:raise ValueError('Insertion orientation departs from fixed tool pose')
        waypoints.append(b)
    if not all(c['clear'] for c in clearances):raise ValueError('Path approaches central column')
    return dict(status='kinematic_preview',frame='base_link',position_unit='mm',joint_unit='rad',
        approach=dict(kind='joint_space',start_rad=q.tolist(),end_rad=first.tolist()),
        tool_points_mm=points.tolist(),wrist_pre_mm=wrists[0].tolist(),wrist_end_mm=wrists[-1].tolist(),
        joint_waypoints_rad=[v.tolist() for v in waypoints],
        interpolation_position_bound_mm=position_bound,sampled_orientation_error_deg=angle_error,
        column_checked=True,column_lower_mm=min(c['lower_mm'] for c in clearances),
        environment_collision_checked=False,executable=False,
        blockers=['尚未核对夹爪、左右臂及环境碰撞','尚未生成受速度和加速度约束的执行时序',
                  '连续位置界限仅适用于所检查的关节线性插值；控制服务必须采用匹配的插值'])
