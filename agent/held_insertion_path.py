"""Fixed-Z aperture translation while preserving the observed tool rotation."""
import numpy as np
from scipy.spatial.transform import Rotation
from .grasp_path import TimedJointPath

def plan_insert(model, start_q, lift_rad, pre_mm, end_mm, base_tool_rotation, wrist_from_tool, limits_deg, *, timing_limits=None,fixed_z_axis=None,fixed_height_up=None,max_step_mm=2.):
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
    distance=float(np.linalg.norm(delta))
    direction=delta/distance if distance else delta
    axis=orientation[:3,0]
    if fixed_z_axis is not None:
        axis=np.asarray(fixed_z_axis,float)
        up=np.asarray([0.,0.,1.] if fixed_height_up is None else fixed_height_up,float)
        if (axis.shape!=(3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1)>1e-6
                or up.shape!=(3,) or not np.isfinite(up).all() or abs(np.linalg.norm(up)-1)>1e-6
                or abs(axis@up)>1e-6 or axis@orientation[:3,0]<np.cos(np.radians(3.))):
            raise ValueError('Fixed-Z aperture axis must be horizontal and within 3deg of tool +X')
    elif fixed_height_up is not None:raise ValueError('Explicit insertion axis required for fixed height basis')
    if not 0<distance<=200 or not np.allclose(direction,axis,atol=1e-6):
        raise ValueError('Insertion must follow tool +X and be at most 200 mm')
    if np.linalg.norm(tcp[:3,3])>1:raise ValueError('TCP uses metres; offset exceeds tool size bound')
    wrist_rotation=orientation[:3,:3]@tcp[:3,:3].T
    rpy=Rotation.from_matrix(wrist_rotation).as_euler('xyz',degrees=True)
    if not np.isfinite(max_step_mm) or not .25<=max_step_mm<=2.:raise ValueError('Invalid bounded insertion sampling')
    points=np.linspace(pre,end,int(np.ceil(distance/max_step_mm))+1)
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
        if bound>2:raise ValueError('Joint interpolation cannot keep TCP within 2 mm of insertion line')
        count=max(2,int(np.ceil(np.max(np.abs(dq))/np.radians(.1)))+1)
        for t in np.linspace(0,1,count):
            actual=(model.wrist_transform(a+t*dq,lift_rad)@tcp)[:3,:3]
            error=np.degrees(Rotation.from_matrix(orientation[:3,:3].T@actual).magnitude())
            angle_error=max(angle_error,float(error))
        if angle_error>.7:raise ValueError('Insertion orientation departs from fixed tool pose')
        waypoints.append(b)
    if not all(c['clear'] for c in clearances):raise ValueError('Path approaches central column')
    timing=(TimedJointPath(waypoints,model.reach+np.linalg.norm(tcp[:3,3]),
            tcp_position=lambda angles:(model.wrist_transform(angles,lift_rad)@tcp)[:3,3],
            **timing_limits).receipt()
            if timing_limits is not None else None)
    return dict(status='kinematic_preview',timing=timing,frame='base_link',position_unit='mm',joint_unit='rad',
        approach=dict(kind='joint_space',start_rad=q.tolist(),end_rad=first.tolist()),
        tool_points_mm=points.tolist(),wrist_pre_mm=wrists[0].tolist(),wrist_end_mm=wrists[-1].tolist(),
        insertion_axis_base=direction.tolist(),
        joint_waypoints_rad=[v.tolist() for v in waypoints],
        interpolation_position_bound_mm=position_bound,sampled_orientation_error_deg=angle_error,
        column_checked=True,column_lower_mm=min(c['lower_mm'] for c in clearances),
        environment_collision_checked=False,executable=False,
        blockers=['尚未核对夹爪、左右臂及环境碰撞',('尚未生成受速度和加速度约束的执行时序' if timing is None else '驱动尚未接入该时序，不可按异步关节目标执行'),
                  '连续位置界限仅适用于所检查的关节线性插值；控制服务必须采用匹配的插值'])
