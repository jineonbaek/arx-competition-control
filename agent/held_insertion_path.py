"""Fixed-Z aperture translation while preserving the observed tool rotation."""
import numpy as np
from scipy.spatial.transform import Rotation
from .grasp_path import TimedJointPath

ENTRY_ANGLE_DEG = 10.


def plan_insert(model, start_q, lift_rad, pre_mm, end_mm, base_tool_rotation, wrist_from_tool, limits_deg, *, timing_limits=None,fixed_z_axis=None,fixed_height_up=None,max_step_mm=2.,measured_gap_mm=None,entry_tolerance_deg=3.,adaptive_stroke=False,terminal_timing_limits=None):
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
                or abs(axis@up)>1e-6):
            raise ValueError('推进轴和高度基向量无效')
        if entry_tolerance_deg is not None and (not 0<entry_tolerance_deg<=ENTRY_ANGLE_DEG
                or axis@orientation[:3,0]<np.cos(np.radians(entry_tolerance_deg))-1e-12):
            raise ValueError(f'入仓方向与末端 +X 的夹角须在 {entry_tolerance_deg:g}° 以内，推进轴须水平')
    elif fixed_height_up is not None:raise ValueError('Explicit insertion axis required for fixed height basis')
    if distance<=0 or not np.allclose(direction,axis,atol=1e-6):
        raise ValueError('Insertion must follow the measured entry axis')
    legs = [distance]
    if distance > 200.:
        if measured_gap_mm is None or fixed_z_axis is None:
            raise ValueError('Travel without a measured approach is limited to 200 mm')
        gap = float(measured_gap_mm)
        if (not np.isfinite(gap) or not 0 < gap <= 200. or not 0 < distance-gap <= 200.
                or distance > 300. + 1e-6):
            raise ValueError('Measured approach and insertion exceed the planned travel bounds')
        legs = [gap, distance-gap]
    if np.linalg.norm(tcp[:3,3])>1:raise ValueError('TCP uses metres; offset exceeds tool size bound')
    wrist_rotation=orientation[:3,:3]@tcp[:3,:3].T
    rpy=Rotation.from_matrix(wrist_rotation).as_euler('xyz',degrees=True)
    if not np.isfinite(max_step_mm) or not .25<=max_step_mm<=2.:raise ValueError('Invalid bounded insertion sampling')
    # Sample each bounded leg, including the actual mouth crossing. Both legs
    # share one IK chain and one time law; the entire path is checked up front.
    terminal_start=None
    if terminal_timing_limits is not None:
        if len(legs)!=1 or timing_limits is None or distance<=10.:
            raise ValueError('Terminal timing requires one timed stroke longer than 10 mm')
        legs=[distance-10.,10.]
        terminal_start=int(np.ceil(legs[0]/max_step_mm))
    offsets=[0.]
    for leg in legs:
        offsets.extend(np.linspace(offsets[-1], offsets[-1]+leg, int(np.ceil(leg/max_step_mm))+1)[1:])
    points=pre+np.asarray(offsets)[:,None]*direction
    wrists=points-(wrist_rotation@tcp[:3,3])*1000
    reach_mm=(model.reach+np.linalg.norm(tcp[:3,3]))*1000.
    checked_ik=getattr(model,'inverse_wrist_with_clearance',None) if adaptive_stroke else None
    def solve(start,index):
        if checked_ik is not None:
            solution,clearance=checked_ik(start,lift_rad,wrists[index],rpy,limits,adaptive_intervals=True)
            return np.asarray(solution),clearance
        solution=np.asarray(model.inverse_wrist(start,lift_rad,wrists[index],rpy,limits))
        return solution,model.check(start,solution,lift_rad)

    first,clearance=solve(q,0)
    if not clearance['clear']:raise ValueError('Path approaches central column')
    waypoints=[first];indices=[0];position_bound=0.;angle_error=0.;lower_mm=clearance['lower_mm']

    def extend(last,*,subdivide=True):
        nonlocal position_bound,angle_error,lower_mm
        previous=indices[-1];a=waypoints[-1]
        try:
            b,clearance=solve(a,last)
            if not clearance['clear']:raise ValueError('Path approaches central column')
            dq=b-a
            if np.max(np.abs(dq))>np.radians(5):
                raise ValueError('Insertion IK branch jump or singular configuration')
            intervals=max(1,int(np.ceil(np.max(abs(dq))/np.radians(.1))))
            max_sample_error=0.;max_angle=0.
            for t in np.linspace(0.,1.,intervals+1):
                pose=model.wrist_transform(a+t*dq,lift_rad)@tcp
                if adaptive_stroke or t==0. or t==1.:
                    point=points[previous]+t*(points[last]-points[previous])
                    max_sample_error=max(max_sample_error,float(np.linalg.norm(pose[:3,3]*1000.-point)))
                angle=Rotation.from_matrix(orientation[:3,:3].T@pose[:3,:3]).magnitude()
                max_angle=max(max_angle,float(np.degrees(angle)))
            # For joint-linear interpolation, the chord error between FK samples
            # is bounded by reach * sum(abs(dq))**2 * (sample interval)**2 / 8.
            subdivisions=intervals if adaptive_stroke else 1
            bound=max_sample_error+reach_mm*(np.sum(abs(dq))/subdivisions)**2/8.
            if bound>2.:raise ValueError('Joint interpolation cannot keep TCP within 2 mm of insertion line')
            if max_angle>.7:raise ValueError('Insertion orientation departs from fixed tool pose')
            if adaptive_stroke and last-previous>1 and (bound>.5 or max_angle>.15):
                raise ValueError('Coarse joint interpolation requires subdivision')
        except ValueError:
            if not subdivide or not adaptive_stroke or last-previous<=1:raise
            extend((previous+last)//2)
            extend(last)
            return
        waypoints.append(b);indices.append(last)
        position_bound=max(position_bound,bound);angle_error=max(angle_error,max_angle)
        lower_mm=min(lower_mm,clearance['lower_mm'])

    # Keep explicit mouth crossings and endpoints. Split a proposed span on the
    # original fine grid if the actual joint-interpolated motion needs more knots.
    last=0
    stride=max(1,int(24./max_step_mm)) if adaptive_stroke else 1
    for leg in legs:
        last+=int(np.ceil(leg/max_step_mm))
        while indices[-1]<last:
            previous=indices[-1]
            distance_left=last-previous
            proposed=min(distance_left,stride)
            if not adaptive_stroke:
                extend(previous+proposed)
                continue
            # An IK span often just exceeds the per-joint increment. Fit the
            # largest nearby admissible span instead of forcing two half-stops.
            while proposed>1:
                try:
                    extend(previous+proposed,subdivide=False)
                    break
                except ValueError:
                    proposed=max(1,proposed-max(1,proposed//4))
            if indices[-1]==previous:extend(previous+1)
            stride=min(max(1,int(30./max_step_mm)),max(2,proposed+2))
    points=points[indices]
    def time_knots(knots,parameters):
        return TimedJointPath(knots,model.reach+np.linalg.norm(tcp[:3,3]),
            tcp_position=lambda angles:(model.wrist_transform(angles,lift_rad)@tcp)[:3,3],
            **parameters).receipt()
    timing=None
    if terminal_start is not None:
        split=indices.index(terminal_start)
        timing=time_knots(waypoints[:split+1],timing_limits)
        tail=time_knots(waypoints[split:],terminal_timing_limits)
        timing['terminal_start_knot']=split
        timing['terminal_limits']=tail['limits']
        timing['time_from_start_s'].extend(timing['duration_s']+t for t in tail['time_from_start_s'][1:])
        timing['duration_s']=timing['time_from_start_s'][-1]
    elif timing_limits is not None:
        timing=time_knots(waypoints,timing_limits)
    return dict(status='kinematic_preview',timing=timing,frame='base_link',position_unit='mm',joint_unit='rad',
        approach=dict(kind='joint_space',start_rad=q.tolist(),end_rad=first.tolist()),
        tool_points_mm=points.tolist(),wrist_pre_mm=wrists[0].tolist(),wrist_end_mm=wrists[-1].tolist(),
        insertion_axis_base=direction.tolist(),
        joint_waypoints_rad=[v.tolist() for v in waypoints],
        interpolation_position_bound_mm=position_bound,sampled_orientation_error_deg=angle_error,
        column_checked=True,column_lower_mm=lower_mm,
        environment_collision_checked=False,executable=False,
        blockers=['尚未核对夹爪、左右臂及环境碰撞',('尚未生成受速度和加速度约束的执行时序' if timing is None else '驱动尚未接入该时序，不可按异步关节目标执行'),
                  '连续位置界限仅适用于所检查的关节线性插值；控制服务必须采用匹配的插值'])
