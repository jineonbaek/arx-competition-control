"""Offline insertion IK with an explicit calibrated tool transform; no command transport."""
import numpy as np
from scipy.spatial.transform import Rotation


def plan_insert(model, start_q, lift_rad, pre_mm, end_mm, base_tool_rotation, wrist_from_tool, limits_deg, *, timing_limits=None):
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
    if not 0<distance<=200 or not np.allclose(direction,orientation[:3,0],atol=1e-6):
        raise ValueError('Insertion must follow tool +X and be at most 200 mm')
    if np.linalg.norm(tcp[:3,3])>1:raise ValueError('TCP uses metres; offset exceeds tool size bound')
    wrist_rotation=orientation[:3,:3]@tcp[:3,:3].T
    rpy=Rotation.from_matrix(wrist_rotation).as_euler('xyz',degrees=True)
    points=np.linspace(pre,end,int(np.ceil(distance/2))+1)
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


class TimedJointPath:
    """Synchronized rest-to-rest segments; no transport and no physical defaults.

    q(s)=q0+s*dq retains the checked joint-space curve. A quintic scalar changes
    only timing. Every knot stops; upgrade to blended timing only with a new swept
    path check. Bounds describe references, not motor tracking or braking behavior.
    """
    def __init__(self,waypoints,reach_m,*,joint_speed_deg_s,joint_acceleration_deg_s2,
                 tcp_speed_mm_s,tcp_acceleration_mm_s2,tcp_position=None):
        q=np.asarray(waypoints,float)
        v=np.asarray(joint_speed_deg_s,float);a=np.asarray(joint_acceleration_deg_s2,float)
        if (q.ndim!=2 or q.shape[1]!=6 or len(q)<2 or v.shape!=(6,) or a.shape!=(6,)
                or not np.isfinite(q).all() or not np.isfinite(v).all() or not np.isfinite(a).all()
                or np.any(v<=0) or np.any(v>8.55) or np.any(a<=0)
                or not np.isfinite([reach_m,tcp_speed_mm_s,tcp_acceleration_mm_s2]).all()
                or min(reach_m,tcp_speed_mm_s,tcp_acceleration_mm_s2)<=0):
            raise ValueError('Explicit finite six-joint timing limits required; X5 speed cap is 8.55 deg/s')
        delta=np.abs(np.diff(q,axis=0));length=delta.sum(axis=1)
        if np.any(length==0):raise ValueError('Duplicate adjacent trajectory knots')
        speed_bound=reach_m*1000*length
        curvature_bound=reach_m*1000*length**2
        if tcp_position is not None:
            if not callable(tcp_position):raise ValueError('TCP position function required')
            for i,(start,end) in enumerate(zip(q,q[1:])):
                steps=max(2,int(np.ceil(np.max(np.abs(end-start))/np.radians(.25))))
                xyz=np.asarray([tcp_position(start+s*(end-start))
                                for s in np.linspace(0,1,steps+1)],float)*1000
                if xyz.shape!=(steps+1,3) or not np.isfinite(xyz).all():
                    raise ValueError('Finite metre-unit TCP positions required')
                # Serial revolute-chain derivative bounds cover the gaps between FK samples.
                speed_bound[i]=(np.linalg.norm(np.diff(xyz,axis=0),axis=1).max()*steps
                                +reach_m*1000*length[i]**2/(2*steps))
                curvature_bound[i]=(np.linalg.norm(np.diff(xyz,n=2,axis=0),axis=1).max()*steps**2
                                    +2*reach_m*1000*length[i]**3/steps)
        # Exact extrema of s=10u^3-15u^4+6u^5: max s'=15/8, max |s''|=10/sqrt(3).
        peak_v=15/8;peak_a=10/np.sqrt(3)
        durations=np.maximum.reduce([
            peak_v*np.max(delta/np.radians(v),axis=1),
            np.sqrt(peak_a*np.max(delta/np.radians(a),axis=1)),
            speed_bound*peak_v/tcp_speed_mm_s,
            np.sqrt((speed_bound*peak_a+curvature_bound*peak_v**2)/tcp_acceleration_mm_s2)])
        if not np.isfinite(durations).all() or np.any(durations<=0):raise ValueError('Invalid trajectory duration')
        self.q=q.copy();self.times=np.r_[0.,np.cumsum(durations)]
        if not np.isfinite(self.times).all() or np.any(np.diff(self.times)<=0):
            raise ValueError('Trajectory time resolution invalid')
        self.limits=dict(joint_speed_deg_s=v.tolist(),joint_acceleration_deg_s2=a.tolist(),
            tcp_speed_mm_s=float(tcp_speed_mm_s),tcp_acceleration_mm_s2=float(tcp_acceleration_mm_s2))
        self.reach_m=float(reach_m)

    def sample(self,elapsed_s):
        if not np.isfinite(elapsed_s) or elapsed_s<0 or elapsed_s>self.times[-1]:
            raise ValueError('Sample outside trajectory time interval')
        i=min(np.searchsorted(self.times,elapsed_s,side='right')-1,len(self.q)-2)
        duration=self.times[i+1]-self.times[i]
        u=(elapsed_s-self.times[i])/duration
        scalar=u**3*(10+u*(-15+6*u))
        velocity=30*u**2*(1-u)**2/duration
        acceleration=60*u*(1-u)*(1-2*u)/duration**2
        delta=self.q[i+1]-self.q[i]
        return self.q[i]+scalar*delta,velocity*delta,acceleration*delta

    def receipt(self):
        return dict(status='timed_preview',interpolation='segment_quintic_scalar',joint_unit='rad',
            time_unit='s',time_from_start_s=self.times.tolist(),duration_s=float(self.times[-1]),
            limits=self.limits.copy(),reach_bound_m=self.reach_m,stop_at_each_knot=True,
            driver_supported=False,executable=False,
            note='仅参考轨迹的速度/加速度界；不保证实机跟踪精度，不可逐点异步发送替代驱动内时序。')
