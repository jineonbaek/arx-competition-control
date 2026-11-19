"""Versioned taught placement route and monotonic phase receipts."""
from dataclasses import dataclass
import json, math, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class TaughtRoute:
    version:int
    lift_target_rad:float
    lift_tolerance_rad:float
    full_depth_mm:float
    fixed_entry_waypoint:str
    d405_preobservation_waypoint:str
    navigation_handoff:str
    visual_chassis_policy:str
    visual_chassis_handoff:str
    d405_step_limit_mm:float
    d405_total_limit_mm:float
    d405_attempts:int
    d405_target_offset_mm:float
    d405_convergence_mm:float
    d405_capture_timeout_s:float
    preobservation_axis_tolerance_deg:float
    preobservation_z_tolerance_mm:float
    preobservation_rotation_tolerance_deg:float
    preobservation_j6_target_deg:float
    preobservation_j6_tolerance_deg:float
    preobservation_j5_margin_deg:float
    preobservation_elbow_margin_deg:float
    learned_vertical_bias_mm:float
    outside_retreat_mm:float
    observation_joints_deg:tuple


def load_taught_route(path):
    d=json.loads(Path(path).read_text())
    numeric=('version','lift_target_rad','lift_tolerance_rad','full_depth_mm','d405_step_limit_mm','d405_total_limit_mm',
        'd405_attempts','d405_target_offset_mm','d405_convergence_mm','d405_capture_timeout_s',
        'preobservation_axis_tolerance_deg','preobservation_z_tolerance_mm','preobservation_rotation_tolerance_deg',
        'preobservation_j6_target_deg','preobservation_j6_tolerance_deg','preobservation_j5_margin_deg',
        'preobservation_elbow_margin_deg','learned_vertical_bias_mm','outside_retreat_mm')
    try:values=[d[k] for k in numeric]
    except KeyError as error:raise ValueError('Invalid taught placement route') from error
    fixed=d.get('fixed_entry_waypoint')
    preobserve=d.get('d405_preobservation_waypoint')
    handoff=d.get('navigation_handoff')
    chassis_policy=d.get('visual_chassis_policy')
    chassis_handoff=d.get('visual_chassis_handoff')
    q=np.asarray(d['observation_joints_deg'],float)
    if (d['version']!=2 or q.shape!=(6,) or not np.isfinite(np.r_[values,q]).all()
            or not 2.5<=d['lift_target_rad']<=3.5 or not .1<=d['lift_tolerance_rad']<=.5
            or not 100<=d['full_depth_mm']<=180 or not isinstance(fixed,str) or not fixed.startswith('reports/manual-waypoints/') or '..' in fixed
            or not isinstance(preobserve,str) or not preobserve.startswith('reports/manual-waypoints/') or '..' in preobserve
            or handoff!='visual_chassis_locked_admitted'
            or chassis_policy!='config/visual-chassis-placement.json'
            or chassis_handoff!='reports/visual-chassis-active/handoff.json'
            or d['d405_step_limit_mm']!=3. or d['d405_total_limit_mm']!=15. or d['d405_attempts']!=4
            or d['d405_target_offset_mm']!=42.4 or d['d405_convergence_mm']!=.75 or d['d405_capture_timeout_s']!=8.
            or not 0<d['preobservation_axis_tolerance_deg']<=.2
            or not 0<d['preobservation_z_tolerance_mm']<=.5
            or not 0<d['preobservation_rotation_tolerance_deg']<=.2
            or d['preobservation_j6_target_deg']!=-90. or not 0<d['preobservation_j6_tolerance_deg']<=3.
            or d['preobservation_j5_margin_deg']<5. or d['preobservation_elbow_margin_deg']<10.):
        raise ValueError('Invalid taught placement route')
    fields=tuple(TaughtRoute.__dataclass_fields__)
    return TaughtRoute(**{name:(tuple(q.tolist()) if name=='observation_joints_deg' else d[name]) for name in fields})


def load_preobservation_waypoint(path,current_lift):
    try:data=json.loads(Path(path).read_text())
    except (OSError,ValueError) as error:raise ValueError('Invalid D405 pre-observation waypoint') from error
    q=np.asarray(data.get('joints_rad'),float);w=np.asarray(data.get('wrist_base_mm_deg'),float)
    tool=np.asarray(data.get('tool_base_matrix'),float);tcp_hash=data.get('source_hashes',{}).get('config/right-grip-tcp.json')
    try:values=np.r_[q,w,float(data.get('lift_rad')),float(current_lift),float(data.get('captured_at'))]
    except (TypeError,ValueError) as error:raise ValueError('Invalid D405 pre-observation waypoint') from error
    if (data.get('version')!=1 or data.get('arm')!='right' or q.shape!=(6,) or w.shape!=(6,)
            or tool.shape!=(4,4) or not np.isfinite(tool).all() or not np.allclose(tool[3],[0,0,0,1])
            or not np.allclose(tool[:3,:3].T@tool[:3,:3],np.eye(3),atol=1e-6)
            or not isinstance(tcp_hash,str) or len(tcp_hash)!=64
            or not np.isfinite(values).all() or data.get('gripper_command_target')!=0
            or abs(float(data['lift_rad'])-float(current_lift))>.10
            or not isinstance(data.get('source'),str) or not data['source']):
        raise ValueError('Invalid D405 pre-observation waypoint')
    return data


def validate_preobservation_pose(tool,joints,joint_limits_deg,route):
    """Validate and return the FK-derived lock reference for D405 corrections."""
    T=np.asarray(tool,float);q=np.asarray(joints,float);limits=np.asarray(joint_limits_deg,float)
    if (T.shape!=(4,4) or q.shape!=(6,) or limits.shape!=(6,2)
            or not np.isfinite(np.r_[T.ravel(),q,limits.ravel()]).all()
            or not np.allclose(T[3],[0,0,0,1])
            or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)
            or np.linalg.det(T[:3,:3])<.999999):
        raise ValueError('Invalid pre-observation pose')
    degrees=np.degrees(q)
    if np.any(degrees<limits[:,0]) or np.any(degrees>limits[:,1]):
        raise ValueError('Pre-observation joint limits exceeded')
    axis=T[:3,0]/np.linalg.norm(T[:3,0])
    axis_error=float(np.degrees(np.arccos(np.clip(axis@[1.,0.,0.],-1.,1.))))
    if axis_error>route.preobservation_axis_tolerance_deg:
        raise ValueError('Pre-observation insertion axis is not base +X')
    j6_error=abs(float(degrees[5]-route.preobservation_j6_target_deg))
    if j6_error>route.preobservation_j6_tolerance_deg:
        raise ValueError('Pre-observation J6 branch unavailable')
    j5_margin=float(min(degrees[4]-limits[4,0],limits[4,1]-degrees[4]))
    if j5_margin<route.preobservation_j5_margin_deg:
        raise ValueError('Pre-observation J5 software-limit margin insufficient')
    elbow_margin=float(min(abs(degrees[2]),abs(180.-abs(degrees[2]))))
    if elbow_margin<route.preobservation_elbow_margin_deg:
        raise ValueError('Pre-observation elbow margin insufficient')
    return dict(insertion_axis_base=axis.tolist(),tcp_z_m=float(T[2,3]),
        rotation_base_from_tool=T[:3,:3].tolist(),axis_error_deg=axis_error,
        z_tolerance_mm=float(route.preobservation_z_tolerance_mm),
        rotation_tolerance_deg=float(route.preobservation_rotation_tolerance_deg),
        j6_error_deg=j6_error,j5_margin_deg=j5_margin,elbow_margin_deg=elbow_margin)


def validate_preobservation_waypoint(waypoint,planned_wrist,planned_tool,joints,joint_limits_deg,route,tcp_sha256):
    """Bind a recorded waypoint to FK and validate it before dispatch."""
    wrist=np.asarray(planned_wrist,float);tool=np.asarray(planned_tool,float)
    record=np.asarray(waypoint.get('wrist_base_mm_deg'),float);recorded_tool=np.asarray(waypoint.get('tool_base_matrix'),float)
    recorded_hash=waypoint.get('source_hashes',{}).get('config/right-grip-tcp.json')
    if (wrist.shape!=(4,4) or record.shape!=(6,) or not np.isfinite(np.r_[wrist.ravel(),record]).all()
            or not np.allclose(wrist[3],[0,0,0,1])):
        raise ValueError('Invalid recorded wrist pose')
    if (tool.shape!=(4,4) or recorded_tool.shape!=(4,4)
            or not np.isfinite(np.r_[tool.ravel(),recorded_tool.ravel()]).all()
            or not np.allclose(recorded_tool[3],[0,0,0,1])
            or not np.allclose(recorded_tool[:3,:3].T@recorded_tool[:3,:3],np.eye(3),atol=1e-6)):
        raise ValueError('Invalid recorded tool pose')
    if (not isinstance(tcp_sha256,str) or len(tcp_sha256)!=64 or recorded_hash!=tcp_sha256):
        raise ValueError('Pre-observation TCP hash changed')
    expected=np.eye(4);expected[:3,3]=record[:3]/1000.
    expected[:3,:3]=Rotation.from_euler('xyz',record[3:],degrees=True).as_matrix()
    position_error=float(np.linalg.norm(wrist[:3,3]-expected[:3,3])*1000.)
    relative=expected[:3,:3].T@wrist[:3,:3]
    rotation_error=float(Rotation.from_matrix(relative).magnitude()*180./np.pi)
    if position_error>route.preobservation_z_tolerance_mm or rotation_error>route.preobservation_rotation_tolerance_deg:
        raise ValueError('Pre-observation recorded wrist does not match FK')
    tool_position_error=float(np.linalg.norm(tool[:3,3]-recorded_tool[:3,3])*1000.)
    tool_relative=recorded_tool[:3,:3].T@tool[:3,:3]
    tool_rotation_error=float(Rotation.from_matrix(tool_relative).magnitude()*180./np.pi)
    if tool_position_error>route.preobservation_z_tolerance_mm or tool_rotation_error>route.preobservation_rotation_tolerance_deg:
        raise ValueError('Pre-observation recorded tool does not match FK')
    result=validate_preobservation_pose(planned_tool,joints,joint_limits_deg,route)
    result.update(recorded_wrist_position_error_mm=position_error,
        recorded_wrist_rotation_error_deg=rotation_error)
    result.update(recorded_tool_position_error_mm=tool_position_error,
        recorded_tool_rotation_error_deg=tool_rotation_error,tcp_sha256=tcp_sha256)
    return result


def reverse_reached_route(records):
    q=np.asarray(records,float)
    if q.ndim!=2 or q.shape[1]!=6 or len(q)<1 or not np.isfinite(q).all():
        raise ValueError('Invalid reached route')
    return q[::-1].tolist()


def outer_alignment_error(fit,window,slot,*,cross_tolerance_mm=5.,standoff_range_mm=(40.,100.)):
    points=np.asarray(fit.get('footprint_uv_mm'),float)
    measured=np.asarray(window.get('measured_size_mm'),float)
    nominal=np.asarray(slot.get('size_mm'),float)
    standoff=float(fit.get('leading_standoff_mm',float('nan')))
    limits=np.asarray(standoff_range_mm,float)
    if (points.ndim!=2 or points.shape[1]!=2 or not len(points) or measured.shape!=(2,) or nominal.shape!=(2,)
            or limits.shape!=(2,) or not np.isfinite(np.r_[points.ravel(),measured,nominal,standoff,limits,cross_tolerance_mm]).all()
            or np.any(measured<=0) or np.any(nominal<=0) or not 0<cross_tolerance_mm<=10. or not 0<=limits[0]<limits[1]):
        raise ValueError('Invalid current D435 alignment observation')
    target=np.array([min(measured[0],nominal[0])/4.,0.])
    center=points.mean(0);residual=target-center
    return dict(target_uv_mm=target.tolist(),center_uv_mm=center.tolist(),residual_uv_mm=residual.tolist(),
        leading_standoff_mm=standoff,converged=bool(np.max(np.abs(residual))<=cross_tolerance_mm and limits[0]<=standoff<=limits[1]))


class PhaseTimeline:
    def __init__(self,monotonic=time.monotonic,wall=time.time):
        self.monotonic=monotonic;self.wall=wall;self.events=[];self.active=None
        self.total_started_at=wall();self._total_start=monotonic();self.total_finished_at=None;self._total_duration=None
    def start(self,phase,*,attempt=1,planned_duration_s=None,**metrics):
        if self.active is not None:raise ValueError('Previous phase still active')
        if not isinstance(phase,str) or not phase:raise ValueError('Invalid phase')
        self.active=dict(phase=phase,attempt=int(attempt),started_at=self.wall(),_start=self.monotonic(),planned_duration_s=planned_duration_s,**metrics)
        return self.active
    def plan(self,seconds):
        if (self.active is None or self.active.get('planned_duration_s') is not None
                or type(seconds) not in (int,float) or not math.isfinite(seconds) or seconds<=0):
            raise ValueError('Active phase requires one positive planned duration')
        self.active['planned_duration_s']=float(seconds);return self.active
    def finish(self,status='ok',reason='',**metrics):
        if self.active is None:raise ValueError('No active phase')
        row=self.active;self.active=None;end=self.monotonic();duration=max(0.,float(end-row.pop('_start')))
        cumulative=sum(float(x['duration_s']) for x in self.events)+duration
        row.update(finished_at=self.wall(),duration_s=duration,cumulative_duration_s=cumulative,status=status,reason=reason,**metrics);self.events.append(row);return row
    def complete(self):
        if self.active is not None:raise ValueError('Active phase remains')
        self.total_finished_at=self.wall();self._total_duration=max(0.,float(self.monotonic()-self._total_start));return self.snapshot()
    def snapshot(self):
        duration=self._total_duration if self._total_duration is not None else max(0.,float(self.monotonic()-self._total_start))
        phase_duration=0. if self.active is None else max(0.,float(self.monotonic()-self.active['_start']))
        active=None if self.active is None else {k:v for k,v in self.active.items() if k!='_start'}
        if active is not None:active.update(duration_s=phase_duration,cumulative_duration_s=sum(float(x['duration_s']) for x in self.events)+phase_duration,status='running')
        return dict(total_started_at=self.total_started_at,total_finished_at=self.total_finished_at,total_duration_s=duration,
                    phase_duration_s=phase_duration,active_phase=active,timeline=[dict(x) for x in self.events])
