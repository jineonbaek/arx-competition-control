"""Versioned taught placement route and monotonic phase receipts."""
from dataclasses import dataclass
import json, math, time
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class TaughtRoute:
    version:int
    lift_target_rad:float
    lift_tolerance_rad:float
    full_depth_mm:float
    d405_step_limit_mm:float
    d405_total_limit_mm:float
    d405_attempts:int
    learned_vertical_bias_mm:float
    outside_retreat_mm:float
    observation_joints_deg:tuple


def load_taught_route(path):
    d=json.loads(Path(path).read_text())
    values=[d[k] for k in ('version','lift_target_rad','lift_tolerance_rad','full_depth_mm','d405_step_limit_mm','d405_total_limit_mm','d405_attempts','learned_vertical_bias_mm','outside_retreat_mm')]
    q=np.asarray(d['observation_joints_deg'],float)
    if (d['version']!=1 or q.shape!=(6,) or not np.isfinite(np.r_[values,q]).all()
            or not 2.5<=d['lift_target_rad']<=3.5 or not .1<=d['lift_tolerance_rad']<=.5
            or not 100<=d['full_depth_mm']<=180 or not 0<d['d405_step_limit_mm']<=5
            or not d['d405_step_limit_mm']<=d['d405_total_limit_mm']<=15
            or not 1<=d['d405_attempts']<=6):
        raise ValueError('Invalid taught placement route')
    return TaughtRoute(*(d[k] for k in ('version','lift_target_rad','lift_tolerance_rad','full_depth_mm','d405_step_limit_mm','d405_total_limit_mm','d405_attempts','learned_vertical_bias_mm','outside_retreat_mm')),tuple(q.tolist()))


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
