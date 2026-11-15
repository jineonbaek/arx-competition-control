"""Bounded local docking proposals and cumulative budgets; never actuator commands."""
import math
import numpy as np
from .held_battery import finite


class SearchBudget:
    def __init__(self,*,radius_m=.30,max_distance_m=.60,max_yaw_rad=math.radians(15),max_total_yaw_rad=math.radians(30),max_views=7):
        if any(not finite(v) or v<=0 for v in (radius_m,max_distance_m,max_yaw_rad,max_total_yaw_rad)) or type(max_views) is not int or max_views<1:raise ValueError('Invalid local budget')
        self.radius=radius_m;self.max_distance=max_distance_m;self.max_yaw=max_yaw_rad;self.max_total_yaw=max_total_yaw_rad;self.max_views=max_views
        self.xy=np.zeros(2);self.yaw=0.;self.distance=0.;self.rotation=0.;self.views=0

    def commit(self,xy,yaw):
        xy=np.asarray(xy,float)
        if xy.shape!=(2,) or not np.isfinite(xy).all() or not finite(yaw):raise ValueError('Invalid local waypoint')
        distance=self.distance+np.linalg.norm(xy-self.xy);rotation=self.rotation+abs(yaw-self.yaw)
        if (np.linalg.norm(xy)>self.radius+1e-12 or abs(yaw)>self.max_yaw+1e-12 or distance>self.max_distance+1e-12
                or rotation>self.max_total_yaw+1e-12 or self.views>=self.max_views):raise ValueError('Search/correction region or cumulative budget exhausted')
        self.distance=float(distance);self.rotation=float(rotation);self.views+=1;self.xy=xy.copy();self.yaw=float(yaw)
        return self.snapshot()

    def snapshot(self):return dict(xy_m=self.xy.tolist(),yaw_rad=self.yaw,total_distance_m=self.distance,total_rotation_rad=self.rotation,views=self.views)

    def reconcile(self,xy,yaw,*,distance_upper_m=None,rotation_upper_rad=None):
        xy=np.asarray(xy,float)
        if xy.shape!=(2,) or not np.isfinite(xy).all() or not finite(yaw):raise ValueError('Invalid measured budget pose')
        delta=float(np.linalg.norm(xy-self.xy));turn=abs(yaw-self.yaw)
        travel=delta if distance_upper_m is None else distance_upper_m;rotation=turn if rotation_upper_rad is None else rotation_upper_rad
        if (not finite(travel) or not finite(rotation) or travel+1e-12<delta or rotation+1e-12<turn
                or np.linalg.norm(xy)>self.radius+1e-12 or abs(yaw)>self.max_yaw+1e-12
                or self.distance+travel>self.max_distance+1e-12 or self.rotation+rotation>self.max_total_yaw+1e-12):
            raise ValueError('Measured/unobserved travel exceeds cumulative budget')
        self.distance+=travel;self.rotation+=rotation;self.xy=xy.copy();self.yaw=float(yaw)


class DockingPlanner:
    def propose(self,target_m,*,current_lift_rad,check=None):
        target=np.asarray(target_m,float)
        if target.shape!=(3,) or not np.isfinite(target).all() or not finite(current_lift_rad) or not 0<=current_lift_rad<=18:raise ValueError('Invalid metric target or raw lift')
        preferred=np.array([target[0]-.45,target[1]+.25]);length=np.linalg.norm(preferred)
        if length>.25:preferred*=.25/length
        candidates=[];rejected=[]
        for dx in (-.08,0.,.08):
            for dy in (-.08,0.,.08):
                xy=preferred+[dx,dy]
                if np.linalg.norm(xy)>.30:continue
                for yaw in (-.15,0.,.15):
                    for lift in sorted(set(np.clip([current_lift_rad-1,current_lift_rad,current_lift_rad+1],0,18).tolist())):
                        c=dict(base_delta_m=xy.tolist(),yaw_rad=yaw,lift_rad=lift,target_local_m=target.tolist())
                        if check is None:rejected.append('Missing independent kinematic, obstacle and stopping checks');continue
                        try:
                            row=check(c)
                            if (not isinstance(row,dict) or not all(row.get(k) is True for k in ('ik_clear','normal_clear','stopping_clear','coverage_complete','retreat_clear'))
                                    or not finite(row.get('min_clearance_mm')) or row['min_clearance_mm']<=0
                                    or not finite(row.get('joint_margin_deg')) or row['joint_margin_deg']<=0):raise ValueError('Candidate normal/stopping/reach/retreat checks rejected')
                            c['check']=row;candidates.append(c)
                        except (ValueError,TypeError,KeyError):rejected.append('Candidate checks failed')
        candidates.sort(key=lambda c:(-c['check']['min_clearance_mm'],-c['check']['joint_margin_deg'],
            np.linalg.norm(c['base_delta_m'])+.02*abs(c['lift_rad']-current_lift_rad)+.02*abs(c['yaw_rad'])))
        return dict(status='candidate' if candidates else 'blocked',selected=candidates[0] if candidates else None,
            candidate_count=len(candidates),rejected_count=len(rejected),reasons=sorted(set(rejected)),
            mode='offline',executable=False,physical_clearance_verified=False)
