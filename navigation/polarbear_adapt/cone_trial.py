"""Track an RViz path past one measured cone at a fixed chassis heading."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs_py.point_cloud2 import read_points_numpy

from axis_trial import AxisTrial,body_delta
from follow_preview import angle
from path_audit import planar


def clearance(poly,pose,center):
    delta=body_delta(pose,np.r_[center,pose[2]])[:2]
    edge=np.roll(poly,-1,axis=0)-poly
    rel=delta-poly
    u=np.clip(np.sum(rel*edge,axis=1)/np.sum(edge*edge,axis=1),0,1)
    cross=edge[:,0]*rel[:,1]-edge[:,1]*rel[:,0]
    if np.all(cross>=0) or np.all(cross<=0):return 0.
    return np.linalg.norm(rel-u[:,None]*edge,axis=1).min()


class ConeTrial(AxisTrial):
    def __init__(self,center,radius,goal,levels):
        super().__init__()
        self.center=center;self.radius=radius;self.target=goal;self.levels=levels
        self.route=None;self.failure=None;self.started=False;self.hits=[]
        self.min_gap=math.inf
        self.travel_yaw=None
        self.final_turn=False
        self.validated_segment=None
        self.selected=None
        self.resuming=False
        self.display=self.create_publisher(Marker,"/pb/path_issue",1)
        self.create_subscription(PointCloud2,'/pb/obstacles',self.observe,qos_profile_sensor_data)

    def observe(self,m):
        try:t=self.tf.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
        except Exception:return
        q=t.rotation;v=t.translation;a=read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
        pts=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(a)+[v.x,v.y,v.z]
        self.hits=(self.hits+[(time.monotonic(),bool(np.any(np.linalg.norm(pts[:,:2]-self.center,axis=1)<self.radius+.07)))])[-12:]

    def changed(self,_):
        if self.started:self.failure='收到新目标'

    def receive(self,m):
        self.route=np.array([planar(p.pose) for p in m.poses]) if m.header.frame_id=='map' else None
        self.stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.started and (self.route is None or not len(self.route)):
            self.brake()
            self.validated_segment=None

    def check(self):
        if self.failure:raise RuntimeError(self.failure)
        if not self.ready():raise RuntimeError('传感器或地图过期')
        pose=self.pose('map',self.now())
        gap=clearance(self.poly,pose,self.center)-self.radius
        self.min_gap=min(self.min_gap,float(gap))
        if gap<.06 or (not self.final_turn and self.travel_yaw is not None and abs(angle(pose[2]-self.travel_yaw))>.23):raise RuntimeError('净空或车头朝向越界')
        if self.grids['map'][1].inspect(pose)['first_cell']:raise RuntimeError('当前全局轮廓被挡')
        localpose=self.pose('odom',self.now())
        if self.grids['odom'][1].inspect(localpose)['first_cell']:raise RuntimeError('当前局部轮廓被挡')
        return pose

    def route_segment(self,pose):
        if self.route is None or not len(self.route) or not 0<=self.now()-self.stamp<4:
            if self.active:raise RuntimeError('运动中规划过期')
            end=time.monotonic()+10
            while time.monotonic()<end and not self.failure:
                self.spin(.08)
                if self.route is not None and len(self.route) and 0<=self.now()-self.stamp<4:break
            else:raise RuntimeError('没有新鲜的规划路径')
        if np.linalg.norm(self.route[-1,:2]-self.target[:2])>.06 or abs(angle(self.route[-1,2]-self.target[2]))>.15:
            raise RuntimeError('实际路径终点不匹配')
        last_error=None
        for source in (self.selected,self.route):
            if source is None:continue
            try:
                result=self.audit_remaining(source,pose)
            except ValueError as exc:
                last_error=exc
                continue
            if source is not self.selected:self.show_route(result)
            self.selected=source
            return result
        if self.active:raise RuntimeError(str(last_error))
        # The cone's measured/confirmed base extends beyond some visible laser cells.
        # Plan a right-hand bypass at fixed heading and audit every swept cell.
        diff=self.target[:2]-pose[:2];distance=np.linalg.norm(diff)
        if distance<.25:raise RuntimeError(str(last_error))
        ahead=diff/distance;right=np.array([ahead[1],-ahead[0]])
        for offset in [.85,.95,1.05,1.15]:
            mid=self.center+offset*right
            for length in [.28,.45]:
                corners=[pose[:2],mid-length*ahead,mid+length*ahead,self.target[:2]]
                if (corners[1]-pose[:2])@ahead<.04:continue
                xy=np.vstack([a+u*(b-a) for a,b in zip(corners,corners[1:])
                              for u in np.linspace(0,1,max(2,math.ceil(np.linalg.norm(b-a)/.07)),endpoint=False)]+[corners[-1]])
                candidate=np.column_stack((xy,np.full(len(xy),pose[2])))
                try:result=self.audit_remaining(candidate,pose)
                except ValueError:continue
                self.selected=candidate
                self.show_route(result)
                return result
        raise RuntimeError(str(last_error))

    def show_route(self,route):
        m=Marker();m.header.frame_id="map";m.ns="cone_trial_corridor";m.id=0
        m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=.045
        m.color.a=1.;m.color.g=.8;m.color.b=.95
        m.points=[Point(x=float(x),y=float(y),z=.15) for x,y,_ in route]
        self.display.publish(m)
        print(json.dumps(dict(accepted_route=route.tolist())),flush=True)

    def clear_route(self):
        m=Marker();m.header.frame_id="map";m.ns="cone_trial_corridor";m.id=0;m.action=Marker.DELETE
        self.display.publish(m)

    def audit_remaining(self,source,pose):
        j=np.argmin(np.linalg.norm(source[:,:2]-pose[:2],axis=1))
        if np.linalg.norm(source[j,:2]-pose[:2])>.18:raise ValueError('偏离当前路径')
        route=np.vstack((pose,source[j:]))
        route[:,2]=pose[2]
        stamp=self.grids['map'][0]
        key=(id(source),j,stamp.sec,stamp.nanosec,round(pose[2],2))
        if self.validated_segment==key:return route
        if self.grids['map'][1].along(route)['first']:raise ValueError('保持当前车头方向时路径不可通行')
        for a,b in zip(route,route[1:]):
            steps=max(1,math.ceil(np.linalg.norm(b[:2]-a[:2])/.025))
            for u in np.linspace(0.,1.,steps+1):
                if clearance(self.poly,a+u*(b-a),self.center)<self.radius+.075:
                    raise ValueError('规划路径距离锥桶不足')
        self.validated_segment=key
        return route

    def guarded_pose(self,direction):
        pose=self.check()
        if self.final_turn:
            if np.linalg.norm(pose[:2]-self.target[:2])>.05:raise RuntimeError('终点转向时位置超差')
        elif not self.active:self.route_segment(pose)
        elif self.route is None or not len(self.route) or not 0<=self.now()-self.stamp<4:
            raise RuntimeError('运动中规划过期')
        dx,dy,w=direction
        end=pose+[(math.cos(pose[2])*dx-math.sin(pose[2])*dy)*.09,
                  (math.sin(pose[2])*dx+math.cos(pose[2])*dy)*.09,.2*w]
        for u in np.linspace(0.,1.,7):
            if clearance(self.poly,pose+u*(end-pose),self.center)<self.radius+.06:
                raise RuntimeError('点动会侵入锥桶净空')
        return super().guarded_pose(direction)

    def run(self):
        self.spin(5)
        pose=self.check()
        deadline=time.monotonic()+10
        while True:
            try:
                route=self.route_segment(pose)
                break
            except RuntimeError:
                if time.monotonic()>=deadline:raise
                self.spin(.15)
                pose=self.check()
        if not self.resuming and sum(ok for t,ok in self.hits if time.monotonic()-t<2)<3:raise RuntimeError('未证实锥桶有实时回波')
        if not self.resuming and min(clearance(self.poly,pose+u*(np.r_[self.target[:2],pose[2]]-pose),self.center)
               for u in np.linspace(0.,1.,81))>self.radius:raise RuntimeError('直线没有被锥桶挡住')
        if not self.resuming and self.grids['map'][1].along([pose,np.r_[self.target[:2],pose[2]]])['first'] is None:
            raise RuntimeError('直线路径未被代价地图阻挡')
        self.travel_yaw=pose[2]
        self.started=True
        stalls=0
        for _ in range(175):
            p=self.check();e=body_delta(p,self.target)
            if np.linalg.norm(e[:2])<=.045 and abs(e[2])<=.09:
                self.spin(.45);rows=[];until=time.monotonic()+1
                while time.monotonic()<until:
                    err=body_delta(self.check(),self.target)
                    if np.linalg.norm(err[:2])>.045 or abs(err[2])>.09:raise RuntimeError('停车后超差')
                    rows.append(self.pose('odom',self.now()));self.spin(.08)
                span=np.ptp(rows,axis=0)
                if np.linalg.norm(span[:2])>.009 or span[2]>.02:raise RuntimeError('未停稳')
                return dict(status='arrived_stopped',goal=self.target.tolist(),error=err.tolist(),
                            min_sampled_cone_gap_m=self.min_gap,stationary_span=span.tolist())
            self.final_turn=np.linalg.norm(e[:2])<=.045
            route=np.array([p,self.target]) if self.final_turn else self.route_segment(p)
            look=next((v for v in route if np.linalg.norm(v[:2]-p[:2])>=.105),route[-1])
            local=body_delta(p,look)
            yaw_error=e[2] if self.final_turn else angle(self.travel_yaw-p[2])
            if self.final_turn or abs(yaw_error)>.075:
                axis=2;value=yaw_error
            else:
                axis=int(abs(local[1])>abs(local[0]));value=local[axis]
            sign=np.sign(value);direction=np.zeros(3);direction[axis]=sign
            if not sign or (axis==0 and sign<0):raise RuntimeError('当前路线需要未标定的后退')
            name=['forward','left' if sign>0 else 'right','ccw' if sign>0 else 'cw'][axis]
            row=self.pulse(direction,self.levels[name],seconds=1.05 if name=='right' else .65)
            print(json.dumps(dict(pose=p.tolist(),remaining=e.tolist(),axis=name,delta=row['delta'],command_count=row['command_count'],command_span=row['command_span'])),flush=True)
            stalls=stalls+1 if np.dot(row['delta'],direction)<(.005 if axis!=2 else .012) else 0
            if stalls>=2:raise RuntimeError('连续两次无进展')
        raise RuntimeError('动作次数上限')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('calibration',type=Path)
    parser.add_argument('--resume-audit',type=Path)
    parser.add_argument('--previous-log',type=Path)
    parser.add_argument('geometry',nargs=6,type=float,help='cone x y radius; goal x y yaw')
    a=parser.parse_args();x,y,r,gx,gy,w=a.geometry
    if not 0<r<.5:raise ValueError('invalid cone radius')
    levels={k:rows[-1]['level'] for k,rows in json.loads(a.calibration.read_text()).items()}
    levels['forward']=.4
    rclpy.init();n=ConeTrial(np.array([x,y]),r,np.array([gx,gy,w]),levels)
    if a.resume_audit:
        initial=json.loads(a.resume_audit.read_text())
        assert initial['direct_audit']['first'] is not None and initial['cone_hits']>=3
        assert np.linalg.norm(np.array(initial['target'])-n.target)<.01
        records=[json.loads(line) for line in a.previous_log.read_text().splitlines() if line.startswith('{')]
        routes=[v['accepted_route'] for v in records if 'accepted_route' in v]
        if not routes:raise ValueError('no verified earlier route')
        n.selected=np.array(routes[-1]);n.resuming=True
    try:print(json.dumps(n.run()),flush=True)
    finally:
        try:n.brake()
        finally:n.clear_route();n.spin(.12);n.destroy_node();rclpy.try_shutdown()
