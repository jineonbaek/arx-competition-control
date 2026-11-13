"""Follow an audited short lateral or in-place yaw RViz goal using calibrated pulses."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Path as RosPath

from axis_trial import AxisTrial,body_delta
from follow_preview import angle
from path_audit import planar


class AxisPath(AxisTrial):
    def __init__(self,mode,goal,levels):
        super().__init__()
        self.mode=mode
        self.expected=np.array(goal)
        self.levels=levels
        self.route=None
        self.committed=False
        self.failure=None

    def changed(self,message):
        if self.committed:self.failure='目标已变更'

    def receive(self,m:RosPath):
        if m.header.frame_id!='map':return
        self.route=np.array([planar(p.pose) for p in m.poses])
        self.route_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.committed and not len(self.route):
            self.failure='规划器撤回路径'

    def guard_route(self,at_goal=False):
        if self.failure:raise RuntimeError(self.failure)
        if not self.ready():raise RuntimeError('传感器或代价图过期')
        p=self.pose('map',self.now())
        for f in ('map','odom'):
            current=self.pose(f,self.now())
            if self.grids[f][1].inspect(current)['first_cell']:raise RuntimeError('当前轮廓被挡')
        if self.grids['map'][1].inspect(self.expected)['first_cell']:raise RuntimeError('终点被挡')
        if at_goal:return p
        if self.route is None or not len(self.route) or not 0<=self.now()-self.route_stamp<3:
            if self.active:raise RuntimeError('运动中规划路径过期')
            deadline=time.monotonic()+5
            while time.monotonic()<deadline and not self.failure:
                self.spin(.08)
                if self.route is not None and len(self.route) and 0<=self.now()-self.route_stamp<3:
                    return self.guard_route()
            raise RuntimeError('缺少新鲜规划路径')
        if np.linalg.norm(self.route[-1,:2]-self.expected[:2])>.06 or abs(angle(self.route[-1,2]-self.expected[2]))>.13:
            raise RuntimeError('路径终点与箭头不一致')
        if self.grids['map'][1].along(self.route)['first']:raise RuntimeError('路径被挡')
        relative=np.array([body_delta(p,v) for v in self.route])
        if self.mode=='yaw' and max(np.linalg.norm(relative[:,:2],axis=1))>.055:
            raise RuntimeError('规划路径需要平移，超出原地旋转测试范围')
        if self.mode=='lateral' and max(abs(relative[:,0]))>.055:
            raise RuntimeError('规划路径超出侧移走廊')
        if not self.ready():raise RuntimeError('规划检查超时')
        return p

    def guarded_pose(self,direction):
        self.guard_route()
        return super().guarded_pose(direction)

    def run(self):
        self.spin(4)
        self.guard_route()
        self.committed=True
        stalls=0
        for _ in range(20):
            p=self.guard_route(at_goal=True)
            e=body_delta(p,self.expected)
            if np.linalg.norm(e[:2])<=.045 and abs(e[2])<=.09:
                self.spin(.45)
                settled=[]
                end=time.monotonic()+1.
                while time.monotonic()<end:
                    p=self.guard_route(at_goal=True)
                    e=body_delta(p,self.expected)
                    if np.linalg.norm(e[:2])>.045 or abs(e[2])>.09:raise RuntimeError('停车后超出目标容差')
                    settled.append(self.pose('odom',self.now()))
                    self.spin(.07)
                span=np.ptp(settled,axis=0)
                if np.linalg.norm(span[:2])>.009 or span[2]>.02:raise RuntimeError('停车后仍有位移')
                return dict(status='arrived_stopped',map_pose=p.tolist(),goal=self.expected.tolist(),
                            error=e.tolist(),settled_axis_span=span.tolist())
            self.guard_route()
            if self.mode=='yaw':
                if np.linalg.norm(e[:2])>.045 or abs(e[2])>.55:raise RuntimeError('超出原地旋转范围')
                axis=2;name='ccw' if e[2]>0 else 'cw'
            else:
                if abs(e[0])>.033 or abs(e[2])>.09 or abs(e[1])>.38:raise RuntimeError('超出短程侧移范围')
                axis=1;name='left' if e[1]>0 else 'right'
            direction=np.zeros(3);direction[axis]=np.sign(e[axis])
            data=self.pulse(direction,self.levels[name])
            progress=np.dot(data['delta'],direction)
            print(json.dumps(dict(axis=name,progress=progress,remaining=e.tolist())),flush=True)
            stalls=stalls+1 if progress<(.006 if axis==1 else .012) else 0
            if stalls>=2:raise RuntimeError('连续两次未取得进展')
        raise RuntimeError('动作次数上限')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=['lateral','yaw'])
    parser.add_argument('calibration',type=Path)
    parser.add_argument('goal',nargs=3,type=float)
    args=parser.parse_args()
    results=json.loads(args.calibration.read_text())
    levels={}
    for name,axis,sign in [('left',1,1),('right',1,-1),('ccw',2,1),('cw',2,-1)]:
        row=results[name][-1]
        if row['delta'][axis]*sign<(.012 if axis==1 else .025):raise ValueError('方向标定尚未通过 '+name)
        levels[name]=row['level']
    rclpy.init();n=AxisPath(args.mode,args.goal,levels)
    try:print(json.dumps(n.run()),flush=True)
    finally:n.brake();n.destroy_node();rclpy.try_shutdown()
