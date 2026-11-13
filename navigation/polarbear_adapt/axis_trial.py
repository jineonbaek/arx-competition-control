"""Guarded low-speed lateral/yaw pulse commissioning in both costmaps."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from follow_preview import Follow, angle
from chassis_command import WebChassis


def body_delta(start, end):
    c,s=math.cos(start[2]),math.sin(start[2])
    x,y=end[:2]-start[:2]
    return np.array([c*x+s*y,-s*x+c*y,angle(end[2]-start[2])])


class AxisTrial(Follow):
    def __init__(self):
        super().__init__(False)
        self.transport=WebChassis()
        self.active=False

    def step(self):
        pass

    def spin(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.015)

    def brake(self):
        if self.active:
            self.transport.send((0,0,0))
            self.active=False

    def guarded_pose(self,direction):
        if not self.ready():raise RuntimeError('stale sensor/grid')
        poses={}
        for frame in ('map','odom'):
            pose=self.pose(frame,self.now())
            c,s=math.cos(pose[2]),math.sin(pose[2])
            x,y,w=direction
            end=pose+[.09*(c*x-s*y),.09*(s*x+c*y),.20*w]
            if self.grids[frame][1].along([pose,end])['first']:
                raise RuntimeError(frame+' swept footprint blocked')
            poses[frame]=pose
        if not self.ready():raise RuntimeError('stale after grid check')
        return poses['odom']

    def pulse(self,direction,level,seconds=.65):
        direction=np.asarray(direction)
        start=self.guarded_pose(direction)
        t=time.monotonic()
        sent=[]
        try:
            while time.monotonic()-t<seconds:
                delta=body_delta(start,self.guarded_pose(direction))
                dt=time.monotonic()-t
                if dt>.24 and (np.linalg.norm(delta[:2])/dt>.11 or abs(delta[2])/dt>.38):
                    raise RuntimeError('measured speed exceeded test bound')
                progress=delta@direction
                if progress<-(.009 if not direction[2] else .025):
                    raise RuntimeError('opposite direction')
                if np.linalg.norm(delta[:2])>.025 or abs(delta[2])>.055:break
                self.active=True
                sent.append(time.monotonic())
                self.transport.send(level*direction)
                self.spin(.065)
        finally:self.brake()
        brake_pose=self.pose('odom',self.now())
        self.spin(1.35)
        end=self.pose('odom',self.now())
        self.spin(.4)
        stopped=self.pose('odom',self.now())
        drift=body_delta(end,stopped)
        if np.linalg.norm(drift[:2])>.010 or abs(drift[2])>.025:raise RuntimeError('not stationary')
        return dict(level=level,command_count=len(sent),command_span=sent[-1]-sent[0] if sent else 0.,delta=body_delta(start,stopped).tolist(),
                    after_brake=body_delta(brake_pose,stopped).tolist(),settled_drift=drift.tolist())

    def calibrate(self,output):
        records={}
        for name,vector in [('left',[0,1,0]),('right',[0,-1,0]),('ccw',[0,0,1]),('cw',[0,0,-1])]:
            rows=records[name]=[]
            threshold=.012 if vector[2]==0 else .025
            for level in [.12,.20,.30,.40,.50]:
                row=self.pulse(vector,level)
                rows.append(row)
                output.write_text(json.dumps(records,indent=2))
                print(name,row,flush=True)
                d=np.array(row['delta'])
                if vector[2] and np.linalg.norm(d[:2])>.035:raise RuntimeError('excess translation')
                if not vector[2] and (abs(d[0])>.03 or abs(d[2])>.09):raise RuntimeError('excess cross-axis motion')
                if d@vector>=threshold:break
            else:raise RuntimeError(name+' insufficient response')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('report',type=Path)
    args=parser.parse_args()
    rclpy.init();node=AxisTrial()
    try:
        deadline=time.monotonic()+12
        while not node.ready() and time.monotonic()<deadline:node.spin(.1)
        node.calibrate(args.report)
    finally:
        node.brake();node.destroy_node();rclpy.try_shutdown()
