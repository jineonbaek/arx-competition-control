"""Commissioning mode for a short straight path at the measured web input."""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from follow_preview import Follow, angle
from chassis_command import WebChassis


def near_goal(pose, goal):
    return np.linalg.norm(pose[:2]-goal[:2])<=.045 and abs(angle(pose[2]-goal[2]))<=.09


class ForwardTrial(Follow):
    def __init__(self, goal):
        self.start = None
        self.web_started = False
        self.reason = None
        self.settling = False
        super().__init__(False)
        self.expected = np.array(goal)
        self.transport = WebChassis()
        self.input_pub = self.create_publisher(Float64,'/pb/forward_web_input',1)

    def step(self):
        # The outer loop advances only after checking fresh feedback.
        pass

    def stop(self, why):
        if not self.done:
            self.reason = why
            if self.web_started:
                self.web_started = False
                self.transport.send((0,0,0))
            super().stop(why)

    def terminal(self):
        if self.path is None or self.done:return False
        if not self.ready():
            self.stop('传感器或代价图过期');return False
        for f in ('map','odom'):
            pose=self.pose(f,self.now())
            if f=='map' and not near_goal(pose,self.expected):return False
            if self.grids[f][1].inspect(pose)['first_cell']:
                self.stop('停车轮廓被挡');return False
        if self.grids['map'][1].inspect(self.expected)['first_cell']:
            self.stop('目标轮廓被挡');return False
        if not self.settling:
            if self.web_started:
                self.transport.send((0,0,0));self.web_started=False
            self.input_pub.publish(Float64(data=0.))
            self.settling=True
        return True

    def receive(self,m):
        # Once inside tolerance, stop first; a new lattice detour is irrelevant
        # while verifying standstill. New operator goals still cancel the trial.
        if self.terminal() or self.done:return
        super().receive(m)

    def verify_arrival(self):
        if not self.terminal():
            if self.settling:self.stop('停车后超出容差')
            return self.done
        self.spin(.45)
        samples=[]
        until=time.monotonic()+1.0
        while time.monotonic()<until and not self.done:
            if not self.terminal():
                self.stop('停车核验未通过');return True
            samples.append(self.pose('odom',self.now()))
            self.spin(.07)
        if self.done:return True
        drift=np.ptp(np.asarray(samples),axis=0)
        if np.linalg.norm(drift[:2])>.009 or drift[2]>.02:
            self.stop('尚未停稳');return True
        print(json.dumps(dict(goal=self.expected.tolist(),final=self.pose('map',self.now()).tolist(),
                              stationary_span=drift.tolist())),flush=True)
        self.stop('到达前向目标，已停稳');return True

    def spin(self, seconds):
        until = time.monotonic()+seconds
        while not self.done and time.monotonic()<until:
            rclpy.spin_once(self,timeout_sec=.015)

    def check(self):
        if not self.ready() or self.path is None:
            raise ValueError('路径或传感器未就绪')
        p = {f:self.pose(f,self.now()) for f in ('map','odom')}
        xy = p['map'][:2]
        if self.start is None:
            self.start = p['odom'].copy()
        yaw = p['map'][2]
        c,s = math.cos(yaw),math.sin(yaw)
        delta = self.expected[:2]-xy
        forward,left = np.array([[c,s],[-s,c]])@delta
        if abs(angle(yaw-self.expected[2]))>.10 or abs(left)>.032 or forward>.40 or forward<-.025:
            raise ValueError('前向短程测试超出方向或位置范围')
        route=self.path[:,:2]
        lateral=(route-xy)@np.array([-s,c])
        if max(abs(lateral))>.05 or np.linalg.norm(route[-1]-self.expected[:2])>.045:
            raise ValueError('规划路线偏离直行走廊')
        if self.grids['map'][1].along(self.path)['first']:
            raise ValueError('全局路径被挡')
        for frame,pose in p.items():
            distance=max(0,min(.10,forward+.025))
            end=pose+[distance*math.cos(pose[2]),distance*math.sin(pose[2]),0]
            if self.grids[frame][1].along([pose,end])['first']:
                raise ValueError('实时行进段被挡')
        if not self.ready():raise ValueError('检查期间传感器超时')
        return forward,p['odom']

    def run(self):
        self.spin(2)
        sender=self.create_publisher(PoseStamped,'/pb/plan_goal',1)
        self.spin(.5)
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.pose.position.x,m.pose.position.y=map(float,self.expected[:2])
        m.pose.orientation.z=math.sin(self.expected[2]/2);m.pose.orientation.w=math.cos(self.expected[2]/2)
        sender.publish(m)
        deadline=time.monotonic()+6
        while self.path is None and not self.done and time.monotonic()<deadline:self.spin(.05)
        stalls=0
        for _ in range(24):
            if self.done or self.verify_arrival():return
            remaining,initial=self.check()
            start=time.monotonic()
            while time.monotonic()-start<.80 and not self.done:
                if self.terminal() or self.done:break
                dist,pose=self.check()
                elapsed=time.monotonic()-start
                moved=np.linalg.norm(pose[:2]-initial[:2])
                if elapsed>.25 and moved/elapsed>.08:raise ValueError('实测平均速度超过 0.08 m/s')
                if moved>.030:break
                self.web_started=True
                self.transport.send((.4,0,0))
                self.input_pub.publish(Float64(data=.4))
                self.spin(.065)
            if self.done:return
            self.transport.send((0,0,0));self.web_started=False;self.input_pub.publish(Float64(data=0.))
            self.spin(1.4)
            if self.done:return
            if self.verify_arrival():return
            dist,_=self.check()
            print(json.dumps(dict(before=remaining,after=dist)),flush=True)
            stalls=stalls+1 if remaining-dist<.004 else 0
            if stalls>=2:raise ValueError('连续两次无进展')
        raise ValueError('脉冲次数上限')


if __name__=='__main__':
    rclpy.init();node=ForwardTrial([float(x) for x in sys.argv[1:4]])
    try:node.run()
    except (KeyboardInterrupt, Exception) as exc:node.stop(str(exc));raise
    finally:node.stop('试验结束');node.destroy_node();rclpy.try_shutdown()
