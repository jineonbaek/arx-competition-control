#!/usr/bin/env python3
"""Gamepad mapper and opt-in ROS transport; default is command preview only."""
import argparse
import json
import math
import time
from urllib.request import urlopen

DEADZONE = .08


def deadzone(value):
    if not math.isfinite(value):
        raise ValueError('non-finite input')
    value = max(-1., min(1., value))
    return 0. if abs(value) <= DEADZONE else math.copysign((abs(value)-DEADZONE)/(1-DEADZONE), value)


def command_from_axes(axes):
    forward, left, yaw = -deadzone(axes[3]), -deadzone(axes[2]), -deadzone(axes[0])
    radius = math.hypot(forward, left)
    if radius > 1:
        forward, left = forward/radius, left/radius
    return 2*forward, 2*left, 2*yaw


class EnableGate:
    def __init__(self):
        self.released = False
        self.active = False
        self.previous = False

    def update(self, connected, held, neutral, healthy):
        if not connected or not healthy:
            self.released = False
            self.active = False
            self.previous = held
            return False
        if not held:
            self.active = False
            self.released = bool(neutral)
        elif not self.previous:
            self.active = self.released and neutral
            self.released = False
        self.previous = held
        return self.active


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true', help='Publish to dedicated gamepad driver topic')
    args=parser.parse_args()
    import rclpy
    from arm_control.msg import PosCmd
    from std_msgs.msg import Bool, String
    rclpy.init()
    node=rclpy.create_node('arx_gamepad_chassis')
    publisher=node.create_publisher(PosCmd,'/web/gamepad/command',1) if args.execute else None
    health={'ready':False,'ready_at':0.,'ack_at':0.,'rejected':False}
    def ready(msg):
        health['ready']=msg.data;health['ready_at']=time.monotonic()
    def status(msg):
        if msg.data=='accepted:gamepad':
            health['ack_at']=time.monotonic()
        elif msg.data.startswith('rejected:') or msg.data=='stop:web':
            health['rejected']=True
    node.create_subscription(Bool,'/web/body/calibrated',ready,1)
    node.create_subscription(String,'/web/gamepad/status',status,10)
    gate=EnableGate();was_active=False;active_since=0.
    report='/tmp/arx-gamepad-teleop-state.json'
    def send(command):
        if publisher is not None:
            msg=PosCmd();msg.chx,msg.chy,msg.chz=map(float,command)
            msg.mode1=1 if any(command) else 2
            publisher.publish(msg)
    try:
        while rclpy.ok():
            start=time.monotonic()
            rclpy.spin_once(node,timeout_sec=0.)
            try:
                with urlopen('http://127.0.0.1:9984/api/state',timeout=.10) as response:
                    state=json.load(response)
                connected=state['connected'] and state.get('initialized',False)
                axes=state['normalized'];held=bool(state['buttons'][7])
                command=command_from_axes(axes)
                neutral=not any(command)
            except (OSError,ValueError,KeyError,IndexError,TypeError):
                connected=False;held=False;neutral=False;command=(0.,0.,0.)
            healthy=health['ready'] and start-health['ready_at']<.25 and not health['rejected']
            if args.execute and was_active and start-active_since>.25 and start-health['ack_at']>.25:
                healthy=False
            enabled=gate.update(connected,held,neutral,healthy)
            if enabled and not was_active:
                active_since=start;health['ack_at']=0.
            if enabled:
                send(command)
            elif was_active:
                send((0.,0.,0.))
            if not held:
                health['rejected']=False
            reason='按住 LT 控制' if enabled else ('松开 LT 并回中，再按住 LT' if healthy else '底盘未就绪或控制权被占用')
            record={'execute':args.execute,'enabled':enabled,'sdk_target':list(command) if connected else [0,0,0],
                    'sdk_command':list(command) if enabled and args.execute else [0,0,0],
                    'reason':reason,'at':time.time()}
            temp=report+'.tmp'
            with open(temp,'w') as output:json.dump(record,output,ensure_ascii=False)
            import os
            os.replace(temp,report)
            was_active=enabled
            time.sleep(max(0,.05-(time.monotonic()-start)))
    finally:
        if was_active:
            send((0.,0.,0.));rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
