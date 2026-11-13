#!/usr/bin/env python3
"""Read-only CAN feedback. No SDK objects, CAN transmission, or command topics."""
import argparse
from pathlib import Path
import socket
import struct
import time
import json
import math
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from odometry_math import decode_sdk_wheel_feedback, wheel_twist, kinematic_matrix, integrate_with_covariance, calibration_errors


class WheelRx(Node):
    def __init__(self, cfg, use_can=True):
        super().__init__('wheel_odometry_rx')
        self.cfg = cfg
        kinematic_matrix(cfg['wheels'])
        self.blocks = calibration_errors(cfg)
        self.pose = np.zeros(3)
        self.P = np.eye(3)*1e-6
        self.last_stamp = None
        self.last_rx = None
        self.last_valid_rx = None
        self.frames = 0
        self.rejected = 0
        self.fault_latched = False
        self.raw = self.create_publisher(String, '/wheel/feedback_raw', 20)
        self.odom = self.create_publisher(Odometry, '/wheel/odometry', 20)
        self.diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.sock = None
        if use_can:
            # Receive filter is local to this socket. Existing controller is untouched.
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                                 struct.pack('=II', cfg['feedback_can_id'], 0xC00007FF))
            self.sock.setsockopt(socket.SOL_SOCKET, getattr(socket, 'SO_TIMESTAMPNS', 35), 1)
            self.sock.bind((cfg['can_interface'],))
            self.sock.setblocking(False)
            self.timer = self.create_timer(.005, self.poll)
        self.dtimer = self.create_timer(1.0, self.diagnostics)

    def poll(self):
        for _ in range(100):
            try:
                frame, ancillary, flags, _ = self.sock.recvmsg(16, 128)
            except BlockingIOError:
                break
            # Reject echoes generated locally by other applications.
            if flags & socket.MSG_DONTROUTE:
                continue
            cid, dlc, data = struct.unpack('=IB3x8s', frame)
            if cid != self.cfg['feedback_can_id'] or dlc != 8:
                continue
            stamp_ns = None
            for level, kind, payload in ancillary:
                if level == socket.SOL_SOCKET and kind == getattr(socket, 'SO_TIMESTAMPNS', 35):
                    sec, ns = struct.unpack('@ll', payload[:16])
                    stamp_ns = sec*1_000_000_000+ns
            if stamp_ns is None:
                self.rejected += 1
                continue
            self.receive(data, stamp_ns)

    def receive(self, data, stamp_ns):
        raw, sdk = decode_sdk_wheel_feedback(data)
        self.frames += 1
        self.last_rx = time.monotonic()
        message = String()
        message.data = json.dumps({'stamp_ns': stamp_ns, 'raw_signed16': raw.astype(int).tolist(),
                                   'sdk_scaled_values': sdk.tolist(), 'physical_units_confirmed': not self.blocks})
        self.raw.publish(message)
        if self.blocks or self.fault_latched:
            return
        stamp = stamp_ns*1e-9
        age = time.time()-stamp
        if not -.05 <= age <= self.cfg['feedback_timeout_s']:
            self.rejected += 1
            return
        if self.last_stamp is not None and not 0 < stamp-self.last_stamp <= self.cfg['feedback_timeout_s']:
            self.fault_latched = True
            self.get_logger().error('Wheel feedback discontinuity; restart local odometry after diagnosis')
            return
        velocity, C = wheel_twist(sdk, self.cfg)
        if self.last_stamp is not None:
            self.pose, self.P = integrate_with_covariance(self.pose, self.P, velocity, C, stamp-self.last_stamp)
        self.last_stamp = stamp
        m = Odometry()
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        m.header.frame_id = self.cfg['odom_frame']
        m.child_frame_id = self.cfg['base_frame']
        m.pose.pose.position.x, m.pose.pose.position.y = map(float, self.pose[:2])
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(self.pose[2]/2), math.cos(self.pose[2]/2)
        m.twist.twist.linear.x, m.twist.twist.linear.y = map(float, velocity[:2])
        m.twist.twist.angular.z = float(velocity[2])
        pc, tc = np.eye(6)*1e6, np.eye(6)*1e6
        for i, a in enumerate([0, 1, 5]):
            for j, b in enumerate([0, 1, 5]):
                pc[a,b], tc[a,b] = self.P[i,j], C[i,j]
        m.pose.covariance, m.twist.covariance = pc.ravel().tolist(), tc.ravel().tolist()
        self.odom.publish(m)
        self.last_valid_rx = time.monotonic()

    def diagnostics(self):
        age = None if self.last_rx is None else time.monotonic()-self.last_rx
        valid_age = None if self.last_valid_rx is None else time.monotonic()-self.last_valid_rx
        healthy = not self.blocks and not self.fault_latched and valid_age is not None and valid_age < self.cfg['feedback_timeout_s']
        status = DiagnosticStatus()
        status.name = 'wheel_odometry/read_only_can'
        status.hardware_id = self.cfg['can_interface']
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = 'fresh validated wheel feedback' if healthy else 'WAITING_FOR_VALID_WHEEL_FEEDBACK'
        status.values = [KeyValue(key=k,value=str(v)) for k,v in {'can_id':hex(self.cfg['feedback_can_id']),
            'rx_frames':self.frames,'last_rx_age_s':age,'last_valid_odom_age_s':valid_age,'rejected':self.rejected,
            'calibration_blocks':self.blocks,'fault_latched':self.fault_latched,'transmitted_frames':0}.items()]
        m=DiagnosticArray();m.header.stamp=self.get_clock().now().to_msg();m.status=[status];self.diag.publish(m)


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);a=p.parse_args()
    rclpy.init();n=WheelRx(yaml.safe_load(a.config.read_text()))
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if n.sock:n.sock.close()
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
