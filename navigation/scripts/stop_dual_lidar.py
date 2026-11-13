#!/usr/bin/env python3
"""Save both maps then stop only the dual-lidar launch. No actuator commands."""
from pathlib import Path
import os
import signal
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

root = Path(__file__).resolve().parents[1]
pid = int((root/'runtime/dual-lidar.pid').read_text())
cmd = Path(f'/proc/{pid}/cmdline').read_bytes()
if b'config/dual_lidar/dual_lidar.launch.py' not in cmd:
    raise SystemExit('Unexpected PID; no signal sent')
rclpy.init()
node = Node('save_stop_dual_lidar')
try:
    for name in ('arm', 'top'):
        client = node.create_client(Trigger, f'/dual/{name}/map_save')
        if client.wait_for_service(timeout_sec=2):
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            print(name, future.result() if future.done() else 'save timeout')
finally:
    node.destroy_node()
    rclpy.shutdown()
os.kill(pid, signal.SIGINT)
