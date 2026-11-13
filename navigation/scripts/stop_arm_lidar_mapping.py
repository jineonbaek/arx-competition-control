#!/usr/bin/env python3
"""Save the map then stop ONLY this mapping launch; no chassis commands."""
from pathlib import Path
import os,signal,time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
root=Path(__file__).resolve().parents[1];pid=int((root/'runtime/arm-mapping.pid').read_text());cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
if b'config/mapping_test/arm_lidar.launch.py' not in cmd:raise SystemExit('PID is not the expected mapping launch; no signal sent')
rclpy.init();node=Node('save_and_stop_arm_mapping');client=node.create_client(Trigger,'/mapping_test/map_save')
try:
 if client.wait_for_service(timeout_sec=3):
  future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=5)
  print('Map save:',future.result() if future.done() else 'timed out')
finally:
 node.destroy_node();rclpy.shutdown()
os.kill(pid,signal.SIGINT);print('Requested mapping-only launch shutdown, PID',pid)
