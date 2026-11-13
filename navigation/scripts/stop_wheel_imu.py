#!/usr/bin/env python3
from pathlib import Path
import os
import signal
root=Path(__file__).resolve().parents[1]
pid=int((root/'runtime/wheel-imu-inputs.pid').read_text())
cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
if b'/config/wheel_imu_fusion/wheel_imu' not in cmd or b'.launch.py' not in cmd:
    raise SystemExit('Unexpected PID; no signal sent')
os.kill(pid,signal.SIGINT)
print('Stopped only the navigation wheel/IMU processing launch',pid)
