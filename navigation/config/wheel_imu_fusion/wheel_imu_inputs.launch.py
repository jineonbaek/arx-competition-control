"""Read-only wheel telemetry and calibrated virtual IMUs; no final EKF output."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    root=Path(__file__).resolve().parents[2]
    cal=Path((root/'config/wheel_imu_fusion/latest-imu-calibration.txt').read_text().strip())
    procs=[ExecuteProcess(cmd=[sys.executable,str(root/'scripts/wheel_odometry_rx.py'),'--config',str(root/'config/wheel_imu_fusion/wheels.yaml')],output='screen')]
    procs += [ExecuteProcess(cmd=[sys.executable,str(root/'scripts/imu_yaw_adapter.py'),'--sensor',name,'--calibration',str(cal)],output='screen') for name in ['arm','top']]
    handlers=[RegisterEventHandler(OnProcessExit(target_action=p,on_exit=[EmitEvent(event=Shutdown(reason='Read-only odometry input process exited'))])) for p in procs]
    return LaunchDescription(procs+handlers)
