"""Validated local odometry only; never starts motor SDK or Nav2 controllers."""
from pathlib import Path
import json
import sys
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parents[2]
    cfg=root/'config/wheel_imu_fusion/wheels.yaml'
    data=yaml.safe_load(cfg.read_text())
    blocked=[k for k in ['geometry_confirmed','feedback_order_signs_confirmed','feedback_units_confirmed'] if data.get(k) is not True]
    if blocked:raise RuntimeError('Wheel calibration not confirmed: '+', '.join(blocked)+'. Run read-only wheel telemetry first.')
    cal=Path((root/'config/wheel_imu_fusion/latest-imu-calibration.txt').read_text().strip())
    params=json.loads(cal.read_text())
    if params.get('boot_id')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():raise RuntimeError('New boot: stationary IMU calibration required')
    return LaunchDescription([
        ExecuteProcess(cmd=[sys.executable,str(root/'scripts/wheel_odometry_rx.py'),'--config',str(cfg)],output='screen'),
        *[ExecuteProcess(cmd=[sys.executable,str(root/'scripts/imu_yaw_adapter.py'),'--sensor',name,'--calibration',str(cal)],output='screen') for name in ['arm','top']],
        Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',parameters=[str(root/'config/wheel_imu_fusion/ekf.yaml')],remappings=[('odometry/filtered','/fusion/odometry_internal')],output='screen'),
        ExecuteProcess(cmd=[sys.executable,str(root/'scripts/fused_odom_gate.py')],output='screen')])
