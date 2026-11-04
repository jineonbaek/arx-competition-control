#!/usr/bin/env python3
"""Finite checks: no ROS nodes spun, no driver/SDK objects instantiated."""
import ast
import importlib.util
import json
import math
import subprocess
import tempfile
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml
from catkin_pkg.package import parse_package
from launch import LaunchContext
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs_py.point_cloud2 import read_points_numpy

ROOT = Path(__file__).resolve().parents[1]
package = ROOT / 'workspace/src/arx_nav_bringup'
checks = []


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


for p in (ROOT / 'workspace/src').rglob('package.xml'):
    assert parse_package(str(p)).get_build_type() == 'ament_cmake', str(p)
checks.append('all eight selected packages have valid ament manifests')
for p in (ROOT / 'overlay').rglob('*.py'):
    ast.parse(p.read_text())
checks.append('overlay Python syntax')

converter = load('arx_cloud_converter', package / 'scripts/custom_to_cloud.py')
msg = CustomMsg()
msg.header.frame_id = 'livox_frame'
msg.header.stamp.sec = 42
msg.header.stamp.nanosec = 1234
msg.points = [CustomPoint(x=1.0, y=2.0, z=-3.0, reflectivity=25, offset_time=100),
              CustomPoint(x=-4.0, y=5.0, z=6.0, reflectivity=90, offset_time=200)]
msg.point_num = 2
cloud = converter.to_cloud(msg)
assert cloud.header == msg.header
np.testing.assert_array_equal(read_points_numpy(cloud),
                              np.array([[1, 2, -3, 25], [-4, 5, 6, 90]], dtype=np.float32))
empty = CustomMsg()
assert converter.to_cloud(empty).width == 0
assert msg.points[1].offset_time == 200
checks.append('CustomMsg to PointCloud2 data, timestamps, empty cloud; original LIO timing preserved')

nav = yaml.safe_load((package / 'config/nav2_jazzy.yaml').read_text())
controller = nav['controller_server']['ros__parameters']
assert controller['FollowPath']['plugin'] == 'dwb_core::DWBLocalPlanner'
assert 'progress_checker_plugins' in controller and 'progress_checker_plugin' not in controller
assert 'recoveries_server' not in nav
assert 'base_link_fake' not in yaml.safe_dump(nav)
assert nav['behavior_server']['ros__parameters']['spin']['plugin'] == 'nav2_behaviors::Spin'
checks.append('Jazzy Nav2 parameter migration and removal of sentry fake frame')

launch = load('arx_launch', package / 'launch/bringup.launch.py')
launch.get_package_share_directory = lambda name: str(package)
with tempfile.TemporaryDirectory(prefix='arx-nav-check-') as tmp:
    robot = yaml.safe_load((package / 'config/robot.template.yaml').read_text())
    config = Path(tmp) / 'robot.yaml'
    config.write_text(yaml.safe_dump(robot))
    context = LaunchContext()
    context.launch_configurations.update(robot_config=str(config), lio='fastlio', mode='mapping',
        map='', use_sim_time='true', start_driver='false', start_navigation='false')
    try:
        launch.runtime(context)
    except RuntimeError as e:
        assert 'lidar_xyz_m' in str(e)
    else:
        raise AssertionError('Unmeasured mounting was accepted')
    checks.append('incomplete physical calibration rejected before node execution')
    # Synthetic geometry solely for launch-description checks, never a live profile.
    robot.update(lidar_xyz_m=[0.0, 0.0, 1.0], lidar_rpy_rad=[math.pi, 0.0, 0.0],
        base_link_height_above_ground_m=0.2, lidar_origin_height_above_ground_m=1.2,
        lidar_imu_extrinsic_T_m=[0.0, 0.0, 0.0],
        lidar_imu_extrinsic_R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        footprint=[[-0.5, -0.4], [0.5, -0.4], [0.5, 0.4], [-0.5, 0.4]])
    config.write_text(yaml.safe_dump(robot))
    original_temp = tempfile.NamedTemporaryFile
    with patch('tempfile.NamedTemporaryFile', side_effect=lambda *a, **kw:
               original_temp(*a, dir=tmp, **kw)):
        for lio in ('fastlio', 'pointlio'):
            for navigation in ('false', 'true'):
                context.launch_configurations.update(lio=lio, start_navigation=navigation)
                actions = launch.runtime(context)
                nodes = [a for a in actions if type(a).__name__ == 'Node']
                assert len(nodes) == (7 if navigation == 'false' else 14), len(nodes)
    checks.append('FAST-LIO and Point-LIO mapping launch descriptions, with/without Nav2, no processes executed')
    context.launch_configurations.update(start_driver='true')
    try:
        launch.runtime(context)
    except RuntimeError as e:
        assert 'Live driver' in str(e)
    else:
        raise AssertionError('Live hardware combined with bag clock')
    checks.append('live sensor and bag clock conflict rejected')

shown = subprocess.run(['ros2', 'launch', 'arx_nav_bringup', 'bringup.launch.py', '--show-args'],
                       text=True, capture_output=True, timeout=30)
assert shown.returncode == 0, shown.stdout + shown.stderr
checks.append('installed launch resolves and --show-args succeeds without starting nodes')
result = {'passed': True, 'count': len(checks), 'checks': checks,
          'hardware_access': False, 'sensor_runtime_validated': False,
          'navigation_runtime_validated': False}
(ROOT / 'reports/offline-check.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
