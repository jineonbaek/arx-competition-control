#!/usr/bin/env python3
"""Stage isolated, editable copies. Never modify the upstream checkouts."""
import argparse
import difflib
import json
import re
import shutil
import subprocess
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--root', type=Path, default=ROOT)
args = parser.parse_args()
root = args.root.resolve()
up = root / 'upstream/rm_navigation_ws'
src = root / 'workspace/src'
src.mkdir(parents=True, exist_ok=True)
packages = [
    'rm_localization/FAST_LIO', 'rm_localization/point_lio',
    'rm_perception/imu_complementary_filter',
    'rm_perception/pointcloud_to_laserscan',
    'rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation',
    'rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros',
]
origins = {}
for rel in packages:
    source = up / 'src' / rel
    if not (source / 'package.xml').exists():
        raise SystemExit(f'Missing source/submodule: {source}')
    target = src / source.name
    if target.exists():
        raise SystemExit(f'Refusing to overwrite existing staged package: {target}')
    shutil.copytree(source, target, ignore=shutil.ignore_patterns('.git', 'Log', 'PCD'))
    origins[target] = source

driver = src / 'livox_ros_driver2'
shutil.copytree(root / 'deps/livox_ros_driver2', driver,
                ignore=shutil.ignore_patterns('.git'))
origins[driver] = root / 'deps/livox_ros_driver2'
shutil.copyfile(driver / 'package_ROS2.xml', driver / 'package.xml')
shutil.copytree(driver / 'launch_ROS2', driver / 'launch')

# Jazzy rclcpp/PCL require C++17. Remove every old conflicting -std flag.
for name in ('FAST_LIO', 'point_lio'):
    cmake = src / name / 'CMakeLists.txt'
    text = cmake.read_text().replace('c++14', 'c++17').replace('c++0x', 'c++17')
    text = text.replace('CMAKE_CXX_STANDARD 14', 'CMAKE_CXX_STANDARD 17')
    if name == 'point_lio':
        text = text.replace('find_package(Eigen3 REQUIRED)',
            'find_package(Eigen3 REQUIRED)\nfind_package(PCL REQUIRED COMPONENTS common io)')
    cmake.write_text(text)

# These parameters were read without declaration and silently ignored in the port.
ground_cpp = src / 'linefit_ground_segmentation_ros/src/ground_segmentation_node.cc'
text = ground_cpp.read_text().replace(
    'double r_min, r_max, max_fit_error;',
    'double r_min, r_max, max_fit_error;\n'
    '  this->declare_parameter("r_min", std::sqrt(params_.r_min_square));\n'
    '  this->declare_parameter("r_max", std::sqrt(params_.r_max_square));\n'
    '  this->declare_parameter("max_fit_error", std::sqrt(params_.max_error_square));')
text = '#include <cmath>\n' + text
# An unavailable gravity transform must not silently classify an inverted cloud.
text = text.replace('                  ex.what());\n    }',
                    '                  ex.what());\n      return;\n    }')
ground_cpp.write_text(text)

bringup = src / 'arx_nav_bringup'
shutil.copytree(root / 'overlay/arx_nav_bringup', bringup)
cfg = bringup / 'config'
cfg.mkdir(exist_ok=True)
for name in ('fastlio_mid360_real.yaml', 'pointlio_mid360_real.yaml',
             'mapper_params_online_async_real.yaml', 'mapper_params_localization_real.yaml',
             'segmentation_real.yaml'):
    shutil.copyfile(up / 'src/rm_nav_bringup/config/reality' / name, cfg / name)

# Reuse the PB mapping/AMCL parameters; replace legacy Nav2 plugin configuration.
nav = yaml.safe_load((up / 'src/rm_nav_bringup/config/reality/nav2_params_real.yaml').read_text())
nav['bt_navigator']['ros__parameters'].pop('plugin_lib_names', None)
controller = nav['controller_server']['ros__parameters']
controller.pop('progress_checker_plugin', None)
controller['progress_checker_plugins'] = ['progress_checker']
controller['enable_stamped_cmd_vel'] = False
controller['odom_topic'] = '/Odometry'
controller['min_y_velocity_threshold'] = 0.001
controller['general_goal_checker']['yaw_goal_tolerance'] = 0.15
controller['general_goal_checker']['xy_goal_tolerance'] = 0.15
controller['FollowPath'] = {
    'plugin': 'dwb_core::DWBLocalPlanner', 'debug_trajectory_details': False,
    'min_vel_x': -0.2, 'min_vel_y': 0.0, 'max_vel_x': 0.2, 'max_vel_y': 0.0,
    'max_vel_theta': 0.3, 'min_speed_xy': 0.0, 'max_speed_xy': 0.2,
    'min_speed_theta': 0.0, 'acc_lim_x': 0.2, 'acc_lim_y': 0.0,
    'acc_lim_theta': 0.3, 'decel_lim_x': -0.2, 'decel_lim_y': 0.0,
    'decel_lim_theta': -0.3, 'vx_samples': 15, 'vy_samples': 1,
    'vtheta_samples': 20, 'sim_time': 1.5, 'linear_granularity': 0.05,
    'angular_granularity': 0.025, 'transform_tolerance': 0.3,
    'xy_goal_tolerance': 0.15, 'trans_stopped_velocity': 0.03,
    'short_circuit_trajectory_evaluation': True, 'stateful': True,
    'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
                'PathAlign', 'PathDist', 'GoalDist'],
    'BaseObstacle.scale': 0.02, 'PathAlign.scale': 32.0,
    'PathAlign.forward_point_distance': 0.1, 'GoalAlign.scale': 24.0,
    'GoalAlign.forward_point_distance': 0.1, 'PathDist.scale': 32.0,
    'GoalDist.scale': 24.0, 'RotateToGoal.scale': 32.0,
    'RotateToGoal.slowing_factor': 5.0, 'RotateToGoal.lookahead_time': -1.0,
}
nav['planner_server']['ros__parameters']['GridBased']['plugin'] = 'nav2_navfn_planner::NavfnPlanner'
nav.pop('recoveries_server', None)
nav['behavior_server'] = {'ros__parameters': {
    'local_costmap_topic': 'local_costmap/costmap_raw',
    'global_costmap_topic': 'global_costmap/costmap_raw',
    'local_footprint_topic': 'local_costmap/published_footprint',
    'global_footprint_topic': 'global_costmap/published_footprint',
    'cycle_frequency': 10.0, 'behavior_plugins': ['spin', 'backup', 'wait'],
    'spin': {'plugin': 'nav2_behaviors::Spin'},
    'backup': {'plugin': 'nav2_behaviors::BackUp'},
    'wait': {'plugin': 'nav2_behaviors::Wait'},
    'local_frame': 'odom', 'global_frame': 'map', 'robot_base_frame': 'base_link',
    'transform_tolerance': 0.3, 'simulate_ahead_time': 1.0,
    'max_rotational_vel': 0.3, 'min_rotational_vel': 0.1,
    'rotational_acc_lim': 0.3, 'enable_stamped_cmd_vel': False,
}}
nav['smoother_server'] = {'ros__parameters': {
    'smoother_plugins': ['simple_smoother'],
    'simple_smoother': {'plugin': 'nav2_smoother::SimpleSmoother',
                         'tolerance': 1.0e-10, 'max_its': 1000, 'do_refinement': True}}}
smooth = nav['velocity_smoother']['ros__parameters']
smooth.update(max_velocity=[0.2, 0.0, 0.3], min_velocity=[-0.2, 0.0, -0.3],
              max_accel=[0.2, 0.0, 0.3], max_decel=[-0.2, 0.0, -0.3],
              odom_topic='/Odometry', enable_stamped_cmd_vel=False)
for scope in ('local_costmap', 'global_costmap'):
    cost = nav[scope][scope]['ros__parameters']
    # Filled from measured chassis geometry by the launch file. Never run the
    # original sentry's 0.2 m footprint against an ARX chassis.
    cost.pop('robot_radius', None)
    cost['footprint'] = '[]'
    cost['global_frame'] = 'odom' if scope == 'local_costmap' else 'map'
    cost['plugins'] = (['obstacle_layer', 'inflation_layer'] if scope == 'local_costmap'
                       else ['static_layer', 'obstacle_layer', 'inflation_layer'])
    cost.pop('stvl_layer', None)
    cost.pop('voxel2d_layer', None)
    cost['obstacle_layer'] = {
        'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
        'observation_sources': 'scan',
        'scan': {'topic': '/scan', 'data_type': 'LaserScan', 'marking': True,
                 'clearing': True, 'obstacle_max_range': 8.0,
                 'raytrace_max_range': 10.0, 'inf_is_valid': True}}
    cost['inflation_layer'] = {'plugin': 'nav2_costmap_2d::InflationLayer',
                               'cost_scaling_factor': 3.0, 'inflation_radius': 0.7}

def replace_frames(value):
    if isinstance(value, dict):
        return {k: replace_frames(v) for k, v in value.items()}
    if isinstance(value, list):
        return [replace_frames(v) for v in value]
    return 'base_link' if value == 'base_link_fake' else value

nav = replace_frames(nav)
(cfg / 'nav2_jazzy.yaml').write_text(yaml.safe_dump(nav, sort_keys=False))
manifest = {}
for key, path in {'upstream': root / 'upstream', 'navigation': up,
                  'fast_lio': up / 'src/rm_localization/FAST_LIO',
                  'point_lio': up / 'src/rm_localization/point_lio',
                  'livox_ros_driver2': root / 'deps/livox_ros_driver2',
                  'livox_sdk2': root / 'deps/Livox-SDK2'}.items():
    manifest[key] = {'commit': subprocess.check_output(
        ['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()}
(root / 'reports/source-lock.json').write_text(json.dumps(manifest, indent=2) + '\n')
patches = []
for target, source in origins.items():
    for f in sorted(target.rglob('*')):
        if f.is_file() and f.suffix in ('.txt', '.cpp', '.cc', '.h', '.hpp', '.xml'):
            old = source / f.relative_to(target)
            if old.is_file():
                a, b = old.read_text(errors='replace'), f.read_text(errors='replace')
                if a != b:
                    patches.extend(difflib.unified_diff(a.splitlines(True), b.splitlines(True),
                        fromfile=str(old.relative_to(root)), tofile=str(f.relative_to(root))))
(root / 'reports/jazzy-source.patch').write_text(''.join(patches))
print(f'Staged {len(packages) + 2} packages in {src}')
