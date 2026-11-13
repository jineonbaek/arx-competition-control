"""Independent MID360S/IMU LIOs; does not start any controller."""
from pathlib import Path
import os
import json
import sys
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[2]
    run = Path(os.environ['ARX_DUAL_RUN'])
    names = tuple(os.environ.get('ARX_LIO_SENSORS', 'arm,top').split(','))
    if names not in (('top',), ('arm',), ('arm', 'top')):
        raise ValueError('ARX_LIO_SENSORS must be top, arm or arm,top')
    ips = {'arm': '192.168.1.3', 'top': '192.168.1.164'}
    driver = json.loads((root/'config/mid360s_dual_capture.json').read_text())
    # Keep known powered device addresses registered, even when only top LIO runs.
    # Otherwise unsolicited packets on shared SDK ports flood unknown-device logs.
    (run/'livox.json').write_text(json.dumps(driver, indent=2))
    nodes = [Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                  name='dual_mid360s', output='screen', parameters=[{
                      'xfer_format': 1, 'multi_topic': 1, 'data_src': 0,
                      'publish_freq': 10.0, 'output_data_type': 0,
                      'frame_id': 'livox_native',
                      'user_config_path': str(run/'livox.json')}],
                  remappings=[('/livox/lidar_192_168_1_3', '/dual/arm/lidar'),
                              ('/livox/imu_192_168_1_3', '/dual/arm/imu'),
                              ('/livox/lidar_192_168_1_164', '/dual/top/lidar'),
                              ('/livox/imu_192_168_1_164', '/dual/top/imu')])]
    for name in names:
        p = yaml.safe_load((root/'config/mapping_test/fastlio_arm.yaml').read_text())['/**']['ros__parameters']
        p['common'].update(lid_topic=f'/dual/{name}/lidar', imu_topic=f'/dual/{name}/imu', max_imu_gap_sec=1.0, align_gravity_world=True)
        p['frames'] = {'odom': f'{name}_lidar_odom', 'imu': f'{name}_lio_imu', 'lidar': f'{name}_livox_frame'}
        p['map_file_path'] = str(run/f'{name}.pcd')
        (run/f'{name}.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': p}}, sort_keys=False))
        lio = Node(package='fast_lio', executable='fastlio_mapping', name=f'{name}_lio',
                       namespace=f'dual/{name}', output='screen', parameters=[p],
                       remappings=[(src, f'/dual/{name}/{dst}') for src, dst in [
                           ('/cloud_registered', 'cloud_registered'), ('/cloud_registered_body', 'cloud_body'),
                           ('/cloud_effected', 'cloud_effected'), ('/Laser_map', 'map'),
                           ('/Odometry', 'odometry'), ('/path', 'path')]])
        nodes += [TimerAction(period=8.0, actions=[lio]), RegisterEventHandler(OnProcessExit(target_action=lio, on_exit=[
                       EmitEvent(event=Shutdown(reason=f'{name} LIO ended; reinitialize both sensors while stationary'))])),
                  Node(package='tf2_ros', executable='static_transform_publisher', name=f'{name}_internal_tf',
                       arguments=['--x', '-0.011', '--y', '-0.02329', '--z', '0.04412',
                                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                                  '--frame-id', f'{name}_lio_imu', '--child-frame-id', f'{name}_livox_frame']),
                  ExecuteProcess(cmd=[sys.executable, str(root/'scripts/smooth_lio_twist.py'), '--sensor', name], output='screen'),
                  ExecuteProcess(cmd=[sys.executable, str(root/'scripts/pointmap_grid.py'),
                                      '--cloud-topic', f'/dual/{name}/map',
                                      '--odom-topic', f'/dual/{name}/odometry',
                                      '--map-topic', '/mapping/grid' if name == 'arm' else '/mapping/top_grid',
                                      '--frame', f'{name}_lidar_odom',
                                      '--output', str(root/f'maps/dual_{name}_live_grid')], output='screen')]
    return LaunchDescription(nodes)
