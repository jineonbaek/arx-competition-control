"""Single LiDAR mapping pipeline; chassis command connection is external."""
import json
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def runtime(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    flag = lambda key: get(key).lower() == 'true'
    use_sim_time = flag('use_sim_time')
    share = Path(get_package_share_directory('arx_nav_bringup'))
    robot_path = Path(get('robot_config')).expanduser()
    if not robot_path.is_file():
        raise RuntimeError('Provide robot_config:=<measured robot.yaml>; see config/robot.template.yaml')
    robot = yaml.safe_load(robot_path.read_text())

    def vector(key, count):
        value = robot.get(key)
        if not isinstance(value, list) or len(value) != count or not all(
                isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x)
                for x in value):
            raise RuntimeError(f'{key} must contain {count} measured finite numbers')
        return [float(x) for x in value]

    xyz, rpy = vector('lidar_xyz_m', 3), vector('lidar_rpy_rad', 3)
    height = robot.get('lidar_origin_height_above_ground_m')
    base_height = robot.get('base_link_height_above_ground_m')
    if not isinstance(height, (int, float)) or not math.isfinite(height) or height <= 0:
        raise RuntimeError('Measure lidar_origin_height_above_ground_m; bottom-face height is not equivalent')
    if not isinstance(base_height, (int, float)) or not math.isfinite(base_height) or base_height < 0:
        raise RuntimeError('Measure base_link_height_above_ground_m')
    lio = get('lio')
    if lio not in ('fastlio', 'pointlio'):
        raise RuntimeError('lio must be fastlio or pointlio')
    mode = get('mode')
    if mode not in ('mapping', 'localization'):
        raise RuntimeError('mode must be mapping or localization')
    # Use native driver coordinates. Rotating only point clouds inside the SDK
    # while leaving IMU unchanged would invalidate the LIO measurement model.
    driver_path = robot.get('livox_config_file')
    if flag('start_driver'):
        if use_sim_time:
            raise RuntimeError('Live driver and use_sim_time=true must not be combined')
        if not driver_path or not Path(driver_path).is_file():
            raise RuntimeError('Set livox_config_file to an official config with actual sensor/host IPs')
        config = json.loads(Path(driver_path).read_text())
        for item in config.get('lidar_configs', []):
            extrinsic = item.get('extrinsic_parameter', {})
            if any(float(extrinsic.get(k, 0)) != 0.0 for k in ('roll', 'pitch', 'yaw', 'x', 'y', 'z')):
                raise RuntimeError('Keep driver extrinsic identity; configure mounting in lidar_xyz_m/lidar_rpy_rad')

    time = {'use_sim_time': use_sim_time}
    nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_lidar', arguments=[
                 '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                 '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                 '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'], parameters=[time]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='odom_to_lidar_odom',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'lidar_odom'], parameters=[time]),
        Node(package='arx_nav_bringup', executable='custom_to_cloud.py', parameters=[time]),
    ]
    if flag('start_driver'):
        nodes.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
            parameters=[{'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                         'publish_freq': 10.0, 'output_data_type': 0,
                         'frame_id': 'livox_frame', 'user_config_path': str(driver_path)}]))
    params_file = share / 'config' / f'{lio}_mid360_real.yaml'
    params = yaml.safe_load(params_file.read_text())['/**']['ros__parameters']
    params['mapping']['extrinsic_T'] = vector('lidar_imu_extrinsic_T_m', 3)
    params['mapping']['extrinsic_R'] = vector('lidar_imu_extrinsic_R', 9)
    if lio == 'pointlio':
        params.update(use_imu_as_input=False, prop_at_freq_of_imu=True, check_satu=True,
                      init_map_size=10, point_filter_num=3, space_down_sample=True,
                      filter_size_surf=0.3, filter_size_map=0.3, ivox_nearby_type=6,
                      runtime_pos_log_enable=False)
    params.update(time)
    nodes.append(Node(package='fast_lio' if lio == 'fastlio' else 'point_lio',
        executable='fastlio_mapping' if lio == 'fastlio' else 'pointlio_mapping',
        name='laserMapping', parameters=[params], output='screen'))
    seg = yaml.safe_load((share / 'config/segmentation_real.yaml').read_text())['ground_segmentation']['ros__parameters']
    seg.update(time)
    seg.update(sensor_height=float(height), gravity_aligned_frame='odom')
    nodes.append(Node(package='linefit_ground_segmentation_ros', executable='ground_segmentation_node',
                      parameters=[seg], output='screen'))
    nodes.append(Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan', parameters=[time, {
            'target_frame': 'base_link', 'transform_tolerance': 0.2,
            'min_height': 0.08 - base_height, 'max_height': 2.0 - base_height,
            'angle_min': -math.pi, 'angle_max': math.pi, 'angle_increment': 0.0043,
            'scan_time': 0.1, 'range_min': 0.5, 'range_max': 10.0, 'use_inf': True}],
        remappings=[('cloud_in', '/segmentation/obstacle'), ('scan', '/scan')]))
    slam_file = ('mapper_params_online_async_real.yaml' if mode == 'mapping'
                 else 'mapper_params_localization_real.yaml')
    slam = yaml.safe_load((share / 'config' / slam_file).read_text())['slam_toolbox']['ros__parameters']
    slam.update(time)
    if mode == 'localization':
        map_path = Path(get('map')).expanduser()
        if not Path(str(map_path) + '.posegraph').is_file():
            raise RuntimeError('localization map:=<absolute path without .posegraph extension>')
        slam.update(map_file_name=str(map_path), map_start_pose=[0.0, 0.0, 0.0])
    nodes.append(Node(package='slam_toolbox', executable=(
        'async_slam_toolbox_node' if mode == 'mapping' else 'localization_slam_toolbox_node'),
        name='slam_toolbox', parameters=[slam], output='screen'))

    if flag('start_navigation'):
        footprint = robot.get('footprint')
        if not isinstance(footprint, list) or len(footprint) < 3 or any(
                not isinstance(p, list) or len(p) != 2 or not all(
                    isinstance(v, (int, float)) and math.isfinite(v) for v in p) for p in footprint):
            raise RuntimeError('Set footprint to a measured base_link polygon before navigation')
        nav = yaml.safe_load((share / 'config/nav2_jazzy.yaml').read_text())
        for cost in ('local_costmap', 'global_costmap'):
            nav[cost][cost]['ros__parameters']['footprint'] = json.dumps(footprint)
        # Inline parameter dictionaries preserve nested costmap nodes when dumped.
        import tempfile
        temp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='arx-nav-', delete=False)
        yaml.safe_dump(nav, temp, sort_keys=False)
        temp.close()
        from launch.actions import RegisterEventHandler
        from launch.event_handlers import OnShutdown
        nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _context: Path(temp.name).unlink(missing_ok=True))])))
        names = []
        for package, name in [('nav2_controller', 'controller_server'),
                              ('nav2_planner', 'planner_server'),
                              ('nav2_smoother', 'smoother_server'),
                              ('nav2_behaviors', 'behavior_server'),
                              ('nav2_bt_navigator', 'bt_navigator'),
                              ('nav2_velocity_smoother', 'velocity_smoother')]:
            names.append(name)
            executable = 'velocity_smoother' if name == 'velocity_smoother' else name
            remaps = [('cmd_vel', '/navigation/cmd_vel_raw')]
            if name == 'velocity_smoother':
                remaps.append(('cmd_vel_smoothed', '/navigation/cmd_vel'))
            nodes.append(Node(package=package, executable=executable, name=name,
                              parameters=[temp.name, time], remappings=remaps, output='screen'))
        nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', parameters=[time, {'autostart': True, 'node_names': names}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value=''),
        DeclareLaunchArgument('lio', default_value='fastlio', choices=['fastlio', 'pointlio']),
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localization']),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_driver', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_navigation', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=runtime),
    ])
