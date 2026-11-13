from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[2]
    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server', name='archive_grid_server',
             parameters=[{'yaml_filename': str(root/'maps/arm_manual_203414_grid.yaml'),
                          'frame_id': 'lidar_odom', 'topic_name': '/mapping/saved_grid'}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='archive_grid_lifecycle',
             parameters=[{'autostart': True, 'node_names': ['archive_grid_server']}], output='screen')])
