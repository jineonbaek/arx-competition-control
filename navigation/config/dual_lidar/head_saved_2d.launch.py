from pathlib import Path
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    root=Path(__file__).resolve().parents[2]
    folder=Path((root/'maps/latest_head_2d_path.txt').read_text().strip())
    view=Node(package='rviz2',executable='rviz2',name='head_saved_2d_viewer',arguments=['-d',str(root/'config/dual_lidar/head_saved_2d.rviz')],output='screen')
    return LaunchDescription([
        Node(package='nav2_map_server',executable='map_server',name='head_saved_2d_server',parameters=[{'yaml_filename':str(folder/'map.yaml'),'frame_id':'top_lidar_odom','topic_name':'/mapping/head_saved_grid'}],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='head_saved_2d_lifecycle',parameters=[{'autostart':True,'node_names':['head_saved_2d_server']}],output='screen'),
        view,RegisterEventHandler(OnProcessExit(target_action=view,on_exit=[EmitEvent(event=Shutdown(reason='2D map viewer closed'))]))])
