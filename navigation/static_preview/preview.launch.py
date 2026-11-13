"""Passive perception, SLAM map comparison, standalone costmap and planner only."""
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
 d=Path(__file__).resolve().parent
 bridge=ExecuteProcess(cmd=[sys.executable,str(d/'bridge.py')],output='screen')
 seg=Node(package='linefit_ground_segmentation_ros',executable='ground_segmentation_node',name='preview_ground_segmentation',parameters=[str(d/'segmentation.yaml')],output='screen')
 scan=Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',name='preview_cloud_to_scan',parameters=[{'target_frame':'bottom_livox','transform_tolerance':.3,'min_height':-.20,'max_height':1.1,'angle_min':-3.141592653589793,'angle_max':3.141592653589793,'angle_increment':.008726646259971648,'scan_time':.1,'range_min':.2,'range_max':8.,'use_inf':True}],remappings=[('cloud_in','/preview/obstacles'),('scan','/preview/scan_unchecked')],output='screen')
 slam=Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',parameters=[str(d/'slam.yaml')],remappings=[('map','/preview/slam_map'),('map_metadata','/preview/slam_map_metadata')],output='screen')
 cost=Node(package='nav2_costmap_2d',executable='nav2_costmap_2d',name='local_costmap',namespace='local_costmap',parameters=[str(d/'nav2.yaml')],output='screen')
 plan=Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[str(d/'nav2.yaml')],output='screen')
 client=ExecuteProcess(cmd=[sys.executable,str(d/'plan_only.py')],output='screen')
 nodes=[bridge,seg,scan,slam,cost,plan,client,ExecuteProcess(cmd=[sys.executable,str(d/'grid_display.py')],output='screen'),Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='preview_lifecycle',parameters=[{'autostart':True,'node_names':['slam_toolbox','/local_costmap/local_costmap','planner_server'],'bond_timeout':0.0}],output='screen')]
 for n in [bridge,seg,scan,slam,cost,plan,client]:nodes.append(RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='Preview component stopped'))])))
 return LaunchDescription(nodes)
