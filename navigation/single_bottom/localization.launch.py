"""Saved-map AMCL localization only. No planning, control or actuator nodes.

Set ARX_LOCALIZATION_MAP and initial pose explicitly. This does not supply
global automatic relocalization or certify mapping/localization accuracy.
"""
import os,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from core import localization_initial_pose
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
 path=os.environ['ARX_LOCALIZATION_MAP'];x,y,yaw=localization_initial_pose(os.environ)
 return LaunchDescription([
  Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',name='bottom_cloud_to_scan',parameters=[{'target_frame':'bottom_livox','transform_tolerance':.15,'min_height':-.15,'max_height':.8,'angle_min':-3.141592653589793,'angle_max':3.141592653589793,'angle_increment':.008726646259971648,'scan_time':.1,'range_min':.2,'range_max':20.,'use_inf':True}],remappings=[('cloud_in','/single/cloud_body'),('scan','/single/scan')]),
  Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[{'yaml_filename':path,'frame_id':'map','topic_name':'map','use_sim_time':False}]),
  Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[{'base_frame_id':'base_link','global_frame_id':'map','odom_frame_id':'odom','scan_topic':'/single/scan','robot_model_type':'nav2_amcl::OmniMotionModel','set_initial_pose':False,'max_beams':90,'min_particles':500,'max_particles':2000,'update_min_d':.05,'update_min_a':.05,'laser_min_range':.2,'laser_max_range':20.,'transform_tolerance':.3,'tf_broadcast':True}]),
  ExecuteProcess(cmd=[sys.executable,str(Path(__file__).resolve().parent/'seed_localization.py')],output='screen'),
  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='localization_lifecycle',parameters=[{'autostart':True,'node_names':['map_server','amcl']}])])
