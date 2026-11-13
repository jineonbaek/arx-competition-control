from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import sys

def generate_launch_description():
 root=Path(__file__).resolve().parents[2]
 run=Path((root/'reports/latest-arm-mapping-path.txt').read_text().strip())
 return LaunchDescription([
  Node(package='livox_ros_driver2',executable='livox_ros_driver2_node',name='arm_mid360s',output='screen',
   parameters=[{'xfer_format':1,'multi_topic':0,'data_src':0,'publish_freq':10.0,'output_data_type':0,'frame_id':'livox_frame','user_config_path':str(root/'config/mapping_test/mid360s_arm.json')}],
   remappings=[('/livox/lidar','/mapping_test/lidar'),('/livox/imu','/mapping_test/imu')]),
  Node(package='tf2_ros',executable='static_transform_publisher',name='lio_imu_to_lidar',output='screen',
   arguments=['--x','-0.011','--y','-0.02329','--z','0.04412','--roll','0','--pitch','0','--yaw','0','--frame-id','lio_imu','--child-frame-id','livox_frame']),
  Node(package='fast_lio',executable='fastlio_mapping',name='arm_lio',namespace='mapping_test',output='screen',parameters=[str(run/'fastlio.yaml'),{'use_sim_time':False}],
   remappings=[('/cloud_registered','/mapping_test/cloud_registered'),('/cloud_registered_body','/mapping_test/cloud_body'),('/cloud_effected','/mapping_test/cloud_effected'),('/Laser_map','/mapping_test/map'),('/Odometry','/mapping_test/odometry'),('/path','/mapping_test/path')]),
  ExecuteProcess(cmd=[sys.executable,str(root/'scripts/observe_arm_mapping.py'),str(run)],output='screen')])
