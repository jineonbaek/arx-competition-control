"""Unmounted D435 + independent dual MID360S streams; no chassis control."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
PROFILES={
 'balanced':('640,480,30','848,480,30'),
 'high_speed':('640,480,60','848,480,90'),
 'max_resolution':('1920,1080,30','1280,720,30'),
}
def runtime(context):
 color,depth=PROFILES[LaunchConfiguration('profile').perform(context)]
 config=Path(__file__).with_name('mid360s_dual_capture.json')
 realsense=Path(get_package_share_directory('realsense2_camera'))/'launch/rs_launch.py'
 return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(realsense)),launch_arguments={
    'camera_namespace':'sensors','camera_name':'d435',
    'serial_no':'_'+LaunchConfiguration('camera_serial').perform(context),
    'enable_color':'true','enable_depth':'true','enable_infra1':'false','enable_infra2':'false',
    'rgb_camera.color_profile':color,'depth_module.depth_profile':depth,
    'enable_sync':'false','align_depth.enable':'false','pointcloud.enable':'false'}.items()),
  Node(package='livox_ros_driver2',executable='livox_ros_driver2_node',name='dual_mid360s_capture',
       output='screen',parameters=[{'xfer_format':1,'multi_topic':1,'data_src':0,'publish_freq':10.0,
        'output_data_type':0,'frame_id':'livox_unmounted','user_config_path':str(config)}])]
def generate_launch_description():
 return LaunchDescription([
  DeclareLaunchArgument('profile',default_value='balanced',choices=list(PROFILES)),
  DeclareLaunchArgument('camera_serial',default_value='349622074226'),
  OpaqueFunction(function=runtime)])
