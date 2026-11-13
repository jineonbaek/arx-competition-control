"""Independent top LIO only. Reuses existing dual driver, no controllers."""
import os
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    run = Path(os.environ['ARX_DYNAMIC_RUN'])
    run.mkdir(parents=True, exist_ok=True)
    p = yaml.safe_load((root/'config/mapping_test/fastlio_arm.yaml').read_text())['/**']['ros__parameters']
    p['common'].update(lid_topic='/dual/top/lidar', imu_topic='/dual/top/imu',
                       max_imu_gap_sec=0.1, align_gravity_world=True)
    p['frames'] = {'odom':'top_lidar_odom','imu':'top_lio_imu','lidar':'top_livox_frame'}
    p['preprocess']['blind'] = 0.5
    p['map_file_path'] = str(run/'top.pcd')
    (run/'top.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':p}}))
    lio = Node(package='fast_lio', executable='fastlio_mapping', name='top_lio', namespace='dual/top',
               parameters=[p], output='screen', remappings=[(src, '/dual/top/'+dst) for src,dst in [
               ('/cloud_registered','cloud_registered'),('/cloud_registered_body','cloud_body'),
               ('/cloud_effected','cloud_effected'),('/Laser_map','map'),('/Odometry','odometry'),('/path','path')]])
    internal = Node(package='tf2_ros', executable='static_transform_publisher', name='top_internal_tf',
         arguments=['--x','-0.011','--y','-0.02329','--z','0.04412','--roll','0','--pitch','0','--yaw','0',
                    '--frame-id','top_lio_imu','--child-frame-id','top_livox_frame'])
    return LaunchDescription([lio, internal, RegisterEventHandler(OnProcessExit(target_action=lio,
        on_exit=[EmitEvent(event=Shutdown(reason='top LIO exited; reinitialize while stationary'))]))])
