"""Independent bottom LIO only. Reuses existing dual driver, no controllers."""
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
    p['common'].update(lid_topic='/dual/arm/lidar', imu_topic='/dual/arm/imu',
                       max_imu_gap_sec=0.1, align_gravity_world=True)
    p['frames'] = {'odom':'bottom_lidar_odom','imu':'bottom_lio_imu','lidar':'bottom_livox_frame'}
    p['preprocess']['blind'] = 0.65
    p['map_file_path'] = str(run/'bottom.pcd')
    (run/'bottom.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':p}}))
    lio = Node(package='fast_lio', executable='fastlio_mapping', name='bottom_lio', namespace='dual/bottom',
               parameters=[p], output='screen', remappings=[(src, '/dual/bottom/'+dst) for src,dst in [
               ('/cloud_registered','cloud_registered'),('/cloud_registered_body','cloud_body'),
               ('/cloud_effected','cloud_effected'),('/Laser_map','map'),('/Odometry','odometry'),('/path','path')]])
    internal = Node(package='tf2_ros', executable='static_transform_publisher', name='bottom_internal_tf',
         arguments=['--x','-0.011','--y','-0.02329','--z','0.04412','--roll','0','--pitch','0','--yaw','0',
                    '--frame-id','bottom_lio_imu','--child-frame-id','bottom_livox_frame'])
    return LaunchDescription([lio, internal, RegisterEventHandler(OnProcessExit(target_action=lio,
        on_exit=[EmitEvent(event=Shutdown(reason='bottom LIO exited; reinitialize while stationary'))]))])
