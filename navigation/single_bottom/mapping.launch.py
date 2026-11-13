"""Bottom-only LIO mapping, chassis pose and grid. No controller nodes."""
import os,sys,json
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess,RegisterEventHandler,EmitEvent,TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
 root=Path(__file__).resolve().parents[1];run=Path(os.environ['ARX_SINGLE_RUN']);run.mkdir(parents=True,exist_ok=True)
 sys.path.insert(0,str(root/'fixed_dual'))
 from geometry import enabled,configuration
 dual=enabled(root)
 cfg=root/'config/bottom_upright_20261105.json';mesh=root/'models/lift2s_bottom_upright_20261105/body_voxels.npz'
 params=yaml.safe_load((root/'config/mapping_test/fastlio_arm.yaml').read_text())['/**']['ros__parameters']
 if dual:mesh=root/configuration(root)['mesh']
 params['common'].update(lid_topic='/fixed_dual/lidar' if dual else '/single/lidar_filtered',imu_topic='/single/imu',max_imu_gap_sec=.1,align_gravity_world=True)
 params['frames']={'odom':'lio_world','imu':'bottom_imu','lidar':'bottom_livox'};params['preprocess']['blind']=.2;params['map_file_path']=str(run/'map_lio_world.pcd');params['mapping']['extrinsic_est_en']=False
 params['publish'].update(sensor_only_tf=True,scan_bodyframe_pub_en=True,map_en=True)
 (run/'fastlio.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}));(run/'geometry.json').write_text(cfg.read_text())
 nodes=[]
 for mode in (['body','map'] if dual else ['filter','body','map']):
  nodes.append(ExecuteProcess(cmd=[sys.executable,str(root/'single_bottom/runtime.py'),mode,'--config',str(cfg),'--mesh',str(mesh),'--run',str(run)],output='screen'))
 if dual:
  for name,source in [('input_filter.py',''),('observations.py','bottom'),('observations.py','head')]:
   nodes.append(ExecuteProcess(cmd=[sys.executable,str(root/'fixed_dual'/name)],additional_env={'ARX_DESKEW_SOURCE':source,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'},output='screen'))
  nodes.append(ExecuteProcess(cmd=[str(root/'fixed_dual/build/frontend'),str(root),str(run)],output='screen'))
 required=list(nodes)
 lio=Node(package='fast_lio',executable='fastlio_mapping',name='bottom_upright_lio',namespace='single',parameters=[params],output='screen',remappings=[('/tf','/single/private_lio_tf'),('/cloud_registered','/single/cloud_registered'),('/cloud_registered_body','/single/cloud_body'),('/cloud_effected','/single/cloud_effected'),('/Laser_map','/single/map_raw'),('/Odometry','/single/odometry_raw'),('/path','/single/path_raw')])
 nodes.append(TimerAction(period=2.,actions=[lio]));nodes.append(ExecuteProcess(cmd=[sys.executable,str(root/'scripts/pointmap_grid.py'),'--cloud-topic','/mapping/cloud_map','--odom-topic','/localization/odometry','--map-topic','/mapping/grid','--frame','odom','--floor','0','0','0','--resolution','0.05','--output',str(run/'grid')],output='screen'))
 for process in required+[lio]:nodes.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason='Required single LIO process stopped; reinitialize stationary'))])))
 return LaunchDescription(nodes)
