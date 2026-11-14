"""PolarBear mapping/costmaps/planning, with selectable guarded path tracking."""
import os,sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess,RegisterEventHandler,EmitEvent,TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
 if os.getenv('ARX_REPLAY','0')=='1' and os.getenv('ROS_DOMAIN_ID')!='199':raise RuntimeError('Replay requires domain199')
 d=Path(__file__).resolve().parent;sim=os.getenv('ARX_REPLAY','0')=='1';blind=float(os.getenv('ARX_BLIND','.35'));nodes=[]
 import json
 selection=d.parent/'config/path_controller.json'
 controller=os.getenv('ARX_PATH_CONTROLLER') or (json.loads(selection.read_text()).get('mode','legacy') if selection.exists() else 'legacy')
 if controller not in ('legacy','mppi'):raise ValueError('Unknown path controller '+controller)
 mppi=controller=='mppi'
 sys.path.insert(0,str(d.parent/'fixed_dual'))
 from geometry import enabled
 dual=enabled(d.parent)
 def py(name,environment=None):return ExecuteProcess(additional_env=environment or {},cmd=[sys.executable,str(d/name),'--ros-args','-p',f'use_sim_time:={str(sim).lower()}'],output='screen')
 bridge=(ExecuteProcess(cmd=[sys.executable,str(d/'bridge.py'),'--ros-args','-r','/pb/mapping_scan:=/fixed_dual/bottom/mapping_scan'],output='screen') if dual else py('bridge.py'));nodes.append(bridge)
 seg=Node(package='linefit_ground_segmentation_ros',executable='ground_segmentation_node',name='pb_ground_segmentation',parameters=[str(d/'segmentation.yaml')],output='screen');nodes.append(seg)
 for name,lo,hi in [('scan',-8.,8.),('mapping_scan',-8.,8.)]:
  # Navigation cloud already has the world-height gate. Gravity-aligned scan
  # projection must not reintroduce a second, tilted optical-height threshold.
  nodes.append(Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',name='pb_'+name,parameters=[{'use_sim_time':sim,'target_frame':'pb_lidar_level' if name=='scan' else 'bottom_livox','transform_tolerance':.3,'min_height':lo,'max_height':hi,'angle_min':-3.141592653589793,'angle_max':3.141592653589793,'angle_increment':.008726646259971648,'scan_time':.1,'range_min':blind,'range_max':8.,'use_inf':True}],remappings=[('cloud_in','/pb/nav_obstacles'),('scan','/pb/'+name+'_unchecked')],output='screen'))
 if dual:
  desktop_mode=os.getenv('ARX_DESKTOP_MODE')
  default_hold_ms='80' if not sim and desktop_mode in ('mapping','navigation','mapping-navigation') else '150'
  hold_ms=os.getenv('ARX_SCAN_REORDER_HOLD_MS',default_hold_ms)
  nodes.append(ExecuteProcess(
   cmd=[sys.executable,str(d.parent/'fixed_dual/scan_order.py'),
        '--ros-args','-p',f'reorder_hold_ms:={hold_ms}'],output='screen'))
  mapping={'/pb/mapping_scan':'/fixed_dual/head/mapping_scan','/pb/level_cloud':'/pb/rear/level_cloud','/pb/obstacles':'/pb/rear/obstacles','/pb/ground':'/pb/rear/ground',
   '/pb/ground_raw':'/pb/rear/ground_raw','/pb/obstacle_raw':'/pb/rear/obstacle_raw','/pb/nav_obstacles':'/pb/rear/nav_obstacles','/pb/scan':'/pb/head/direct_scan',
   '/pb/scan_unchecked':'/pb/rear/scan_unchecked','/pb/mapping_scan_unchecked':'/pb/rear/mapping_scan_unchecked',
   '/pb/bottom_observations':'/pb/head_observations','/pb/global_mark':'/pb/rear/global_mark'}
  args=[sys.executable,str(d/'bridge.py'),'--ros-args','-p',f'use_sim_time:={str(sim).lower()}']
  for a,b in mapping.items():args.extend(['-r',a+':='+b])
  nodes.append(ExecuteProcess(cmd=args,additional_env={'ARX_OBSERVATION_SENSOR':'head'},output='screen'))
  nodes.append(Node(package='linefit_ground_segmentation_ros',executable='ground_segmentation_node',name='pb_rear_ground_segmentation',parameters=[str(d/'rear_segmentation.yaml')],output='screen'))
  for name,lo,hi in [('scan',-8.,8.),('mapping_scan',-8.,8.)]:
   nodes.append(Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',name='pb_rear_'+name,parameters=[{'use_sim_time':sim,'target_frame':'pb_rear_lidar_level' if name=='scan' else 'rear_livox','transform_tolerance':.3,'min_height':lo,'max_height':hi,'angle_min':-3.141592653589793,'angle_max':3.141592653589793,'angle_increment':.008726646259971648,'scan_time':.1,'range_min':blind,'range_max':8.,'use_inf':True}],remappings=[('cloud_in','/pb/rear/nav_obstacles'),('scan','/pb/rear/'+name+'_unchecked')],output='screen'))
 if os.getenv('ARX_SCAN_BOOTSTRAP')=='1':return LaunchDescription(nodes)
 saved=os.getenv('ARX_SAVED_MAP')
 mapping_mode=os.getenv('ARX_DESKTOP_MODE') in ('mapping','mapping-navigation')
 dual_grid=(mapping_mode and not saved and not os.getenv('ARX_RESUME_GRAPH')
            and not sim)
 if dual_grid and not dual:
  raise RuntimeError('Live dual-origin mapping requires fixed front and rear lidars')
 localization_nodes=[]
 if saved and not sim:
  import json
  seed=json.loads(Path(os.environ['ARX_MAP_SEED']).read_text())
  if not seed['acceptable']:raise ValueError('Saved map seed failed scan validation')
  x,y,yaw=seed['base_pose']
  localization_nodes=['map_server','amcl']
  nodes.append(Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[{'yaml_filename':saved,'topic_name':'/pb/reference_map','frame_id':'map'}],output='screen'))
  nodes.append(Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[{'base_frame_id':'base_link','odom_frame_id':'odom','global_frame_id':'map','map_topic':'/pb/reference_map','scan_topic':'/pb/mapping_scan','robot_model_type':'nav2_amcl::OmniMotionModel','set_initial_pose':True,'initial_pose.x':x,'initial_pose.y':y,'initial_pose.yaw':yaw,'update_min_d':.04,'update_min_a':.06,'transform_tolerance':.3,'laser_min_range':.35,'laser_max_range':8.}],remappings=[('map','/pb/reference_map')],output='screen'))
 elif dual_grid:
  nodes.append(py('dual_origin_grid.py'))
 else:
  localization_nodes=['slam_toolbox']
  nodes.append(Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',parameters=[str(d/'slam.yaml')],remappings=[('map','/pb/reference_map'),('map_metadata','/pb/map_metadata')],output='screen'))
 nodes.append(py('chassis_pose.py'))
 nodes.append(py('live_static_map.py'))
 local_controller=(Node(package='nav2_controller',executable='controller_server',name='controller_server',parameters=[str(d/'nav2.yaml')],remappings=[('cmd_vel','/pb/mppi/raw_velocity')],output='screen') if mppi else
  Node(package='nav2_costmap_2d',executable='nav2_costmap_2d',name='local_costmap',namespace='local_costmap',parameters=[str(d/'nav2.yaml')],output='screen'))
 nodes.extend([
  local_controller,
  Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[str(d/'nav2.yaml')],output='screen'),py('plan_only.py'),py('grid_display.py')])
 if mppi:
  args=[sys.executable,str(d/'mppi_trial/preview_gate.py'),'--ros-args','-r','__node:=pb_mppi_command_gate','-p',f'use_sim_time:={str(sim).lower()}']
  for suffix in ('raw_velocity','preview_velocity','sdk_preview','preview_status','inputs_ready'):
   args.extend(['-r',f'/arx_mppi_trial/{suffix}:=/pb/mppi/{suffix}'])
  args.extend(['-r','/arx_mppi_trial/local_costmap/costmap:=/local_costmap/costmap'])
  nodes.append(ExecuteProcess(cmd=args,output='screen'))
 if sim:nodes.append(py('replay_tf.py'))
 if not sim or os.getenv('ARX_EXPERIMENTAL_CLEAR')!='1':
  nodes.append(py('dual_observations.py',{'ARX_FAST_SENSOR_NODE':'1','ARX_STVL_EVIDENCE':'1'}))
 required=list(nodes)
 if not sim and not dual:nodes.append(py("stationary_head.py"))
 if not sim and os.getenv("ROS_DOMAIN_ID")=="192" and os.getenv("ARX_EXTERNAL_GOAL_DRIVER")!="1" and os.getenv("ARX_DESKTOP_MODE")!="mapping":nodes.append(py("mppi_goal_drive.py" if mppi else "goal_drive.py"))
 manager=Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='pb_lifecycle',parameters=[{'use_sim_time':sim,'autostart':not sim,'node_names':localization_nodes+['controller_server' if mppi else '/local_costmap/local_costmap','planner_server'],'bond_timeout':0.0}],output='screen')
 nodes.append(TimerAction(period=2.,actions=[manager]) if dual_grid else manager)
 for n in required:nodes.append(RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='PolarBear preview component exited'))])))
 return LaunchDescription(nodes)
