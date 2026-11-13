"""PolarBear architecture, Jazzy plugins, measured LIFT geometry; no controllers."""
import json,os,math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
root=Path(__file__).resolve().parents[1];out=root/'polarbear_adapt';blind=float(os.environ.get('ARX_BLIND','.35'));sim=os.getenv('ARX_REPLAY','0')=='1'
assert .2<=blind<=.6
foot=json.loads((root/'static_preview/footprint.json').read_text());(out/'footprint.json').write_text(json.dumps(foot,indent=2));radius=foot['covering_radius_m']
seg=yaml.safe_load((root/'static_preview/segmentation.yaml').read_text())['/**']['ros__parameters'];seg.update(use_sim_time=sim,input_topic='/pb/level_cloud',ground_output_topic='/pb/ground_raw',obstacle_output_topic='/pb/obstacle_raw',r_min=blind)
import sys
sys.path.insert(0,str(root/'fixed_dual'))
from geometry import enabled
if enabled(root):
 rear=dict(seg,input_topic='/pb/rear/level_cloud',ground_output_topic='/pb/rear/ground_raw',obstacle_output_topic='/pb/rear/obstacle_raw',sensor_height=.228)
 (out/'rear_segmentation.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':rear}}))
(out/'segmentation.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':seg}}))
source={'topic':'/pb/scan','sensor_frame':'bottom_livox','data_type':'LaserScan','marking':True,'clearing':True,'inf_is_valid':False,'obstacle_min_range':blind,'obstacle_max_range':8.,'raytrace_min_range':blind,'raytrace_max_range':8.,'min_obstacle_height':0.,'max_obstacle_height':1.6,'observation_persistence':0.,'expected_update_rate':.5}
voxel={'plugin':'nav2_costmap_2d::VoxelLayer','enabled':True,'footprint_clearing_enabled':False,'origin_z':0.,'z_resolution':.10,'z_voxels':16,'max_obstacle_height':1.6,'mark_threshold':0,'unknown_threshold':15,'publish_voxel_map':False,'observation_sources':'scan','scan':source}
floor_source=dict(source,topic='/pb/floor_clear_scan',marking=False,clearing=True)
if sim and os.getenv('ARX_EXPERIMENTAL_CLEAR')=='1':
 voxel.update(observation_sources='scan floor',floor=floor_source)
stvl={'plugin':'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer','enabled':True,'voxel_decay':15.,'decay_model':0,'voxel_size':.05,'track_unknown_space':True,'mark_threshold':0,'update_footprint_enabled':False,'combination_method':1,'publish_voxel_map':True,'transform_tolerance':.3,'mapping_mode':False,'observation_sources':'mark clear',
 'mark':{'data_type':'PointCloud2','topic':'/pb/global_mark','sensor_frame':'bottom_livox','marking':True,'clearing':False,'obstacle_range':8.,'min_obstacle_height':.03,'max_obstacle_height':1.35,'expected_update_rate':.5,'observation_persistence':0.,'inf_is_valid':False,'filter':'voxel','voxel_min_points':0,'clear_after_reading':True},
 'clear':{'enabled':True,'data_type':'PointCloud2','topic':'/pb/obstacles','sensor_frame':'bottom_livox','marking':False,'clearing':True,'obstacle_range':8.,'min_z':1.,'max_z':6.,'vertical_fov_angle':math.radians(4),'vertical_fov_padding':0.,'horizontal_fov_angle':math.radians(60),'decay_acceleration':0.,'model_type':1,'observation_persistence':0.,'expected_update_rate':0.,'clear_after_reading':True}}
if sim and os.getenv('ARX_EXPERIMENTAL_CLEAR')=='1':
 stvl['confirmed_clear_topic']='/pb/confirmed_clear'
# STVL2.5.5 clearing-only BufferCloud returns before updating last_updated.
# Disable only its broken rate check; marking on the same topic retains .5s watchdog.
# clear_after_reading prevents reuse of old frustum observations.
# Keep a conservative frustum definition, but acceleration=0 until occlusion validation.
infl={'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':radius+.25,'cost_scaling_factor':5.}
mask_config=root/'config/base_envelope.json'
if mask_config.exists() and json.loads(mask_config.read_text()).get('enabled',False):
 infl.update(plugin='arx_base_mask::InflationLayer',chassis_mask_file=str(root/'config/base_envelope.wkt'))
 voxel.update(plugin='arx_base_mask::VoxelLayer',chassis_mask_file=str(root/'config/base_envelope.wkt'))
base={'use_sim_time':sim,'global_frame':'odom','robot_base_frame':'base_link','transform_tolerance':.3,'update_frequency':5.,'publish_frequency':2.,'resolution':.05,'track_unknown_space':True,'always_send_full_costmap':True,'footprint':json.dumps(foot['polygon_m']),'footprint_padding':.03}
local=dict(base,rolling_window=True,track_unknown_space=False,width=10,height=10,plugins=['voxel','inflation'],voxel=voxel,inflation=infl)
globalc=dict(base,global_frame='map',rolling_window=False,plugins=['static','stvl','inflation'],static={'plugin':'nav2_costmap_2d::StaticLayer','map_topic':'/pb/map','map_subscribe_transient_local':True,'subscribe_to_updates':True,'footprint_clearing_enabled':False},stvl=stvl,inflation=infl)
dual_mode = not sim or os.getenv('ARX_EXPERIMENTAL_CLEAR')!='1'
if dual_mode:
 local.update(update_frequency=20.,publish_frequency=10.)
 voxel.update(observation_sources='bottom head')
 voxel.pop('scan',None)
 stvl.update(observation_sources='bottom_mark bottom_clear head_mark head_clear')
 mark_template=stvl.pop('mark')
 clear_template=stvl.pop('clear')
 for sensor in ('bottom','head'):
  topic=f'/pb/{sensor}/nav_mark'
  frame=f'pb_nav_{sensor}_sensor'
  rate=.5 if sensor=='bottom' else 0.
  voxel[sensor]=dict(source,topic='/pb/scan' if sensor=='bottom' else ('/pb/head/direct_scan' if enabled(root) else '/pb/head/nav_scan'),sensor_frame='bottom_livox' if sensor=='bottom' else ('rear_livox' if enabled(root) else 'pb_nav_head_scan'),expected_update_rate=rate)
  stvl[sensor+'_mark']=dict(mark_template,topic=topic,sensor_frame=frame,expected_update_rate=rate)
  stvl[sensor+'_clear']=dict(clear_template,topic=topic,sensor_frame=frame,min_z=blind,max_z=8.,
                             horizontal_fov_angle=math.radians(275 if sensor=='bottom' else 360),
                             vertical_fov_angle=math.radians(12),decay_acceleration=5.)
 # STVL owns the live global occupancy. Native clear buffers supply the
 # frustums; source-stamped packets carry actual marks and finite rays together.
 stvl['observation_sources']='bottom_clear head_clear'
 stvl.pop('bottom_mark')
 stvl.pop('head_mark')
 stvl['evidence_topics']=['/pb/bottom/stvl_evidence','/pb/head/stvl_evidence']
 stvl['initial_history']=os.getenv('ARX_STVL_INITIAL_HISTORY','')
 globalc.update(update_frequency=10.,publish_frequency=10.)
primitives = Path(get_package_share_directory('nav2_smac_planner')) / 'sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json'
if not primitives.is_file():
 raise RuntimeError(f'Missing omni motion primitives: {primitives}')
planner={'use_sim_time':sim,'expected_planner_frequency':2.,'costmap_update_timeout':1.,'planner_plugins':['GridBased'],'GridBased':{'plugin':'nav2_smac_planner::SmacPlannerLattice','tolerance':.05,'allow_unknown':False,'allow_reverse_expansion':True,'smooth_path':False,'lattice_filepath':str(primitives)}}
(out/'nav2.yaml').write_text(yaml.safe_dump(json.loads(json.dumps({'global_costmap':{'global_costmap':{'ros__parameters':globalc}},'local_costmap':{'local_costmap':{'ros__parameters':local}},'planner_server':{'ros__parameters':planner}}))))
slam=yaml.safe_load((Path(get_package_share_directory('slam_toolbox'))/'config/mapper_params_online_async.yaml').read_text())['slam_toolbox']['ros__parameters'];slam.update(use_sim_time=sim,odom_frame='odom',map_frame='map',base_frame='base_link',scan_topic='/pb/mapping_scan',resolution=.05,map_update_interval=2.,check_min_dist_and_heading_precisely=True,minimum_travel_distance=.05,minimum_travel_heading=.05,minimum_time_interval=.5,min_laser_range=blind,max_laser_range=8.,enable_interactive_mode=False,min_pass_through=2)
if graph:=os.getenv('ARX_RESUME_GRAPH'):
 prefix=Path(graph)
 if seed:=os.getenv('ARX_RESUME_SEED'):
  match=json.loads(Path(seed).read_text())
  if not match['acceptable']:raise ValueError('SLAM resume seed did not pass validation')
  pose=match['base_pose']
 else:
  hint=json.loads((prefix.parent/'pose_hint.json').read_text())
  pose=[hint[k] for k in ('x_m','y_m','yaw_rad')]
 slam.update(map_file_name=str(prefix),map_start_pose=pose)
(out/'slam.yaml').write_text(yaml.safe_dump({'slam_toolbox':{'ros__parameters':slam}}))
(out/'policy.json').write_text(json.dumps({'perception_blind_m':blind,'range_metric':'horizontal from optical origin','mapping_height_optical_m':[-.18,.45],'obstacle_height_optical_m':[-.20,1.1],'lio_blind_changed':False,'obstacle_policy':('legacy STVL replay' if not dual_mode else 'dual VoxelLayer scans, dual STVL marks and visibility decay'),'commands_enabled':False},indent=2))
