"""Build preview configs from installed upstream packages and delivered CAD cache."""
import json,math
from pathlib import Path
import numpy as np,yaml
from scipy.spatial import ConvexHull
from ament_index_python.packages import get_package_share_directory
root=Path(__file__).resolve().parents[1];out=root/'static_preview';out.mkdir(exist_ok=True)
a=np.load(root/'models/lift2s_bottom_upright_20261105/body_voxels.npz');head=a['head']
xy=np.vstack([a['rigid'],head])[:,:2];hull=xy[ConvexHull(xy).vertices];radius=math.ceil((np.linalg.norm(xy,axis=1).max()+.00684+.03)*1000)/1000
(out/'footprint.json').write_text(json.dumps({'polygon_m':hull.tolist(),'covering_radius_m':radius,'padding_m':.03,'source':'delivered CAD voxel hull; nominal arm/lift pose','head_margin_for_collision_not_flex_tracking':True},indent=2))
seg={'n_threads':2,'r_min':.2,'r_max':20.,'n_bins':120,'n_segments':60,'max_dist_to_line':.035,'sensor_height':.23,'min_slope':0.,'max_slope':.3,'max_fit_error':.025,'long_threshold':1.,'max_long_height':.06,'max_start_height':.10,'line_search_angle':.1,'gravity_aligned_frame':'','visualize':False,'input_topic':'/preview/level_cloud','obstacle_output_topic':'/preview/obstacle_raw','ground_output_topic':'/preview/ground_raw'}
# Inputs already sensor-centred gravity level; declared overrides are checked at runtime.
(out/'segmentation.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':seg}}))
obs={'plugin':'nav2_costmap_2d::ObstacleLayer','enabled':True,'footprint_clearing_enabled':False,'observation_sources':'scan','scan':{'topic':'/preview/scan','data_type':'LaserScan','marking':True,'clearing':True,'inf_is_valid':False,'obstacle_min_range':.2,'obstacle_max_range':8.,'raytrace_min_range':.2,'raytrace_max_range':8.,'min_obstacle_height':-.5,'max_obstacle_height':1.6,'observation_persistence':0.,'expected_update_rate':.5}}
infl={'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':radius+.25,'cost_scaling_factor':5.}
base={'use_sim_time':False,'global_frame':'odom','robot_base_frame':'base_link','transform_tolerance':.3,'update_frequency':5.,'publish_frequency':2.,'resolution':.05,'track_unknown_space':True,'always_send_full_costmap':True,'robot_radius':radius,'footprint_padding':0.,'plugins':['obstacles','inflation'],'obstacles':obs,'inflation':infl}
local=dict(base,rolling_window=True,width=10,height=10)
globalc=dict(base,global_frame='preview_map',rolling_window=True,width=16,height=16)
planner={'use_sim_time':False,'expected_planner_frequency':2.,'costmap_update_timeout':1.,'planner_plugins':['GridBased'],'GridBased':{'plugin':'nav2_navfn_planner::NavfnPlanner','tolerance':.10,'use_astar':False,'allow_unknown':False}}
(out/'nav2.yaml').write_text(yaml.safe_dump(json.loads(json.dumps({'global_costmap':{'global_costmap':{'ros__parameters':globalc}},'local_costmap':{'local_costmap':{'ros__parameters':local}},'planner_server':{'ros__parameters':planner}}))))
# Use installed Jazzy defaults, overlay PolarBear mapping choices and actual frames.
slam=yaml.safe_load((Path(get_package_share_directory('slam_toolbox'))/'config/mapper_params_online_async.yaml').read_text())['slam_toolbox']['ros__parameters']
slam.update(odom_frame='odom',map_frame='preview_map',base_frame='base_link',scan_topic='/preview/scan',resolution=.05,map_update_interval=2.,minimum_travel_distance=.05,minimum_travel_heading=.05,minimum_time_interval=.5,min_laser_range=.2,max_laser_range=8.,enable_interactive_mode=False,min_pass_through=1)
(out/'slam.yaml').write_text(yaml.safe_dump({'slam_toolbox':{'ros__parameters':slam}}))
print(json.dumps({'covering_radius_m':radius,'hull_vertices':len(hull)}))
