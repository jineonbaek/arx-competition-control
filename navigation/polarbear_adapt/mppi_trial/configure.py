"""Create isolated MPPI parameters from the commissioned ARX local costmap."""
import argparse
from pathlib import Path
import yaml

p = argparse.ArgumentParser()
p.add_argument('nav2', type=Path)
p.add_argument('output', type=Path)
p.add_argument('--sim-time', action='store_true')
args = p.parse_args()
costmap = yaml.safe_load(args.nav2.read_text())['local_costmap']['local_costmap']['ros__parameters']
costmap['use_sim_time'] = args.sim_time
params = dict(
    use_sim_time=args.sim_time,
    odom_topic='/localization/odometry',
    controller_frequency=20.0,
    costmap_update_timeout=.3,
    enable_stamped_cmd_vel=True,
    min_x_velocity_threshold=.001,
    min_y_velocity_threshold=.001,
    min_theta_velocity_threshold=.001,
    controller_plugins=['FollowPath'],
    progress_checker_plugins=['progress_checker'],
    goal_checker_plugins=['goal_checker'],
    progress_checker=dict(plugin='nav2_controller::SimpleProgressChecker',
                          required_movement_radius=.04, movement_time_allowance=12.),
    goal_checker=dict(plugin='nav2_controller::SimpleGoalChecker',
                      xy_goal_tolerance=.045, yaw_goal_tolerance=.09, stateful=True),
    FollowPath=dict(
        plugin='nav2_mppi_controller::MPPIController', motion_model='Omni',
        model_dt=.05, time_steps=40, batch_size=600, iteration_count=1,
        vx_max=.10, vx_min=-.10, vy_max=.10, wz_max=.18,
        ax_max=.20, ax_min=-.20, ay_max=.20, ay_min=-.20, az_max=.35,
        vx_std=.07, vy_std=.07, wz_std=.12, transform_tolerance=.3,
        prune_distance=1., visualize=False,
        critics=['ConstraintCritic','CostCritic','GoalCritic','GoalAngleCritic',
                 'PathAlignCritic','PathFollowCritic'],
        CostCritic=dict(enabled=True, consider_footprint=True, trajectory_point_step=1),
        PathAlignCritic=dict(enabled=True, use_path_orientations=True),
    ),
)
args.output.write_text(yaml.safe_dump({
    '/arx_mppi_trial/controller_server': {'ros__parameters': params},
    '/arx_mppi_trial/local_costmap/local_costmap': {'ros__parameters': costmap},
}))
