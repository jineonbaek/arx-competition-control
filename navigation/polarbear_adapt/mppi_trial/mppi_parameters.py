"""Measured forward-stage Omni controller; costmap geometry is supplied unchanged."""
if __package__:
    from .speed_profile import NORMAL_XY_SPEED,NORMAL_YAW_SPEED
else:
    from speed_profile import NORMAL_XY_SPEED,NORMAL_YAW_SPEED

def controller_parameters(sim=False):
    return dict(
        use_sim_time=sim, odom_topic='/localization/odometry',
        speed_limit_topic='/pb/mppi/speed_limit',
        controller_frequency=20., costmap_update_timeout=.3,
        enable_stamped_cmd_vel=True,
        min_x_velocity_threshold=.001, min_y_velocity_threshold=.001,
        min_theta_velocity_threshold=.001,
        controller_plugins=['FollowPath'],
        progress_checker_plugins=['progress_checker'],
        goal_checker_plugins=['goal_checker'],
        progress_checker=dict(plugin='nav2_controller::PoseProgressChecker',
            required_movement_radius=.03, required_movement_angle=.025,
            movement_time_allowance=12.),
        goal_checker=dict(plugin='nav2_controller::SimpleGoalChecker',
            xy_goal_tolerance=.045, yaw_goal_tolerance=.09, stateful=True),
        FollowPath=dict(
            plugin='nav2_mppi_controller::MPPIController', motion_model='Omni',
            model_dt=.05, time_steps=40, batch_size=600, iteration_count=1,
            vx_max=NORMAL_XY_SPEED, vx_min=-NORMAL_XY_SPEED, vy_max=NORMAL_XY_SPEED, wz_max=NORMAL_YAW_SPEED,
            ax_max=1.2, ax_min=-1.2, ay_max=1.2, ay_min=-1.2, az_max=1.2,
            vx_std=.30, vy_std=.30, wz_std=.40, transform_tolerance=.3,
            prune_distance=2., visualize=False,
            critics=['ConstraintCritic','CostCritic','GoalCritic','GoalAngleCritic',
                     'PathAlignCritic','PathFollowCritic'],
            CostCritic=dict(enabled=True, consider_footprint=True, trajectory_point_step=1),
            GoalCritic=dict(cost_weight=35.,threshold_to_consider=.45),
            GoalAngleCritic=dict(cost_weight=15.,threshold_to_consider=.5),
            PathAlignCritic=dict(enabled=True,use_path_orientations=True,
                                offset_from_furthest=5,threshold_to_consider=.10),
            PathFollowCritic=dict(cost_weight=18.,offset_from_furthest=20,
                                 threshold_to_consider=.12)))
