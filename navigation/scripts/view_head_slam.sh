#!/usr/bin/env bash
set -eo pipefail
nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export NAV_ROS_DOMAIN_ID=192
source "$nav_root/sensor_env.sh"
exec rviz2 -d "$nav_root/config/dual_lidar/head_slam.rviz" --ros-args -r __node:=head_slam_viewer
