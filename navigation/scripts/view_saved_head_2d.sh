#!/usr/bin/env bash
set -eo pipefail
nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export NAV_ROS_DOMAIN_ID=192
source "$nav_root/sensor_env.sh"
exec ros2 launch "$nav_root/config/dual_lidar/head_saved_2d.launch.py"
