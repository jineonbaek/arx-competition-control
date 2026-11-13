#!/usr/bin/env bash
set -eo pipefail
nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$nav_root/sensor_env.sh"
mkdir -p "$nav_root/runtime" "$nav_root/workspace/src/FAST_LIO/Log"
exec 9>"$nav_root/runtime/arm-mapping.lock"
flock -n 9 || { echo 'Arm lidar mapping is already running'; exit 1; }
run_dir="$(python3 "$nav_root/scripts/prepare_arm_mapping.py")"
export ROS_LOG_DIR="$run_dir/ros"
printf '%s\n' "$$" > "$nav_root/runtime/arm-mapping.pid"
exec ros2 launch "$nav_root/config/mapping_test/arm_lidar.launch.py"
