#!/usr/bin/env bash
set -e -o pipefail
cd "$(dirname "$0")/../.."
trial_dir=$1
params=$2
domain=${3:-192}
source sensor_env.sh
export ROS_DOMAIN_ID="$domain"
mkdir -p "$trial_dir"
ros2 run nav2_controller controller_server --ros-args -r __ns:=/arx_mppi_trial \
  -r /arx_mppi_trial/local_costmap/tf:=/tf -r /arx_mppi_trial/local_costmap/tf_static:=/tf_static \
  -r cmd_vel:=/arx_mppi_trial/raw_velocity --params-file "$params" &
pid=$!
trap 'kill -INT "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true' EXIT
trap 'exit 0' INT TERM
python3 polarbear_adapt/mppi_trial/lifecycle_wait.py
wait "$pid"
