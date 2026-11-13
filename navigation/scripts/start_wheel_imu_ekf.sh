#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export NAV_ROS_DOMAIN_ID=192
source "$root/sensor_env.sh"
mkdir -p "$root/runtime"
exec 9>"$root/runtime/wheel-imu-inputs.lock"
flock -n 9 || { echo 'Stop the read-only input pipeline before starting the full EKF'; exit 1; }
printf '%s\n' "$$" > "$root/runtime/wheel-imu-inputs.pid"
exec ros2 launch "$root/config/wheel_imu_fusion/wheel_imu.launch.py"
