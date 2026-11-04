#!/usr/bin/env bash
set -eo pipefail
ROOT=/home/arx/arx-competition-control
case "${1:-}" in left|right) side="$1" ;; *) echo "Usage: start-arm.sh left|right" >&2; exit 2 ;; esac
source "$ROOT/env.sh"
export LD_LIBRARY_PATH="$ROOT/repos/ARX_X5/ROS2/X5_ws/src/arx_x5_ros2/arx_x5_controller/lib/arx_x5_src:$LD_LIBRARY_PATH"
export LD_BIND_NOW=1
export ROS_DOMAIN_ID=191
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec "$ROOT/web/hardware/build/x5_controller_web" --ros-args -r __node:="web_x5_$side" -p side:="$side"
