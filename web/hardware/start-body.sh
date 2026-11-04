#!/usr/bin/env bash
set -eo pipefail
ROOT=/home/arx/arx-competition-control
BODY_LIB="$ROOT/repos/LIFT/body/ROS2/src/ARX_LIFT_ros2/arx_lift_controller/lib/arx_lift_src"
source "$ROOT/env.sh"
export LD_LIBRARY_PATH="$BODY_LIB:$LD_LIBRARY_PATH"
export LD_BIND_NOW=1
export ROS_DOMAIN_ID=191
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec "$ROOT/web/hardware/build/lift_controller_web" --ros-args -p robot_type:=0 -p fixed_body_height:=0.0
