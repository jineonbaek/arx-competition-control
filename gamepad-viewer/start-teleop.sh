#!/usr/bin/env bash
set -eo pipefail
source /home/arx/arx-competition-control/env.sh
export ROS_DOMAIN_ID=191
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec python3 /home/arx/arx-competition-control/gamepad-viewer/teleop.py "$@"
