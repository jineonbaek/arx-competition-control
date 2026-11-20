#!/usr/bin/env bash
set -eo pipefail
ROOT=/home/arx/arx-competition-control
source "$ROOT/env.sh"
export ROS_DOMAIN_ID=191
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec python3 -m arx_chassis.node --config "$ROOT/web/config/control.json"
