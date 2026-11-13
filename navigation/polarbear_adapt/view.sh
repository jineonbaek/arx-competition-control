#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source sensor_env.sh
export ARX_DESKTOP_LAUNCHER="$PWD/desktop/start_navigation.sh"
source staging/waypoint-rviz/install/local_setup.bash
config="$PWD/polarbear_adapt/polarbear.rviz"
python3 "$PWD/polarbear_adapt/rviz_cloud_display.py" &
rear_cloud_pid=$!
cleanup() {
    kill "$rear_cloud_pid" 2>/dev/null || true
    wait "$rear_cloud_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
rviz2 -d "$config"
