#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source sensor_env.sh
source "$PWD/staging/effective-stvl/install/local_setup.bash"
if [[ "${ARX_REPLAY:-0}" == 1 ]]; then
 export ROS_DOMAIN_ID=199
else
 nodes="$(ros2 node list --no-daemon --spin-time 3 2>/dev/null)"
 if grep -Eq '^/(preview_level_bridge|pb_level_bridge)$' <<< "$nodes"; then
  echo 'Existing preview active: stop its recorded launch PID before switching.' >&2; exit 1
 fi
 if ! grep -qx /single/bottom_upright_lio <<< "$nodes"; then
  echo 'Bottom LIO must already be running. This script does not initialize IMU.' >&2;exit 1
 fi
fi
: "${ARX_ADAPT_RUN:?Set new report directory}"
mkdir -p "$ARX_ADAPT_RUN"
python3 polarbear_adapt/prepare.py
python3 polarbear_adapt/rviz_config.py
cp polarbear_adapt/{nav2.yaml,slam.yaml,segmentation.yaml,policy.json} "$ARX_ADAPT_RUN/"
exec ros2 launch polarbear_adapt/navigation.launch.py
