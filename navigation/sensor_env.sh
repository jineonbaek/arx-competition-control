# Source only for the isolated sensor/navigation test environment.
_sensor_env_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_sensor_env_root/env.sh"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE="$_sensor_env_root/config/sensors_fastdds_local.xml"
unset FASTDDS_BUILTIN_TRANSPORTS
unset _sensor_env_root
