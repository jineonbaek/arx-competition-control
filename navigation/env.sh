# source navigation/env.sh; does not launch any node or access hardware.
_arx_nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
if [ -f "$_arx_nav_root/workspace/install/setup.bash" ]; then
  source "$_arx_nav_root/workspace/install/setup.bash"
fi
export LD_LIBRARY_PATH="$_arx_nav_root/install-sdk/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${NAV_ROS_DOMAIN_ID:-192}"
export ROS_LOG_DIR="$_arx_nav_root/logs/ros"
export ROS_HOME="$_arx_nav_root/.ros"
# Competition chassis mask: native-layer subclasses, no additional layers.
_arx_mask_prefix="$_arx_nav_root/overlays/base_mask/install"
if [ -d "$_arx_mask_prefix/share/ament_index" ]; then
  export AMENT_PREFIX_PATH="$_arx_mask_prefix:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$_arx_mask_prefix:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$_arx_mask_prefix/lib:${LD_LIBRARY_PATH:-}"
fi
unset _arx_mask_prefix
unset _arx_nav_root
