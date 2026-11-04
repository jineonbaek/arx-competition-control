# Source this file. It only selects the independent environment; no hardware is started.
_arx_control_root=/home/arx/arx-competition-control
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source "$_arx_control_root/workspace/install/setup.bash"
source "$_arx_control_root/.venv/bin/activate"
export ROS_LOG_DIR="$_arx_control_root/logs/ros"
export ROS_HOME="$_arx_control_root/.ros"
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPYCACHEPREFIX="$_arx_control_root/.pycache"
export PYTHONPATH="$_arx_control_root/install-python/x5:$_arx_control_root/install-python/lift:$_arx_control_root/install-python/lift/lift_controller/api/arx_lift_python:$PYTHONPATH"
export LD_LIBRARY_PATH="$_arx_control_root/install-python/x5/bimanual/api/arx_x5_src:$_arx_control_root/install-python/x5/bimanual/api:$_arx_control_root/install-python/lift/lift_controller/api/arx_lift_src:$LD_LIBRARY_PATH"
unset _arx_control_root
