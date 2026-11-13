#!/usr/bin/env bash
set -eo pipefail
nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
python3 "$nav_root/scripts/generate_sdk_exports.py"
export CMAKE_BUILD_PARALLEL_LEVEL="${NAV_BUILD_JOBS:-2}"
cmake -S "$nav_root/deps/Livox-SDK2" -B "$nav_root/build-sdk" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$nav_root/install-sdk" \
  -DCMAKE_SHARED_LINKER_FLAGS="-Wl,--version-script=$nav_root/config/livox_sdk_exports.map"
cmake --build "$nav_root/build-sdk" --parallel "$CMAKE_BUILD_PARALLEL_LEVEL"
cmake --install "$nav_root/build-sdk"
export CMAKE_PREFIX_PATH="$nav_root/install-sdk:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$nav_root/install-sdk/lib:${LD_LIBRARY_PATH:-}"
cd "$nav_root/workspace"
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy \
  -DCMAKE_CXX_STANDARD=17 \
  -DLIVOX_LIDAR_SDK_LIBRARY="$nav_root/install-sdk/lib/liblivox_lidar_sdk_shared.so"
