#!/usr/bin/env bash
set -eo pipefail
nav=$(cd "$(dirname "$0")/../.." && pwd)
src=${1:?Supply the upstream STVL git checkout}
out=${2:?Supply a fresh build directory}
mkdir -p "$out/src"
git -C "$src" archive 2.5.5 spatio_temporal_voxel_layer |tar -xf - -C "$out/src"
(cd "$out/src" && patch -p1 <"$nav/polarbear_adapt/stvl/evidence.patch")
cp "$nav/polarbear_adapt/stvl/effective_grid.cpp" "$nav/polarbear_adapt/fast_rays.cpp" "$out/src/spatio_temporal_voxel_layer/src/"
source "$nav/sensor_env.sh"
cd "$out"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-select spatio_temporal_voxel_layer --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
