#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/sensor_env.sh"
exec rviz2 -d "$ROOT/single_bottom/bottom_slam.rviz"
