#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source sensor_env.sh
exec rviz2 -d "$PWD/static_preview/preview.rviz"
