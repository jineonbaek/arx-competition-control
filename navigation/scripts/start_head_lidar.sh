#!/usr/bin/env bash
set -eo pipefail
export ARX_LIO_SENSORS=top
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$root/scripts/start_dual_lidar.sh"
