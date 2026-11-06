#!/usr/bin/env bash
set -eo pipefail
TASK="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MKL_NUM_THREADS=4
exec "$TASK/.venv/bin/python" "$TASK/run_official_demo.py" "$@"
