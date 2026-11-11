#!/usr/bin/env bash
FP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
FP_BASE=/home/arx/miniconda3/envs/act
export PATH="$FP_ROOT/.venv/bin:$FP_BASE/bin:$FP_BASE/nvvm/bin:$PATH"
export CUDA_HOME="$FP_BASE/targets/x86_64-linux"
export TORCH_CUDA_ARCH_LIST=12.0 MAX_JOBS=2
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MKL_NUM_THREADS=4
export PYTHONPATH="$FP_ROOT/upstream:$FP_ROOT/upstream/mycpp/build${PYTHONPATH:+:$PYTHONPATH}"
