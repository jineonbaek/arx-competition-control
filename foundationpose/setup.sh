#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
FP_BASE=/home/arx/miniconda3/envs/act
clone_pin() {
  local repo_url="$1" repo_dir="$2" revision="$3"
  if [ ! -d "$repo_dir/.git" ]; then
    git clone --no-checkout --filter=blob:none "$repo_url" "$repo_dir"
    git -C "$repo_dir" checkout --detach "$revision"
  fi
  test "$(git -C "$repo_dir" rev-parse HEAD)" = "$revision"
}
clone_pin https://github.com/NVlabs/FoundationPose.git upstream a1b694b83e633c2cb6115b9063d940a687759392
clone_pin https://github.com/facebookresearch/pytorch3d.git upstream/pytorch3d e4ad6e5687a360ccdc6ebd91348e8dbbac97bf31
clone_pin https://github.com/NVlabs/nvdiffrast.git upstream/nvdiffrast 253ac4fcea7de5f396371124af597e6cc957bfae
test -x .venv/bin/python || "$FP_BASE/bin/python" -m venv --system-site-packages .venv
source ./env.sh
python -c 'import torch; assert torch.__version__=="2.10.0+cu128"'
python -m pip install --index-url https://pypi.tuna.tsinghua.edu.cn/simple -r requirements-local.txt
python -m pip install --no-build-isolation --no-deps ./upstream/pytorch3d ./upstream/nvdiffrast
cmake -S upstream/mycpp -B upstream/mycpp/build -DCMAKE_BUILD_TYPE=Release   -Dpybind11_DIR="$(python -m pybind11 --cmakedir)" -DPYTHON_EXECUTABLE="$FP_ROOT/.venv/bin/python"
cmake --build upstream/mycpp/build --parallel 2
mkdir -p weights demo_data runtime
test -e upstream/weights || ln -s "$FP_ROOT/weights" upstream/weights
test -e upstream/demo_data || ln -s "$FP_ROOT/demo_data" upstream/demo_data
python download-weights.py
python upstream/check_env.py
