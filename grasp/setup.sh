#!/usr/bin/env bash
set -eo pipefail
TASK="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BASE=/home/arx/miniconda3/envs/act
mkdir -p "$TASK/upstream" "$TASK/models" "$TASK/runtime"
test -x "$BASE/bin/python"
for spec in 'graspnet-baseline 280c215129f759ed8649cb4e89fc5dfee55f4f80' 'graspnetAPI eb57dd2092d8dbe05312a29c3d0c22f3226efbfc'; do
    read -r name revision <<< "$spec"
    if [ ! -d "$TASK/upstream/$name/.git" ]; then
        git clone "https://github.com/graspnet/$name.git" "$TASK/upstream/$name"
        git -C "$TASK/upstream/$name" checkout --detach "$revision"
    fi
    test "$(git -C "$TASK/upstream/$name" rev-parse HEAD)" = "$revision"
done
"$BASE/bin/python" "$TASK/apply_compatibility.py"
if [ ! -x "$TASK/.venv/bin/python" ]; then
    "$BASE/bin/python" -m venv --system-site-packages "$TASK/.venv"
fi
export PATH="$TASK/.venv/bin:$BASE/bin:$BASE/nvvm/bin:$PATH"
export CUDA_HOME="$BASE/targets/x86_64-linux"
export TORCH_CUDA_ARCH_LIST=12.0 MAX_JOBS=2
python -m pip install --index-url https://pypi.tuna.tsinghua.edu.cn/simple -c "$TASK/constraints.txt" ninja requests open3d numpy
python -m pip install --no-build-isolation --no-deps "$TASK/upstream/graspnet-baseline/pointnet2"
python -m pip install --no-build-isolation --no-deps "$TASK/upstream/graspnet-baseline/knn"
python -m pip install --index-url https://pypi.tuna.tsinghua.edu.cn/simple -c "$TASK/constraints.txt" -e "$TASK/upstream/graspnetAPI"
python "$TASK/download_weights.py"
python "$TASK/test_install.py"
