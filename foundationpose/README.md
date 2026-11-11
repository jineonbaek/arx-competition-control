# ROG FoundationPose 独立环境

工作树 /home/arx/arx-foundationpose，分支 codex/foundationpose，从 main 0dc075e 创建。只运行离线位姿估计，不连接机器人驱动。

## 安装与来源

已在 RTX 5070 Laptop 8 GB、Ubuntu 24.04 上安装。独立 .venv 复用现有 ACT 环境的 PyTorch 2.10.0+cu128 和 CUDA 12.8 编译工具；额外依赖与扩展只写入本目录。未修改共享 ACT 环境、9983 或机器人驱动。

- 官方源码 https://github.com/NVlabs/FoundationPose ，固定提交见 sources.json。
- Python 依赖使用清华 PyPI https://pypi.tuna.tsinghua.edu.cn/simple ，局部版本见 requirements-local.txt。
- 模型通过 https://hf-mirror.com/gpue/foundationpose-weights 下载，固定版本 42d49e0633d245b3cf4dea6b1e7ec2b31d5b7654。两组权重共 258449498 字节（约 246.5 MiB），文件大小与镜像仓库 LFS SHA256 校验一致。原作者 Google Drive 直连下载超时，因此未独立核对完整权重与 Google Drive 的字节一致性。来源和 SHA256 见 weight-sources.json。
- CUDA 扩展 PyTorch3D、NVDiffRast 为 sm_120 编译。mycpp 使用本机已有 Eigen、Boost、CMake 和独立 pybind11 构建。
- 模型基于 CAD 推理，不包含可选的 BundleSDF/NeRF 无模型重建。

重建（本机需保留 ACT 基础环境和系统 C++ 开发依赖）：

    cd /home/arx/arx-foundationpose/foundationpose
    bash setup.sh

安装自检：

    source env.sh
    python upstream/check_env.py --demo-data

官方 check_env 对缺少权重或示例只发警告；实际可运行性以 demo 成功输出为准。源码和权重不常驻运行，启动 demo 时才加载模型。

本目录的 upstream、weights、.venv、demo_data、runtime 均忽略，不提交二进制、下载内容或日志。
