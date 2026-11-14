# CNOS 离线候选复核实验

只读取 2026-11-14 自动抓取保存的图片、检测框和电池 CAD。不连接相机、网页控制接口、ROS 或机械臂。
不改变 D435、D405 或任务链路；输出全部候选的匹配分数和排序，不设置线上接受/中止阈值。

## 测试方法

- CNOS 官方源码：<https://github.com/nv-nguyen/cnos>，固定提交 `298d1f3366171464ca271659f0e2f7a6eb8e39b4`。
- DINOv2 官方源码：<https://github.com/facebookresearch/dinov2>，固定提交 `7764ea0f912e53c92e82eb78a2a1631e92725fc8`。
- 使用 CNOS 的 `CustomDINOv2`、`PairwiseSimilarity` 和 `CNOS.find_matched_proposals`，默认 ViT-L/14、224 像素、42 视角、top-5 余弦相似度均值。
- 模板采用官方 `pyrender.render` 和 level-0 姿态，CAD 为已有 `battery-print.stl`，毫米转米。
- 两组模板：完整 CAD；沿物体 Z 中点截取的端部 CAD。端部组是实验性局部模板，不是无人机遮挡仿真，也不是 CNOS 官方默认模板。
- 模板裁剪、归一化顺序沿用官方 BOP loader；候选使用官方 CNOS descriptor 的掩膜裁剪流程。
- 候选框沿用冻结 Grounding-DINO 输出，使用同一现场已有 SAM2 权重重新生成掩膜。因此测试的是 **CNOS 匹配模块作为二次复核器**，不是原版 SAM 全图自动提议的端到端基准。
- 加入人工框电池和固定夹爪作诊断对照；这些框不计入原检测器成功率或自动排序命中率。
- 图像真值区域由人工查看照片指定；`overlap_label` 中的几何规则仅用于离线标注，不用于机器人接受/拒绝目标。
- 一张无目标电池的负例仍输出分数，不强行给出“通过/不通过”。分数不是电池身份概率。

## 环境隔离

使用已有 FoundationPose Python 的 Torch/CUDA 和 pyrender，只把缺少的包安装到本实验 `runtime/deps`：

```bash
FP=/home/arx/arx-foundationpose/foundationpose/.venv/bin/python
$FP -m pip install --no-deps --target vision/offline_cnos/runtime/deps \
  -i https://pypi.tuna.tsinghua.edu.cn/simple \
  pytorch-lightning==2.5.6 torchmetrics==1.8.2 lightning-utilities==0.15.2 \
  hydra-core==1.3.2 antlr4-python3-runtime==4.9.3 ruamel.yaml==0.19.1 \
  distinctipy==1.3.4 pycocotools==2.0.11 shapely==2.2.0
```

官方权重地址：<https://dl.fbaipublicfiles.com/dinov2/dinov2_vitl14/dinov2_vitl14_pretrain.pth>。
依赖、第三方源码、权重和中间文件只放在本地实验目录，不提交 Git、不修改生产环境。

```bash
PYTHONPATH=vision/offline_cnos/runtime/deps \
PYOPENGL_PLATFORM=egl XFORMERS_DISABLED=1 HF_HUB_OFFLINE=1 \
flock -n /tmp/arx-grasp-integration-gpu.lock "$FP" vision/offline_cnos/evaluate.py \
  --upstream /home/arx/cnos-upstream-offline-20261114 \
  --dino-source vision/offline_cnos/runtime/dinov2 \
  --weights vision/offline_cnos/runtime/dinov2_vitl14_pretrain.pth \
  --mesh /home/arx/arx-competition-control/web/cad/gripper-v2/battery-print.stl \
  --output vision/offline_cnos/runtime/evaluation_v2
```

运行前需取得官方源码和完整权重；使用 `--prepare-only` 可只制作掩膜与模板。
执行 `python -m unittest discover -s vision/offline_cnos -p test_offline.py` 检查离线数据标注与导入边界。

## 输出与边界

`manifest.json` 保存样本来源、原选择和输入 SHA256；`templates/mesh.json` 保存 CAD 校验与渲染说明；
每张图片的 `scores.json` 保存所有候选分数、原候选排序、目标是否进入候选集；`ranking.png` 展示候选；
`results.json` 汇总权重 SHA256、上游版本、匹配耗时与全部结果。

这是少量现场图片的可行性实验，不是训练/测试独立划分的精度评测，不能据此确定生产阈值。
仅检查本次打印件和观察角度；最终实物材质、额外遮挡和不同摆放仍需单独验证。
