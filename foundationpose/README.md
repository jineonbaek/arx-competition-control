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

## 怎么运行

    cd /home/arx/arx-foundationpose/foundationpose
    bash run-demo.sh --frames 5

默认使用已下载的官方 mustard0 前五帧。每次新建 runtime/demo-时间戳/，保存 overlay-0000.png 等效果图、pose-0000.txt 等 4×4 位姿矩阵及 result.json；不弹窗口、不操作相机、不发机器人命令。不能把返回矩阵直接发给机械臂。

重新下载相同官方数据子集：

    source env.sh
    python download-demo.py

下载器从官方 Google Drive ZIP 通过 HTTP Range 取出同一模型和五帧，逐文件校验 ZIP CRC，SHA256 记录在 demo-sources.json；子集约 10.2 MiB，无须完整下载 345 MiB 视频包。

实测验证：check_env 全部核心检查通过；首次定位约 3.420 s，后四帧分别 56.9 / 15.1 / 14.9 / 15.5 ms（准确值见 validation-results.json）；PyTorch 峰值已分配显存约 2942.9 MiB。时间包含 CUDA 同步，排除加载模型、磁盘读取和效果图保存，首帧包含冷内核开销。五帧样例不能作为电池精度或实时帧率结论。

## 换成我们的电池

FoundationPose 需要相匹配的 CAD、彩色图、对齐彩色的深度图、该分辨率的彩色内参，以及首帧电池二值遮罩。遮罩指定完整电池区域，而非仅提手；后续同一连续序列使用跟踪。

已把当前电池打印 STL 按 0.001 转为米，放在 runtime/battery/battery-m.obj，来源和尺寸见同目录 mesh.json。这个步骤没有改变原始 STL。

准备文件格式与官方示例相同的电池序列：
- rgb/0000.png 等彩色图（原尺寸）；
- depth/0000.png 等 uint16 深度图，数值单位毫米，须与彩色对齐；
- masks/0000.png 首帧二值电池遮罩，非零表示电池；
- cam_K.txt 为对应彩色相机的 3×3 内参矩阵。

    bash run-demo.sh --scene /绝对路径/电池序列 --mesh runtime/battery/battery-m.obj --frames 1

结果 T_camera_from_object 把 CAD 中的米制点转换到相机光学坐标（X 右、Y 下、Z 前）。接入机器人时，还要使用拍照时的手眼外参与真实腕位，转换到 base_link。9983 的 D405 相机卡片现已提供 FoundationPose 位姿验证二级页面（集成源码在 main 的 web/ 下），尚未验收电池实拍精度，也未生成或执行机械臂动作。

当前可先继续用既有 D405 提手预览。下一阶段把冻结 RGB-D 与分割遮罩送入此独立环境，检查 CAD 叠加和提手坐标，再讨论运动。


## D405 网页验证入口

http://192.168.58.14:9983/foundationpose.html

点击「采集 RGB-D」后，D405 临时开启深度，采集结束恢复原开关。框住完整电池，再点击「估计位姿」，后台运行本目录的 run-snapshot.sh。先用 CPU GrabCut 提取轮廓，再按 D405 内参同步校正 RGB-D 与轮廓畸变，调用 FoundationPose，并把当前电池 CAD 轮廓投回原图。页面显示原图、实际深度、前景轮廓、CAD 叠加、整车 base_link 下的中心位置与姿态。

模型按次加载，进程完成后退出释放显存。关闭页面或「取消计算」可结束当前任务，最长 90 秒后强制停止。打开页面和采集快照都不加载 GPU 模型。web 服务不导入 torch，只通过子进程使用本独立环境。每次推理的 NPZ 与结果保存在主工作区 web/hardware/runtime/foundationpose/，与提手示教数据隔离。

电池 STL 默认读取本工作树 web/cad/gripper-v2/battery-print.stl，按 0.001 转成米；结果包含 mesh_sha256，便于核对。UI 的物体坐标显示 CAD 包围盒中心；T_base_from_object 的原点仍是 CAD 原点，二者不能混用。CAD 叠加效果和遮罩必须通过实拍核验后，才适合做抓取目标计算。
