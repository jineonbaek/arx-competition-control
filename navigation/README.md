# ARX ROS 2 Jazzy 导航工作区

NUC 路径：`/home/arx/arx-competition-control/navigation`。

目标：先完成单台固定倒装 MID360S 的建图、已知地图定位与 Nav2 导航，随后接入第二雷达、D435 和电池识别。当前为基础适配，不代表传感器或底盘实车验收完成。

## 目录

- `upstream/`：用户指定的北极熊总仓库，导航子模块按总仓库固定提交检出。原始源码保持不改动；未拉取无关的决策、视觉子模块及旧 TEB/旧 Livox 驱动。
- `deps/`：官方 Livox SDK2 和支持 MID360S/Jazzy 的 `livox_ros_driver2` 1.2.6。
- `workspace/src/`：独立适配副本：FAST-LIO、Point-LIO、Livox 驱动、地面分割、点云转扫描、IMU 滤波及 `arx_nav_bringup`。
- `overlay/`：自行维护的 ARX 启动包原始文件；通过 prepare 脚本首次复制到 workspace。
- `config/robot.template.yaml`：待填写的实际机械安装参数；不包含猜测的外参或默认雷达 IP。
- `scripts/`：源码准备、编译和有限离线验证。
- `install-sdk/`、`build-sdk/`：SDK 的本目录安装及编译产物，不覆盖系统 `/usr/local`。
- `reports/`：固定版本、源码差异、依赖安装、编译和离线验证报告。
- `Dockerfile`：Ubuntu 24.04 + ROS 2 Jazzy 构建入口；需先有完整 deps 和已准备的 workspace/src。

## 本次适配

1. 选取实车所需包，排除 Gazebo Classic、旧 TEB、STVL 和哨兵自旋速度转换；保留上游工程供对照。
2. LIO 编译标准统一为 C++17；补齐 Point-LIO 的 PCL 查找，并按实际编译结果保存兼容修正。
3. 官方 Livox 驱动保留 CustomMsg 为 LIO 输入；另加轻量转换节点为感知提供 PointCloud2。该转换不替换 LIO 输入，不丢弃其原消息的逐点时间。
4. Jazzy Nav2 使用 Navfn + DWB，更新进度检查器、行为插件及 BT 配置，统一 `base_link`；代价地图使用标准 ObstacleLayer。
5. 输出仅为 `/navigation/cmd_vel`，没有接入 ARX 的运动指令话题。候选限速为 0.2 m/s、0.3 rad/s，侧移先禁用；不声称这些是厂商验证的限值。
6. 地面分割使用 `odom` 的重力对齐旋转；缺失 TF 时丢弃该帧，不把倒装点云当正装处理。修正原移植中三个未声明的地面参数。
7. 雷达底面离地 1.267332 m 只作为测量记录，不直接充当雷达坐标原点的离地高度或 TF z 值。

## 构建和离线验证

已经准备好的 workspace 不要重复运行 prepare 脚本，以免覆盖本地修改。

```bash
cd /home/arx/arx-competition-control/navigation
bash scripts/build.sh
source env.sh
python3 scripts/check_offline.py
ros2 launch arx_nav_bringup bringup.launch.py --show-args
```

`source env.sh` 仅选择本目录环境，默认 `ROS_DOMAIN_ID=192`，与已有控制系统的 191 区分。可用 `NAV_ROS_DOMAIN_ID` 显式指定。它不启动节点、驱动或运动。

## 配置实际安装

复制模板为 `config/robot.yaml`，填写：底盘到雷达原生坐标系的 XYZ/RPY、base_link 离地高度、雷达坐标原点离地高度、MID360S 的内部 LiDAR–IMU 外参，以及底盘真实足迹。内部外参不能用倒装安装角替代。

现场接入前，还需要确认 MID360S 点云字段、线号、IMU 单位/频率与时序兼容旧 LIO 预处理；原 MID360 的参数没有被当作已验证的 MID360S 标定。

驱动使用官方配置结构，填写传感器及接收网卡的实际 IP；保持驱动中的点云额外旋转/平移为零，在 TF 中描述安装关系。

## 数据回放入口

完成上述几何参数后，在同一导航环境和域中回放只含原始 `/livox/lidar` CustomMsg 与 `/livox/imu` 的 rosbag2：

```bash
ros2 bag play /absolute/path/to/bag --clock
ros2 launch arx_nav_bringup bringup.launch.py \
  robot_config:=/home/arx/arx-competition-control/navigation/config/robot.yaml \
  lio:=fastlio use_sim_time:=true start_driver:=false start_navigation:=false
```

不要同时回放已由本链路生成的 TF、点云转换结果、/scan 或 /map，以免重复发布。当前固定外参模式仅用于刚性固定雷达；不处理升降平台动态外参。

首次建图仍以 FAST-LIO 为默认；可切换 `lio:=pointlio`。保存 slam_toolbox 的栅格地图和 `.posegraph/.data` 后，可使用 `mode:=localization map:=<地图前缀绝对路径>`。当前适配入口选择 slam_toolbox 定位；AMCL/ICP 等原仓库实现仍保留在 upstream，尚未接入此入口。

设置 `start_navigation:=true` 会运行 Nav2，但命令仍仅输出到 `/navigation/cmd_vel`。暂未提供通往 ARX 底盘的桥接；还需适配 LIO 位姿/速度到 base_link 里程计、实测足迹、控制语义、速度限制与停止行为。

真实雷达入口需填写 `livox_config_file`，并显式设置 `use_sim_time:=false start_driver:=true`；本次不会执行此入口。

## Docker

```bash
sudo docker build --platform linux/amd64 -t arx-navigation:jazzy .
```

NUC 采用 amd64；Apple Silicon 日常开发可尝试原生 arm64 构建，但当前编译结果只覆盖 NUC。Docker 镜像下载与构建结果以 reports 为准；生成 Dockerfile 不代表镜像已经构建成功。构建时的包版本及源码版本分别记录在 installed-ros-packages.txt 与 source-lock.json，尚未锁定所有传递 apt 依赖。

## 验证范围

离线验证不运行硬件驱动、不实例化 SDK 控制对象、不执行 launch 的节点动作。编译通过与离线配置检查不代表真实数据下的定位、导航效果或实时性能通过。传感器目前未接入，机器人安装参数尚未完成。
