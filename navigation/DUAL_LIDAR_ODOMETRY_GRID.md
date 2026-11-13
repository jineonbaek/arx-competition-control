双 MID360S 内置 IMU 里程计与二维栅格地图
====================================

所有运行命令在 NUC 的 `/home/arx/arx-competition-control/navigation` 下执行，先 `source sensor_env.sh`。只接收传感器、计算和发布定位/地图；没有底盘控制、CAN、运动或停车命令。

当前实现
--------

- 一个 Livox SDK 驱动分别发布两台 MID360S 的点云与内置 IMU；每台独立运行 FAST-LIO。点云/里程计 10 Hz、内置 IMU 200 Hz。
- `/dual/arm/odometry` 与 `/dual/top/odometry` 包含原始滤波器线速度和去偏角速度。位姿在各自 `arm_lidar_odom` / `top_lidar_odom` 世界坐标中，速度在各自 `arm_lio_imu` / `top_lio_imu` 子坐标中，符合 [ROS Odometry 消息约定](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/nav_msgs/msg/Odometry.msg)。单位 m/s 和 rad/s。
- `odometry_smoothed` 为对应的速度平滑输出，时间常数 0.2 秒；位姿保持原始值，协方差保守地沿用输入。存在滤波延迟，未经移动速度真值验证。
- 可选的静止 IMU 重力对齐让两个世界的 Z 轴朝上。两套世界的原点、水平朝向仍各自独立，不能直接合并点云或平均位姿。
- IMU 时间回退或间隔超过 1 秒会退出旧 LIO；任一 LIO 退出时，本双雷达 launch 关闭，需静止后重新初始化。断流期间不会补发旧速度。没有无缝断电续图。
- 三维地图投影成 `nav_msgs/OccupancyGrid`，分辨率 0.10 m。地面支持点为空闲，0.10–2.0 m 高度内点为障碍，其他保留未知；有轨迹时，只沿观测地面射线清空，并在投影障碍处停止。不是全图凸包填白。

输出与查看
----------

| 内容 | 话题 | RViz Fixed Frame |
| --- | --- | --- |
| 臂上雷达原始/平滑里程计 | `/dual/arm/odometry`、`/dual/arm/odometry_smoothed` | `arm_lidar_odom` |
| 顶部 IP 雷达原始/平滑里程计 | `/dual/top/odometry`、`/dual/top/odometry_smoothed` | `top_lidar_odom` |
| 臂上雷达实时二维图 | `/mapping/grid` | `arm_lidar_odom` |
| 顶部 IP 雷达实时二维图 | `/mapping/top_grid` | `top_lidar_odom` |
| 上一轮移动测试的保存图 | `/mapping/saved_grid` | `lidar_odom` |

`arm` 暂按 192.168.1.3，`top` 暂按 192.168.1.164 命名，物理对应仍需现场核对。不能用这些名称代替标定。

在 NUC 图形终端选择一种视图：

```bash
rviz2 -d config/dual_lidar/saved.rviz
rviz2 -d config/dual_lidar/arm.rviz
rviz2 -d config/dual_lidar/top.rviz
```

保存的移动测试图：`maps/arm_manual_203414_grid.yaml` / `.pgm`，254×131、0.10 m/格。其原三维地图与轨迹为 `reports/manual-mapping-20261104-203414/map-after.pcd` 和 `trajectory.csv`。当前坐标系重新初始化过，这份旧图不能与新实时图叠加。`maps/dual_arm_live_grid.*` / `dual_top_live_grid.*` 为新会话实时图。JSON 记录地面拟合、坐标系、已知/未知格统计与验证状态。

需要启动保存图服务时执行 `ros2 launch config/dual_lidar/saved_grid.launch.py`。该服务仅加载地图，不启动 Nav2 或控制器。

启动、停止与重现
----------------

停止现有单雷达建图后，确认底盘、升降均静止，再运行：

```bash
bash scripts/start_dual_lidar.sh
```

记录目录指针：`reports/latest-dual-lidar-path.txt`。只停止本导航会话：

```bash
python3 scripts/stop_dual_lidar.py
```

单/双雷达启动共享 `runtime/arm-mapping.lock`；本机接收地址 192.168.1.50 和两台有线直连 IP 都必须存在。不要同时启动 sensors_only 驱动或另一个 SDK 进程。

`scripts/patch_fastlio_dual.py workspace/src/FAST_LIO` 对暂存源码增加独立 frame 参数、速度/协方差输出、重力对齐和时间保护；上游原始检出保持不变。`prepare_workspace.py` 已接入补丁。补丁后重新 colcon build fast_lio；已有运行进程需重新启动才使用新二进制。

离线重建保存的二维图：

```bash
python3 scripts/pointmap_grid.py \
  --pcd reports/manual-mapping-20261104-203414/map-after.pcd \
  --trajectory reports/manual-mapping-20261104-203414/trajectory.csv \
  --frame lidar_odom --output maps/arm_manual_203414_grid
```

关键待完成项
------------

1. 当前两路 IMU 里程计不是底盘 `base_link` 里程计，也没有双雷达融合。缺少实测外参，不能虚构平移/朝向或把倒装轴当作底盘轴。
2. `config/dual_lidar/body_extrinsics.template.yaml` 中填写经过验证的 `T_base_imu`，确认升降锁定后，`python3 scripts/body_odometry.py --calibration <文件>` 可输出杆臂修正后的两套独立底盘估计与主源 `/localization/odometry`（含 TF）。旋转时减去传感器杆臂速度；主源只是选择，不是融合。
3. 真正双雷达融合还需共同世界对齐、时间同步误差测量、相关误差处理、故障选择与移动验收；外参有效后才能接入。
4. 地面高度来自自动拟合，不等同于实测。两台雷达当前拟合高度约 0.2–0.3 m，用户已确认顶部倒装雷达当前确实在此临时低位；此前最终安装的底面高度 1.267332 m 不适用于当前测试。移回最终位置后须重建/重新定位、核验地面平面并重测外参。可用 `--floor a b c` 指定已验证的 `z=a*x+b*y+c`。所有导出 JSON 均标记 `navigation_validated:false`。
5. 栅格只完成生成、消息发布和 Nav2 map_server 加载。尚未验收地面/障碍阈值、自由空间连通性、动态障碍残影、车体尺寸、地图定位与底盘执行接口，不能据此启动自主运动。

验证
----

运行 `python3 staging/dual_lidar/tests/test_geometry_grid.py`（6 项几何/投影测试）、`python3 scripts/test_imu_gap_guard.py`（隔离合成 IMU 时间中断测试）、`python3 scripts/check_dual_lidar.py --output <报告.json>`（纯订阅帧率、速度和位姿检查）。报告和地图均留在 NUC，不导出现场点云、图像或录包。


本轮最终验证目录：/home/arx/arx-competition-control/navigation/reports/dual-lidar-20261104-211937。dual-validation.json验证20秒双雷达点云/里程计10Hz、内置IMU200Hz；final-acceptance.json验证速度平滑、位姿/速度协方差对称半正定、三路地图；publisher-audit.json记录无运动发布者。最新独立PCD软链接maps/dual_arm_latest.pcd和dual_top_latest.pcd。


2026-11-05头部主源：静止后运行bash scripts/start_head_lidar.sh。SDK登记两台IP，但仅头部进入LIO/速度平滑/二维图；LIO延迟8秒避开初始化数据过渡，断流保护保留。查看rviz2 -d config/dual_lidar/top.rviz，输出/dual/top/odometry与/mapping/top_grid。当前报告/home/arx/arx-competition-control/navigation/reports/dual-lidar-20261105-104828。底盘外参与导航闭环仍待标定。


头部SLAM三维查看配置：rviz2 -d config/dual_lidar/head_slam.rviz。包括三维点云地图、当前扫描、轨迹和可选二维投影层。只显示，不发送控制。
