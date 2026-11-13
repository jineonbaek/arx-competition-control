# 双 MID360S 避障与北极熊 RM2024 代价地图

底盘 MID360S 持续提供里程计、建图扫描和避障观测。头部 MID360S 在整车停稳且配准通过后补充避障。NUC 桌面的「雷达开关」独立控制两台雷达的数据接收，底盘 IMU 保持开启。建图入口用于人工建图；保存地图导航入口可执行 RViz 单点或多点目标。Publish Point 用于预览。回放不启动底盘执行器。

## 复用与分工

- 复用北极熊工作空间的 linefit 地面分割及 pointcloud_to_laserscan。
- 复用 Jazzy SLAM Toolbox：扫描累积、扫描匹配、回环、输出 `/pb/map`。不再用 `/mapping/grid` 作为正式持久地图。
- 全局：非滚动 StaticLayer + STVL + InflationLayer；局部：10×10m 滚动 VoxelLayer + InflationLayer，局部扫描源为 `/pb/scan` 和 `/pb/head/nav_scan`。
- SmacPlannerLattice 只调用 ComputePathToPose。RViz 的 2D Goal Pose 在地图上按下并拖动以指定最终朝向，Publish Point 保留原有点选行为。
- 本车新增部分：光学中心/重力对齐桥接、地面高度复核、盲区遮挡保护、数据新鲜度检查、图形驱动兼容显示。

雷达输入保留车体模型自过滤。历史动态障碍由 STVL 在实际三维体素上根据看穿证据和命中时间管理。底盘投影只用于既有的未知格处理。

## 路径预览与贴桌筛选

蓝线是规划器返回的候选路径。节点用请求时刻的全局代价地图，沿整条路径检查带余量的 CAD 多边形；琥珀色球表示首个待确认的未知格，红球表示碰到障碍格。结果在活动报告目录的 `plan-status.json` 和 `plan-events.jsonl`，最近 12 张请求地图按序号循环保存。地图更新时间与规划器内部实际检查时间可能略有差异。只有 `path_ready` 通过本次栅格检查。

`/pb/plan_goal` 和 `/pb/dock_goal` 都接收 map 系的 PoseStamped。后者表示期望的**底盘中心**及车头朝向，按反向每 5 cm 搜索到 60 cm。它先检查目标轮廓，再向 Nav2 请求路径，并检查沿途轮廓。抓取可达性需用实际电池位姿及当前机械臂姿态单独验证。模拟控制验证脚本 `offline/omni_check.py` 只在隔离域 195 与合成地图运行。

## 当前几何与处理

底雷达正装；安装面中心 `[0.185330,-0.000003,0.183]`m，光学中心Z为0.230m。保留现有CAD车体过滤及实际整车约0.529m保守碰撞半径，不因近墙或无路径而缩小机器人。

感知盲区为光学中心水平距离0.35m，独立于 FAST-LIO 的0.20m设置。此数值来自旧录包候选比较，不代表盲区里没有障碍。删除近点前保存其光学方位；相同时间戳扫描的对应角度及左右各一束输出NaN，防止远点替代近回波后产生穿透清障。缺失掩码也按无效扫描处理。CAD已提前滤掉的自体点、未采到的遮挡和不同高度之间的遮挡，不能凭此完整恢复。

建图扫描仅保留光学高度[-0.18,0.45]m；避障扫描为[-0.20,1.10]m。前者减少桌面/高处投影对地面地图的影响，但会漏掉悬空障碍，后者/STVL须保留高处碰撞检查。此设置以平地和当前姿态为假设，机械臂变姿后须重新审计轮廓。

全局实时障碍由 STVL 的三维 OpenVDB 网格管理，局部图使用原生 VoxelLayer。两图均无独立 confirmation 层。对于同一 STVL 体素，同一雷达连续 4 次有效看穿（不限时间），或最近 1 秒有效观测至少 5 次且看穿占比达到 80%，即可提前清除。未观测到该高度的帧跳过，命中打断连续看穿。遮挡期间从最后真实命中时间计算 15 秒线性衰减。新鲜射线确实到达体素且它位于有效视场时，STVL 使用 5.0 的加速系数。再次命中按新位置、高度和采集时间标记。头部关闭后停止提供新观测，已有体素继续老化。
 
NUC 使用 `staging/effective-stvl/install` 中的插件覆盖。用 `polarbear_adapt/stvl/build.sh <含 2.5.5 标签的上游 STVL 仓库> <新的构建目录>` 可重建，原版系统 ROS 包保留。

建图和已保存地图导航都用 `/pb/reference_map` 作原始底图，规划使用 `/pb/map`。底图旧格只使用 1 秒、至少 5 帧、80% 的判据。确认后，在当前地图会话的规划副本中屏蔽该旧格；普通新回波交由动态层表达，不恢复原来的静态占用。原始 SLAM 图和定位用参考图独立保留。

## 验证与运行

单元测试：
```bash
PYTHONPATH=navigation_setup/polarbear_adapt python3 -m unittest discover -s navigation_setup/polarbear_adapt/tests -p 'test_*.py'
```

NUC隔离回放使用 `replay.sh`：仅domain199，播放cloud_body和base odometry，重建传感器TF；不播放原录包TF，避免旧地图TF与新SLAM争用。不重新估计LIO，因此回放不能证明更改LIO盲区后的效果。必须设置新的 `ARX_ADAPT_RUN` 与原记录 `ARX_BAG_RUN`，不能覆盖旧回放。

实机启动 `start.sh` 需要既有底盘LIO、`ARX_REPLAY=0`、新的报告目录。该脚本不会初始化IMU；IMU重启必须先确认静止。`view.sh` 打开 RViz，地图+X位于画面上方。正式切换前先检查现有预览进程，避免双地图/双代价地图服务。

## 未宣称完成

未验证近桌自动靠泊、导航执行、所有近车回波清除、几何地图精度、完整房间可观测性、动态障碍在静态图中的自动遗忘，以及遮挡安全的FOV加速。灰色未知区保持未知，不伪造自由空间来展示路径。

## 上游依据

- 北极熊仓库：https://gitee.com/SMBU-POLARBEAR/RM2024_SMBU_auto_sentry_ws （本地读取rm_navigation_ws配置，按本车尺寸适配）
- STVL2.5.5缓冲实现：https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/2.5.5/spatio_temporal_voxel_layer/src/measurement_buffer.cpp
- STVL原理与参数：https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/ros2/README.md

“自然衰减”“有限射线清空”“持久地图更新”是三套不同规则。未知格不会因STVL计时到期就自动成为可行驶区域；局部滚动窗口离开的内容也不等于SLAM地图丢失。验收须分别看`/pb/map`、global、local，不能只看叠加截图。

### RViz 拉箭头与单次跟踪

在 NUC 的导航 RViz 中选择 **2D Goal Pose**。在目标车心位置按下鼠标左键，拖向期望车头方向并松开。蓝线显示规划器返回的候选路径。查看目标旁的状态文字：Path ready 表示通过全局轮廓检查，Start occupied 表示起点被占用；黄色或红色球标出问题格。全局规划地图只把刚性底盘实体投影完全覆盖的未知格改为自由。

在雷达、里程计、局部与全局代价地图运行后，可以单独启动一次速度预览，再拉一个新箭头：

```bash
cd /home/arx/arx-competition-control/navigation
source sensor_env.sh
python3 polarbear_adapt/follow_preview.py
```

预览速度为 `/pb/chassis_preview`（TwistStamped，base_link）。90 秒内等待新箭头和新鲜路径。程序按全向底盘跟踪路径，平移合速度限 0.08 m/s，角速度限 0.18 rad/s，经过实际速度标定后再调整。实车模式需要操作员单独启动同一程序并显式指定 `--drive`，由本机网页控制台接收速度请求。每次只执行一个目标。失去雷达、里程计、TF 或代价地图，路径覆盖未知或致命障碍，收到新目标，或运行中断均请求停车。车载控制台的指令租约为 0.3 秒。抓取载物时需先验证整车外伸轮廓。这一模式需要另做实车低速与停车验证。
