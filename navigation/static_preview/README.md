# 底盘单 MID360S 静态导航预览

已批准范围：原地采样、地面/障碍处理、二维地图、车体轮廓、代价地图和仅计算路径。没有 controller_server、bt_navigator、behavior_server、速度桥或CAN接口。头部雷达、D435不参与。

## 当前链路与开源复用
- 原有 Livox SDK/ROS驱动 → 车体网格过滤 → FAST-LIO，保持运行未重新初始化。
- FAST-LIO去畸变 cloud_body（IMU原点）→ optical-origin重力对齐适配 → 北极熊 linefit_ground_segmentation_ros。
- 地面安全检查将超过名义地面3cm的误分类点重新归入障碍；地面高度按当前名义安装，尚不是现场平面精确标定。
- 北极熊 pointcloud_to_laserscan（Jazzy现有包）：实际 bottom_livox 固定传感器坐标，0.2–8m、720束、光学系高度[-.20,1.1]m。无回波统一NaN；Nav2 inf_is_valid=false，不用缺失回波清空未知区。
- 北极熊支持的 SLAM Toolbox（Jazzy上游包）生成 /preview/slam_map，独立 preview_map→odom；现有 /mapping/grid 保留做LIO投影对照。
- 北极熊 Nav2 配置方式＋上游 ObstacleLayer、InflationLayer、NavFn：局部10×10m、全局预览16×16m，均5cm栅格。STVL本机未安装，未强行新增依赖。
- NavFn ComputePathToPose仅产生路径，默认禁止未知区。单独RViz Publish Point客户端；不调用NavigateToPose。

## 重要界限
全局预览是实时观测窗口，不是完整保存地图导航。SLAM地图仅展示/对照，不作为当前规划的StaticLayer：静止时SLAM运动阈值通常不接收新位姿扫描，地图重新发布不等于新增覆盖，历史障碍也不能被max组合的实时层擦除。现有LIO投影地图仍实时更新。避障实测物体移除验收未完成。
实际CAD凸包作为黄色轮廓；NavFn采用覆盖整车的保守圆（由缓存计算，见footprint.json，约0.529m含3cm余量）。关闭圆形footprint clearing，避免把轮廓外、圆内的真实障碍误清除。机械臂/升降假设保持交付模型零位，关节姿态变化后必须更新。
底盘仍不能自动行驶。画出路径也不代表三维净空或动力学可行性已验收。

## 查看和选择预览目标
在NUC图形桌面：
```bash
bash /home/arx/arx-competition-control/navigation/static_preview/view.sh
```
当前RViz已打开。固定坐标preview_map；显示当前点云、障碍、二维图、CAD轮廓、保守圆和全局实时代价地图。使用工具栏 **Publish Point** 点击目标，仅计算路径；成功后约每2秒重新计算。无可行路径会清空蓝线并在运行目录plan-status.json解释。未添加Nav2 Goal工具。
勾选Local costmap或LIO projection切换对照；未知区域不被涂为自由。
本机RViz原生Map显示遇到 indexed_8bit_image GLSL sampler错误，GPU和软件渲染均复现。默认改用RViz MarkerArray/CUBE_LIST显示栅格；原地图话题不变，原生Map显示保留但禁用。

## 启动与停止
当前单LIO运行时才可启动；已运行会拒绝重复启动：
```bash
cd /home/arx/arx-competition-control/navigation
bash static_preview/start.sh
```
本入口只启预览，不启雷达/LIO或底盘。单LIO停止后应按原流程静止初始化，不能通过预览脚本绕过。
当前运行目录见 reports/active-static-preview.txt，preview.pid 是本次预览launch。用SIGINT停止该PID，只停止本次预览，不影响LIO。不要使用泛化pkill或运行北极熊完整实车bringup。

## 验证记录
- 180s静态采样：IMU199.990Hz、点云/LIO约10Hz，时间戳无逆序；底盘位置跨度15.37/7.96/13.80mm。不是定位精度保证。
- 原始185s rosbag留在NUC；后续约23:35–23:39用户确认人工碰动/挪动，约0.39m变化不当作静态漂移。
- 实机感知稳态约10Hz；SLAM/投影地图约0.5Hz；代价地图实测约1.67Hz发布（配置2Hz、更新5Hz）。
- 地面可见点稀疏：每帧约53点在名义地面带；linefit减为60角分区后输出少量地面点。保守复核后仍少，不能宣称完整地面分割质量通过。
- 7项单元/隔离ROS回归测试通过。独立审查发现的圆形误清障、历史地图阻碍清除、下游扫描新鲜度三个问题已修正。
- ROS_DOMAIN_ID=199合成Nav2集成测试：近障碍100；无回波后仍100；真实远处回波后旧格0；未观测区域-1；合成有效路径38点。未把合成数据送入实车域。
- 实车初轮规划返回208无有效路径；越界目标204。连通性检查起点在代价99膨胀区，周围无出口；没有缩小车体或开放未知区来凑路径。最终复测见运行目录plan-probe.json。
- 实际搬动锥桶的标记/消除、完整地图、移动定位、路径跟踪和停车避障仍待验收。

## 文件
prepare.py生成配置与CAD轮廓；preview.launch.py仅白名单节点；bridge.py处理坐标及分类边界；plan_only.py目标与新鲜度监控；grid_display.py兼容显示；health_check.py只读统计；probe_plans.py只调用规划动作；tests/integration_costmap.py强制域199。
部署配置保留北极熊来源，未覆盖仓库原始参数或历史模型。
