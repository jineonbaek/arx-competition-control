# 底盘 MID360S 正装单雷达：部署与验证
2026-11-05。NUC工作目录：/home/arx/arx-competition-control/navigation。
本次运行：reports/bottom-upright-20261105-223312/。

## 完成与当前状态
新外参与车体过滤模型已在NUC生效，底盘单FAST-LIO、底盘位姿转换、三维地图及二维投影栅格正在运行。用户确认静止后完成IMU初始化；之后用户确认人工移动过底盘，故后续约20cm位移不作静态漂移解释。
保存地图定位（map_server + AMCL + cloud-to-scan）已完成数据链路及静止观测更新验证。测试用AMCL/map_server随后停止，当前只保留建图模式，避免将局部测试地图用于后续未知区域移动。
没有启动规划器/控制器，没有发布cmd_vel/CAN运动命令，没有使用轮式里程计、头部LIO或双点云融合。驱动仍登记两台雷达以避免同SDK端口未登记设备报文，头部话题隔离在/unused_head/*，不参与估计。D435未接入。

## 新几何
单位m，车体O为底盘整体外接圆中心的地面投影；导航base_link定义在O，与官方URDF原base_link不是同一点。
- 安装底面中心：(0.185330,-0.000003,0.183000)。
- 光学原点：(0.185330,-0.000003,0.230000)。
- IMU原点：(0.196330,0.023287,0.185880)。
- 车体到雷达/IMU旋转：单位阵，RPY=(0,0,0)，X前/Y左/Z上。
- 内部激光到IMU变换仍为R=I、平移(-0.011,-0.02329,0.04412)。安装翻转不能改内部外参。
- 原front_hang_mount_link及其固定关节删除；lower_mid360_link重新挂到base_link，使用新姿态。
- 生成26个部件模型，底盘雷达网格Z范围0.182999998～0.242999954m，与安装面183mm、设备高60mm一致。

文件：
- config/bottom_upright_20261105.json：当前新配置。
- config/current_bottom_sensor.json：当前单雷达配置与模型路径指针。
- models/lift2s_bottom_upright_20261105/：新URDF、body_voxels.npz及审计JSON。
- 历史config/cad_nominal_20261105.json、models/lift2s_20261105/及三轮录包模型未覆盖，不能用旧倒装启动脚本启动当前新安装。
新支撑板/线缆几何未提供，未虚构网格。机械臂/升降仍按用户此前确认的模型零位过滤，姿态变化需要同步模型。

## 数据链路
/single/lidar_raw → 实际网格过滤 → /single/lidar_filtered → FAST-LIO
/single/imu → FAST-LIO
/single/odometry_raw → 杆臂修正及初始坐标规约 → /localization/odometry
/single/map_raw → 固定世界坐标转换 → /mapping/cloud_map → /mapping/grid

保留原CustomPoint顺序、offset_time、line、tag、reflectivity、timebase；不会把雷达坐标旋转后再误送原始IMU。刚性车体过滤余量2cm，头部名义几何额外使用10cm余量容纳未跟踪柔性；底盘滤波不依赖头部动态里程计。体素分辨率5mm，沿用6.83mm保守离散误差界。FAST-LIO blind设0.2m，IMU断流/时间逆序保护保留0.1s。
示例过滤健康统计：平均每帧17.4ms、最大26.5ms，约42%输入点被删除；此比例包括车体/遮挡相关回波，不是过滤准确率。

TF：
- odom → base_link：底盘位姿。
- base_link → bottom_imu → bottom_livox：新静态外参。
- odom → lio_world：初始化后固定的世界坐标转换。
- FAST-LIO原TF仅在/single/private_lio_tf，避免同一IMU出现两个父节点。
- 地图定位模式另由AMCL发布map → odom。
实际TF查询验证base_link到bottom_livox=(.18533,-.000003,.23)，到IMU=(.19633,.023287,.18588)。

## 实机验证结果
- PTP稳定段：底盘点云和IMU时间类型均1、时间单调。软件PTP不是绝对同步精度标定。
- 稳态检查：IMU约199.99Hz，最大间隔约6ms；原始/过滤点云、LIO/底盘里程计均约10Hz；二维扫描约10Hz，720束中示例约498～503束有回波。
- 首个30秒静态检查底盘位置分轴跨度约7.3/7.4/6.6mm；后续窗口约8.5/8.8/6.5mm。不是定位精度保证，也不代表长时漂移。
- 二维地图5cm分辨率，示例261×295格；原地扫描自由空间有限，大量区域为未知，navigation_validated=false。
- 三维PCD通过/single/map_save保存成功；raw PCD位于lio_world，world-gauge.json保存其到odom变换。二维grid已在odom系。
- 保存地图定位节点生命周期active；显式种子后无运动更新请求使AMCL产生5次有效位姿更新、非零协方差，map到base_link的TF可查询。这验证的是链路，不是位置真值、收敛精度或完整重定位能力。
- 启动时cloud-to-scan因TF缓存未就绪出现少量丢弃/乱序；稳态12秒检查扫描无逆序、最大间隔约0.102s。初次监测器IMU队列过浅导致统计约195Hz，改深队列后约200Hz且连续；并非将此当传感器额定性能不足。

保存地图定位的初始位姿必须显式提供；禁止默认缺失参数时落在地图(0,0,0)。初始化搜索标准差设0.1m/10°，不是标定后的定位精度。
实测发现仅通过set_initial_pose参数初始化可能产生零协方差；改为等待AMCL激活后，使用带非零协方差的SetInitialPose服务。源码依据：[Nav2 Jazzy AMCL](https://api.nav2.org/nav2-jazzy/html/amcl__node_8cpp_source.html)。不通过反复静止更新声称精度收敛。

## 查看当前地图
在NUC图形桌面终端运行：
```bash
bash /home/arx/arx-competition-control/navigation/single_bottom/view.sh
```
RViz固定坐标为odom，默认显示三维地图、当前扫描和轨迹；勾选“2D projected grid”显示二维图。配置不含运动目标工具。

## 保存地图
```bash
cd /home/arx/arx-competition-control/navigation
source sensor_env.sh
ros2 service call /single/map_save std_srvs/srv/Trigger '{}'
```
当前运行目录由 reports/active-bottom-single.txt 指出：
- map_lio_world.pcd、world-gauge.json；
- grid.pgm、grid.yaml、grid.json（约每10秒更新）；
- localization_map/grid.* 是本次定位测试的冻结快照，不会跟随建图更新；
- static-check.json、steady-check.json、final-localization-check.json、tf-check.json；
- mapping.log、filter-health.json、deployment.json。

## 下次重启后的建图启动
必须先确保底盘、升降及机械臂静止。启动脚本会拒绝与已有驱动/LIO重复启动；恢复雷达网口、启动PTP与驱动、审计时钟，再启动单LIO。此封装脚本通过语法检查；其中各步骤本次已逐步实机执行，未为测试脚本额外重启正常运行的系统。
```bash
bash /home/arx/arx-competition-control/navigation/single_bottom/start_mapping.sh --stationary-confirmed
```
保持静止完成初始化，检查日志及数据频率后再移动。若时钟审计失败，不启动LIO。sudo -n需要已有适当授权；脚本不会运行底盘控制程序。

## 保存地图定位模式
应先完成地图覆盖并检查，再冻结地图文件。当前测试快照不作为完整导航地图。
在已有单LIO运行的环境中，显式填写地图中的初始位姿（不是里程计原点），再运行：
```bash
cd /home/arx/arx-competition-control/navigation
source sensor_env.sh
export ARX_SINGLE_RUN="$(cat reports/active-bottom-single.txt)"
export ARX_LOCALIZATION_MAP=/absolute/path/to/saved/grid.yaml
# 必须设置ARX_INITIAL_X、ARX_INITIAL_Y、ARX_INITIAL_YAW；m、rad。
ros2 launch single_bottom/localization.launch.py
```
本启动仅包含点云转扫描、地图服务器、AMCL、生命周期管理和显式初始位姿，不包含运动控制。用同次扫描生成的地图在同一位置启动只证明接口连通；移动定位、不同位置重启及锥桶可见范围仍待专门验证。

## 测试与复核
8项新测试通过：新原点/内部外参、模型翻转平移、纯转动杆臂补偿、初始坐标规约、初始位姿显式/有限性、非零搜索协方差、CustomPoint字段及时间保持/逆序闭锁、协方差传播。另11项mesh/flex和8项独立LIO工具测试通过，共27项。
ROS测试需保留环境PYTHONPATH，并用独立domain：
```bash
ROS_DOMAIN_ID=199 PYTHONPATH="single_bottom:$PYTHONPATH" OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s single_bottom/tests -v
```
时间逆序测试会主动输出一次闭锁错误日志，这是预期测试行为。首次ROS测试因覆盖PYTHONPATH而无法导入rclpy，修正运行命令后全通过。
独立代码复核确认变换/杆臂符号/TF结构与字段保持，提出初始位姿不能默认零的意见已用先失败后通过的测试修正。运行中的AMCL曾零协方差问题亦已增加回归测试并修正。无git仓库，未创建提交。

## 尚未完成
移动建图质量、锥桶各方向遮挡/检测距离、完整地图定位及重启重定位精度尚未验收。当前是底盘单雷达建图模式，可进行下一轮“先记录、再人工移动”的验证；不能据此开启自主行驶。
