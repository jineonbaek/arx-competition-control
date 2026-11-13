轮式里程计＋双 MID360S IMU 融合

> 当前方案已于2026-11-05改为头部MID360S点云＋内置IMU的FAST-LIO。下文轮式融合为历史实现，不作为当前导航里程计。当前头部LIO已恢复，车体base_link输出待完整外参验证。

=============================

本地源码镜像：本目录。NUC部署位置：`/home/arx/arx-competition-control/navigation/scripts/` 和 `config/wheel_imu_fusion/`。

当前状态（2026-11-05，断电后已恢复监测）
----------------------------------

**程序、模拟验证已完成，实车最终里程计尚未启用。** 需要先解决持续轮速反馈、单位/编号/正方向、实际轮心位置的确认。断电后原IMU零偏标定必须重做，恢复时先核验文件。

- 用户确认：三全向轮、轮径150mm（半径0.075m），臂上雷达仍正装、+X朝前、升降锁定；头部倒装、+X朝前。
- 官方 `ARX_Model/LIFT/lift_urdf.zip` 取得三轮CAD位置。它的网格外径约160mm，与实车150mm不一致，所以只借用XY/轴向作为未验证的候选值，半径以用户给出的75mm为准。不能自动将该型号几何标记为实车确认。
- 现有 `/body_information` 无Header；静止采到500条、前三个轮速槽位始终1.6116333，第四个0。不能给缓存重新打时间戳冒充新轮速。
- 只读检查当前SDK二进制：`unpackChassisFeedback` 解析标准CAN ID `0x702`，8字节为4个大端有符号int16；返回值为 `raw*10/32768`。这只证明SDK解码方式，尚未确认固件数值是轮侧rad/s、电机侧速度、目标速度或其他量。
- 静止15秒：CAN有0x706/707/708姿态/角速度/加速度反馈，没有0x702。之后用户手动运动97秒记录中，出现21帧0x702，集中于7.143秒；平均2.800Hz，间隔0.038–1.558秒；停止运动后33秒仍无后续轮速。不能只因运动时存在回包就认定连续里程计可用。
- 同段LIO轨迹约2.64m（含噪声），最大离起点1.476m、最大估计平移速度0.403m/s，证明用户实际有运动。三个SDK槽位约[-0.31,+0.35]、[-8.99,-8.34]、[8.37,9.03]，第四槽0。未据此猜测减速比、单位或直接启用融合。

坐标和几何
----------

底盘参考点建议为三轮轮心三角形的几何中心。**X车前、Y车左、Z向上**。轮心坐标单位m，配置中只用XY；用户可用mm提供，再换算。若是等边三角形，可提供相邻轮心距和哪个轮在车前来推导；反馈通道编号/正方向仍需实测或厂商协议确认。

官方候选：wheel1在(-0.182349,+0.315838)，wheel2在(-0.182349,-0.315838)，wheel3在(+0.364699,0)。这些不是用户实测值，软件默认确认开关均为false。

运动学使用每轮滚动方向t、轮心(x,y)、轮半径r：

`r*omega = tx*vx + ty*vy + (-tx*y + ty*x)*wz`

三轮联合求解vx、vy、wz，SE(2)积分产生轮式位姿及协方差。全向底盘的vy不能被强制为0。

数据通路
--------

1. `wheel_odometry_rx.py` 在can5上创建**只接收**的SocketCAN套接字，过滤0x702、拒绝本机发送回环，使用内核接收时间戳。不创建第二个SDK/CAN控制对象，不发CAN帧，不改变现有驱动。
2. 真实反馈先发布`/wheel/feedback_raw`（JSON，明确SDK值未确认物理单位）。只有配置全部确认且反馈新鲜，才发布`/wheel/odometry`。无新帧不会发布假的零速里程计。轮式节点不发布TF。
3. `imu_yaw_adapter.py` 使用操作员确认静止时的均值去除gyro零偏，并将角速度投影到底盘Z轴；倒装头部自动取正确符号。它输出**虚拟的仅Z角速度有效的IMU测量**，不冒充完整安装外参或绝对航向。
4. 两路`/fusion/imu/{arm,top}_yaw_rate`供EKF使用；orientation和linear_acceleration均明确不可用，角速度X/Y不参与融合。`*_raw_si`提供原生传感器坐标下SI数据，原始加速度由g转换为m/s²。第一阶段不融合加速度，避免重力、升降和未知杆臂影响。
5. `robot_localization`平面EKF只融合轮式vx/vy/wz与两路IMU的wz。轮式位姿不再次加入，以免重复计算同源数据。IMU按各自观测协方差加保守膨胀，不等权硬平均。
6. EKF内部输出`/fusion/odometry_internal`，TF禁用。`fused_odom_gate.py`仅在轮式与两路IMU均新鲜时发布最终`/odometry/filtered`和唯一的`odom→base_link`。输入中断后锁定并停止最终输出，恢复需要重新初始化，不自动继续旧积分。

这里的最终输出目标是**Nav2底盘里程计**。现有FAST-LIO仍直接使用原始雷达/IMU，不会重复注入同一IMU的融合结果；FAST-LIO源码当前也没有外部轮式里程计输入适配。轮速＋陀螺只能提供局部里程计，不能消除长期漂移或替代地图重定位的`map→odom`。

配置与运行
----------

所有命令在NUC导航根目录，先：

```bash
export NAV_ROS_DOMAIN_ID=192
source sensor_env.sh
```

静止平整地面标定（需要操作员确认，原始雷达驱动必须已运行）：

```bash
python3 scripts/calibrate_wheel_imus.py --confirm-static
```

标定与当前boot_id绑定，重启后拒绝旧标定；传感器重新上电/移动安装位置也需要重做，即使NUC没有重启。标定仅对当前刚性安装和初始平面坐标假设有效。

只读输入监测：

```bash
bash scripts/start_wheel_imu_inputs.sh
```

当前wheels.yaml的`geometry_confirmed`、`feedback_order_signs_confirmed`、`feedback_units_confirmed`全为false。它会报告等待有效轮速，不会产生最终有效里程计。不要为了看到输出而把确认开关改成true。轮速噪声参数和rad_per_sdk_unit目前也是候选值。

只停止本导航处理进程（不停止机器人）：

```bash
python3 scripts/stop_wheel_imu.py
```

待参数和反馈全部验证后才运行：

```bash
bash scripts/start_wheel_imu_ekf.sh
```

两个入口共享锁；完整EKF入口在标定缺失时直接拒绝启动。当前没有替换Nav2输入或启用自动导航。现有控制域191和SDK进程未修改，融合域192通过只读CAN反馈取数。

验证与记录
----------

- 7项独立几何/解码/倒装零偏/SE2积分/协方差/断流锁定测试通过。
- 真实`robot_localization`＋最终门控在隔离ROS域197中使用模拟输入通过：340条有效融合输出，速度约[0.2,0.1,0.2]（m/s、m/s、rad/s）；轮速中断后停止输出，恢复消息不会自动解除故障锁定。没有使用硬件输入或CAN发送。
- 双雷达静态标定：每路4116样本、约200Hz；头部去偏前Z轴误差明显大于臂上。保存的协方差反映两路噪声差异，不能宣称“双IMU一定更准”。
- NUC报告`reports/wheel-imu-fusion/`：`can-feedback-audit.json`、`ekf-synthetic-result.json`、`ekf-test-driver.log`。实车运动记录`reports/wheel-imu-motion-20261105-121059/`包含CAN CSV、LIO轨迹、`wheel-feedback-analysis.json`。原始数据留NUC，未导出。
- 断电后已核验CAN记录、21条反馈CSV和模拟EKF报告可读；重新采集双IMU零偏，当前boot_id绑定新标定。10秒实车输入检查两路均约200Hz，去偏wz均值约+9.6e-6/-3.7e-5rad/s；轮式消息0，最终里程计发布者0，诊断明确等待真实轮速和标定确认。因此实际最终融合仍未跑通。
- 重启时USB2CAN已枚举为串口，但SocketCAN网口不存在。只按原控制台参数恢复`/dev/arxcan5→can5`的slcand适配器及link up，没有启动底盘/升降SDK；接口TX计数0。只读10秒收到706/707/708各264帧，702为0。此段没有原底盘控制器的命令负载，与此前人工运动条件不同；不能据此推断运动时永远不会回包。

尚需验证的数据
--------------

用户确认只有ARXroboticsX公开的LIFT与X5官方资料，没有额外协议；后续以官方仓库、当前SDK只读分析和被动实测推进，不再等待用户提供厂商文档。

1. 能否在运动和静止时都以稳定频率持续上报实测轮速或累计编码器位置（建议先争取20–50Hz），并具有明确的采样时序。
2. 0x702中的值到底是什么单位、是否减速前/后、是否真实测量而非目标值，以及四个槽位对应哪三个轮。
3. 各通道正方向、实际轮心位置/轮距；150mm轮径是否需要有效滚动半径标定。

本轮SDK核对（2026-11-05）
------------------------

- 官方LIFT旧Python包的`packChassisCmd2`为三个整数参数，ROS/ROS2包为四个double参数。不能将旧Python头文件与ROS2共享库混用，也不能把四个API槽位解释成实车有四轮。
- 车上`web/hardware/build/lift_controller_web`的RUNPATH明确选择项目`repos/LIFT/body/ROS2/.../libarx_lift_src.so`，SHA256为`426a07074efc420c0a0cf5b7c8fbab07bb67dcec1030a1b8d87cf3ffc8027f66`；与`/home/arx/LIFT`中的ROS2库相同。
- ROS2接收回调调用解包函数，`readFromBuffer()`交换已收数据；已查看的公开接口没有独立的轮速订阅开关。官方ROS2示例400Hz循环直接发布getter返回值，没有轮速接收时间或新数据标志。该发布频率不能证明测量更新率。
- `ARX_all_in_one_readme`在LIFT树中是gitlink（160000），不是可读取的说明文件；其GitHub入口目前404，未取得这部分内容，不能声称已读完整。
- 官方`X7-SDK-V1.5`的文档也包含LIFT接口，但示例轮速接口为四轮，仍未注明轮速单位。不能直接将其轮子编号图套到三全向底盘。
- 只读审计文件保存在NUC `reports/wheel-imu-fusion/sdk-protocol-audit/`：三个版本的反汇编摘录、库哈希及`manifest.json`。没有实例化控制SDK或发送CAN负载，没有替换共享库。
- “反馈是否依赖控制心跳”仍未排除。当前只读采样时控制SDK未运行，先前人工运动时控制SDK运行；不能将两种条件的结果直接归因为运动触发，也不能声称已有不含动作的开启反馈命令。

接下来应在原有人工遥控流程下被动记录静止→前后→左右→转向→静止，结合头部LIO/双IMU比对量纲、时延与符号。若轮速停止后仍不刷新，必须解决反馈连续性或取得有明确定义的编码器源，不能靠长时间保持旧轮速、命令积分或IMU补零来启用最终轮式融合。

参考：

- [LIFT官方ROS2示例](https://github.com/ARXroboticsX/LIFT/blob/006984b693967d4e6aca51952acfd7f9b8f0df64/body/ROS2/src/ARX_LIFT_ros2/arx_lift_controller/src/lift_controller.cpp)
- [官方机身SDK补充文档](https://github.com/ARXroboticsX/X7-SDK-V1.5/blob/75cdd47ea5fcd779d5aab14b1603c497cc92080e/X7%20python%20SDK-V1.5.md)
- [robot_localization配置原则](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ros2/doc/configuring_robot_localization.rst)
- [传感器坐标、单位、协方差要求](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ros2/doc/preparing_sensor_data.rst)
